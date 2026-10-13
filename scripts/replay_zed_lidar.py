#!/usr/bin/env python3
"""ZEDのSVO2とLivoxのrosbag2を時刻を合わせて再生する。

ZED_Explorerで映像を開き、ros2 bag playで点群を配信する。
既定ではRViz2も起動し、必要に応じてCustomMsgをPointCloud2へ変換する。
"""

from __future__ import annotations

import argparse
import datetime as dt
import json
import math
import os
import re
import select
import shutil
import signal
import subprocess
import sys
import tempfile
import time
from dataclasses import dataclass, replace
from pathlib import Path
from typing import Optional

from playback_timeline import (
    PlaybackTimeline,
    inspect_bag_timeline,
    inspect_playback_timeline,
    inspect_svo_timeline,
)

from record_zed import (
    DEFAULT_CONFIG_PATH,
    PROJECT_ROOT,
    RecorderError,
    build_ros_environment,
    load_config,
)


class ReplayError(RuntimeError):
    """再生設定や子プロセスで起きた想定内のエラー。"""


@dataclass
class ReplaySettings:
    """設定ファイルのreplay項目で指定する再生条件。"""

    point_cloud_topic: str = "/livox/lidar"
    point_cloud_type: str = "auto"
    converted_point_cloud_topic: str = "/livox/points"
    fixed_frame: str = "livox_frame"
    rviz_enabled: bool = True
    rviz_executable: str = "rviz2"
    bag_rate: float = 1.0
    bag_delay_seconds: float = 0.0
    loop: bool = False
    rviz_config: Optional[Path] = None


@dataclass
class ManagedProcess:
    """子プロセスと画面表示用の名前を組にする。"""

    name: str
    process: subprocess.Popen


def _resolve_path(value: str | Path, base_dir: Path) -> Path:
    path = Path(value).expanduser()
    if not path.is_absolute():
        path = base_dir / path
    return path.resolve()


def _read_json_object(config_path: Path) -> dict:
    try:
        with config_path.open("r", encoding="utf-8") as config_file:
            raw_config = json.load(config_file)
    except FileNotFoundError as exc:
        raise ReplayError(f"Config file not found: {config_path}") from exc
    except json.JSONDecodeError as exc:
        raise ReplayError(
            f"Invalid JSON in config file {config_path}: {exc.msg}"
        ) from exc
    except OSError as exc:
        raise ReplayError(f"Could not read config file {config_path}: {exc}") from exc
    if not isinstance(raw_config, dict):
        raise ReplayError("Config file must contain a JSON object.")
    return raw_config


def _config_string(raw: dict, key: str, default: str) -> str:
    value = raw.get(key, default)
    if not isinstance(value, str) or not value.strip():
        raise ReplayError(f"Config value 'replay.{key}' must be a non-empty string.")
    return value.strip()


def _config_bool(raw: dict, key: str, default: bool) -> bool:
    value = raw.get(key, default)
    if not isinstance(value, bool):
        raise ReplayError(f"Config value 'replay.{key}' must be true or false.")
    return value


def _config_number(
    raw: dict, key: str, default: float, positive: bool = False
) -> float:
    value = raw.get(key, default)
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ReplayError(f"Config value 'replay.{key}' must be a number.")
    if not math.isfinite(float(value)):
        raise ReplayError(f"Config value 'replay.{key}' must be finite.")
    invalid = value <= 0 if positive else value < 0
    if invalid:
        qualifier = "positive" if positive else "non-negative"
        raise ReplayError(f"Config value 'replay.{key}' must be {qualifier}.")
    return float(value)


def load_replay_settings(config_path: Path, livox_topic: str) -> ReplaySettings:
    """再生設定を読み込み、録画設定と共通の設定ファイルを利用する。"""

    raw_config = _read_json_object(config_path)
    replay_raw = raw_config.get("replay", {})
    if not isinstance(replay_raw, dict):
        raise ReplayError("Config value 'replay' must be a JSON object.")

    settings = ReplaySettings()
    settings.point_cloud_topic = _config_string(
        replay_raw, "point_cloud_topic", livox_topic
    )
    settings.point_cloud_type = _config_string(
        replay_raw, "point_cloud_type", settings.point_cloud_type
    ).lower()
    if settings.point_cloud_type not in {"auto", "pointcloud2", "custom"}:
        raise ReplayError(
            "Config value 'replay.point_cloud_type' must be auto, pointcloud2, or custom."
        )
    settings.converted_point_cloud_topic = _config_string(
        replay_raw,
        "converted_point_cloud_topic",
        settings.converted_point_cloud_topic,
    )
    settings.fixed_frame = _config_string(
        replay_raw, "fixed_frame", settings.fixed_frame
    )
    settings.rviz_enabled = _config_bool(
        replay_raw, "rviz_enabled", settings.rviz_enabled
    )
    settings.rviz_executable = _config_string(
        replay_raw, "rviz_executable", settings.rviz_executable
    )
    settings.bag_rate = _config_number(
        replay_raw, "bag_rate", settings.bag_rate, positive=True
    )
    settings.bag_delay_seconds = _config_number(
        replay_raw, "bag_delay_seconds", settings.bag_delay_seconds
    )
    settings.loop = _config_bool(replay_raw, "loop", settings.loop)

    rviz_config = replay_raw.get("rviz_config")
    if rviz_config is not None:
        if not isinstance(rviz_config, str) or not rviz_config.strip():
            raise ReplayError(
                "Config value 'replay.rviz_config' must be a path or null."
            )
        settings.rviz_config = _resolve_path(rviz_config, config_path.parent)
    return settings


def validate_svo_path(svo_path: Path) -> None:
    if not svo_path.is_file():
        raise ReplayError(f"SVO file does not exist: {svo_path}")
    if svo_path.suffix.lower() not in {".svo", ".svo2"}:
        raise ReplayError(
            f"SVO input must have a .svo or .svo2 extension: {svo_path}"
        )
    if svo_path.stat().st_size <= 0:
        raise ReplayError(f"SVO file is empty (0 bytes): {svo_path}")


def validate_bag_path(bag_path: Path) -> None:
    """子プロセス起動前にrosbag2の保存物を検証する。"""

    if not bag_path.is_dir():
        raise ReplayError(
            f"LiDAR bag directory does not exist: {bag_path}\n"
            "Use --bag to specify the rosbag2 directory containing metadata.yaml."
        )
    metadata = bag_path / "metadata.yaml"
    if not metadata.is_file() or metadata.stat().st_size <= 0:
        raise ReplayError(f"rosbag2 metadata.yaml is missing or empty: {metadata}")
    storage_files = list(bag_path.glob("*.mcap")) + list(bag_path.glob("*.db3"))
    if not storage_files:
        raise ReplayError(
            f"No .mcap or .db3 storage file was found in rosbag2 directory: {bag_path}"
        )
    if not any(path.is_file() and path.stat().st_size > 0 for path in storage_files):
        raise ReplayError(f"All rosbag2 storage files are empty: {bag_path}")


def infer_bag_path(svo_path: Path, bag_dir: Path) -> Path:
    """同じ録画フォルダ内を優先し、旧形式の同名rosbag2も探す。"""

    colocated_candidate = svo_path.parent / "lidar"
    if colocated_candidate.exists():
        return colocated_candidate

    direct_candidate = bag_dir / svo_path.stem
    if direct_candidate.exists():
        return direct_candidate

    candidates = sorted(
        candidate
        for candidate in bag_dir.glob(f"{svo_path.stem}*")
        if candidate.is_dir()
    )
    if len(candidates) == 1:
        return candidates[0]
    if not candidates:
        raise ReplayError(
            f"Could not infer a LiDAR bag for {svo_path.name}. "
            f"Expected {direct_candidate}; pass --bag <rosbag2-directory>."
        )
    names = ", ".join(candidate.name for candidate in candidates)
    raise ReplayError(
        f"Multiple LiDAR bags match {svo_path.stem}: {names}. "
        "Pass the intended one with --bag."
    )


def _shell_quote(value: str) -> str:
    """画面にコマンドを表示するためだけに引数を引用する。"""

    import shlex

    return shlex.quote(value)


def build_bag_play_command(
    ros2_executable: str,
    bag_path: Path,
    settings: ReplaySettings,
) -> list[str]:
    """ros2 bag playの起動引数を一か所で組み立てる。"""

    command = [
        ros2_executable,
        "bag",
        "play",
        os.fspath(bag_path),
        "--clock",
        "--rate",
        str(settings.bag_rate),
        "--disable-keyboard-controls",
    ]
    if settings.loop:
        command.append("--loop")
    if settings.bag_delay_seconds > 0:
        command.extend(["--delay", str(settings.bag_delay_seconds)])
    return command


def build_converter_command(
    input_topic: str, output_topic: str, python_executable: str = sys.executable
) -> list[str]:
    """必要な場合のCustomMsg変換プロセスの引数を作る。"""

    converter = Path(__file__).with_name("convert_livox_custom.py")
    return [
        python_executable,
        os.fspath(converter),
        "--input-topic",
        input_topic,
        "--output-topic",
        output_topic,
    ]


def detect_point_cloud_type(
    ros2_executable: str,
    environment: dict[str, str],
    bag_path: Path,
    topic: str,
    timeout: float = 10.0,
) -> Optional[str]:
    """ros2 bag infoから指定トピックのメッセージ型を判定する。"""

    try:
        result = subprocess.run(
            [ros2_executable, "bag", "info", os.fspath(bag_path)],
            env=environment,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            timeout=timeout,
            check=False,
        )
    except (OSError, subprocess.TimeoutExpired) as exc:
        raise ReplayError(f"Could not inspect LiDAR bag topic types: {exc}") from exc
    if result.returncode != 0:
        detail = result.stderr.strip().splitlines()
        raise ReplayError(
            "ros2 bag info failed"
            + (f": {detail[-1]}" if detail else f" (exit {result.returncode})")
        )

    # ROS 2 Jazzyのトピック型は次の1行形式から読み取る:
    # Topic: /livox/lidar | Type: livox_ros_driver2/msg/CustomMsg | Count: ...
    topic_pattern = re.compile(
        r"Topic:\s*(\S+)\s*\|\s*Type:\s*(\S+)", re.IGNORECASE
    )
    for match in topic_pattern.finditer(result.stdout):
        if match.group(1) != topic:
            continue
        message_type = match.group(2)
        if message_type == "sensor_msgs/msg/PointCloud2":
            return "pointcloud2"
        if message_type.endswith("/CustomMsg"):
            return "custom"
        return message_type
    return None


def build_rviz_config(point_cloud_topic: str, fixed_frame: str) -> str:
    """録画済み点群を表示する最小限のRViz2設定を作る。"""

    # JSONの引用符はYAMLでも有効。トピック名や座標系名の記号を保護する。
    topic_yaml = json.dumps(point_cloud_topic)
    frame_yaml = json.dumps(fixed_frame)
    return f'''Panels:
  - Class: rviz_common/Displays
    Name: Displays
Visualization Manager:
  Class: ""
  Displays:
    - Alpha: 1
      Autocompute Intensity Bounds: true
      Autocompute Value Bounds: true
      Class: rviz_default_plugins/PointCloud2
      Color: 255; 255; 255
      Color Transformer: Intensity
      Enabled: true
      Name: LiDAR PointCloud2
      Position Transformer: XYZ
      Queue Size: 10
      Selectable: true
      Size (Pixels): 3
      Style: Flat Squares
      Topic: {topic_yaml}
      Use Fixed Frame: true
      Value: true
    - Alpha: 0.35
      Class: rviz_default_plugins/Grid
      Color: 160; 160; 160
      Enabled: true
      Name: Grid
      Plane: XY
      Plane Cell Count: 20
      Reference Frame: {frame_yaml}
      Value: true
  Enabled: true
  Global Options:
    Background Color: 48; 48; 48
    Fixed Frame: {frame_yaml}
    Frame Rate: 30
  Name: root
  Tools:
    - Class: rviz_default_plugins/Interact
    - Class: rviz_default_plugins/MoveCamera
    - Class: rviz_default_plugins/Select
  Views:
    Current:
      Class: rviz_default_plugins/Orbit
      Distance: 8
      Focal Point: 0; 0; 0
      Pitch: 0.5
      Yaw: 0.8
Window Geometry:
  Height: 800
  Width: 1200
'''


class ReplayRunner:
    """ZED、rosbag2、RViz2の子プロセスを管理して終了させる。"""

    def __init__(self, log_path: Path) -> None:
        self.log_path = log_path
        self.log_file = None
        self.processes: list[ManagedProcess] = []

    def open_log(self) -> None:
        self.log_path.parent.mkdir(parents=True, exist_ok=True)
        self.log_file = self.log_path.open("a", encoding="utf-8")

    def start(
        self,
        name: str,
        command: list[str],
        environment: Optional[dict[str, str]] = None,
        capture_output: bool = False,
    ) -> subprocess.Popen:
        stdout = self.log_file if capture_output else None
        stderr = subprocess.STDOUT if capture_output else None
        try:
            process = subprocess.Popen(
                command,
                env=environment,
                stdin=subprocess.DEVNULL,
                stdout=stdout,
                stderr=stderr,
                start_new_session=True,
            )
        except OSError as exc:
            raise ReplayError(f"Could not start {name}: {exc}") from exc
        self.processes.append(ManagedProcess(name, process))
        return process

    def forget(self, process: subprocess.Popen) -> None:
        self.processes = [
            managed for managed in self.processes if managed.process is not process
        ]

    def stop_all(self) -> None:
        """各再生プロセスへ正常終了を要求し、待機後は停止を完了させる。"""

        active = [
            managed for managed in self.processes if managed.process.poll() is None
        ]
        for managed in active:
            try:
                managed.process.send_signal(signal.SIGINT)
            except ProcessLookupError:
                pass

        for managed in active:
            try:
                managed.process.wait(timeout=8.0)
            except subprocess.TimeoutExpired:
                print(
                    f"{managed.name} did not stop after SIGINT; sending SIGTERM.",
                    file=sys.stderr,
                )
                managed.process.terminate()
                try:
                    managed.process.wait(timeout=3.0)
                except subprocess.TimeoutExpired:
                    print(
                        f"{managed.name} is still running; sending SIGKILL.",
                        file=sys.stderr,
                    )
                    managed.process.kill()
                    managed.process.wait()

        if self.log_file is not None and not self.log_file.closed:
            self.log_file.close()


def _unique_log_path(svo_path: Path) -> Path:
    timestamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
    base = PROJECT_ROOT / "logs" / f"replay_{svo_path.stem}_{timestamp}.log"
    candidate = base
    number = 1
    while candidate.exists():
        candidate = base.with_name(f"{base.stem}_{number:02d}{base.suffix}")
        number += 1
    return candidate


def _check_executable(
    executable: str, display_name: str, search_path: Optional[str] = None
) -> str:
    resolved = shutil.which(executable, path=search_path)
    if not resolved:
        raise ReplayError(f"{display_name} was not found in PATH: {executable}")
    return resolved


def _print_command(label: str, command: list[str]) -> None:
    print(f"{label:<8}: " + " ".join(_shell_quote(argument) for argument in command))


def _wait_until_alive(process: subprocess.Popen, name: str, timeout: float = 1.0) -> None:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        return_code = process.poll()
        if return_code is not None:
            raise ReplayError(f"{name} exited during startup (exit code {return_code}).")
        time.sleep(0.05)


def _wait_before_start(seconds: float) -> None:
    """2つ目の再生を始める時刻まで短い間隔で待機する。"""

    deadline = time.monotonic() + max(0.0, seconds)
    while True:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            return
        time.sleep(min(0.1, remaining))


def wait_for_replay(
    runner: ReplayRunner,
    zed_process: subprocess.Popen,
    bag_process: subprocess.Popen,
    converter_process: Optional[subprocess.Popen] = None,
) -> str:
    """Enter、Ctrl+C、または主再生プロセスの終了を待つ。"""

    try:
        stdin_fd = sys.stdin.fileno()
    except (AttributeError, OSError, ValueError):
        input()
        return "enter"

    rviz_reported = False
    while True:
        zed_return_code = zed_process.poll()
        bag_return_code = bag_process.poll()
        if zed_return_code is not None:
            print(f"\nZED_Explorer playback ended (exit code {zed_return_code}).")
            return "process-exited"
        if bag_return_code is not None:
            print(f"\nrosbag2 playback ended (exit code {bag_return_code}).")
            return "process-exited"
        if converter_process is not None and converter_process.poll() is not None:
            print(
                "\nWARNING: Livox CustomMsg converter stopped; "
                "the LiDAR bag is still playing but RViz2 will receive no points."
            )
            converter_process = None

        for managed in list(runner.processes):
            if managed.name == "RViz2" and managed.process.poll() is not None:
                if not rviz_reported:
                    print(
                        "\nWARNING: RViz2 closed; rosbag2 playback will continue. "
                        f"See {runner.log_path}."
                    )
                    rviz_reported = True
                runner.forget(managed.process)

        try:
            # ZEDはSVO終端で先頭へ戻るため、rosbag終了を短い間隔で確認する。
            ready, _, _ = select.select([stdin_fd], [], [], 0.05)
        except (OSError, ValueError):
            input()
            return "enter"
        if ready:
            sys.stdin.readline()
            return "enter"


def _parse_args(argv: Optional[list[str]]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("svo", type=Path, help="SVO or SVO2 file to open")
    parser.add_argument(
        "--bag",
        "--lidar-bag",
        dest="bag",
        type=Path,
        default=None,
        help="rosbag2 directory; inferred from the SVO filename when omitted",
    )
    parser.add_argument(
        "--config",
        type=Path,
        default=DEFAULT_CONFIG_PATH,
        help=f"JSON config file (default: {DEFAULT_CONFIG_PATH})",
    )
    parser.add_argument(
        "--point-cloud-topic",
        "--topic",
        dest="point_cloud_topic",
        default=None,
        help="Recorded LiDAR topic to replay and display",
    )
    parser.add_argument(
        "--point-cloud-type",
        choices=("auto", "pointcloud2", "custom"),
        default=None,
        help="Recorded LiDAR message type (auto-detected by default)",
    )
    parser.add_argument(
        "--converted-point-cloud-topic",
        default=None,
        help="Output topic for CustomMsg conversion (default: /livox/points)",
    )
    parser.add_argument(
        "--fixed-frame",
        "--frame",
        dest="fixed_frame",
        default=None,
        help="RViz2 fixed frame; must match the PointCloud2 header frame_id",
    )
    parser.add_argument("--rate", type=float, default=None, help="LiDAR bag playback rate")
    parser.add_argument(
        "--bag-delay",
        type=float,
        default=None,
        help="Additional LiDAR delay after automatic recorded-timestamp alignment",
    )
    parser.add_argument("--loop", action="store_true", help="Loop the LiDAR bag")
    parser.add_argument(
        "--no-rviz",
        dest="rviz",
        action="store_false",
        help="Publish the LiDAR bag without opening RViz2",
    )
    parser.add_argument(
        "--rviz",
        dest="rviz",
        action="store_true",
        help="Open RViz2 even when disabled in config.json",
    )
    parser.set_defaults(rviz=None)
    parser.add_argument(
        "--rviz-config", type=Path, default=None, help="Use an existing RViz2 config"
    )
    parser.add_argument(
        "--zed-explorer",
        default=None,
        help="Override the ZED_Explorer executable from the config file",
    )
    parser.add_argument(
        "--ros2",
        default=None,
        help="Override the ros2 executable from livox config",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Validate paths and print the commands without starting playback",
    )
    return parser.parse_args(argv)


def main(argv: Optional[list[str]] = None) -> int:
    args = _parse_args(argv)
    runner: Optional[ReplayRunner] = None
    temporary_rviz_config: Optional[Path] = None
    try:
        config_path = _resolve_path(args.config, Path.cwd())
        recorder_settings = load_config(config_path)
        replay_settings = load_replay_settings(
            config_path, recorder_settings.livox.detection_topic
        )

        if args.point_cloud_topic is not None:
            replay_settings.point_cloud_topic = args.point_cloud_topic.strip()
        if args.point_cloud_type is not None:
            replay_settings.point_cloud_type = args.point_cloud_type
        if args.converted_point_cloud_topic is not None:
            replay_settings.converted_point_cloud_topic = (
                args.converted_point_cloud_topic.strip()
            )
        if args.fixed_frame is not None:
            replay_settings.fixed_frame = args.fixed_frame.strip()
        if args.rate is not None:
            replay_settings.bag_rate = args.rate
        if args.bag_delay is not None:
            replay_settings.bag_delay_seconds = args.bag_delay
        if args.loop:
            replay_settings.loop = True
        if args.rviz is not None:
            replay_settings.rviz_enabled = args.rviz
        if args.rviz_config is not None:
            replay_settings.rviz_config = _resolve_path(args.rviz_config, Path.cwd())
        if args.zed_explorer is not None:
            recorder_settings.zed_explorer = args.zed_explorer.strip()
        if args.ros2 is not None:
            recorder_settings.livox.ros2_executable = args.ros2.strip()

        if not replay_settings.point_cloud_topic:
            raise ReplayError("LiDAR point cloud topic must not be empty.")
        if replay_settings.point_cloud_type not in {"auto", "pointcloud2", "custom"}:
            raise ReplayError(
                "LiDAR point cloud type must be auto, pointcloud2, or custom."
            )
        if not replay_settings.converted_point_cloud_topic:
            raise ReplayError("Converted PointCloud2 topic must not be empty.")
        if not replay_settings.fixed_frame:
            raise ReplayError("RViz2 fixed frame must not be empty.")
        if not math.isfinite(replay_settings.bag_rate) or replay_settings.bag_rate <= 0:
            raise ReplayError("LiDAR bag playback rate must be positive and finite.")
        if (
            not math.isfinite(replay_settings.bag_delay_seconds)
            or replay_settings.bag_delay_seconds < 0
        ):
            raise ReplayError("LiDAR bag delay must be non-negative and finite.")

        svo_path = _resolve_path(args.svo, Path.cwd())
        validate_svo_path(svo_path)
        bag_path = (
            _resolve_path(args.bag, Path.cwd())
            if args.bag is not None
            else infer_bag_path(svo_path, recorder_settings.livox.output_dir)
        )
        validate_bag_path(bag_path)

        timeline = inspect_playback_timeline(svo_path, bag_path)
        svo_launch_delay_seconds = 0.0
        bag_timeline_delay_seconds = 0.0
        if timeline is not None:
            svo_launch_delay_seconds = timeline.svo_start_delay_seconds
            bag_timeline_delay_seconds = timeline.bag_start_delay_seconds

        effective_bag_delay_seconds = (
            replay_settings.bag_delay_seconds + bag_timeline_delay_seconds
        )
        command_settings = replace(
            replay_settings,
            bag_delay_seconds=effective_bag_delay_seconds,
        )
        zed_command = [recorder_settings.zed_explorer, os.fspath(svo_path)]
        ros2_command = build_bag_play_command(
            recorder_settings.livox.ros2_executable,
            bag_path,
            command_settings,
        )
        converter_command: Optional[list[str]] = None
        display_topic = replay_settings.point_cloud_topic
        if replay_settings.rviz_enabled and replay_settings.point_cloud_type == "custom":
            converter_command = build_converter_command(
                replay_settings.point_cloud_topic,
                replay_settings.converted_point_cloud_topic,
            )
            display_topic = replay_settings.converted_point_cloud_topic
        rviz_command: Optional[list[str]] = None
        if replay_settings.rviz_enabled:
            rviz_command = [replay_settings.rviz_executable]
            if replay_settings.rviz_config is not None:
                rviz_command.extend(["-d", os.fspath(replay_settings.rviz_config)])
            else:
                rviz_command.extend(["-d", "<generated replay config>"])

        print("=" * 56)
        print(" ZED + LiDAR REPLAY")
        print("=" * 56)
        print(f"SVO        : {svo_path}")
        print(f"LiDAR bag  : {bag_path}")
        print(f"LiDAR topic: {replay_settings.point_cloud_topic}")
        print(f"Message type: {replay_settings.point_cloud_type}")
        print(f"RViz topic : {display_topic}")
        print(f"Fixed frame: {replay_settings.fixed_frame}")
        if timeline is not None:
            print(
                "Recorded timeline: "
                f"SVO {timeline.svo_duration_seconds:.3f}s / "
                f"LiDAR {timeline.bag_duration_seconds:.3f}s"
            )
            print(
                "Automatic start alignment: "
                f"SVO +{svo_launch_delay_seconds:.3f}s / "
                f"LiDAR +{bag_timeline_delay_seconds:.3f}s"
            )
        else:
            print(
                "Recorded timestamp alignment unavailable; "
                "using process-start synchronization."
            )
        print()
        _print_command("ZED", zed_command)
        _print_command("ROS 2", ros2_command)
        if converter_command is not None:
            _print_command("Convert", converter_command)
        if rviz_command is not None:
            _print_command("RViz2", rviz_command)

        if args.dry_run:
            print("\nDry run: no playback process was started.")
            return 0

        zed_explorer = _check_executable(
            recorder_settings.zed_explorer, "ZED_Explorer"
        )
        ros_environment = build_ros_environment(recorder_settings.livox)
        ros2_executable = _check_executable(
            recorder_settings.livox.ros2_executable,
            "ros2",
            search_path=ros_environment.get("PATH"),
        )
        zed_command[0] = zed_explorer
        ros2_command[0] = ros2_executable

        if replay_settings.rviz_enabled and replay_settings.point_cloud_type == "auto":
            detected_type = detect_point_cloud_type(
                ros2_executable,
                ros_environment,
                bag_path,
                replay_settings.point_cloud_topic,
            )
            if detected_type is None:
                raise ReplayError(
                    f"Topic {replay_settings.point_cloud_topic} was not found in "
                    f"rosbag2 metadata. Use --topic or --point-cloud-type explicitly."
                )
            if detected_type not in {"pointcloud2", "custom"}:
                raise ReplayError(
                    f"Topic {replay_settings.point_cloud_topic} has unsupported "
                    f"message type: {detected_type}"
                )
            replay_settings.point_cloud_type = detected_type
            print(f"Detected LiDAR message type: {detected_type}")

        if replay_settings.rviz_enabled and replay_settings.point_cloud_type == "custom":
            converter_command = build_converter_command(
                replay_settings.point_cloud_topic,
                replay_settings.converted_point_cloud_topic,
            )
            display_topic = replay_settings.converted_point_cloud_topic
            print(
                "Livox CustomMsg detected; converting to "
                f"PointCloud2 on {display_topic}."
            )

        if replay_settings.rviz_enabled:
            rviz_executable = _check_executable(
                replay_settings.rviz_executable,
                "RViz2",
                search_path=ros_environment.get("PATH"),
            )
            if replay_settings.rviz_config is not None:
                if not replay_settings.rviz_config.is_file():
                    raise ReplayError(
                        f"RViz2 config file does not exist: {replay_settings.rviz_config}"
                    )
            else:
                with tempfile.NamedTemporaryFile(
                    mode="w",
                    encoding="utf-8",
                    suffix=".rviz",
                    prefix="experiment_replay_",
                    delete=False,
                ) as rviz_file:
                    rviz_file.write(
                        build_rviz_config(
                            display_topic,
                            replay_settings.fixed_frame,
                        )
                    )
                    temporary_rviz_config = Path(rviz_file.name)
                rviz_command = [rviz_executable, "-d", os.fspath(temporary_rviz_config)]
            if rviz_command is not None:
                rviz_command[0] = rviz_executable

        log_path = _unique_log_path(svo_path)
        runner = ReplayRunner(log_path)
        runner.open_log()
        command_list = [zed_command, ros2_command]
        if converter_command is not None:
            command_list.append(converter_command)
        if rviz_command is not None:
            command_list.append(rviz_command)
        runner.log_file.write(
            "Replay commands:\n"
            + "\n".join(
                " ".join(_shell_quote(argument) for argument in command)
                for command in command_list
            )
            + "\n"
        )
        runner.log_file.flush()

        # 点群の先頭を受信できるよう、RVizを先に起動して購読を準備する。
        if rviz_command is not None:
            rviz_process = runner.start(
                "RViz2", rviz_command, ros_environment, capture_output=True
            )
            time.sleep(0.2)
            if rviz_process.poll() is not None:
                print(
                    "WARNING: RViz2 could not be started; continuing with "
                    f"ZED and rosbag2. See {log_path}."
                )
                runner.forget(rviz_process)

        converter_process: Optional[subprocess.Popen] = None
        if converter_command is not None:
            converter_process = runner.start(
                "Livox converter",
                converter_command,
                ros_environment,
                capture_output=True,
            )
            _wait_until_alive(converter_process, "Livox CustomMsg converter")

        # 記録された共通時刻を基準に、先に記録を始めた媒体から再生する。
        # LiDARが先ならrosbagを先に起動し、ZEDが先なら--delayでLiDARを遅らせる。
        if svo_launch_delay_seconds > 0:
            bag_process = runner.start(
                "rosbag2", ros2_command, ros_environment, capture_output=True
            )
            print(
                f"Waiting {svo_launch_delay_seconds:.3f}s for the recorded "
                "SVO start offset..."
            )
            _wait_before_start(svo_launch_delay_seconds)
            zed_process = runner.start("ZED_Explorer", zed_command)
        else:
            zed_process = runner.start("ZED_Explorer", zed_command)
            bag_process = runner.start(
                "rosbag2", ros2_command, ros_environment, capture_output=True
            )
        _wait_until_alive(zed_process, "ZED_Explorer")
        _wait_until_alive(bag_process, "rosbag2")

        print("\nPlayback started. Press ENTER or Ctrl+C to stop both streams.")
        print(f"Replay log: {log_path}")
        stop_reason = wait_for_replay(
            runner, zed_process, bag_process, converter_process
        )
        if stop_reason == "process-exited":
            print("Stopping the remaining playback process...")
        return 0
    except KeyboardInterrupt:
        print("\nStopping playback...", file=sys.stderr)
        return 130
    except (RecorderError, ReplayError, OSError, ValueError) as exc:
        print(f"Error: {exc}", file=sys.stderr)
        return 1
    finally:
        if runner is not None:
            runner.stop_all()
        if temporary_rviz_config is not None:
            try:
                temporary_rviz_config.unlink(missing_ok=True)
            except OSError:
                pass


if __name__ == "__main__":
    raise SystemExit(main())
