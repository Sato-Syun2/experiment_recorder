#!/usr/bin/env python3
"""ZEDのSVO2とLivoxのrosbag2を試行単位で録画する。

ZEDを必須、Livoxを任意として扱う。終了時は各プロセスへSIGINTを送り、
書き込みを正常に完了させる。
"""

from __future__ import annotations

import argparse
import contextlib
import datetime as dt
import json
import os
import re
import select
import shutil
import signal
import subprocess
import sys
import time
import unicodedata
from pathlib import Path
from typing import Optional, Union

from experiment_plan import DEFAULT_PLAN_PATH, PlanError, load_run_order
from recording_progress import ProgressError, RecordingProgress


PROJECT_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_OUTPUT_DIR = Path("/data/experiment_data/recordings")
DEFAULT_CONFIG_PATH = PROJECT_ROOT / "config.json"
DEFAULT_CONDITION = "test"
DEFAULT_TRIAL = 1
DEFAULT_GROUP_NUMBER = 1
DEFAULT_FILENAME_TEMPLATE = "{condition}_{trial:03d}_{timestamp}.svo2"
DEFAULT_COMPRESSION_MODE = 1
DEFAULT_RESOLUTION = "HD720"
DEFAULT_FPS = 30
DEFAULT_LIVOX_OUTPUT_DIR = DEFAULT_OUTPUT_DIR
DEFAULT_LIVOX_TOPICS = ["/livox/lidar", "/livox/imu"]
DEFAULT_LIVOX_DETECTION_TOPIC = "/livox/lidar"
DEFAULT_ROS_SETUP_FILES = [
    Path("/opt/ros/jazzy/setup.bash"),
    Path("/home/hp-ubuntu/ros2_ws/install/setup.bash"),
]
STOP_WAIT_WARNING_SECONDS = 30.0


class RecorderError(RuntimeError):
    """録画設定や子プロセスで起きた想定内のエラー。"""


class LivoxSettings:
    """Livoxドライバとrosbag2の設定値。"""

    def __init__(self) -> None:
        self.enabled = True
        self.output_dir = DEFAULT_LIVOX_OUTPUT_DIR
        self.topics = list(DEFAULT_LIVOX_TOPICS)
        self.detection_topic = DEFAULT_LIVOX_DETECTION_TOPIC
        self.connection_timeout_seconds = 5.0
        self.sensor_ip = "192.168.1.197"
        self.ping_timeout_seconds = 1.0
        self.ros2_executable = "ros2"
        self.setup_files = list(DEFAULT_ROS_SETUP_FILES)
        self.start_driver = True
        self.driver_package = "livox_ros_driver2"
        self.driver_launch_file = "rviz_MID360s_launch.py"
        self.storage = "mcap"


class LivoxSession:
    """1試行中に使用するLivoxのプロセス、ログ、接続状態。"""

    def __init__(self, settings: LivoxSettings) -> None:
        self.settings = settings
        self.ros2_executable: Optional[str] = None
        self.environment: Optional[dict[str, str]] = None
        self.driver_process: Optional[subprocess.Popen] = None
        self.driver_log = None
        self.driver_log_path: Optional[Path] = None
        self.bag_process: Optional[subprocess.Popen] = None
        self.bag_log = None
        self.bag_log_path: Optional[Path] = None
        self.connected = False
        self.reason = "not checked"


class RecorderSettings:
    """config.jsonとCLI指定から組み立てる録画設定。"""

    def __init__(
        self,
        trial: int = DEFAULT_TRIAL,
        output_dir: Path = DEFAULT_OUTPUT_DIR,
        group_number: int = DEFAULT_GROUP_NUMBER,
        compression_mode: int = DEFAULT_COMPRESSION_MODE,
        resolution: str = DEFAULT_RESOLUTION,
        fps: int = DEFAULT_FPS,
        zed_explorer: str = "ZED_Explorer",
        livox: Optional[LivoxSettings] = None,
    ) -> None:
        self.trial = trial
        self.output_dir = output_dir
        self.group_number = group_number
        self.compression_mode = compression_mode
        self.resolution = resolution
        self.fps = fps
        self.zed_explorer = zed_explorer
        self.livox = livox or LivoxSettings()


def _resolve_path(value: Union[str, Path], base_dir: Path) -> Path:
    """チルダを展開し、相対パスは設定ファイルの場所を基準に解決する。"""

    path = Path(value).expanduser()
    if not path.is_absolute():
        path = base_dir / path
    return path.resolve()


def _text_setting(raw_config: dict, key: str, default: str) -> str:
    value = raw_config.get(key, default)
    if not isinstance(value, str):
        raise RecorderError(f"Config value '{key}' must be a string.")
    return value


def _bool_setting(raw_config: dict, key: str, default: bool) -> bool:
    value = raw_config.get(key, default)
    if not isinstance(value, bool):
        raise RecorderError(f"Config value '{key}' must be true or false.")
    return value


def load_livox_config(raw_config: dict, base_dir: Path) -> LivoxSettings:
    """任意のlivox設定を読み込み、接続と保存に必要な値を検証する。"""

    livox_raw = raw_config.get("livox", {})
    if not isinstance(livox_raw, dict):
        raise RecorderError("Config value 'livox' must be a JSON object.")

    settings = LivoxSettings()
    settings.enabled = _bool_setting(livox_raw, "enabled", settings.enabled)
    settings.output_dir = _resolve_path(
        livox_raw.get("output_dir", os.fspath(DEFAULT_LIVOX_OUTPUT_DIR)), base_dir
    )
    settings.detection_topic = _text_setting(
        livox_raw, "detection_topic", settings.detection_topic
    ).strip()
    settings.sensor_ip = _text_setting(
        livox_raw, "sensor_ip", settings.sensor_ip
    ).strip()
    settings.ros2_executable = _text_setting(
        livox_raw, "ros2_executable", settings.ros2_executable
    ).strip()
    settings.start_driver = _bool_setting(
        livox_raw, "start_driver", settings.start_driver
    )
    settings.driver_package = _text_setting(
        livox_raw, "driver_package", settings.driver_package
    ).strip()
    settings.driver_launch_file = _text_setting(
        livox_raw, "driver_launch_file", settings.driver_launch_file
    ).strip()
    settings.storage = _text_setting(
        livox_raw, "storage", settings.storage
    ).strip()

    topics = livox_raw.get("topics", settings.topics)
    if not isinstance(topics, list) or not topics or not all(
        isinstance(topic, str) and topic.strip() for topic in topics
    ):
        raise RecorderError("Config value 'livox.topics' must be a non-empty list of topics.")
    settings.topics = [topic.strip() for topic in topics]

    setup_files = livox_raw.get("setup_files", settings.setup_files)
    if not isinstance(setup_files, list) or not all(
        isinstance(path, (str, Path)) for path in setup_files
    ):
        raise RecorderError("Config value 'livox.setup_files' must be a list of paths.")
    settings.setup_files = [_resolve_path(path, base_dir) for path in setup_files]

    timeout = livox_raw.get(
        "connection_timeout_seconds", settings.connection_timeout_seconds
    )
    if isinstance(timeout, bool) or not isinstance(timeout, (int, float)):
        raise RecorderError(
            "Config value 'livox.connection_timeout_seconds' must be a number."
        )
    if timeout < 0:
        raise RecorderError(
            "Config value 'livox.connection_timeout_seconds' must not be negative."
        )
    settings.connection_timeout_seconds = float(timeout)

    ping_timeout = livox_raw.get(
        "ping_timeout_seconds", settings.ping_timeout_seconds
    )
    if isinstance(ping_timeout, bool) or not isinstance(ping_timeout, (int, float)):
        raise RecorderError(
            "Config value 'livox.ping_timeout_seconds' must be a number."
        )
    if ping_timeout <= 0:
        raise RecorderError(
            "Config value 'livox.ping_timeout_seconds' must be positive."
        )
    settings.ping_timeout_seconds = float(ping_timeout)

    if not settings.detection_topic:
        raise RecorderError("Config value 'livox.detection_topic' must not be empty.")
    if not settings.ros2_executable:
        raise RecorderError("Config value 'livox.ros2_executable' must not be empty.")
    if not settings.driver_package or not settings.driver_launch_file:
        raise RecorderError("Livox driver package and launch file must not be empty.")
    if settings.storage not in {"mcap", "sqlite3"}:
        raise RecorderError("Config value 'livox.storage' must be 'mcap' or 'sqlite3'.")
    return settings


def load_config(config_path: Path) -> RecorderSettings:
    """JSON設定を読み込み、録画に必要な値を検証する。"""

    config_path = Path(config_path).expanduser().resolve()
    try:
        with config_path.open("r", encoding="utf-8") as config_file:
            raw_config = json.load(config_file)
    except FileNotFoundError as exc:
        raise RecorderError(f"Config file not found: {config_path}") from exc
    except json.JSONDecodeError as exc:
        raise RecorderError(
            f"Invalid JSON in config file {config_path}: {exc.msg}"
        ) from exc
    except OSError as exc:
        raise RecorderError(f"Could not read config file {config_path}: {exc}") from exc

    if not isinstance(raw_config, dict):
        raise RecorderError("Config file must contain a JSON object.")

    settings = RecorderSettings()
    settings.trial = raw_config.get("trial", settings.trial)
    settings.output_dir = _resolve_path(
        raw_config.get("output_dir", os.fspath(DEFAULT_OUTPUT_DIR)), config_path.parent
    )
    settings.group_number = raw_config.get("group_number", DEFAULT_GROUP_NUMBER)
    settings.compression_mode = raw_config.get(
        "compression_mode", settings.compression_mode
    )
    settings.resolution = _text_setting(
        raw_config, "resolution", settings.resolution
    ).strip().upper()
    settings.fps = raw_config.get("fps", settings.fps)
    settings.zed_explorer = _text_setting(
        raw_config, "zed_explorer", settings.zed_explorer
    ).strip()
    settings.livox = load_livox_config(raw_config, config_path.parent)

    if not isinstance(settings.trial, int) or isinstance(settings.trial, bool):
        raise RecorderError("Config value 'trial' must be an integer.")
    if settings.trial < 1:
        raise RecorderError("Config value 'trial' must be a positive integer.")
    if not isinstance(settings.group_number, int) or isinstance(
        settings.group_number, bool
    ):
        raise RecorderError("Config value 'group_number' must be an integer.")
    if settings.group_number < 1:
        raise RecorderError("Config value 'group_number' must be a positive integer.")
    if not isinstance(settings.compression_mode, int) or isinstance(
        settings.compression_mode, bool
    ):
        raise RecorderError("Config value 'compression_mode' must be an integer.")
    if not 0 <= settings.compression_mode <= 6:
        raise RecorderError("Config value 'compression_mode' must be between 0 and 6.")
    if not isinstance(settings.fps, int) or isinstance(settings.fps, bool):
        raise RecorderError("Config value 'fps' must be an integer.")
    if settings.fps <= 0:
        raise RecorderError("Config value 'fps' must be positive.")
    if not settings.resolution.strip():
        raise RecorderError("Config value 'resolution' must not be empty.")
    if not settings.zed_explorer.strip():
        raise RecorderError("Config value 'zed_explorer' must not be empty.")
    return settings


def check_zed_explorer(executable: str = "ZED_Explorer") -> str:
    """ZED_Explorerの実行ファイルを探し、見つからなければ理由を示す。"""

    resolved = shutil.which(executable)
    if not resolved:
        raise RecorderError(
            "ZED_Explorer was not found in PATH. "
            "Install the ZED SDK and make sure ZED_Explorer is available."
        )
    return resolved


def _safe_condition(condition: str) -> str:
    """実験条件を安全なファイル名の一部分へ整形する。"""

    safe = re.sub(r"[^A-Za-z0-9._-]+", "_", condition.strip())
    safe = safe.strip("._-")
    return safe or DEFAULT_CONDITION


def _render_output_name(
    condition: str,
    trial: int,
    timestamp: dt.datetime,
    template: str,
    setting_name: str,
) -> str:
    """保存名テンプレートを展開し、パスとして危険な名前を除く。"""

    try:
        name = template.format(
            condition=_safe_condition(condition),
            trial=trial,
            timestamp=timestamp.strftime("%Y%m%d_%H%M%S"),
        ).strip()
    except (IndexError, KeyError, ValueError) as exc:
        raise RecorderError(
            f"Invalid {setting_name}. Use {{condition}}, {{trial}}, "
            "and {timestamp}."
        ) from exc
    if not name:
        raise RecorderError(f"{setting_name} produced an empty name.")
    if "/" in name or "\\" in name:
        raise RecorderError(f"{setting_name} must describe a name, not a path.")
    if Path(name).is_absolute() or name in {".", ".."}:
        raise RecorderError(f"{setting_name} must describe a relative name.")
    return name


def generate_filename(
    condition: str,
    trial: int,
    output_dir: Path,
    timestamp: Optional[dt.datetime] = None,
    filename_template: str = DEFAULT_FILENAME_TEMPLATE,
) -> Path:
    """SVO2の未使用保存先を作る。

    録画開始時刻を名前に含め、同名があれば連番を付けて上書きを防ぐ。
    """

    if trial < 1:
        raise ValueError("trial must be a positive integer")

    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    timestamp = timestamp or dt.datetime.now()
    base_name = _render_output_name(
        condition, trial, timestamp, filename_template, "filename_template"
    )
    if Path(base_name).suffix.lower() != ".svo2":
        base_name = f"{base_name}.svo2"

    base_path = output_dir / base_name
    candidate = base_path
    suffix_number = 1
    while candidate.exists() or candidate.is_symlink():
        candidate = output_dir / (
            f"{base_path.stem}_{suffix_number:02d}{base_path.suffix}"
        )
        suffix_number += 1
    return candidate


def generate_livox_bag_path(
    condition: str,
    trial: int,
    output_dir: Path,
    timestamp: Optional[dt.datetime] = None,
    bag_name_template: str = "{condition}_{trial:03d}_{timestamp}",
) -> Path:
    """既存ディレクトリと重ならないrosbag2の保存先を作る。"""

    if trial < 1:
        raise ValueError("trial must be a positive integer")
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    timestamp = timestamp or dt.datetime.now()
    base_name = _render_output_name(
        condition, trial, timestamp, bag_name_template, "livox.bag_name_template"
    )
    base_path = output_dir / base_name
    candidate = base_path
    suffix_number = 1
    while candidate.exists() or candidate.is_symlink():
        candidate = output_dir / f"{base_path.name}_{suffix_number:02d}"
        suffix_number += 1
    return candidate


def recording_directory(
    output_root: Path,
    group_number: int,
    timestamp: Optional[dt.datetime] = None,
    *,
    reserve: bool = False,
) -> Path:
    """日付・グループ配下に録画IDフォルダを選び、必要なら確保する。"""

    if (
        not isinstance(group_number, int)
        or isinstance(group_number, bool)
        or group_number < 1
    ):
        raise ValueError("group_number must be a positive integer")
    timestamp = timestamp or dt.datetime.now()
    group_dir = Path(output_root) / (
        f"{timestamp:%Y%m%d}_group{group_number:03d}"
    )
    existing_numbers = []
    if group_dir.is_dir():
        for child in group_dir.iterdir():
            match = re.fullmatch(r"recording_(\d+)", child.name)
            if match:
                existing_numbers.append(int(match.group(1)))
    recording_number = max(existing_numbers, default=0) + 1
    while True:
        candidate = group_dir / f"recording_{recording_number:03d}"
        if reserve:
            try:
                candidate.mkdir(parents=True, exist_ok=False)
                return candidate
            except FileExistsError:
                recording_number += 1
        elif candidate.exists() or candidate.is_symlink():
            recording_number += 1
        else:
            return candidate


def write_recording_metadata(
    recording_dir: Path,
    run_order: dict,
    trial_entry: dict,
    take_number: int,
    started_at: dt.datetime,
    lidar_enabled: bool,
) -> Path:
    """共通順序データの条件・教示とTake番号を録画ファイルへ記録する。"""

    group_number = int(trial_entry["group"][1:])
    metadata = {
        "recording_id": recording_dir.name,
        "recording_started_at": started_at.isoformat(timespec="seconds"),
        "group_number": group_number,
        "trial_number": trial_entry["global_trial_number"],
        "global_trial_number": trial_entry["global_trial_number"],
        "set_number": trial_entry["set_number"],
        "paper_name": trial_entry["paper_name"],
        "paper_trial_number": trial_entry["paper_trial_number"],
        "trial_type": trial_entry["trial_type"],
        "trial_type_label": trial_entry["trial_type_label"],
        "take_number": take_number,
        "condition_id": trial_entry["condition_id"],
        "condition_category": trial_entry["condition_category"],
        "corridor_width": trial_entry["corridor_width"],
        "avoidance": trial_entry["avoidance"],
        "leading_walker_speed": trial_entry["leading_walker_speed"],
        "overtaking_direction": trial_entry["overtaking_direction"],
        "overtaker": trial_entry["overtaker"],
        "overtaken_person": trial_entry["overtaken_person"],
        "solo_target": trial_entry["solo_target"],
        "participants": {
            "subject_1": trial_entry["subject_1"],
            "subject_2": trial_entry["subject_2"],
        },
        "plan_version": run_order["plan_version"],
        "plan_sha256": run_order["plan_sha256"],
        "source_workbooks": run_order["source_workbooks"],
        "data": {
            "svo2": f"{recording_dir.name}.svo2",
            "lidar_bag": "lidar" if lidar_enabled else None,
        },
    }
    metadata_path = recording_dir / "recording.json"
    metadata_path.write_text(
        json.dumps(metadata, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    return metadata_path


def build_ros_environment(settings: LivoxSettings) -> dict[str, str]:
    """ROSのsetupファイルを読み込み、子プロセスへ渡す環境変数を作る。"""

    environment = os.environ.copy()
    ros_log_dir = PROJECT_ROOT / "logs" / "ros"
    ros_log_dir.mkdir(parents=True, exist_ok=True)
    environment["ROS_LOG_DIR"] = os.fspath(ros_log_dir)

    missing = [path for path in settings.setup_files if not path.is_file()]
    if missing:
        raise RecorderError(
            "ROS setup file not found: " + ", ".join(os.fspath(path) for path in missing)
        )
    if not settings.setup_files:
        return environment

    shell_script = (
        'set -e\nfor setup_file in "$@"; do '
        'source "$setup_file" >/dev/null; done\nenv -0'
    )
    try:
        result = subprocess.run(
            ["/bin/bash", "-c", shell_script, "bash"]
            + [os.fspath(path) for path in settings.setup_files],
            env=environment,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            timeout=15.0,
            check=False,
        )
    except (OSError, subprocess.TimeoutExpired) as exc:
        raise RecorderError(f"Could not load the ROS 2 environment: {exc}") from exc
    if result.returncode != 0:
        detail = result.stderr.decode(errors="replace").strip()
        raise RecorderError(
            f"Could not load the ROS 2 environment (exit {result.returncode}): "
            f"{detail or 'no details'}"
        )

    sourced_environment: dict[str, str] = {}
    for entry in result.stdout.split(b"\0"):
        key, separator, value = entry.partition(b"=")
        if separator:
            sourced_environment[os.fsdecode(key)] = os.fsdecode(value)
    sourced_environment["ROS_LOG_DIR"] = os.fspath(ros_log_dir)
    return sourced_environment


def list_ros_topics(
    ros2_executable: str,
    environment: dict[str, str],
    timeout: float = 4.0,
) -> set[str]:
    """デーモンを介さず現在見えるROS 2トピックを取得する。"""

    try:
        result = subprocess.run(
            [ros2_executable, "topic", "list", "--no-daemon"],
            env=environment,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            timeout=timeout,
            check=False,
        )
    except (OSError, subprocess.TimeoutExpired) as exc:
        raise RecorderError(f"Could not inspect ROS 2 topics: {exc}") from exc
    if result.returncode != 0:
        detail = result.stderr.strip().splitlines()
        raise RecorderError(
            "ROS 2 topic discovery failed"
            + (f": {detail[-1]}" if detail else f" (exit {result.returncode})")
        )
    return {line.strip() for line in result.stdout.splitlines() if line.strip()}


def ros_topic_has_publisher(
    ros2_executable: str,
    environment: dict[str, str],
    topic: str,
    timeout: float = 4.0,
) -> bool:
    """指定トピックへ配信するPublisherが実際に存在するか確認する。

    RVizの購読だけでもトピック名は見えるため、配信者数を調べる。
    """

    try:
        result = subprocess.run(
            [ros2_executable, "topic", "info", topic, "--no-daemon"],
            env=environment,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            timeout=timeout,
            check=False,
        )
    except (OSError, subprocess.TimeoutExpired) as exc:
        raise RecorderError(
            f"Could not inspect ROS 2 topic publishers: {exc}"
        ) from exc
    if result.returncode != 0:
        return False

    match = re.search(r"Publisher count:\s*(\d+)", result.stdout, re.IGNORECASE)
    return bool(match and int(match.group(1)) > 0)


def _start_livox_driver(session: LivoxSession, started_at: dt.datetime) -> None:
    """Livoxドライバを起動し、その出力ログを保持する。"""

    assert session.ros2_executable is not None
    assert session.environment is not None
    logs_dir = PROJECT_ROOT / "logs"
    logs_dir.mkdir(parents=True, exist_ok=True)
    session.driver_log_path = logs_dir / (
        f"livox_driver_{started_at:%Y%m%d_%H%M%S}.log"
    )
    session.driver_log = session.driver_log_path.open("a", encoding="utf-8")
    command = [
        session.ros2_executable,
        "launch",
        session.settings.driver_package,
        session.settings.driver_launch_file,
    ]
    try:
        session.driver_process = subprocess.Popen(
            command,
            env=session.environment,
            stdin=subprocess.DEVNULL,
            stdout=session.driver_log,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
    except OSError as exc:
        session.driver_log.close()
        session.driver_log = None
        raise RecorderError(f"Could not start the Livox driver: {exc}") from exc


def probe_livox_sensor(settings: LivoxSettings, environment: dict[str, str]) -> tuple[bool, str]:
    """ドライバ起動前にLiDARのIPへ短時間の接続確認を行う。"""

    if not settings.sensor_ip:
        return True, "IP probe disabled"
    ping_executable = shutil.which("ping", path=environment.get("PATH"))
    if not ping_executable:
        return True, "ping command unavailable; continuing with topic detection"
    wait_seconds = max(1, int(settings.ping_timeout_seconds + 0.999))
    try:
        result = subprocess.run(
            [
                ping_executable,
                "-c",
                "1",
                "-W",
                str(wait_seconds),
                settings.sensor_ip,
            ],
            env=environment,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            timeout=settings.ping_timeout_seconds + 2.0,
            check=False,
        )
    except (OSError, subprocess.TimeoutExpired) as exc:
        return False, f"could not probe LiDAR IP {settings.sensor_ip}: {exc}"
    if result.returncode != 0:
        return False, f"LiDAR IP {settings.sensor_ip} did not respond to ping"
    return True, f"LiDAR IP {settings.sensor_ip} responded"


def prepare_livox_session(
    settings: LivoxSettings,
    started_at: dt.datetime,
) -> LivoxSession:
    """Livoxの起動と接続を確認し、試行ごとの状態を返す。"""

    session = LivoxSession(settings)
    if not settings.enabled:
        session.reason = "disabled in config.json"
        return session

    try:
        session.environment = build_ros_environment(settings)
        session.ros2_executable = shutil.which(
            settings.ros2_executable,
            path=session.environment.get("PATH"),
        )
        if not session.ros2_executable:
            raise RecorderError(
                f"{settings.ros2_executable} was not found in the ROS 2 environment"
            )

        reachable, probe_reason = probe_livox_sensor(settings, session.environment)
        # ping失敗だけでLiDARを未接続と判断しない。UDP通信とROSトピック検出は続行する。
        ping_warning = "" if reachable else f"IP probe warning: {probe_reason}"

        topics = list_ros_topics(session.ros2_executable, session.environment)
        if settings.detection_topic in topics and ros_topic_has_publisher(
            session.ros2_executable,
            session.environment,
            settings.detection_topic,
        ):
            session.connected = True
            session.reason = "Livox topic detected (using an existing driver)"
            if ping_warning:
                session.reason += f"; {ping_warning}"
            return session

        if not settings.start_driver:
            session.reason = (
                f"topic {settings.detection_topic} was not detected and driver startup is disabled"
            )
            if ping_warning:
                session.reason += f"; {ping_warning}"
            return session

        _start_livox_driver(session, started_at)
        deadline = time.monotonic() + settings.connection_timeout_seconds
        last_error = ""
        while True:
            if session.driver_process.poll() is not None:
                session.reason = (
                    "Livox driver exited during startup"
                    + (
                        f"; see {_relative_display_path(session.driver_log_path)}"
                        if session.driver_log_path
                        else ""
                    )
                )
                return session
            try:
                topics = list_ros_topics(
                    session.ros2_executable,
                    session.environment,
                )
                if settings.detection_topic in topics and ros_topic_has_publisher(
                    session.ros2_executable,
                    session.environment,
                    settings.detection_topic,
                ):
                    session.connected = True
                    session.reason = f"topic {settings.detection_topic} detected"
                    return session
            except RecorderError as exc:
                last_error = str(exc)
            if time.monotonic() >= deadline:
                break
            time.sleep(0.5)

        session.reason = (
            f"topic {settings.detection_topic} was not detected within "
            f"{settings.connection_timeout_seconds:.1f} seconds"
        )
        if last_error:
            session.reason += f" ({last_error})"
        if ping_warning:
            session.reason += f"; {ping_warning}"
    except KeyboardInterrupt:
        if session.driver_process is not None:
            stop_process(session.driver_process, "Livox driver")
        close_livox_logs(session)
        raise
    except (RecorderError, OSError) as exc:
        session.reason = str(exc)
    return session


def start_livox_bag_recording(
    session: LivoxSession,
    bag_path: Path,
    startup_timeout: float = 1.0,
) -> subprocess.Popen:
    """設定されたLivoxトピックのrosbag2録画を開始する。"""

    if not session.connected:
        raise RecorderError("Livox is not connected; rosbag2 was not started.")
    assert session.ros2_executable is not None
    assert session.environment is not None

    logs_dir = PROJECT_ROOT / "logs"
    logs_dir.mkdir(parents=True, exist_ok=True)
    session.bag_log_path = logs_dir / f"rosbag_{bag_path.parent.name}.log"
    session.bag_log = session.bag_log_path.open("a", encoding="utf-8")
    command = [
        session.ros2_executable,
        "bag",
        "record",
        "--output",
        os.fspath(bag_path),
        "--storage",
        session.settings.storage,
        "--disable-keyboard-controls",
        "--topics",
        *session.settings.topics,
    ]
    try:
        process = subprocess.Popen(
            command,
            env=session.environment,
            stdin=subprocess.DEVNULL,
            stdout=session.bag_log,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
    except OSError as exc:
        session.bag_log.close()
        session.bag_log = None
        raise RecorderError(f"Could not start ros2 bag record: {exc}") from exc

    session.bag_process = process
    deadline = time.monotonic() + startup_timeout
    while time.monotonic() < deadline:
        return_code = process.poll()
        if return_code is not None:
            raise RecorderError(
                "ros2 bag record exited during startup "
                f"(exit code {return_code}); see "
                f"{_relative_display_path(session.bag_log_path)}"
            )
        time.sleep(0.05)
    return process


def validate_livox_bag(bag_path: Path, return_code: int) -> tuple[bool, str]:
    """rosbag2のメタデータと空でない保存ファイルを確認する。"""

    errors: list[str] = []
    if return_code != 0:
        errors.append(f"ros2 bag record exited with code {return_code}")
    if not bag_path.is_dir():
        errors.append(f"rosbag2 output directory does not exist: {bag_path}")
        return False, "; ".join(errors)
    metadata = bag_path / "metadata.yaml"
    if not metadata.is_file() or metadata.stat().st_size <= 0:
        errors.append(f"rosbag2 metadata is missing or empty: {metadata}")
    storage_files = list(bag_path.glob("*.mcap")) + list(bag_path.glob("*.db3"))
    if not storage_files:
        errors.append(f"rosbag2 storage file is missing: {bag_path}")
    elif not any(path.stat().st_size > 0 for path in storage_files):
        errors.append(f"rosbag2 storage file is empty: {bag_path}")
    return (not errors), "; ".join(errors)


def close_livox_logs(session: LivoxSession) -> None:
    """対応するプロセスの終了後にログファイルを閉じる。"""

    for attribute in ("bag_log", "driver_log"):
        stream = getattr(session, attribute)
        if stream is not None and not stream.closed:
            stream.close()


def start_recording(
    zed_explorer: str,
    output_path: Path,
    startup_timeout: float = 1.0,
    compression_mode: int = DEFAULT_COMPRESSION_MODE,
    resolution: str = DEFAULT_RESOLUTION,
    fps: int = DEFAULT_FPS,
) -> subprocess.Popen:
    """指定された圧縮・解像度・FPSでZED_Explorerを起動する。"""

    command = [
        zed_explorer,
        "-o",
        os.fspath(output_path),
        "--cpm",
        str(compression_mode),
        "--resolution",
        resolution,
        "--frequency",
        str(fps),
    ]
    try:
        # 子プロセスを別グループに置き、Ctrl+Cは親が受けてSIGINTを明示的に転送する。
        process = subprocess.Popen(
            command,
            stdin=subprocess.DEVNULL,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
    except OSError as exc:
        raise RecorderError(
            f"Could not start ZED_Explorer ({exc}). "
            "Check the ZED SDK installation and camera connection."
        ) from exc

    deadline = time.monotonic() + startup_timeout
    try:
        while time.monotonic() < deadline:
            return_code = process.poll()
            if return_code is not None:
                raise RecorderError(
                    "ZED_Explorer exited during startup "
                    f"(exit code {return_code}). Is a ZED camera connected?"
                )
            time.sleep(0.05)
    except KeyboardInterrupt:
        # 起動直後のCtrl+Cでも子プロセスを残さず、SIGINT後の終了を待つ。
        print(
            "\nCtrl+C received during startup; closing ZED_Explorer safely...",
            file=sys.stderr,
        )
        stop_recording(process)
        raise
    return process


def request_process_stop(process: subprocess.Popen) -> None:
    """SIGINTを送り、子プロセスへ正常終了を要求する。"""

    while True:
        try:
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
            return
        except ProcessLookupError:
            # 生存確認後、シグナル送信前に終了した場合は問題にしない。
            return
        except KeyboardInterrupt:
            print(
                "\nStill closing recording processes safely...",
                file=sys.stderr,
            )


def wait_for_process_exit(
    process: subprocess.Popen,
    process_name: str,
    warning_after: float = STOP_WAIT_WARNING_SECONDS,
) -> int:
    """正常終了要求後、保存処理が完了するまで待機する。"""

    deadline = time.monotonic() + warning_after
    warned = False
    while True:
        try:
            return process.wait(timeout=1.0)
        except subprocess.TimeoutExpired:
            if not warned and time.monotonic() >= deadline:
                print(
                    f"{process_name} is taking longer than expected to close; "
                    "continuing to wait without force-killing it...",
                    file=sys.stderr,
                )
                warned = True
        except KeyboardInterrupt:
            # 2回目のCtrl+CでもSVO2とrosbagの書き込み完了まで待つ。
            print(
                f"\nStill waiting for {process_name} to close safely...",
                file=sys.stderr,
            )


def stop_process(
    process: subprocess.Popen,
    process_name: str,
    warning_after: float = STOP_WAIT_WARNING_SECONDS,
) -> int:
    """SIGINTを送り、子プロセスの正常終了を待つ。"""

    request_process_stop(process)
    return wait_for_process_exit(process, process_name, warning_after)


def stop_recording(
    process: subprocess.Popen,
    warning_after: float = STOP_WAIT_WARNING_SECONDS,
) -> int:
    """ZED_ExplorerへSIGINTを送り、録画終了を待つ。"""

    return stop_process(process, "ZED_Explorer", warning_after)


def validate_recording(output_path: Path, return_code: int) -> tuple[bool, str]:
    """終了コードと空でないSVO2ファイルを確認する。"""

    errors: list[str] = []
    if return_code != 0:
        errors.append(f"ZED_Explorer exited with code {return_code}")
    if not output_path.exists():
        errors.append(f"SVO2 file does not exist: {output_path}")
    elif not output_path.is_file():
        errors.append(f"Output path is not a regular file: {output_path}")
    elif output_path.stat().st_size <= 0:
        errors.append(f"SVO2 file is empty (0 bytes): {output_path}")

    if errors:
        return False, "; ".join(errors)
    return True, ""


def format_duration(seconds: float) -> str:
    """経過秒数を時:分:秒の表示へ変換する。"""

    total_seconds = max(0, int(seconds))
    hours, remainder = divmod(total_seconds, 3600)
    minutes, seconds_part = divmod(remainder, 60)
    return f"{hours:02d}:{minutes:02d}:{seconds_part:02d}"


def _relative_display_path(path: Path) -> str:
    """表示用のパスは可能ならプロジェクト相対にする。"""

    try:
        return os.fspath(path.resolve().relative_to(PROJECT_ROOT))
    except ValueError:
        return os.fspath(path)


def _trial_entry(run_order: dict, group_number: int, trial: int) -> dict:
    group_name = f"G{group_number:02d}"
    try:
        return run_order["groups"][group_name]["trials"][trial - 1]
    except (KeyError, IndexError) as exc:
        raise RecorderError(
            f"計画にグループ{group_name}の試行{trial}がありません。"
        ) from exc


def _available_trial_numbers(
    group_plan: dict, progress: RecordingProgress
) -> list[int]:
    maximum = (
        group_plan["basic_trial_count"] + group_plan["additional_trial_count"]
        if progress.additional_enabled
        else group_plan["basic_trial_count"]
    )
    return list(range(1, maximum + 1))


def _trial_entry_for_group(group_plan: dict, trial: int) -> dict:
    return group_plan["trials"][trial - 1]


def _prepare_warnings(group_plan: dict, trial_entry: dict) -> list[str]:
    trial_number = trial_entry["global_trial_number"]
    if trial_number <= 1:
        return []
    previous = _trial_entry_for_group(group_plan, trial_number - 1)
    warnings: list[str] = []
    if previous["set_number"] != trial_entry["set_number"]:
        warnings.append(
            f"セット切替：第{previous['set_number']}セット {previous['paper_name']} "
            f"→ 第{trial_entry['set_number']}セット {trial_entry['paper_name']}"
        )
    if previous["corridor_width"] != trial_entry["corridor_width"]:
        warnings.append(
            f"通路幅変更：{previous['corridor_width'] or '—'} → "
            f"{trial_entry['corridor_width'] or '—'}"
        )
    previous_roles = (previous["subject_1"]["role"], previous["subject_2"]["role"])
    current_roles = (trial_entry["subject_1"]["role"], trial_entry["subject_2"]["role"])
    if previous_roles != current_roles:
        warnings.append(
            "役割変更：被験者1 {0} / 被験者2 {1} → 被験者1 {2} / 被験者2 {3}".format(
                previous_roles[0], previous_roles[1], current_roles[0], current_roles[1]
            )
        )
    return warnings


def _print_ready(
    trial_entry: dict,
    group_plan: dict,
    group_number: int,
    progress: RecordingProgress,
) -> None:
    _clear_screen()
    trial = trial_entry["global_trial_number"]
    print("=" * 54)
    print(f"録画前確認  |  グループ {trial_entry['group']}（{group_number}/18）")
    print(
        f"試行 {trial:02d}  [{trial_entry['trial_type_label']}]  |  "
        f"通路幅: {trial_entry['corridor_width'] or '—'}  |  "
        f"Take: {progress.next_take_number(trial)}"
    )
    for subject_number in (1, 2):
        participant = trial_entry[f"subject_{subject_number}"]
        print(f"被験者{subject_number}（{participant['role']}）: {participant['instruction']}")
    warnings = _prepare_warnings(group_plan, trial_entry)
    rerecord = progress.rerecord_trials()
    if trial in progress.completed_trials() and trial not in rerecord:
        warnings.append(
            f"この試行は完了済みです。再録を開始するとTake {progress.next_take_number(trial)}になります"
        )
    if warnings:
        print("\n" + "!" * 54)
        print("【録画前に必ず確認】")
        for warning in warnings:
            print(f"  ◆ {warning}")
        print("!" * 54)
    print()
    print("← / → グループ選択   ↑ 試行-1   ↓ 試行+1")
    print("Enter 録画開始   Q 終了")


def _print_recording(
    trial_entry: dict,
    take_number: int = 1,
) -> None:
    print("=" * 54)
    print("録画中")
    trial = trial_entry["global_trial_number"]
    print(
        f"グループ {trial_entry['group']}  |  試行 {trial:02d} "
        f"[{trial_entry['trial_type_label']}]  |  通路幅: {trial_entry['corridor_width'] or '—'}  "
        f"|  Take: {take_number}"
    )
    for subject_number in (1, 2):
        participant = trial_entry[f"subject_{subject_number}"]
        print(f"被験者{subject_number}（{participant['role']}）: {participant['instruction']}")
    print()
    print("SPACE / S / ENTER 録画停止   CTRL+C 緊急終了")


def _print_livox_warning(reason: str, lidar_connected: bool = False) -> None:
    print()
    print("!" * 64)
    if lidar_connected:
        print("【警告】Livoxは接続されていますが、点群録画を開始できません")
    else:
        print("【警告】Livox未接続")
    print("この試行では点群を保存せず、ZEDのみ録画します。")
    print(f"理由: {reason}")
    print("!" * 64)
    print()


def _clear_screen() -> None:
    """リダイレクト先へ影響しない方法で対話端末を消去する。"""

    if getattr(sys.stdout, "isatty", lambda: False)():
        print("\033[2J\033[H", end="", flush=True)


@contextlib.contextmanager
def _terminal_key_mode():
    """対話端末から一文字ずつ読める状態へ一時的に切り替える。"""

    try:
        import termios
        import tty

        stdin_fd = sys.stdin.fileno()
        if not os.isatty(stdin_fd):
            yield False
            return
        original_attributes = termios.tcgetattr(stdin_fd)
        tty.setcbreak(stdin_fd)
    except (AttributeError, ImportError, OSError, ValueError):
        yield False
        return

    try:
        yield True
    finally:
        try:
            termios.tcsetattr(stdin_fd, termios.TCSADRAIN, original_attributes)
        except OSError:
            pass


def _read_command_line(prompt: str = "") -> str:
    """対話端末の不正なバイトを無視してコマンドを読み取る。

    保存後の操作はASCIIだけなので、不完全なUTF-8入力が来ても録画操作を続ける。
    """

    if prompt:
        print(prompt, end="", flush=True)

    reconfigure = getattr(sys.stdin, "reconfigure", None)
    if callable(reconfigure):
        try:
            reconfigure(errors="ignore")
        except (OSError, ValueError):
            pass
    try:
        return input()
    except UnicodeDecodeError:
        # reconfigure非対応の入力でも、不正なバイトを無視して録画を継続する。
        stdin_buffer = getattr(sys.stdin, "buffer", None)
        if stdin_buffer is None:
            return ""
        raw_line = stdin_buffer.readline()
        if not raw_line:
            raise EOFError
        return raw_line.decode("utf-8", errors="ignore")


def _read_terminal_key(stdin_fd: int) -> str:
    """端末から矢印キーを含むキー入力を1つ読み取る。"""

    ready, _, _ = select.select([stdin_fd], [], [], 0.5)
    if not ready:
        return ""
    first = os.read(stdin_fd, 1)
    if first in {b"\r", b"\n"}:
        return "enter"
    if first == b"\x1b":
        sequence = bytearray()
        while len(sequence) < 8:
            ready, _, _ = select.select([stdin_fd], [], [], 0.05)
            if not ready:
                break
            value = os.read(stdin_fd, 1)
            if not value:
                break
            sequence.extend(value)
            if value.isalpha():
                break
        return {
            b"[A": "up",
            b"[B": "down",
            b"[C": "right",
            b"[D": "left",
            b"OA": "up",
            b"OB": "down",
            b"OC": "right",
            b"OD": "left",
        }.get(bytes(sequence), "")
    return first.decode("ascii", errors="ignore").lower()


def wait_for_recording_start(
    output_root: Path,
    run_order: dict,
    group_number: int,
    trial: int,
) -> tuple[bool, int, int, RecordingProgress]:
    """run_order.jsonの計画行を表示し、開始前のグループ・試行番号を選択する。"""

    confirmed_completed_trial: tuple[int, int] | None = None
    while True:
        group_name = f"G{group_number:02d}"
        try:
            group_plan = run_order["groups"][group_name]
        except KeyError as exc:
            raise RecorderError(f"計画にグループ{group_name}がありません。") from exc
        progress = RecordingProgress.open(
            output_root, group_name, run_order["plan_sha256"]
        )
        if (
            progress.additional_decision == "pending"
            and progress.completed_count(list(range(1, group_plan["basic_trial_count"] + 1)))
            == group_plan["basic_trial_count"]
            and not any(
                trial_number <= group_plan["basic_trial_count"]
                for trial_number in progress.rerecord_trials()
            )
        ):
            if _ask_optional_additional_set(group_plan, progress):
                trial = group_plan["basic_trial_count"] + 1
        available_trials = _available_trial_numbers(group_plan, progress)
        trial = max(1, min(trial, available_trials[-1]))
        entry = _trial_entry(run_order, group_number, trial)
        _print_ready(
            entry,
            group_plan,
            group_number,
            progress,
        )

        try:
            with _terminal_key_mode() as key_mode:
                if key_mode:
                    stdin_fd = sys.stdin.fileno()
                    while True:
                        key = _read_terminal_key(stdin_fd)
                        if key == "left":
                            group_number = max(1, group_number - 1)
                            confirmed_completed_trial = None
                            break
                        if key == "right":
                            group_number = min(run_order["group_count"], group_number + 1)
                            confirmed_completed_trial = None
                            break
                        if key == "up":
                            trial = max(1, trial - 1)
                            confirmed_completed_trial = None
                            break
                        if key == "down":
                            trial = min(available_trials[-1], trial + 1)
                            confirmed_completed_trial = None
                            break
                        if key == "enter":
                            if (
                                trial in progress.completed_trials()
                                and trial not in progress.rerecord_trials()
                                and confirmed_completed_trial != (group_number, trial)
                            ):
                                confirmed_completed_trial = (group_number, trial)
                                print(
                                    f"\n【再録確認】試行{trial:02d}は完了済みです。"
                                    f"もう一度EnterでTake {progress.next_take_number(trial)}を開始します。"
                                    "矢印キーで選び直せます。",
                                    flush=True,
                                )
                                continue
                            return True, trial, group_number, progress
                        if key in {"q", "Q"}:
                            return False, trial, group_number, progress
                else:
                    command = _read_command_line(
                        "矢印キー（↑は試行-1、↓は試行+1。left/rightでグループ）で変更、"
                        "Enterで開始、qで終了: "
                    ).strip().lower()
                    if command in {"left", "right", "up", "down"}:
                        confirmed_completed_trial = None
                        if command == "left":
                            group_number = max(1, group_number - 1)
                        elif command == "right":
                            group_number = min(run_order["group_count"], group_number + 1)
                        elif command == "up":
                            trial = max(1, trial - 1)
                        else:
                            trial = min(available_trials[-1], trial + 1)
                        continue
                    if command == "q":
                        return False, trial, group_number, progress
                    if (
                        trial in progress.completed_trials()
                        and trial not in progress.rerecord_trials()
                        and confirmed_completed_trial != (group_number, trial)
                    ):
                        confirmed_completed_trial = (group_number, trial)
                        print(
                            f"試行{trial:02d}は完了済みです。再録する場合は、"
                            f"もう一度Enterを押してください（Take {progress.next_take_number(trial)}）。"
                        )
                        continue
                    return True, trial, group_number, progress
        except EOFError:
            return False, trial, group_number, progress


def wait_for_stop(
    process: subprocess.Popen,
    started_at: dt.datetime,
    livox_process: Optional[subprocess.Popen] = None,
) -> str:
    """状態表示を更新しながら停止キーかプロセス終了を待つ。

    戻り値はユーザー操作ならkey、録画側の早期終了ならprocess-exited。
    """

    try:
        stdin_fd = sys.stdin.fileno()
    except (AttributeError, OSError, ValueError):
        try:
            _read_command_line("\n録画停止: Enterを押してください ")
        except EOFError:
            pass
        return "key"

    livox_failure_reported = False
    with _terminal_key_mode() as key_mode:
        if not key_mode:
            # 標準入力がselect非対応でもEnterによる録画停止を受け付ける。
            try:
                _read_command_line("\n録画停止: Enterを押してください ")
            except EOFError:
                pass
            return "key"

        while True:
            elapsed = (dt.datetime.now() - started_at).total_seconds()
            try:
                ready, _, _ = select.select([stdin_fd], [], [], 0.5)
            except (OSError, ValueError):
                print("\n端末入力を取得できないため、Enterで停止します。")
                try:
                    _read_command_line()
                except EOFError:
                    pass
                return "key"

            if ready:
                key = os.read(stdin_fd, 1).decode(
                    getattr(sys.stdin, "encoding", None) or "utf-8",
                    errors="ignore",
                )
                if not key or key in {" ", "s", "S", "x", "X", "\n", "\r"}:
                    print("\n状態      : 停止処理中（保存を完了しています）")
                    return "key"

            if process.poll() is not None:
                print()
                return "process-exited"

            status_line = (
                f"状態: 録画中 | 経過: {format_duration(elapsed)} "
                "| SPACE / S / ENTER: 停止 | CTRL+C: 緊急終了"
            )
            print(f"\r{status_line:<100}", end="", flush=True)
            if (
                livox_process is not None
                and livox_process.poll() is not None
                and not livox_failure_reported
            ):
                print(
                    "\nWARNING: Livox rosbag2 stopped unexpectedly. "
                    "ZED recording is still running."
                )
                livox_failure_reported = True


def _parse_args(argv: Optional[list[str]]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--config",
        type=Path,
        default=DEFAULT_CONFIG_PATH,
        help=f"JSON config file (default: {DEFAULT_CONFIG_PATH})",
    )
    parser.add_argument(
        "--run-order",
        type=Path,
        default=DEFAULT_PLAN_PATH,
        help=f"Verified run_order.json (default: {DEFAULT_PLAN_PATH})",
    )
    parser.add_argument(
        "--trial",
        type=int,
        default=None,
        help="Override the initial trial number from the config file",
    )
    parser.add_argument(
        "--group-number",
        type=int,
        default=None,
        help="Override the group number in the recording directory",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=None,
        help="Override the recordings root directory from the config file",
    )
    parser.add_argument(
        "--compression-mode",
        type=int,
        default=None,
        help="Override compression_mode from the config file",
    )
    parser.add_argument(
        "--resolution",
        default=None,
        help="Override resolution from the config file",
    )
    parser.add_argument(
        "--fps",
        type=int,
        default=None,
        help="Override FPS from the config file",
    )
    parser.add_argument(
        "--zed-explorer",
        default=None,
        help="Override ZED_Explorer executable from the config file",
    )
    return parser.parse_args(argv)


def _apply_cli_overrides(settings: RecorderSettings, args: argparse.Namespace) -> None:
    """設定ファイルへCLI引数を上書きし、録画開始前に値を検証する。"""

    if args.trial is not None:
        settings.trial = args.trial
    if args.group_number is not None:
        settings.group_number = args.group_number
    if args.output_dir is not None:
        settings.output_dir = _resolve_path(args.output_dir, Path.cwd())
    if args.compression_mode is not None:
        settings.compression_mode = args.compression_mode
    if args.resolution is not None:
        settings.resolution = args.resolution.strip().upper()
    if args.fps is not None:
        settings.fps = args.fps
    if args.zed_explorer is not None:
        settings.zed_explorer = args.zed_explorer.strip()

    if settings.trial < 1:
        raise RecorderError("trial must be a positive integer.")
    if settings.group_number < 1:
        raise RecorderError("group_number must be a positive integer.")
    if settings.group_number > 18:
        raise RecorderError("group_number must be between 1 and 18.")
    if not settings.resolution.strip():
        raise RecorderError("resolution must not be empty.")
    if not settings.zed_explorer.strip():
        raise RecorderError("zed_explorer must not be empty.")
    if not 0 <= settings.compression_mode <= 6:
        raise RecorderError("compression_mode must be between 0 and 6.")
    if settings.fps <= 0:
        raise RecorderError("fps must be positive.")


def _ask_optional_additional_set(
    group_plan: dict, progress: RecordingProgress
) -> bool:
    """基本54試行の完了後に、第4セットを明示的に選んでもらう。"""

    basic_trials = list(range(1, group_plan["basic_trial_count"] + 1))
    if progress.additional_enabled or progress.additional_decision == "declined":
        return False
    if progress.completed_count(basic_trials) != len(basic_trials):
        return False
    if any(trial in basic_trials for trial in progress.rerecord_trials()):
        return False
    print("\n" + "!" * 72)
    print(f"【{group_plan['group']} 基本54試行が完了しました】")
    print(
        f"任意の第4セット {group_plan['optional_set']}（18試行）を追加で実施しますか？"
    )
    print("ここではまだ録画を開始しません。Yで追加セットを選択可能にします。")
    print("!" * 72)
    while True:
        try:
            answer = unicodedata.normalize(
                "NFKC", _read_command_line("第4セットを実施しますか [y/N]: ")
            ).strip().lower()
        except EOFError:
            return False
        if answer in {"y", "yes", "はい"}:
            progress.enable_additional()
            print(
                f"第4セット {group_plan['optional_set']} を選択しました。"
                "録画開始前画面で内容を確認し、Enterを押してください。"
            )
            return True
        if answer in {"", "n", "no", "いいえ", "q"}:
            progress.decline_additional()
            print("第4セットは実施しません。")
            return False
        print("YまたはNで回答してください。")


def main(argv: Optional[list[str]] = None) -> int:
    args = _parse_args(argv)
    try:
        config_path = _resolve_path(args.config, Path.cwd())
        settings = load_config(config_path)
        _apply_cli_overrides(settings, args)
        run_order_path = _resolve_path(args.run_order, Path.cwd())
        run_order = load_run_order(run_order_path)
        group_name = f"G{settings.group_number:02d}"
        if group_name not in run_order["groups"]:
            raise RecorderError(f"計画にグループ{group_name}がありません。")
        progress = RecordingProgress.open(
            settings.output_dir, group_name, run_order["plan_sha256"]
        )
        if settings.trial > run_order["basic_trial_count"] and not progress.additional_enabled:
            raise RecorderError(
                "第4セットは基本54試行の完了後に画面で選択してください。"
            )
        settings.output_dir.mkdir(parents=True, exist_ok=True)
        zed_explorer = check_zed_explorer(settings.zed_explorer)
    except (PlanError, ProgressError, RecorderError, OSError, TypeError) as exc:
        print(f"Error: {exc}", file=sys.stderr)
        return 1

    if args.trial is None:
        available = _available_trial_numbers(run_order["groups"][group_name], progress)
        retry_pending = any(
            trial_number in available for trial_number in progress.rerecord_trials()
        )
        configured_trial_is_complete = settings.trial in progress.completed_trials()
        if retry_pending or configured_trial_is_complete or settings.trial == DEFAULT_TRIAL:
            recommended = progress.recommended_trial(available)
        else:
            recommended = None
        if recommended is not None:
            settings.trial = recommended

    # 選択した計画行に沿って、録画開始・保存検証・進捗保存を繰り返す。
    trial = settings.trial
    interrupted = False
    while True:
        try:
            start_requested, trial, settings.group_number, progress = wait_for_recording_start(
                settings.output_dir,
                run_order,
                settings.group_number,
                trial,
            )
        except KeyboardInterrupt:
            print("\nInterrupted before recording started.")
            return 130
        except (PlanError, ProgressError, RecorderError, OSError, ValueError) as exc:
            print(f"Error: {exc}", file=sys.stderr)
            return 1
        if not start_requested:
            print("\nExiting before recording started.")
            return 0
        trial_entry = _trial_entry(run_order, settings.group_number, trial)
        take_number = progress.next_take_number(trial)

        # LiDARは任意のため、接続失敗時もZED単独録画を続行する。
        print("\nChecking Livox LiDAR connection...")
        livox_check_started_at = dt.datetime.now()
        try:
            livox_session = prepare_livox_session(
                settings.livox, livox_check_started_at
            )
        except KeyboardInterrupt:
            print("\nInterrupted while checking Livox. No recording was started.")
            return 130

        # 接続確認後、録画IDと録画基準に使う時刻を一度だけ取得する。
        started_at = dt.datetime.now()
        try:
            recording_dir = recording_directory(
                settings.output_dir,
                settings.group_number,
                started_at,
                reserve=True,
            )
            output_path = recording_dir / f"{recording_dir.name}.svo2"
            take_number = progress.begin_attempt(trial, recording_dir.name)
            write_recording_metadata(
                recording_dir,
                run_order,
                trial_entry,
                take_number,
                started_at,
                livox_session.connected,
            )
        except KeyboardInterrupt:
            if livox_session.driver_process is not None:
                stop_process(livox_session.driver_process, "Livox driver")
            close_livox_logs(livox_session)
            print("\nInterrupted before recording started.")
            return 130
        except (ProgressError, RecorderError, OSError, ValueError) as exc:
            if livox_session.driver_process is not None:
                stop_process(livox_session.driver_process, "Livox driver")
            close_livox_logs(livox_session)
            print(f"Error: {exc}", file=sys.stderr)
            return 1
        livox_bag_path: Optional[Path] = None
        livox_recording_unavailable = False
        if livox_session.connected:
            try:
                livox_bag_path = recording_dir / "lidar"
                print(
                    f"Livox connected: {settings.livox.detection_topic}\n"
                    f"Livox bag     : {_relative_display_path(livox_bag_path)}"
                )
            except KeyboardInterrupt:
                if livox_session.driver_process is not None:
                    stop_process(livox_session.driver_process, "Livox driver")
                close_livox_logs(livox_session)
                print("\nInterrupted before recording started.")
                return 130
            except (RecorderError, OSError, ValueError) as exc:
                livox_recording_unavailable = True
                livox_session.connected = False
                livox_session.reason = f"could not prepare rosbag2 output: {exc}"

        if not livox_session.connected:
            # 点群トピックのない試行ではドライバを停止し、ZEDだけを録画する。
            if livox_session.driver_process is not None:
                stop_process(livox_session.driver_process, "Livox driver")
                livox_session.driver_process = None
            if settings.livox.enabled:
                _print_livox_warning(
                    livox_session.reason,
                    lidar_connected=livox_recording_unavailable,
                )
            else:
                print("Livox無効: 点群を保存せず、ZEDのみ録画します。\n")

        # 録画開始から停止までをtry/finallyで囲み、割り込み時もSIGINTで閉じる。
        process: Optional[subprocess.Popen] = None
        livox_recording_started = False
        livox_return_code: Optional[int] = None
        livox_exited_early = False
        stop_reason = "enter"
        try:
            if livox_session.connected and livox_bag_path is not None:
                try:
                    start_livox_bag_recording(livox_session, livox_bag_path)
                    livox_recording_started = True
                except RecorderError as exc:
                    livox_recording_unavailable = True
                    livox_session.connected = False
                    livox_session.reason = str(exc)
                    _print_livox_warning(
                        livox_session.reason,
                        lidar_connected=True,
                    )

            process = start_recording(
                zed_explorer,
                output_path,
                compression_mode=settings.compression_mode,
                resolution=settings.resolution,
                fps=settings.fps,
            )
            _print_recording(
                trial_entry,
                take_number,
            )
            stop_reason = wait_for_stop(
                process,
                started_at,
                livox_session.bag_process if livox_recording_started else None,
            )
        except KeyboardInterrupt:
            interrupted = True
            print("\nCtrl+C received; closing all recorders safely with SIGINT...")
        except RecorderError as exc:
            print(f"Error: {exc}", file=sys.stderr)
            return 1
        finally:
            if livox_session.bag_process is not None:
                livox_exited_early = livox_session.bag_process.poll() is not None
                request_process_stop(livox_session.bag_process)
            if process is not None:
                request_process_stop(process)
            if livox_session.bag_process is not None:
                livox_return_code = wait_for_process_exit(
                    livox_session.bag_process, "Livox rosbag2"
                )
            if process is not None:
                return_code = wait_for_process_exit(process, "ZED_Explorer")
            else:
                return_code = 1
            if livox_session.driver_process is not None:
                stop_process(livox_session.driver_process, "Livox driver")
            close_livox_logs(livox_session)

        # 終了コードと成果物を確認してから次の試行を受け付ける。
        ok, validation_error = validate_recording(output_path, return_code)
        if stop_reason == "process-exited":
            ok = False
            unexpected_exit = (
                "ZED_Explorer exited before the stop command was received"
            )
            validation_error = (
                f"{unexpected_exit}; {validation_error}"
                if validation_error
                else unexpected_exit
            )
        livox_validation_ok: Optional[bool] = None
        livox_error = ""
        if livox_recording_started:
            if livox_bag_path is not None and livox_return_code is not None:
                livox_validation_ok, livox_error = validate_livox_bag(
                    livox_bag_path, livox_return_code
                )
                if livox_exited_early:
                    livox_validation_ok = False
                    early_error = "Livox rosbag2 exited before the stop command"
                    livox_error = (
                        f"{early_error}; {livox_error}" if livox_error else early_error
                    )
            else:
                livox_validation_ok = False
                livox_error = "Livox rosbag2 did not return a validation result"
        elif livox_recording_unavailable:
            livox_validation_ok = False
            livox_error = livox_session.reason
        trial_recording_ok = ok and livox_validation_ok is not False
        progress_details = validation_error
        if livox_validation_ok is False:
            progress_details = "; ".join(
                detail for detail in (progress_details, f"Livox: {livox_error}") if detail
            )
        try:
            progress.finish_attempt(
                trial,
                take_number,
                success=trial_recording_ok,
                details=progress_details,
            )
        except ProgressError as exc:
            print(f"進捗を更新できません: {exc}", file=sys.stderr)
            return 1
        if ok:
            size_mb = output_path.stat().st_size / (1024 * 1024)
            print("\nZED recording completed successfully.\n")
            print(f"ZED File:\n{_relative_display_path(output_path)}")
            print(f"\nZED Size:\n{size_mb:.2f} MB")
        else:
            print(f"\nZED recording failed: {validation_error}", file=sys.stderr)
        print(
            f"Recording labels:\n"
            f"{_relative_display_path(recording_dir / 'recording.json')}"
        )

        if livox_recording_started and livox_bag_path is not None:
            if livox_validation_ok:
                print("\nLivox録画完了（点群保存済み）。\n")
                print(f"Livox bag:\n{_relative_display_path(livox_bag_path)}")
            else:
                print(
                    "\n【警告】Livox録画に失敗しました。ZEDの録画結果は保存されています。\n"
                    f"理由: {livox_error}",
                    file=sys.stderr,
                )
        elif settings.livox.enabled:
            print(
                "\nLivox状態: 未接続または録画開始失敗。点群は保存されていません。"
            )
            print(f"理由: {livox_session.reason}")
        if ok and not trial_recording_ok:
            print(
                f"試行{trial:02d}は再録が必要な状態で記録しました。"
                "録画前画面の再録一覧に表示します。"
            )

        if interrupted:
            return 130
        if not ok:
            print(
                f"試行{trial:02d}を再録が必要な状態で記録しました。"
                "再起動すると未完了試行として最初に表示します。"
            )
            return 1

        if trial == run_order["basic_trial_count"]:
            try:
                selected_additional = _ask_optional_additional_set(
                    run_order["groups"][f"G{settings.group_number:02d}"], progress
                )
            except KeyboardInterrupt:
                print("\n追加セットの選択を中断しました。")
                return 130
            if selected_additional:
                trial = run_order["basic_trial_count"] + 1
                continue
            if progress.additional_decision == "declined":
                return 0

        group_plan = run_order["groups"][f"G{settings.group_number:02d}"]
        available = _available_trial_numbers(group_plan, progress)
        if trial == available[-1] and progress.completed_count(available) == len(available):
            print(f"\n{group_plan['group']}の計画試行をすべて録画しました。")
            return 0

        print("\n状態      : 保存完了（次の試行を選択できます）")
        print("操作      : ENTER = 次の候補を表示 / Q = 終了")
        while True:
            try:
                command = unicodedata.normalize(
                    "NFKC", _read_command_line()
                ).strip().lower()
            except KeyboardInterrupt:
                print("\nExiting.")
                return 130
            except EOFError:
                print("\nEnd of input; exiting.")
                return 0
            if command == "q":
                return 0
            if not command:
                break
            print("ENTERで次の候補を表示、Qで終了します。")
        recommended = progress.recommended_trial(available, after_trial=trial)
        if recommended is not None:
            trial = recommended


if __name__ == "__main__":
    raise SystemExit(main())
