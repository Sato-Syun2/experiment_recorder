#!/usr/bin/env python3
"""追跡CSVから、単体で開ける歩行軌跡HTMLを生成する。

平面軌跡、現在位置、水平速度を同じ時刻軸で表示する。
CSVに廊下座標が含まれる場合は、廊下中心座標を優先表示できる。
追加のPythonライブラリは使用しない。
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import shutil
import subprocess
import sys
import time
import webbrowser
from collections import defaultdict
from pathlib import Path
from typing import Optional, Sequence

from playback_timeline import (
    PlaybackTimeline,
    inspect_bag_timeline,
    inspect_playback_timeline,
    inspect_svo_timeline,
)


PROJECT_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CONFIG_PATH = PROJECT_ROOT / "config.json"
DEFAULT_TOPIC = "/livox/lidar"


class ReplayError(RuntimeError):
    """CSVまたは再生設定に関する想定内のエラー。"""


# 旧来の内部名を維持しつつ、SVO/bagの時刻解析を共通実装へ集約する。
_PlaybackTimeline = PlaybackTimeline
_inspect_svo_timeline = inspect_svo_timeline
_inspect_bag_timeline = inspect_bag_timeline
_inspect_playback_timeline = inspect_playback_timeline


def _wait_before_start(seconds: float) -> None:
    deadline = time.monotonic() + max(0.0, seconds)
    while True:
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            return
        time.sleep(min(0.1, remaining))


def _open_html(output_path: Path, rate: float) -> None:
    webbrowser.open(output_path.as_uri() + f"?autoplay=1&rate={rate}")
    # HTMLの自動再生が始まるまでの遅延を、SVO起動時刻の調整に使う。
    time.sleep(0.35)


# 表示画面はHTMLテンプレートに分け、Python側ではCSVデータだけを埋め込む。
HTML_PAGE = Path(__file__).with_name("tracking_replay.html").read_text(encoding="utf-8")


def _float_field(
    row: dict[str, str], name: str, line: int, default: Optional[float] = None
) -> float:
    value = row.get(name, "")
    if value == "" and default is not None:
        return default
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise ReplayError(f"CSV line {line}: '{name}' must be a number.") from exc
    if not math.isfinite(result):
        raise ReplayError(f"CSV line {line}: '{name}' must be finite.")
    return result


def load_tracking_csv(
    path: Path,
    fps: float,
    *,
    include_tentative: bool = False,
    coordinate_frame: str = "auto",
) -> dict[str, object]:
    """CSVを読み込み、HTML用のフレーム・軌跡データへ変換する。"""

    if coordinate_frame not in {"auto", "livox", "corridor"}:
        raise ReplayError("coordinate_frame must be auto, livox, or corridor.")
    try:
        stream = path.open("r", encoding="utf-8-sig", newline="")
    except OSError as exc:
        raise ReplayError(f"Could not open CSV: {exc}") from exc

    rows: list[dict[str, object]] = []
    time_by_frame: dict[int, float] = {}
    rows_by_frame: dict[int, list[dict[str, object]]] = defaultdict(list)
    rows_by_track: dict[int, list[dict[str, object]]] = defaultdict(list)
    with stream:
        # CSVの座標系を決めてから行を読み、フレームと人物IDごとに集める。
        reader = csv.DictReader(stream)
        fieldnames = set(reader.fieldnames or [])
        raw_rows = list(reader)
        required = {"frame", "time_sec", "track_id", "x", "y"}
        missing = required - fieldnames
        if missing:
            raise ReplayError(
                "CSV is missing required columns: " + ", ".join(sorted(missing))
            )
        # 廊下座標の列があり、少なくとも1行に値があれば校正済みとみなす。
        corridor_fields = {
            "corridor_x", "corridor_y", "corridor_z",
            "corridor_vx", "corridor_vy",
        }
        corridor_available = corridor_fields <= fieldnames and any(
            all((row.get(name) or "").strip() != "" for name in corridor_fields)
            for row in raw_rows
        )
        if coordinate_frame == "corridor" and not corridor_available:
            raise ReplayError(
                "CSV does not contain calibrated corridor coordinates. "
                "Reprocess it after creating tracking/corridor_frame.json."
            )
        # autoは廊下座標を優先し、livoxは従来の生座標を強制する。
        use_corridor = coordinate_frame == "corridor" or (
            coordinate_frame == "auto" and corridor_available
        )
        selected_frame = "corridor" if use_corridor else "livox"
        for line, row in enumerate(raw_rows, start=2):
            try:
                frame = int(row["frame"])
                track_id = int(row["track_id"])
            except (TypeError, ValueError) as exc:
                raise ReplayError(
                    f"CSV line {line}: 'frame' and 'track_id' must be integers."
                ) from exc
            if frame < 1 or frame > 1_000_000:
                raise ReplayError(f"CSV line {line}: frame must be between 1 and 1,000,000.")
            time_sec = _float_field(row, "time_sec", line)
            time_by_frame.setdefault(frame, time_sec)
            if use_corridor:
                if not all((row.get(name) or "").strip() != "" for name in corridor_fields):
                    raise ReplayError(f"CSV line {line}: corridor coordinates are missing.")
                x = _float_field(row, "corridor_x", line)
                y = _float_field(row, "corridor_y", line)
                z = _float_field(row, "corridor_z", line, 0.0)
                vx = _float_field(row, "corridor_vx", line, 0.0)
                vy = _float_field(row, "corridor_vy", line, 0.0)
                speed = _float_field(
                    row, "corridor_speed_xy", line, math.hypot(vx, vy)
                )
            else:
                x = _float_field(row, "x", line)
                y = _float_field(row, "y", line)
                z = _float_field(row, "z", line, 0.0)
                vx = _float_field(row, "vx", line, 0.0)
                vy = _float_field(row, "vy", line, 0.0)
                speed = _float_field(row, "speed_xy", line, math.hypot(vx, vy))
            status = (row.get("status") or "").strip().lower()
            if (
                not include_tentative
                and "status" in fieldnames
                and status not in {"confirmed", "predicted"}
            ):
                continue
            item: dict[str, object] = {
                "frame": frame,
                "time_sec": time_sec,
                "id": track_id,
                "x": x,
                "y": y,
                "z": z,
                "vx": vx,
                "vy": vy,
                "speed": speed,
                "detected": (row.get("detected") or "1").strip().lower() not in {"0", "false"},
                "status": status,
            }
            rows.append(item)
            rows_by_frame[frame].append(item)
            rows_by_track[track_id].append(item)

    if not rows:
        if include_tentative:
            raise ReplayError("CSV has no tracking rows to replay.")
        raise ReplayError(
            "CSV has no confirmed or predicted tracking rows. "
            "Use --include-tentative only for diagnostics."
        )

    # 欠けたフレームの時刻を補間し、先頭・末尾は指定FPSで補う。
    last_frame = max(time_by_frame)
    if last_frame > 1_000_000:
        raise ReplayError("CSV contains too many frames to build a replay.")
    time_by_index: list[float] = [0.0] * last_frame
    known_frames = sorted(time_by_frame)
    known_index = 0
    for frame in range(1, last_frame + 1):
        while known_index < len(known_frames) and known_frames[known_index] < frame:
            known_index += 1
        if frame in time_by_frame:
            current_time = time_by_frame[frame]
        elif known_index == 0:
            first_known = known_frames[0]
            current_time = time_by_frame[first_known] - (first_known - frame) / fps
        elif known_index == len(known_frames):
            last_known = known_frames[-1]
            current_time = time_by_frame[last_known] + (frame - last_known) / fps
        else:
            before = known_frames[known_index - 1]
            after = known_frames[known_index]
            fraction = (frame - before) / (after - before)
            current_time = time_by_frame[before] + fraction * (
                time_by_frame[after] - time_by_frame[before]
            )
        if frame > 1 and current_time <= time_by_index[frame - 2]:
            current_time = time_by_index[frame - 2] + 1.0 / fps
        time_by_index[frame - 1] = current_time

    # 空フレームがあっても、再生・ラベル・グラフで同じ単調な時刻軸を使う。
    for item in rows:
        item["time_sec"] = time_by_index[int(item["frame"]) - 1]
    frames = [
        {
            "frame": frame,
            "time_sec": time_by_index[frame - 1],
            "rows": rows_by_frame.get(frame, []),
        }
        for frame in range(1, last_frame + 1)
    ]
    tracks = [
        {"id": track_id, "samples": sorted(samples, key=lambda item: int(item["frame"]))}
        for track_id, samples in sorted(rows_by_track.items())
    ]
    return {
        "frames": frames,
        "tracks": tracks,
        "bounds": _bounds(rows),
        "coordinate_frame": selected_frame,
    }


def _bounds(rows: Sequence[dict[str, object]]) -> dict[str, float]:
    xs = [float(row["x"]) for row in rows]
    ys = [float(row["y"]) for row in rows]
    x_pad = max(0.5, (max(xs) - min(xs)) * 0.08)
    y_pad = max(0.5, (max(ys) - min(ys)) * 0.08)
    return {
        "xmin": min(xs) - x_pad,
        "xmax": max(xs) + x_pad,
        "ymin": min(ys) - y_pad,
        "ymax": max(ys) + y_pad,
    }


def _parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv", type=Path, help="people tracking CSV to replay")
    parser.add_argument("--output", type=Path, default=None, help="generated HTML path")
    media = parser.add_mutually_exclusive_group()
    media.add_argument(
        "--svo",
        type=Path,
        default=None,
        help="also open this ZED SVO/SVO2 camera recording in ZED_Explorer",
    )
    media.add_argument(
        "--bag",
        type=Path,
        default=None,
        help="also replay this source rosbag in RViz with the CSV track markers",
    )
    parser.add_argument(
        "--sync-bag",
        type=Path,
        default=None,
        help="LiDAR bag used to align --svo playback; inferred from the SVO name by default",
    )
    parser.add_argument(
        "--zed-explorer",
        default=None,
        help="override ZED_Explorer executable from config.json",
    )
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG_PATH)
    parser.add_argument("--topic", default=None, help=f"Point cloud topic (default: {DEFAULT_TOPIC})")
    parser.add_argument("--rate", type=float, default=1.0, help="shared playback rate for HTML and RViz")
    parser.add_argument(
        "--fps",
        type=float,
        default=10.0,
        help="time step used for empty or missing frames",
    )
    parser.add_argument(
        "--no-open", action="store_true", help="generate HTML without opening a browser"
    )
    parser.add_argument(
        "--include-tentative",
        action="store_true",
        help="also display unconfirmed candidate tracks for diagnostics",
    )
    parser.add_argument(
        "--coordinate-frame",
        choices=("auto", "livox", "corridor"),
        default="auto",
        help="HTMLに表示する座標系（autoは校正済みの廊下座標を優先）",
    )
    return parser.parse_args(argv)


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = _parse_args(argv)
    if not math.isfinite(args.fps) or args.fps <= 0:
        print("Error: --fps must be positive and finite.", file=sys.stderr)
        return 2
    if not math.isfinite(args.rate) or args.rate <= 0:
        print("Error: --rate must be positive and finite.", file=sys.stderr)
        return 2
    if args.bag is not None and not args.bag.expanduser().resolve().is_dir():
        print(f"Error: rosbag2 directory does not exist: {args.bag}", file=sys.stderr)
        return 2
    if args.svo is not None:
        svo_path = args.svo.expanduser().resolve()
        if not svo_path.is_file() or svo_path.suffix.lower() not in {".svo", ".svo2"}:
            print(f"Error: SVO/SVO2 file does not exist: {args.svo}", file=sys.stderr)
            return 2
        if svo_path.stat().st_size <= 0:
            print(f"Error: SVO/SVO2 file is empty: {args.svo}", file=sys.stderr)
            return 2
    if args.sync_bag is not None:
        if args.svo is None:
            print("Error: --sync-bag requires --svo.", file=sys.stderr)
            return 2
        sync_bag = args.sync_bag.expanduser().resolve()
        if not sync_bag.is_dir() or not (sync_bag / "metadata.yaml").is_file():
            print(f"Error: LiDAR sync bag does not exist: {args.sync_bag}", file=sys.stderr)
            return 2
    csv_path = args.csv.expanduser().resolve()
    try:
        data = load_tracking_csv(
            csv_path,
            args.fps,
            include_tentative=args.include_tentative,
            coordinate_frame=args.coordinate_frame,
        )
        data["filename"] = csv_path.name
        output_path = (
            args.output.expanduser().resolve()
            if args.output is not None
            else csv_path.with_name(f"{csv_path.stem}_replay.html")
        )
        payload = json.dumps(data, ensure_ascii=False, separators=(",", ":"))
        payload = (
            payload.replace("<", "\\u003c")
            .replace(">", "\\u003e")
            .replace("&", "\\u0026")
        )
        # テンプレートへ解析済みデータを埋め、単体で開けるHTMLを書き出す。
        page = HTML_PAGE.replace("__DATA__", payload)
        output_path.parent.mkdir(parents=True, exist_ok=True)
        output_path.write_text(page, encoding="utf-8")
    except (ReplayError, OSError, ValueError) as exc:
        print(f"Error: {exc}", file=sys.stderr)
        return 1

    print(f"Replay HTML: {output_path}")
    if args.svo is not None:
        try:
            svo_path = args.svo.expanduser().resolve()
            csv_recording = csv_path.stem.removesuffix("_tracks")
            if csv_recording != svo_path.stem:
                raise ReplayError(
                    "CSV/SVO trial mismatch: "
                    f"CSV={csv_recording}, SVO={svo_path.stem}. "
                    "Use files with the same trial timestamp."
                )
            if args.sync_bag is not None:
                sync_bag = args.sync_bag.expanduser().resolve()
            else:
                sync_bag = PROJECT_ROOT / "recordings" / "livox" / svo_path.stem
            timeline = (
                _inspect_playback_timeline(svo_path, sync_bag)
                if sync_bag.is_dir()
                else None
            )
            if timeline is None:
                print(
                    "Warning: SVO/LiDAR recorded timestamps were unavailable; "
                    "starting camera and HTML together."
                )
            else:
                print(
                    "Recorded alignment: "
                    f"SVO starts +{timeline.svo_start_delay_seconds:.3f}s, "
                    f"LiDAR starts +{timeline.bag_start_delay_seconds:.3f}s; "
                    f"durations SVO={timeline.svo_duration_seconds:.3f}s, "
                    f"LiDAR={timeline.bag_duration_seconds:.3f}s"
                )
                if args.rate != 1.0:
                    print(
                        "Warning: camera remains at ZED_Explorer speed 1.0x; "
                        "use --rate 1.0 for sustained synchronization."
                    )
            config = json.loads(args.config.expanduser().resolve().read_text(encoding="utf-8"))
            configured_executable = config.get("zed_explorer", "ZED_Explorer")
            executable = args.zed_explorer or configured_executable
            resolved_executable = shutil.which(executable)
            if resolved_executable is None:
                executable_path = Path(executable).expanduser()
                if executable_path.is_file():
                    resolved_executable = str(executable_path.resolve())
                else:
                    raise ReplayError(
                        f"ZED_Explorer was not found: {executable}. "
                        "Set --zed-explorer or check config.json."
                    )

            process: Optional[subprocess.Popen] = None
            if args.no_open:
                process = subprocess.Popen(
                    [resolved_executable, str(svo_path)]
                )
            elif timeline is not None and timeline.bag_start_delay_seconds > 0:
                # カメラ録画の方が先に始まっている場合は、カメラを先に起動する。
                # 記録上のLiDAR開始差分だけ待ってからHTMLを開く。
                process = subprocess.Popen([resolved_executable, str(svo_path)])
                _wait_before_start(max(0.0, timeline.bag_start_delay_seconds - 0.35))
                _open_html(output_path, args.rate)
            else:
                # LiDAR/HTMLの方が先に始まっている場合は、HTMLを先に再生する。
                # 記録上のSVO開始差分だけ待ってからカメラを起動する。
                _open_html(output_path, args.rate)
                if timeline is not None:
                    _wait_before_start(timeline.svo_start_delay_seconds)
                process = subprocess.Popen([resolved_executable, str(svo_path)])
        except (OSError, json.JSONDecodeError, ReplayError, TypeError) as exc:
            print(f"Error: could not start camera replay: {exc}", file=sys.stderr)
            return 1
        print("ZED camera video started with recorded-timestamp alignment. Close ZED_Explorer to finish replay.")
        if args.rate != 1.0:
            print(
                "Note: --rate controls the HTML graph; change playback speed "
                "separately in ZED_Explorer."
            )
        try:
            return process.wait()
        except KeyboardInterrupt:
            if process.poll() is None:
                process.terminate()
                try:
                    process.wait(timeout=3.0)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait(timeout=3.0)
            return 130

    if args.bag is not None:
        command = [
            sys.executable,
            str(PROJECT_ROOT / "scripts" / "replay_tracking_rviz.py"),
            str(args.bag.expanduser().resolve()),
            "--tracks-csv",
            str(csv_path),
            "--rate",
            str(args.rate),
            "--exit-after-playback",
        ]
        if not args.no_open:
            command.extend(["--browser-html", str(output_path)])
        if args.config is not None:
            command.extend(["--config", str(args.config.expanduser().resolve())])
        if args.topic is not None:
            command.extend(["--topic", args.topic])
        try:
            return subprocess.run(command, check=False).returncode
        except OSError as exc:
            print(f"Error: could not start RViz replay: {exc}", file=sys.stderr)
            return 1
    if not args.no_open:
        webbrowser.open(output_path.as_uri())
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
