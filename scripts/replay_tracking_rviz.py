#!/usr/bin/env python3
"""録画済みのPointCloud2 bagを再生し、人物追跡結果をRViz2へ表示する。

点群はlivox_frameで表示し、人物の球・軌跡・速度矢印を重ねる。
廊下座標の校正がある場合は、人物ラベルに廊下中心座標も併記する。
"""

from __future__ import annotations

import argparse
import csv
import math
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import time
import webbrowser
from typing import Optional, Sequence

import numpy as np

from track_people import (
    CsvWriter,
    MarkerPublisher,
    PeopleTracker,
    TrackSnapshot,
    TrackingError,
    _iter_bag_clouds,
    _message_timestamp_ns,
    _pointcloud2_to_numpy,
    load_settings,
)


PROJECT_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CONFIG_PATH = PROJECT_ROOT / "config.json"
DEFAULT_TOPIC = "/livox/lidar"
DEFAULT_POINT_TOPIC = "/tracking/points"


def build_rviz_config(point_topic: str, marker_topic: str, fixed_frame: str) -> str:
    """点群表示と人物マーカー表示に必要な最小RViz設定を作る。"""

    point_topic_yaml = _yaml_string(point_topic)
    marker_topic_yaml = _yaml_string(marker_topic)
    frame_yaml = _yaml_string(fixed_frame)
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
      Color Transformer: FlatColor
      Enabled: true
      Name: Recorded PointCloud2
      Position Transformer: XYZ
      Queue Size: 10
      Selectable: true
      Size (Pixels): 3
      Style: Flat Squares
      Topic: {point_topic_yaml}
      Use Fixed Frame: true
      Value: true
    - Class: rviz_default_plugins/MarkerArray
      Enabled: true
      Name: Tracked people
      Namespaces:
        people: true
        people_labels: true
        people_velocity: true
        people_trajectory: true
      Topic: {marker_topic_yaml}
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


def _yaml_string(value: str) -> str:
    """外部ライブラリを使わずにYAML文字列を安全に引用する。"""

    return '"' + value.replace("\\", "\\\\").replace('"', '\\"') + '"'


def _parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bags", nargs="+", type=Path, help="rosbag2 directories in playback order")
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG_PATH)
    parser.add_argument("--topic", default=None, help=f"PointCloud2 topic (default: {DEFAULT_TOPIC})")
    parser.add_argument("--output-dir", type=Path, default=PROJECT_ROOT / "tracking")
    parser.add_argument("--rate", type=float, default=1.0, help="Recorded-time playback rate")
    parser.add_argument("--stride", type=int, default=1, help="Process every Nth cloud")
    parser.add_argument("--point-topic", default=DEFAULT_POINT_TOPIC, help="RViz PointCloud2 topic")
    parser.add_argument("--markers-topic", default=None, help="RViz MarkerArray topic")
    parser.add_argument("--fixed-frame", default="livox_frame")
    parser.add_argument("--rviz-executable", default="rviz2")
    parser.add_argument("--rviz-config", type=Path, default=None, help="Use an existing RViz2 config")
    parser.add_argument("--no-rviz", action="store_true", help="Publish without launching RViz2")
    parser.add_argument(
        "--exit-after-playback",
        action="store_true",
        help="Close the automatically launched RViz2 when the last bag ends",
    )
    parser.add_argument("--storage-id", default="mcap")
    parser.add_argument(
        "--tracks-csv",
        type=Path,
        default=None,
        help="reuse tracked positions from a CSV instead of detecting people again",
    )
    parser.add_argument(
        "--browser-html",
        type=Path,
        default=None,
        help="open the CSV motion and speed view with this RViz replay",
    )
    parser.add_argument(
        "--include-tentative",
        action="store_true",
        help="also publish unconfirmed CSV tracks for diagnostics",
    )
    return parser.parse_args(argv)


def _validate(args: argparse.Namespace) -> None:
    if not math.isfinite(args.rate) or args.rate <= 0:
        raise TrackingError("--rate must be positive and finite.")
    if args.stride < 1:
        raise TrackingError("--stride must be positive.")
    if not args.point_topic.strip() or not args.fixed_frame.strip():
        raise TrackingError("--point-topic and --fixed-frame must not be empty.")
    for bag in args.bags:
        if not bag.expanduser().resolve().is_dir():
            raise TrackingError(f"rosbag2 directory does not exist: {bag}")
    if args.tracks_csv is not None:
        if len(args.bags) != 1:
            raise TrackingError("--tracks-csv can be used with exactly one bag.")
        if not args.tracks_csv.expanduser().resolve().is_file():
            raise TrackingError(f"tracking CSV does not exist: {args.tracks_csv}")
    if args.browser_html is not None:
        if args.tracks_csv is None:
            raise TrackingError("--browser-html requires --tracks-csv.")
        if not args.browser_html.expanduser().resolve().is_file():
            raise TrackingError(f"browser replay HTML does not exist: {args.browser_html}")


def _start_rviz(args: argparse.Namespace, marker_topic: str) -> tuple[Optional[subprocess.Popen], Optional[Path]]:
    if args.no_rviz:
        return None, None
    config_path = args.rviz_config.expanduser().resolve() if args.rviz_config else None
    temporary_path: Optional[Path] = None
    if config_path is None:
        handle = tempfile.NamedTemporaryFile(
            mode="w", encoding="utf-8", suffix=".rviz", prefix="tracking_", delete=False
        )
        try:
            handle.write(build_rviz_config(args.point_topic, marker_topic, args.fixed_frame))
        finally:
            handle.close()
        config_path = Path(handle.name)
        temporary_path = config_path
    elif not config_path.is_file():
        raise TrackingError(f"RViz2 config file does not exist: {config_path}")

    try:
        process = subprocess.Popen(
            [args.rviz_executable, "-d", os.fspath(config_path)],
            stdin=subprocess.DEVNULL,
        )
    except OSError as exc:
        if temporary_path is not None:
            temporary_path.unlink(missing_ok=True)
        raise TrackingError(f"Could not start RViz2: {exc}") from exc
    return process, temporary_path


def _stop_rviz(process: Optional[subprocess.Popen]) -> None:
    if process is None or process.poll() is not None:
        return
    process.terminate()
    try:
        process.wait(timeout=3.0)
    except subprocess.TimeoutExpired:
        process.kill()
        process.wait(timeout=3.0)


def _load_track_csv(path: Path) -> dict[int, list[TrackSnapshot]]:
    """PointCloud2のタイムスタンプをキーにしてCSVの追跡状態を読み込む。"""

    snapshots_by_timestamp: dict[int, list[TrackSnapshot]] = {}
    required = {"timestamp_ns", "track_id", "status", "x", "y", "z", "vx", "vy"}
    try:
        stream = path.expanduser().resolve().open("r", encoding="utf-8-sig", newline="")
    except OSError as exc:
        raise TrackingError(f"Could not open tracking CSV {path}: {exc}") from exc
    with stream:
        reader = csv.DictReader(stream)
        missing = required - set(reader.fieldnames or [])
        if missing:
            raise TrackingError(
                "Tracking CSV is missing columns: " + ", ".join(sorted(missing))
            )
        for line, row in enumerate(reader, start=2):
            try:
                timestamp_ns = int(row["timestamp_ns"])
                track_id = int(row["track_id"])
                position = np.asarray(
                    [float(row["x"]), float(row["y"]), float(row["z"])],
                    dtype=np.float64,
                )
                velocity = np.asarray(
                    [float(row["vx"]), float(row["vy"]), 0.0], dtype=np.float64
                )
                extent = np.asarray(
                    [
                        float(row.get("width") or 0.0),
                        float(row.get("depth") or 0.0),
                        float(row.get("height") or 0.0),
                    ],
                    dtype=np.float64,
                )
                point_count = int(row.get("point_count") or 0)
                confidence = float(row.get("confidence") or 0.0)
                hits = int(row.get("hits") or 1)
                missed_frames = int(row.get("missed_frames") or 0)
            except (KeyError, TypeError, ValueError) as exc:
                raise TrackingError(f"Invalid tracking CSV row at line {line}: {exc}") from exc
            if (
                not np.isfinite(position).all()
                or not np.isfinite(velocity).all()
                or not np.isfinite(extent).all()
                or not math.isfinite(confidence)
            ):
                raise TrackingError(
                    f"Tracking CSV contains non-finite track values at line {line}."
                )
            status = (row.get("status") or "").strip().lower()
            detected = (row.get("detected") or "1").strip().lower() not in {"0", "false"}
            snapshot = TrackSnapshot(
                track_id=track_id,
                position=position,
                velocity=velocity,
                point_count=point_count,
                extent=extent,
                confidence=confidence,
                hits=hits,
                missed_frames=missed_frames,
                detected=detected,
                confirmed=status in {"confirmed", "predicted"},
            )
            snapshots_by_timestamp.setdefault(timestamp_ns, []).append(snapshot)
    if not snapshots_by_timestamp:
        raise TrackingError(f"Tracking CSV contains no track rows: {path}")
    return snapshots_by_timestamp


def _replay_bag(
    node: object,
    point_publisher: object,
    marker_publisher: MarkerPublisher,
    bag_path: Path,
    settings: object,
    output_path: Path,
    args: argparse.Namespace,
    rclpy: object,
    csv_snapshots: Optional[dict[int, list[TrackSnapshot]]] = None,
) -> int:
    tracker = PeopleTracker(settings) if csv_snapshots is None else None
    writer = (
        CsvWriter(output_path, settings.corridor_frame, settings.lidar_to_zed_frame)
        if csv_snapshots is None
        else None
    )
    frame = 0
    cloud_count = 0
    start_timestamp_ns: Optional[int] = None
    previous_timestamp_ns: Optional[int] = None
    matched_csv_frames = 0
    try:
        marker_publisher.reset_sequence()
        print(f"\nReplay: {bag_path}")
        for message, bag_timestamp_ns, actual_type in _iter_bag_clouds(
            bag_path, settings.topic, "pointcloud2", args.storage_id
        ):
            cloud_count += 1
            if cloud_count % args.stride != 0:
                continue
            if actual_type != "pointcloud2":
                raise TrackingError(
                    f"{bag_path} contains {actual_type}; RViz replay currently requires PointCloud2."
                )
            timestamp_ns = _message_timestamp_ns(message, bag_timestamp_ns)
            if previous_timestamp_ns is not None:
                delay = (timestamp_ns - previous_timestamp_ns) / 1_000_000_000.0 / args.rate
                if math.isfinite(delay) and delay > 0:
                    time.sleep(delay)
            previous_timestamp_ns = timestamp_ns
            if start_timestamp_ns is None:
                start_timestamp_ns = timestamp_ns

            if csv_snapshots is None:
                points = _pointcloud2_to_numpy(message)
                snapshots = tracker.update(points, timestamp_ns)
                if writer is not None:
                    writer.write(frame + 1, timestamp_ns, snapshots, start_timestamp_ns or timestamp_ns)
            else:
                snapshots = csv_snapshots.get(timestamp_ns, [])
                if timestamp_ns in csv_snapshots:
                    matched_csv_frames += 1
            frame += 1
            point_publisher.publish(message)
            header = getattr(message, "header", None)
            frame_id = getattr(header, "frame_id", "") or args.fixed_frame
            marker_publisher.publish(snapshots, frame_id, header.stamp)
            rclpy.spin_once(node, timeout_sec=0.0)
            if frame == 1 or frame % 10 == 0:
                print(
                    f"\r  frames={frame} active_tracks={len(snapshots)} "
                    f"created_ids={tracker.next_id - 1 if tracker is not None else 'CSV'}",
                    end="",
                    flush=True,
                )
        print()
        if tracker is None:
            if matched_csv_frames == 0:
                raise TrackingError(
                    "No CSV timestamps matched this bag. Use the rosbag used to create the CSV."
                )
            print(f"  processed={frame}, CSV track frames matched={matched_csv_frames}")
        else:
            print(f"  processed={frame}, created_ids={tracker.next_id - 1}")
            print(f"  CSV={output_path}")
        return frame
    finally:
        if writer is not None:
            writer.close()


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = _parse_args(argv)
    rviz_process: Optional[subprocess.Popen] = None
    temporary_rviz_config: Optional[Path] = None
    node = None
    rclpy = None
    try:
        _validate(args)
        settings = load_settings(args.config.expanduser().resolve(), args.topic)
        csv_snapshots = (
            _load_track_csv(args.tracks_csv)
            if args.tracks_csv is not None
            else None
        )
        if args.markers_topic is not None:
            settings.markers_topic = args.markers_topic.strip()

        # ROSノードと配信先を用意し、RViz側の購読準備を待つ。
        import rclpy as rclpy_module
        from rclpy.node import Node
        from sensor_msgs.msg import PointCloud2

        rclpy = rclpy_module
        rclpy.init(args=None)
        node = Node("recorded_people_tracking_rviz")
        point_publisher = node.create_publisher(
            # RVizのPointCloud2表示は既定でReliable QoSを要求する。
            # センサードライバのBest Effortではなく、互換性のあるQoSを使う。
            PointCloud2, args.point_topic, 10
        )
        marker_publisher = MarkerPublisher(
            node,
            settings.markers_topic,
            include_tentative=args.include_tentative,
            corridor_frame=settings.corridor_frame,
        )
        rviz_process, temporary_rviz_config = _start_rviz(args, settings.markers_topic)
        if rviz_process is not None:
            deadline = time.monotonic() + 10.0
            while (
                point_publisher.get_subscription_count() == 0
                and rviz_process.poll() is None
                and time.monotonic() < deadline
            ):
                time.sleep(0.1)
            if point_publisher.get_subscription_count() == 0:
                print("Warning: RViz2 has not subscribed to the point cloud topic yet.")
        if args.browser_html is not None:
            browser_url = (
                args.browser_html.expanduser().resolve().as_uri()
                + f"?autoplay=1&rate={args.rate}"
            )
            webbrowser.open(browser_url)
            # ブラウザがHTMLを読み込み、再生を開始する時間を確保する。
            time.sleep(0.5)
        print(f"PointCloud2 topic: {args.point_topic}")
        print(f"MarkerArray topic: {settings.markers_topic}")
        print(f"Fixed frame: {args.fixed_frame}")
        print(
            "Static-cluster background: "
            f"voxel={settings.cluster_background_voxel_size}, "
            f"presence={settings.cluster_background_min_presence}"
        )

        # 複数のbagを順に再生し、それぞれの追跡CSVも保存する。
        for bag in args.bags:
            bag_path = bag.expanduser().resolve()
            output_path = args.output_dir.expanduser().resolve() / f"{bag_path.name}_tracks.csv"
            _replay_bag(
                node,
                point_publisher,
                marker_publisher,
                bag_path,
                settings,
                output_path,
                args,
                rclpy,
                csv_snapshots,
            )
        print("\nAll bags have finished.")
        if rviz_process is not None and not args.exit_after_playback:
            print("RViz2を確認後、EnterまたはCtrl+Cで終了します。")
            try:
                input()
            except (EOFError, KeyboardInterrupt):
                pass
        return 0
    except (TrackingError, OSError, ValueError, RuntimeError) as exc:
        print(f"Error: {exc}", file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        return 130
    finally:
        # 正常終了と割り込みの両方でRViz、設定ファイル、ROSノードを解放する。
        _stop_rviz(rviz_process)
        if temporary_rviz_config is not None:
            temporary_rviz_config.unlink(missing_ok=True)
        if node is not None:
            node.destroy_node()
        if rclpy is not None and rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
