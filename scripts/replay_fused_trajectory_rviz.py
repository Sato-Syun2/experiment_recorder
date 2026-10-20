#!/usr/bin/env python3
"""Replay a LiDAR bag and overlay fused person positions and ZED BODY_34 joints in RViz2."""

from __future__ import annotations

import argparse
import bisect
import colorsys
import csv
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import time
from typing import Any, Optional, Sequence

import numpy as np

from track_people import TrackingError, _iter_bag_clouds, _message_timestamp_ns, load_settings


PROJECT_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CONFIG = PROJECT_ROOT / "config.json"
DEFAULT_FUSED_CSV = PROJECT_ROOT / "tracking/20261013_group001_recording_001_zed_lidar_fusion/fused_trajectory.csv"
DEFAULT_POINT_TOPIC = "/tracking/points"
DEFAULT_MARKER_TOPIC = "/people/fused_skeleton"

# Stereolabs BODY_34 index order. Only edges with two valid joints are drawn.
BODY_34_BONES = [
    (0, 1), (1, 2), (2, 4), (4, 5), (5, 6), (6, 7), (7, 8), (8, 9),
    (7, 10), (2, 11), (11, 12), (12, 13), (13, 14), (14, 15), (15, 16),
    (14, 17), (0, 18), (18, 19), (19, 20), (20, 21), (0, 22), (22, 23),
    (23, 24), (24, 25), (2, 3), (3, 26), (26, 27), (27, 28), (28, 29),
    (27, 30), (30, 31), (20, 32), (24, 33), (32, 21), (33, 25),
]

MARKER_NAMESPACES = (
    "fused_people",
    "fused_labels",
    "fused_skeleton_bones",
    "fused_skeleton_joints",
    "fused_trajectory",
)


def finite_float(value: Any) -> Optional[float]:
    try:
        result = float(value)
    except (TypeError, ValueError):
        return None
    return result if math.isfinite(result) else None


def _parse_point(value: Any) -> Optional[np.ndarray]:
    if not isinstance(value, list) or len(value) < 3:
        return None
    values = [finite_float(item) for item in value[:3]]
    if any(item is None for item in values):
        return None
    return np.asarray(values, dtype=np.float64)


def load_fused_rows(path: Path) -> tuple[list[int], dict[int, list[dict[str, Any]]]]:
    """Load fused observations keyed by their original ZED image timestamp."""

    grouped: dict[int, list[dict[str, Any]]] = {}
    required = {
        "timestamp_ns", "zed_body_id", "motion_valid", "fused_position_corridor_x_m",
        "fused_position_corridor_y_m", "fused_position_corridor_z_m", "keypoints_corridor_json",
    }
    try:
        stream = path.expanduser().resolve().open("r", encoding="utf-8-sig", newline="")
    except OSError as exc:
        raise TrackingError(f"Could not open fused trajectory CSV: {exc}") from exc
    with stream:
        reader = csv.DictReader(stream)
        missing = required - set(reader.fieldnames or [])
        if missing:
            raise TrackingError("Fused trajectory CSV is missing columns: " + ", ".join(sorted(missing)))
        for line, row in enumerate(reader, start=2):
            try:
                timestamp_ns = int(row["timestamp_ns"])
                body_id = int(row["zed_body_id"])
            except (TypeError, ValueError) as exc:
                raise TrackingError(f"Invalid fused trajectory row at line {line}: {exc}") from exc
            try:
                raw_joints = json.loads(row.get("keypoints_corridor_json") or "[]")
            except json.JSONDecodeError as exc:
                raise TrackingError(f"Invalid joint JSON at line {line}: {exc}") from exc
            if not isinstance(raw_joints, list):
                raw_joints = []
            joints = [_parse_point(point) for point in raw_joints[:34]]
            joints.extend([None] * (34 - len(joints)))
            position = None
            if row.get("motion_valid") == "1":
                position = _parse_point([
                    row.get("fused_position_corridor_x_m"),
                    row.get("fused_position_corridor_y_m"),
                    row.get("fused_position_corridor_z_m"),
                ])
            if position is None and not any(point is not None for point in joints):
                continue
            observation = {
                "timestamp_ns": timestamp_ns,
                "body_id": body_id,
                "position_corridor": position,
                "joints_corridor": joints,
                "fusion_status": row.get("fusion_status", ""),
                "speed_mps": finite_float(row.get("fused_speed_corridor_mps")),
                "segment_id": row.get("motion_segment_id", ""),
            }
            grouped.setdefault(timestamp_ns, []).append(observation)
    if not grouped:
        raise TrackingError(f"Fused trajectory CSV has no displayable position or joint data: {path}")
    timestamps = sorted(grouped)
    return timestamps, grouped


class CorridorToCloudFrame:
    """Convert calibrated corridor points and vectors back to the raw bag frame."""

    def __init__(self, settings: Any) -> None:
        corridor = settings.corridor_frame
        if corridor is None:
            raise TrackingError("A corridor calibration is required to overlay the fused trajectory.")
        self.corridor = corridor
        source = str(corridor.source_frame)
        if source not in {"livox_level_frame", "livox_frame"}:
            raise TrackingError(
                f"Unsupported corridor calibration source frame '{source}'; expected livox_level_frame or livox_frame."
            )
        self.level_source = source == "livox_level_frame"
        self.rotation = np.asarray(settings.gravity_alignment_rotation, dtype=np.float64)
        self.translation = np.asarray(settings.gravity_alignment_translation, dtype=np.float64)

    def _level_to_cloud(self, level_point: np.ndarray) -> np.ndarray:
        if not self.level_source:
            return level_point
        # level = R * raw + translation. Use the row-vector form of the inverse.
        return (level_point - self.translation) @ self.rotation

    def point(self, corridor_point: np.ndarray) -> np.ndarray:
        frame = self.corridor
        level = np.array([
            frame.origin_xy[0] + frame.longitudinal_xy[0] * corridor_point[0] + frame.lateral_xy[0] * corridor_point[1],
            frame.origin_xy[1] + frame.longitudinal_xy[1] * corridor_point[0] + frame.lateral_xy[1] * corridor_point[1],
            corridor_point[2],
        ], dtype=np.float64)
        return self._level_to_cloud(level)

    def vector(self, corridor_vector: np.ndarray) -> np.ndarray:
        frame = self.corridor
        level = np.array([
            frame.longitudinal_xy[0] * corridor_vector[0] + frame.lateral_xy[0] * corridor_vector[1],
            frame.longitudinal_xy[1] * corridor_vector[0] + frame.lateral_xy[1] * corridor_vector[1],
            corridor_vector[2],
        ], dtype=np.float64)
        return level @ self.rotation if self.level_source else level


class FusedMarkerPublisher:
    def __init__(self, node: Any, topic: str, transform: CorridorToCloudFrame) -> None:
        try:
            from visualization_msgs.msg import Marker, MarkerArray
        except ImportError as exc:
            raise TrackingError("visualization_msgs is unavailable; source the ROS 2 setup first.") from exc
        from geometry_msgs.msg import Point

        self.Marker = Marker
        self.MarkerArray = MarkerArray
        self.Point = Point
        self.publisher = node.create_publisher(MarkerArray, topic, 10)
        self.transform = transform
        self.previous_ids: set[int] = set()
        self.trajectory: dict[int, list[tuple[int, str, tuple[float, float, float]]]] = {}

    @staticmethod
    def _color(body_id: int) -> tuple[float, float, float]:
        return colorsys.hsv_to_rgb((body_id * 0.61803398875 + 0.08) % 1.0, 0.82, 1.0)

    def _marker(self, frame_id: str, stamp: Any, namespace: str, marker_id: int, kind: int) -> Any:
        marker = self.Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = stamp
        marker.ns = namespace
        marker.id = marker_id
        marker.type = kind
        marker.action = self.Marker.ADD
        marker.pose.orientation.w = 1.0
        return marker

    def _point(self, xyz: np.ndarray) -> Any:
        point = self.Point()
        point.x, point.y, point.z = (float(value) for value in xyz)
        return point

    def _set_color(self, marker: Any, color: tuple[float, float, float], alpha: float) -> None:
        marker.color.r, marker.color.g, marker.color.b = color
        marker.color.a = alpha

    def publish(self, observations: Sequence[dict[str, Any]], frame_id: str, stamp: Any) -> None:
        array = self.MarkerArray()
        active_ids = {int(item["body_id"]) for item in observations}
        for removed_id in self.previous_ids - active_ids:
            for namespace in MARKER_NAMESPACES:
                marker = self.Marker()
                marker.header.frame_id = frame_id
                marker.header.stamp = stamp
                marker.ns = namespace
                marker.id = removed_id
                marker.action = self.Marker.DELETE
                array.markers.append(marker)
            self.trajectory.pop(removed_id, None)

        for observation in observations:
            body_id = int(observation["body_id"])
            color = self._color(body_id)
            position_corridor = observation["position_corridor"]
            position = self.transform.point(position_corridor) if position_corridor is not None else None
            emitted_namespaces: set[str] = set()
            joints = [
                self.transform.point(point) if point is not None else None
                for point in observation["joints_corridor"]
            ]
            valid_joints = [point for point in joints if point is not None]

            if position is not None:
                center = self._marker(frame_id, stamp, "fused_people", body_id, self.Marker.SPHERE)
                center.pose.position.x, center.pose.position.y, center.pose.position.z = (float(v) for v in position)
                center.scale.x = center.scale.y = center.scale.z = 0.24
                self._set_color(center, color, 0.95)
                array.markers.append(center)
                emitted_namespaces.add("fused_people")

                path = self.trajectory.setdefault(body_id, [])
                timestamp_ns = int(observation["timestamp_ns"])
                segment_id = str(observation["segment_id"])
                if path and path[-1][1] != segment_id:
                    path.clear()
                if not path or path[-1][0] != timestamp_ns:
                    path.append((timestamp_ns, segment_id, tuple(float(v) for v in position)))
                if len(path) > 60:
                    del path[:-60]
                trail = self._marker(frame_id, stamp, "fused_trajectory", body_id, self.Marker.LINE_STRIP)
                trail.scale.x = 0.035
                self._set_color(trail, color, 0.75)
                trail.points = [self._point(np.asarray(item[2])) for item in path]
                array.markers.append(trail)
                emitted_namespaces.add("fused_trajectory")
            else:
                # Do not leave a stale location or connect the trail across a rejected frame.
                self.trajectory.pop(body_id, None)

            if valid_joints:
                bones = self._marker(frame_id, stamp, "fused_skeleton_bones", body_id, self.Marker.LINE_LIST)
                bones.scale.x = 0.035
                self._set_color(bones, color, 0.95)
                for first, second in BODY_34_BONES:
                    if first < len(joints) and second < len(joints) and joints[first] is not None and joints[second] is not None:
                        bones.points.extend([self._point(joints[first]), self._point(joints[second])])
                array.markers.append(bones)
                emitted_namespaces.add("fused_skeleton_bones")

                joint_marker = self._marker(frame_id, stamp, "fused_skeleton_joints", body_id, self.Marker.SPHERE_LIST)
                joint_marker.scale.x = joint_marker.scale.y = joint_marker.scale.z = 0.075
                self._set_color(joint_marker, (1.0, 0.78, 0.12), 0.98)
                joint_marker.points = [self._point(point) for point in valid_joints]
                array.markers.append(joint_marker)
                emitted_namespaces.add("fused_skeleton_joints")

            anchor = position if position is not None else next(
                (point for point in joints if point is not None), None
            )
            if anchor is not None:
                label = self._marker(frame_id, stamp, "fused_labels", body_id, self.Marker.TEXT_VIEW_FACING)
                up = self.transform.vector(np.asarray([0.0, 0.0, 0.45]))
                label.pose.position.x = float(anchor[0] + up[0])
                label.pose.position.y = float(anchor[1] + up[1])
                label.pose.position.z = float(anchor[2] + up[2])
                label.scale.z = 0.16
                label.color.r = label.color.g = label.color.b = 1.0
                label.color.a = 0.98
                status = str(observation["fusion_status"])
                speed = observation["speed_mps"]
                speed_text = "" if speed is None else f"  {speed:.2f} m/s"
                position_text = ""
                if position_corridor is not None:
                    position_text = (
                        f"\ncorridor=({position_corridor[0]:.2f}, "
                        f"{position_corridor[1]:.2f}, {position_corridor[2]:.2f}) m"
                    )
                label.text = f"ZED ID {body_id}  {status}{speed_text}{position_text}"
                array.markers.append(label)
                emitted_namespaces.add("fused_labels")

            for namespace in set(MARKER_NAMESPACES) - emitted_namespaces:
                marker = self.Marker()
                marker.header.frame_id = frame_id
                marker.header.stamp = stamp
                marker.ns = namespace
                marker.id = body_id
                marker.action = self.Marker.DELETE
                array.markers.append(marker)

        self.previous_ids = active_ids
        self.publisher.publish(array)


def _yaml_string(value: str) -> str:
    return '"' + value.replace("\\", "\\\\").replace('"', '\\"') + '"'


def build_rviz_config(point_topic: str, marker_topic: str, fixed_frame: str) -> str:
    return f'''Panels:
  - Class: rviz_common/Displays
    Name: Displays
Visualization Manager:
  Class: ""
  Displays:
    - Alpha: 1
      Class: rviz_default_plugins/PointCloud2
      Color: 245; 245; 245
      Color Transformer: FlatColor
      Enabled: true
      Name: Recorded LiDAR point cloud
      Position Transformer: XYZ
      Queue Size: 10
      Size (Pixels): 2
      Style: Flat Squares
      Topic: {_yaml_string(point_topic)}
      Value: true
    - Class: rviz_default_plugins/MarkerArray
      Enabled: true
      Name: Fused person and BODY_34 skeleton
      Namespaces:
        fused_people: true
        fused_labels: true
        fused_skeleton_bones: true
        fused_skeleton_joints: true
        fused_trajectory: true
      Topic: {_yaml_string(marker_topic)}
      Value: true
    - Alpha: 0.35
      Class: rviz_default_plugins/Grid
      Color: 160; 160; 160
      Enabled: true
      Name: Ground grid
      Plane: XY
      Plane Cell Count: 30
      Reference Frame: {_yaml_string(fixed_frame)}
      Value: true
  Enabled: true
  Global Options:
    Background Color: 48; 48; 48
    Fixed Frame: {_yaml_string(fixed_frame)}
    Frame Rate: 30
  Name: root
  Tools:
    - Class: rviz_default_plugins/Interact
    - Class: rviz_default_plugins/MoveCamera
    - Class: rviz_default_plugins/Select
  Views:
    Current:
      Class: rviz_default_plugins/Orbit
      Distance: 16
      Focal Point: 6; 0; 1
      Pitch: 0.65
      Yaw: 0.8
Window Geometry:
  Height: 850
  Width: 1300
'''


def _parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", type=Path, help="rosbag2 directory containing the LiDAR point cloud")
    parser.add_argument("--fused-csv", type=Path, default=DEFAULT_FUSED_CSV, help="fused_trajectory.csv from fuse_zed_lidar.py")
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--topic", default=None, help="PointCloud2 topic; defaults to config.json")
    parser.add_argument("--point-topic", default=DEFAULT_POINT_TOPIC)
    parser.add_argument("--markers-topic", default=DEFAULT_MARKER_TOPIC)
    parser.add_argument("--fixed-frame", default="livox_frame")
    parser.add_argument("--rate", type=float, default=1.0, help="recorded-time playback rate")
    parser.add_argument("--stride", type=int, default=1, help="publish every Nth point cloud")
    parser.add_argument("--max-zed-delta-ms", type=float, default=80.0, help="maximum nearest ZED observation age for a LiDAR frame")
    parser.add_argument("--timestamp-offset-ms", type=float, default=0.0, help="add this correction to the ZED timestamp when matching the LiDAR cloud")
    parser.add_argument("--storage-id", default="mcap")
    parser.add_argument("--rviz-executable", default="rviz2")
    parser.add_argument("--rviz-config", type=Path, default=None)
    parser.add_argument("--no-rviz", action="store_true", help="publish topics without launching RViz2")
    parser.add_argument("--exit-after-playback", action="store_true", help="close RViz2 when the bag ends")
    args = parser.parse_args(argv)
    if not math.isfinite(args.rate) or args.rate <= 0:
        parser.error("--rate must be positive and finite")
    if args.stride < 1:
        parser.error("--stride must be positive")
    if not math.isfinite(args.max_zed_delta_ms) or args.max_zed_delta_ms <= 0:
        parser.error("--max-zed-delta-ms must be positive and finite")
    if not math.isfinite(args.timestamp_offset_ms):
        parser.error("--timestamp-offset-ms must be finite")
    return args


def _start_rviz(args: argparse.Namespace) -> tuple[Optional[subprocess.Popen], Optional[Path]]:
    if args.no_rviz:
        return None, None
    config_path = args.rviz_config.expanduser().resolve() if args.rviz_config else None
    temporary_path: Optional[Path] = None
    if config_path is None:
        handle = tempfile.NamedTemporaryFile(mode="w", encoding="utf-8", suffix=".rviz", prefix="fused_", delete=False)
        try:
            handle.write(build_rviz_config(args.point_topic, args.markers_topic, args.fixed_frame))
        finally:
            handle.close()
        config_path = Path(handle.name)
        temporary_path = config_path
    elif not config_path.is_file():
        raise TrackingError(f"RViz2 config file does not exist: {config_path}")
    try:
        process = subprocess.Popen([args.rviz_executable, "-d", os.fspath(config_path)], stdin=subprocess.DEVNULL)
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


def _nearest_observations(
    timestamp_ns: int,
    timestamps: Sequence[int],
    grouped: dict[int, list[dict[str, Any]]],
    max_delta_ns: int,
    timestamp_offset_ns: int,
) -> list[dict[str, Any]]:
    # A positive offset is added to each ZED timestamp, matching fuse_zed_lidar.py semantics.
    target = timestamp_ns - timestamp_offset_ns
    index = bisect.bisect_left(timestamps, target)
    candidates = timestamps[max(0, index - 1):min(len(timestamps), index + 1)]
    if not candidates:
        return []
    nearest = min(candidates, key=lambda value: abs(value - target))
    return grouped[nearest] if abs(nearest - target) <= max_delta_ns else []


def replay(args: argparse.Namespace) -> None:
    bag_path = args.bag.expanduser().resolve()
    if not bag_path.is_dir() or not (bag_path / "metadata.yaml").is_file():
        raise TrackingError(f"rosbag2 directory or metadata.yaml is missing: {bag_path}")
    fused_path = args.fused_csv.expanduser().resolve()
    timestamps, grouped = load_fused_rows(fused_path)
    settings = load_settings(args.config.expanduser().resolve(), args.topic)
    transform = CorridorToCloudFrame(settings)
    topic = args.topic.strip() if isinstance(args.topic, str) else settings.topic

    try:
        import rclpy
        from rclpy.node import Node
        from sensor_msgs.msg import PointCloud2
    except ImportError as exc:
        raise TrackingError("rclpy/sensor_msgs is unavailable; source /opt/ros/jazzy/setup.bash first.") from exc

    rclpy.init(args=None)
    node = Node("replay_fused_trajectory_rviz")
    rviz_process: Optional[subprocess.Popen] = None
    temporary_config: Optional[Path] = None
    try:
        point_publisher = node.create_publisher(PointCloud2, args.point_topic, 10)
        marker_publisher = FusedMarkerPublisher(node, args.markers_topic, transform)
        rviz_process, temporary_config = _start_rviz(args)
        if rviz_process is not None:
            deadline = time.monotonic() + 10.0
            while (
                (point_publisher.get_subscription_count() == 0
                 or marker_publisher.publisher.get_subscription_count() == 0)
                and rviz_process.poll() is None
                and time.monotonic() < deadline
            ):
                time.sleep(0.1)
            if (point_publisher.get_subscription_count() == 0
                    or marker_publisher.publisher.get_subscription_count() == 0):
                print("Warning: RViz2 has not subscribed to both display topics yet.")

        print(f"Point cloud: {bag_path} [{topic}]")
        print(f"Fused CSV: {fused_path}")
        print(f"Topics: {args.point_topic}, {args.markers_topic}; fixed frame: {args.fixed_frame}")
        print(f"Timestamp correction: {args.timestamp_offset_ms:.3f} ms; nearest-frame limit: {args.max_zed_delta_ms:.1f} ms")

        previous_timestamp_ns: Optional[int] = None
        processed = 0
        matched_clouds = 0
        skeleton_clouds = 0
        max_delta_ns = int(round(args.max_zed_delta_ms * 1_000_000))
        offset_ns = int(round(args.timestamp_offset_ms * 1_000_000))
        for message, bag_timestamp_ns, actual_type in _iter_bag_clouds(bag_path, topic, "pointcloud2", args.storage_id):
            if actual_type != "pointcloud2":
                raise TrackingError(f"{topic} is not a PointCloud2 topic.")
            if processed % args.stride:
                processed += 1
                continue
            timestamp_ns = _message_timestamp_ns(message, bag_timestamp_ns)
            if previous_timestamp_ns is not None:
                delay_s = (timestamp_ns - previous_timestamp_ns) / 1_000_000_000.0 / args.rate
                if math.isfinite(delay_s) and delay_s > 0:
                    time.sleep(delay_s)
            previous_timestamp_ns = timestamp_ns

            observations = _nearest_observations(timestamp_ns, timestamps, grouped, max_delta_ns, offset_ns)
            header = getattr(message, "header", None)
            frame_id = (getattr(header, "frame_id", "") or args.fixed_frame) if header is not None else args.fixed_frame
            stamp = getattr(header, "stamp", None) if header is not None else None
            point_publisher.publish(message)
            marker_publisher.publish(observations, frame_id, stamp)
            if observations:
                matched_clouds += 1
                if any(any(point is not None for point in item["joints_corridor"]) for item in observations):
                    skeleton_clouds += 1
            processed += 1
            rclpy.spin_once(node, timeout_sec=0.0)
            if processed == 1 or processed % 20 == 0:
                print(f"\r  clouds={processed} overlays={matched_clouds} skeletons={skeleton_clouds}", end="", flush=True)
        print()
        print(f"Finished: {processed} point clouds; person overlay={matched_clouds}; skeleton overlay={skeleton_clouds}")
        if matched_clouds == 0:
            print("Warning: no ZED rows were close enough to the LiDAR cloud timestamps; check the clock offset or --max-zed-delta-ms.")
        if rviz_process is not None and not args.exit_after_playback:
            print("RViz2を確認後、EnterまたはCtrl+Cで終了します。")
            try:
                input()
            except (EOFError, KeyboardInterrupt):
                pass
    finally:
        _stop_rviz(rviz_process)
        if temporary_config is not None:
            temporary_config.unlink(missing_ok=True)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = _parse_args(argv)
    try:
        replay(args)
    except (TrackingError, OSError, ValueError, RuntimeError, json.JSONDecodeError) as exc:
        print(f"Error: {exc}", file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        return 130
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
