"""Lazy ROS 2 rosbag2 adapter supporting PointCloud2 and Livox CustomMsg."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Iterator, Optional

import numpy as np
import yaml

from .pointcloud import PointCloud


@dataclass(frozen=True)
class BagPointCloudFrame:
    cloud: PointCloud
    timestamp_ns: int
    frame_id: str
    message_type: str
    topic: str


def _storage_id(bag_path: Path) -> str:
    metadata = bag_path / "metadata.yaml"
    if not metadata.is_file():
        raise ValueError(f"rosbag2 metadata.yaml not found in {bag_path}")
    value = yaml.safe_load(metadata.read_text(encoding="utf-8")) or {}
    info = value.get("rosbag2_bagfile_information", {})
    storage = info.get("storage_identifier")
    if not storage:
        raise ValueError(f"No storage_identifier in {metadata}")
    return str(storage)


def _pointcloud2_frame(message: object, timestamp_ns: int, topic: str, message_type: str) -> BagPointCloudFrame:
    try:
        from sensor_msgs_py import point_cloud2
    except ImportError as exc:
        raise RuntimeError("sensor_msgs_py is required for PointCloud2 rosbag input.") from exc
    fields = {field.name for field in message.fields}
    if not {"x", "y", "z"}.issubset(fields):
        raise ValueError(f"PointCloud2 on {topic} is missing x/y/z fields: {sorted(fields)}")
    field_names = ["x", "y", "z"]
    intensity_name = next((name for name in ("intensity", "reflectivity") if name in fields), None)
    if intensity_name:
        field_names.append(intensity_name)
    try:
        values = point_cloud2.read_points(
            message, field_names=field_names, skip_nans=True, reshape_organized_cloud=False
        )
    except TypeError:
        values = point_cloud2.read_points(message, field_names=field_names, skip_nans=True)
    array = np.asarray(values)
    if array.dtype.names:
        points = np.column_stack([array[name] for name in ("x", "y", "z")]).astype(float)
        intensity = array[intensity_name].astype(float) if intensity_name else None
    else:
        array = np.asarray(list(values), dtype=float).reshape(-1, len(field_names))
        points = array[:, :3]
        intensity = array[:, 3] if intensity_name else None
    cloud = PointCloud(points, intensity, source=f"{topic}@{timestamp_ns}")
    frame_id = str(getattr(getattr(message, "header", None), "frame_id", ""))
    return BagPointCloudFrame(cloud, timestamp_ns, frame_id, message_type, topic)


def _custom_msg_frame(message: object, timestamp_ns: int, topic: str, message_type: str) -> BagPointCloudFrame:
    values = getattr(message, "points", None)
    if values is None:
        raise ValueError(f"Livox CustomMsg on {topic} has no points field.")
    points = []
    intensity = []
    has_intensity = True
    for point in values:
        points.append((float(point.x), float(point.y), float(point.z)))
        reflected = getattr(point, "reflectivity", getattr(point, "intensity", None))
        if reflected is None:
            has_intensity = False
        else:
            intensity.append(float(reflected))
    cloud = PointCloud(
        np.asarray(points, dtype=float).reshape(-1, 3),
        np.asarray(intensity, dtype=float) if has_intensity else None,
        source=f"{topic}@{timestamp_ns}",
    )
    header = getattr(message, "header", None)
    frame_id = str(getattr(header, "frame_id", ""))
    return BagPointCloudFrame(cloud, timestamp_ns, frame_id, message_type, topic)


def iter_bag_pointclouds(
    bag_path: Path,
    *,
    topic: str = "/livox/lidar",
    storage_id: Optional[str] = None,
) -> Iterator[BagPointCloudFrame]:
    """Yield point-cloud messages and reject all unknown ROS message types."""

    try:
        import rosbag2_py
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message
    except ImportError as exc:
        raise RuntimeError(
            "ROS 2 Jazzy Python packages (rosbag2_py, rclpy, rosidl_runtime_py) "
            "are required only for rosbag2 input."
        ) from exc
    bag_path = bag_path.expanduser().resolve()
    selected_storage = storage_id or _storage_id(bag_path)
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_path), storage_id=selected_storage),
        rosbag2_py.ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr"),
    )
    topic_types = {entry.name: entry.type for entry in reader.get_all_topics_and_types()}
    if topic not in topic_types:
        raise ValueError(f"Topic {topic!r} not found in bag. Available: {sorted(topic_types)}")
    message_type = topic_types[topic]
    if message_type not in {"sensor_msgs/msg/PointCloud2", "livox_ros_driver2/msg/CustomMsg"}:
        raise ValueError(
            f"Unsupported point-cloud topic type {message_type!r}. "
            "Expected sensor_msgs/msg/PointCloud2 or livox_ros_driver2/msg/CustomMsg."
        )
    message_class = get_message(message_type)
    while reader.has_next():
        current_topic, serialized, timestamp_ns = reader.read_next()
        if current_topic != topic:
            continue
        message = deserialize_message(serialized, message_class)
        if message_type == "sensor_msgs/msg/PointCloud2":
            yield _pointcloud2_frame(message, int(timestamp_ns), topic, message_type)
        else:
            yield _custom_msg_frame(message, int(timestamp_ns), topic, message_type)


def read_bag_pointcloud(
    bag_path: Path,
    *,
    topic: str = "/livox/lidar",
    frame_index: int = 0,
    storage_id: Optional[str] = None,
) -> BagPointCloudFrame:
    if frame_index < 0:
        raise ValueError("Point-cloud frame index must be non-negative.")
    for index, frame in enumerate(iter_bag_pointclouds(bag_path, topic=topic, storage_id=storage_id)):
        if index == frame_index:
            return frame
    raise ValueError(f"No point-cloud frame {frame_index} exists on {topic} in {bag_path}.")
