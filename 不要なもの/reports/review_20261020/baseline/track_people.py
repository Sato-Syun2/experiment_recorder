#!/usr/bin/env python3
"""録画またはライブのROS 2点群から人物を検出・追跡する。

点群のROI切り出し、床・静的背景の除去、人物候補クラスタの抽出、
フレーム間のID対応付けを行う。CSVには従来のLiDAR座標、重力水平化座標、
校正済みの場合は廊下中心座標も出力する。
"""

from __future__ import annotations

import argparse
import csv
import dataclasses
import datetime as dt
import json
import math
import os
import sys
import time
from pathlib import Path
from typing import Iterable, Iterator, Optional, Sequence

import numpy as np

try:
    from .corridor_frame import CorridorFrame, CorridorFrameError
    from .sensor_transform import SensorTransform, SensorTransformError
except ImportError:
    from corridor_frame import CorridorFrame, CorridorFrameError
    from sensor_transform import SensorTransform, SensorTransformError


PROJECT_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CONFIG_PATH = PROJECT_ROOT / "config.json"
DEFAULT_TOPIC = "/livox/lidar"
DEFAULT_OUTPUT_DIR = PROJECT_ROOT / "tracking"


class TrackingError(RuntimeError):
    """入力・設定・ROS環境に関する想定内のエラー。"""


@dataclasses.dataclass
class ROI:
    """点群座標系での軸に沿った処理範囲。"""

    x_min: float = -20.0
    x_max: float = 20.0
    y_min: float = -10.0
    y_max: float = 10.0
    z_min: float = -3.0
    z_max: float = 5.0


@dataclasses.dataclass
class TrackerSettings:
    """人物検出・追跡・出力座標系の設定。"""

    topic: str = DEFAULT_TOPIC
    voxel_size: float = 0.08
    roi: ROI = dataclasses.field(default_factory=ROI)
    ground_mode: str = "fixed"
    ground_z: float = 0.0
    ground_clearance: float = 0.12
    max_height_above_ground: float = 2.4
    gravity_alignment_enabled: bool = False
    gravity_up_vector: tuple[float, float, float] = (0.0, 0.0, 1.0)
    ground_height_along_up: float = 0.0
    gravity_alignment_rotation: np.ndarray = dataclasses.field(
        default_factory=lambda: np.eye(3, dtype=np.float64)
    )
    gravity_alignment_translation: np.ndarray = dataclasses.field(
        default_factory=lambda: np.zeros(3, dtype=np.float64)
    )
    background_mode: str = "temporal"
    background_learning_frames: int = 30
    background_min_presence: float = 0.85
    background_voxel_size: float = 0.18
    cluster_tolerance: float = 0.30
    min_cluster_points: int = 8
    max_cluster_points: int = 2500
    min_cluster_height: float = 0.15
    min_cluster_width: float = 0.10
    min_cluster_depth: float = 0.10
    max_cluster_base_height: float = 0.45
    max_cluster_height: float = 2.5
    max_cluster_width: float = 2.0
    max_cluster_depth: float = 2.0
    far_detection_start: float = 8.0
    far_min_cluster_points: int = 5
    far_min_cluster_width: float = 0.05
    far_min_cluster_depth: float = 0.05
    far_cluster_tolerance_growth_per_meter: float = 0.012
    max_cluster_tolerance: float = 0.80
    cluster_background_voxel_size: float = 0.90
    cluster_background_min_presence: float = 0.50
    max_match_distance: float = 0.55
    max_track_speed: float = 3.0
    max_missed_frames: int = 12
    min_hits: int = 2
    min_confirmed_consecutive_hits: int = 5
    min_confirmed_point_count: float = 20.0
    far_min_confirmed_point_count: float = 8.0
    min_track_displacement: float = 0.80
    min_motion_consistency: float = 0.55
    motion_consistency_window: int = 15
    velocity_smoothing: float = 0.65
    markers_topic: str = "/people/tracks"
    publish_markers: bool = False
    # 内部の追跡座標（必要なら水平化済み）から、廊下・ZED座標もCSVへ併記する。
    output_coordinate_frame: str = "livox"
    corridor_frame_file: Optional[Path] = None
    corridor_frame: Optional[CorridorFrame] = None
    lidar_to_zed_frame_file: Optional[Path] = None
    lidar_to_zed_frame: Optional[SensorTransform] = None


def _confirmation_point_threshold(distance_m: float, settings: TrackerSettings) -> float:
    """遠距離では点密度が落ちるため、確定に必要な平均点数を下げる。"""

    if distance_m < settings.far_detection_start:
        return settings.min_confirmed_point_count
    range_fraction = min(1.0, settings.far_detection_start / max(distance_m, 1e-6))
    return max(
        float(settings.far_min_cluster_points),
        settings.far_min_cluster_points
        + (settings.far_min_confirmed_point_count - settings.far_min_cluster_points)
        * range_fraction,
    )


def _number(
    value: object,
    name: str,
    *,
    positive: bool = False,
    nonnegative: bool = False,
) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise TrackingError(f"Config value '{name}' must be a number.")
    result = float(value)
    if not math.isfinite(result):
        raise TrackingError(f"Config value '{name}' must be finite.")
    if positive and result <= 0:
        qualifier = "positive"
        raise TrackingError(f"Config value '{name}' must be {qualifier}.")
    if nonnegative and result < 0:
        qualifier = "finite and non-negative"
        raise TrackingError(f"Config value '{name}' must be {qualifier}.")
    return result


def _integer(value: object, name: str, *, positive: bool = False) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise TrackingError(f"Config value '{name}' must be an integer.")
    if (value <= 0 if positive else value < 0):
        qualifier = "positive" if positive else "non-negative"
        raise TrackingError(f"Config value '{name}' must be {qualifier}.")
    return value


def _vector3(
    value: object, name: str, default: tuple[float, float, float]
) -> tuple[float, float, float]:
    if value is None:
        value = default
    if not isinstance(value, (list, tuple)) or len(value) != 3:
        raise TrackingError(f"Config value '{name}' must contain three numbers.")
    return tuple(_number(item, f"{name}[{index}]") for index, item in enumerate(value))


def _rotation_from_up_vector(up_vector: Sequence[float]) -> np.ndarray:
    """重力上方向をlevel-frameの+Zへ回し、センサー前方向を水平面へ投影する。"""

    up = np.asarray(up_vector, dtype=np.float64).reshape(-1)
    norm = float(np.linalg.norm(up))
    if up.size != 3 or not math.isfinite(norm) or norm < 1e-6:
        raise TrackingError("tracking.gravity_alignment.up_vector must be non-zero.")
    up /= norm
    forward = np.array([1.0, 0.0, 0.0], dtype=np.float64)
    level_forward = forward - float(np.dot(forward, up)) * up
    forward_norm = float(np.linalg.norm(level_forward))
    if forward_norm < 1e-6:
        raise TrackingError("Gravity up vector cannot be parallel to the sensor forward axis.")
    level_forward /= forward_norm
    level_left = np.cross(up, level_forward)
    level_left /= np.linalg.norm(level_left)
    return np.vstack((level_forward, level_left, up))


def _text(value: object, name: str, default: str) -> str:
    if value is None:
        value = default
    if not isinstance(value, str) or not value.strip():
        raise TrackingError(f"Config value '{name}' must be a non-empty string.")
    return value.strip()


def _boolean(value: object, name: str, default: bool) -> bool:
    if value is None:
        value = default
    if not isinstance(value, bool):
        raise TrackingError(f"Config value '{name}' must be true or false.")
    return value


def load_settings(config_path: Path, topic_override: Optional[str] = None) -> TrackerSettings:
    """config.jsonのtracking設定を読み込み、値を検証する。"""

    settings = TrackerSettings()
    try:
        with config_path.expanduser().resolve().open("r", encoding="utf-8") as stream:
            raw = json.load(stream)
    except FileNotFoundError:
        raw = {}
    except (OSError, json.JSONDecodeError) as exc:
        raise TrackingError(f"Could not read config file {config_path}: {exc}") from exc

    if not isinstance(raw, dict):
        raise TrackingError("Config file must contain a JSON object.")
    tracking = raw.get("tracking", {})
    if not isinstance(tracking, dict):
        raise TrackingError("Config value 'tracking' must be a JSON object.")

    settings.topic = _text(
        topic_override if topic_override is not None else tracking.get("point_cloud_topic"),
        "tracking.point_cloud_topic",
        settings.topic,
    )
    # 廊下座標を使う場合は、事前に壁から作った校正JSONを読み込む。
    settings.output_coordinate_frame = _text(
        tracking.get("output_coordinate_frame"),
        "tracking.output_coordinate_frame",
        settings.output_coordinate_frame,
    ).lower()
    if settings.output_coordinate_frame not in {"livox", "corridor"}:
        raise TrackingError("tracking.output_coordinate_frame must be livox or corridor.")
    frame_value = _text(
        tracking.get("corridor_frame_file"),
        "tracking.corridor_frame_file",
        "tracking/corridor_frame.json",
    )
    frame_path = Path(frame_value).expanduser()
    if not frame_path.is_absolute():
        frame_path = config_path.expanduser().resolve().parent / frame_path
    frame_path = frame_path.resolve()
    settings.corridor_frame_file = frame_path
    if frame_path.is_file():
        try:
            # 校正ファイルがあれば、livox_frameから廊下座標への変換を保持する。
            settings.corridor_frame = CorridorFrame.from_json(frame_path)
        except CorridorFrameError as exc:
            raise TrackingError(f"Could not load tracking.corridor_frame_file: {exc}") from exc
    elif settings.output_coordinate_frame == "corridor":
        raise TrackingError(
            f"Corridor coordinate output requires calibration file: {frame_path}. "
            "Run scripts/estimate_corridor_frame.py first."
        )
    sensor_frame_value = _text(
        tracking.get("lidar_to_zed_frame_file"),
        "tracking.lidar_to_zed_frame_file",
        "tracking/lidar_to_zed_left_eye.json",
    )
    sensor_frame_path = Path(sensor_frame_value).expanduser()
    if not sensor_frame_path.is_absolute():
        sensor_frame_path = config_path.expanduser().resolve().parent / sensor_frame_path
    sensor_frame_path = sensor_frame_path.resolve()
    settings.lidar_to_zed_frame_file = sensor_frame_path
    if sensor_frame_path.is_file():
        try:
            # 初期取り付け寸法または廊下校正後のLiDAR→ZED外部変換を読み込む。
            settings.lidar_to_zed_frame = SensorTransform.from_json(sensor_frame_path)
        except SensorTransformError as exc:
            raise TrackingError(f"Could not load tracking.lidar_to_zed_frame_file: {exc}") from exc
    settings.voxel_size = _number(
        tracking.get("voxel_size", settings.voxel_size),
        "tracking.voxel_size",
        positive=True,
    )

    roi_raw = tracking.get("roi", {})
    if not isinstance(roi_raw, dict):
        raise TrackingError("Config value 'tracking.roi' must be a JSON object.")
    for field in dataclasses.fields(ROI):
        default = getattr(settings.roi, field.name)
        setattr(
            settings.roi,
            field.name,
            _number(roi_raw.get(field.name, default), f"tracking.roi.{field.name}"),
        )
    if (
        settings.roi.x_min >= settings.roi.x_max
        or settings.roi.y_min >= settings.roi.y_max
        or settings.roi.z_min >= settings.roi.z_max
    ):
        raise TrackingError("Each tracking ROI min value must be smaller than its max value.")

    settings.ground_mode = _text(
        tracking.get("ground_mode"), "tracking.ground_mode", settings.ground_mode
    ).lower()
    if settings.ground_mode not in {"fixed", "auto", "none"}:
        raise TrackingError("tracking.ground_mode must be fixed, auto, or none.")
    settings.ground_z = _number(
        tracking.get("ground_z", settings.ground_z), "tracking.ground_z"
    )
    settings.ground_clearance = _number(
        tracking.get("ground_clearance", settings.ground_clearance),
        "tracking.ground_clearance",
    )
    settings.max_height_above_ground = _number(
        tracking.get("max_height_above_ground", settings.max_height_above_ground),
        "tracking.max_height_above_ground",
        positive=True,
    )
    gravity_alignment = tracking.get("gravity_alignment", {})
    if not isinstance(gravity_alignment, dict):
        raise TrackingError("Config value 'tracking.gravity_alignment' must be a JSON object.")
    settings.gravity_alignment_enabled = _boolean(
        gravity_alignment.get("enabled"),
        "tracking.gravity_alignment.enabled",
        settings.gravity_alignment_enabled,
    )
    settings.gravity_up_vector = _vector3(
        gravity_alignment.get("up_vector"),
        "tracking.gravity_alignment.up_vector",
        settings.gravity_up_vector,
    )
    settings.ground_height_along_up = _number(
        gravity_alignment.get("ground_height_along_up_m", settings.ground_height_along_up),
        "tracking.gravity_alignment.ground_height_along_up_m",
    )
    if settings.gravity_alignment_enabled:
        settings.gravity_alignment_rotation = _rotation_from_up_vector(
            settings.gravity_up_vector
        )
        settings.gravity_alignment_translation = np.array(
            [0.0, 0.0, -settings.ground_height_along_up], dtype=np.float64
        )
    if settings.gravity_alignment_enabled and settings.lidar_to_zed_frame is not None:
        # 点群はlevel frameへ変換済みなので、LiDAR→ZED外部変換も同じ入力座標へ合成する。
        raw_to_level = settings.gravity_alignment_rotation
        level_to_raw = raw_to_level.T
        raw_to_zed = settings.lidar_to_zed_frame
        composed_rotation = raw_to_zed.rotation @ level_to_raw
        composed_translation = (
            raw_to_zed.translation
            - composed_rotation @ settings.gravity_alignment_translation
        )
        settings.lidar_to_zed_frame = SensorTransform(
            rotation=composed_rotation,
            translation=composed_translation,
            source_frame="livox_level_frame",
            target_frame=raw_to_zed.target_frame,
        )
    settings.background_mode = _text(
        tracking.get("background_mode"),
        "tracking.background_mode",
        settings.background_mode,
    ).lower()
    if settings.background_mode not in {"none", "temporal"}:
        raise TrackingError("tracking.background_mode must be none or temporal.")
    settings.background_learning_frames = _integer(
        tracking.get("background_learning_frames", settings.background_learning_frames),
        "tracking.background_learning_frames",
        positive=True,
    )
    settings.background_min_presence = _number(
        tracking.get("background_min_presence", settings.background_min_presence),
        "tracking.background_min_presence",
    )
    if not 0 < settings.background_min_presence <= 1:
        raise TrackingError("tracking.background_min_presence must be in (0, 1].")
    settings.background_voxel_size = _number(
        tracking.get("background_voxel_size", settings.background_voxel_size),
        "tracking.background_voxel_size",
        positive=True,
    )

    settings.cluster_tolerance = _number(
        tracking.get("cluster_tolerance", settings.cluster_tolerance),
        "tracking.cluster_tolerance",
        positive=True,
    )
    settings.min_cluster_points = _integer(
        tracking.get("min_cluster_points", settings.min_cluster_points),
        "tracking.min_cluster_points",
        positive=True,
    )
    settings.max_cluster_points = _integer(
        tracking.get("max_cluster_points", settings.max_cluster_points),
        "tracking.max_cluster_points",
        positive=True,
    )
    settings.min_cluster_height = _number(
        tracking.get("min_cluster_height", settings.min_cluster_height),
        "tracking.min_cluster_height",
    )
    settings.min_cluster_width = _number(
        tracking.get("min_cluster_width", settings.min_cluster_width),
        "tracking.min_cluster_width",
        nonnegative=True,
    )
    settings.min_cluster_depth = _number(
        tracking.get("min_cluster_depth", settings.min_cluster_depth),
        "tracking.min_cluster_depth",
        nonnegative=True,
    )
    settings.max_cluster_base_height = _number(
        tracking.get("max_cluster_base_height", settings.max_cluster_base_height),
        "tracking.max_cluster_base_height",
        nonnegative=True,
    )
    settings.max_cluster_height = _number(
        tracking.get("max_cluster_height", settings.max_cluster_height),
        "tracking.max_cluster_height",
        positive=True,
    )
    settings.max_cluster_width = _number(
        tracking.get("max_cluster_width", settings.max_cluster_width),
        "tracking.max_cluster_width",
        positive=True,
    )
    settings.max_cluster_depth = _number(
        tracking.get("max_cluster_depth", settings.max_cluster_depth),
        "tracking.max_cluster_depth",
        positive=True,
    )
    settings.far_detection_start = _number(
        tracking.get("far_detection_start", settings.far_detection_start),
        "tracking.far_detection_start",
        positive=True,
    )
    settings.far_min_cluster_points = _integer(
        tracking.get("far_min_cluster_points", settings.far_min_cluster_points),
        "tracking.far_min_cluster_points",
        positive=True,
    )
    settings.far_min_cluster_width = _number(
        tracking.get("far_min_cluster_width", settings.far_min_cluster_width),
        "tracking.far_min_cluster_width",
        nonnegative=True,
    )
    settings.far_min_cluster_depth = _number(
        tracking.get("far_min_cluster_depth", settings.far_min_cluster_depth),
        "tracking.far_min_cluster_depth",
        nonnegative=True,
    )
    settings.far_cluster_tolerance_growth_per_meter = _number(
        tracking.get(
            "far_cluster_tolerance_growth_per_meter",
            settings.far_cluster_tolerance_growth_per_meter,
        ),
        "tracking.far_cluster_tolerance_growth_per_meter",
        nonnegative=True,
    )
    settings.max_cluster_tolerance = _number(
        tracking.get("max_cluster_tolerance", settings.max_cluster_tolerance),
        "tracking.max_cluster_tolerance",
        positive=True,
    )
    if settings.max_cluster_tolerance < settings.cluster_tolerance:
        raise TrackingError(
            "tracking.max_cluster_tolerance must be at least cluster_tolerance."
        )
    if settings.far_min_cluster_points > settings.min_cluster_points:
        raise TrackingError(
            "tracking.far_min_cluster_points must not exceed min_cluster_points."
        )
    settings.cluster_background_voxel_size = _number(
        tracking.get(
            "cluster_background_voxel_size", settings.cluster_background_voxel_size
        ),
        "tracking.cluster_background_voxel_size",
        positive=True,
    )
    settings.cluster_background_min_presence = _number(
        tracking.get(
            "cluster_background_min_presence", settings.cluster_background_min_presence
        ),
        "tracking.cluster_background_min_presence",
    )
    if not 0 < settings.cluster_background_min_presence <= 1:
        raise TrackingError(
            "tracking.cluster_background_min_presence must be in (0, 1]."
        )
    if settings.min_cluster_points > settings.max_cluster_points:
        raise TrackingError("tracking.min_cluster_points must not exceed max_cluster_points.")

    settings.max_match_distance = _number(
        tracking.get("max_match_distance", settings.max_match_distance),
        "tracking.max_match_distance",
        positive=True,
    )
    settings.max_track_speed = _number(
        tracking.get("max_track_speed", settings.max_track_speed),
        "tracking.max_track_speed",
        positive=True,
    )
    settings.max_missed_frames = _integer(
        tracking.get("max_missed_frames", settings.max_missed_frames),
        "tracking.max_missed_frames",
    )
    settings.min_hits = _integer(
        tracking.get("min_hits", settings.min_hits), "tracking.min_hits", positive=True
    )
    settings.min_confirmed_consecutive_hits = _integer(
        tracking.get(
            "min_confirmed_consecutive_hits", settings.min_confirmed_consecutive_hits
        ),
        "tracking.min_confirmed_consecutive_hits",
        positive=True,
    )
    settings.min_confirmed_point_count = _number(
        tracking.get("min_confirmed_point_count", settings.min_confirmed_point_count),
        "tracking.min_confirmed_point_count",
        positive=True,
    )
    settings.far_min_confirmed_point_count = _number(
        tracking.get(
            "far_min_confirmed_point_count", settings.far_min_confirmed_point_count
        ),
        "tracking.far_min_confirmed_point_count",
        positive=True,
    )
    if settings.far_min_confirmed_point_count < settings.far_min_cluster_points:
        raise TrackingError(
            "tracking.far_min_confirmed_point_count must be at least far_min_cluster_points."
        )
    settings.min_track_displacement = _number(
        tracking.get("min_track_displacement", settings.min_track_displacement),
        "tracking.min_track_displacement",
        nonnegative=True,
    )
    settings.min_motion_consistency = _number(
        tracking.get("min_motion_consistency", settings.min_motion_consistency),
        "tracking.min_motion_consistency",
        nonnegative=True,
    )
    if settings.min_motion_consistency > 1:
        raise TrackingError("tracking.min_motion_consistency must be in [0, 1].")
    settings.motion_consistency_window = _integer(
        tracking.get("motion_consistency_window", settings.motion_consistency_window),
        "tracking.motion_consistency_window",
        positive=True,
    )
    settings.velocity_smoothing = _number(
        tracking.get("velocity_smoothing", settings.velocity_smoothing),
        "tracking.velocity_smoothing",
    )
    if settings.velocity_smoothing > 1:
        raise TrackingError("tracking.velocity_smoothing must be in [0, 1].")
    settings.markers_topic = _text(
        tracking.get("markers_topic"), "tracking.markers_topic", settings.markers_topic
    )
    settings.publish_markers = _boolean(
        tracking.get("publish_markers"),
        "tracking.publish_markers",
        settings.publish_markers,
    )
    return settings


@dataclasses.dataclass
class Detection:
    """1フレームから抽出した人物候補クラスタ。"""

    position: np.ndarray
    extent: np.ndarray
    point_count: int
    confidence: float


@dataclasses.dataclass
class TrackSnapshot:
    """1フレーム分のトラック状態を保存・表示しやすくした値。"""

    track_id: int
    position: np.ndarray
    velocity: np.ndarray
    point_count: int
    extent: np.ndarray
    confidence: float
    hits: int
    missed_frames: int
    detected: bool
    confirmed: bool

    @property
    def status(self) -> str:
        if not self.confirmed:
            return "tentative"
        return "predicted" if not self.detected else "confirmed"


@dataclasses.dataclass
class _Track:
    track_id: int
    position: np.ndarray
    velocity: np.ndarray
    last_timestamp_ns: int
    hits: int = 1
    missed_frames: int = 0
    point_count: int = 0
    extent: np.ndarray = dataclasses.field(default_factory=lambda: np.zeros(3))
    confidence: float = 0.0
    detected: bool = True
    initial_position: np.ndarray = dataclasses.field(default_factory=lambda: np.zeros(3))
    max_displacement: float = 0.0
    motion_observations: int = 0
    motion_streak: int = 0
    recent_positions: list[np.ndarray] = dataclasses.field(default_factory=list)
    confirmed: bool = False
    point_count_total: int = 0
    detected_observations: int = 1
    consecutive_hits: int = 1


class BackgroundModel:
    """複数フレームに残り続ける壁や家具を除去する占有モデル。"""

    def __init__(self, settings: TrackerSettings) -> None:
        self.enabled = settings.background_mode == "temporal"
        self.learning_frames = settings.background_learning_frames
        self.min_presence = settings.background_min_presence
        self.voxel_size = settings.background_voxel_size
        self.frame_count = 0
        self.counts: dict[tuple[int, int, int], int] = {}

    @property
    def ready(self) -> bool:
        """背景を除去できるだけの学習フレームを受け取ったか判定する。"""

        return not self.enabled or self.frame_count > self.learning_frames

    def _keys(self, points: np.ndarray) -> list[tuple[int, int, int]]:
        quantized = np.floor(points / self.voxel_size).astype(np.int64)
        return [tuple(row) for row in quantized]

    def filter(self, points: np.ndarray) -> np.ndarray:
        """占有回数を更新し、静的に見えるボクセルを点群から除外する。"""

        if not self.enabled or len(points) == 0:
            self.frame_count += 1
            return points

        keys = self._keys(points)
        unique_keys = set(keys)
        for key in unique_keys:
            self.counts[key] = self.counts.get(key, 0) + 1
        previous_frames = self.frame_count
        self.frame_count += 1
        if previous_frames < self.learning_frames:
            return points

        threshold = self.min_presence * max(1, previous_frames)
        dynamic_mask = np.fromiter(
            (self.counts[key] < threshold for key in keys), dtype=bool, count=len(keys)
        )
        return points[dynamic_mask]


class PersistentClusterModel:
    """学習期間中に同じ場所へ残る候補クラスタを除去する。

    点単位の背景除去は疎なLiDARノイズの影響を受けやすく、
    壁が数ボクセル動くだけで候補クラスタとして残ることがある。
    そこで候補中心を粗いセルで追跡し、同じ場所に繰り返し現れる壁や設備を
    人物トラックの起点にしない。
    """

    def __init__(self, settings: TrackerSettings) -> None:
        self.enabled = settings.background_mode == "temporal"
        self.learning_frames = settings.background_learning_frames
        self.min_presence = settings.cluster_background_min_presence
        self.match_distance = settings.cluster_background_voxel_size
        self.frame_count = 0
        self.entries: list[tuple[np.ndarray, int, int]] = []
        self.prototypes: list[np.ndarray] = []

    def _observe(self, detections: Sequence[Detection]) -> None:
        """学習期間中の候補中心を対応付け、静的候補の履歴を作る。"""

        used_entries: set[int] = set()
        for detection in detections:
            if not self.entries:
                self.entries.append((detection.position.copy(), 1, self.frame_count))
                continue
            distances = np.array(
                [
                    np.linalg.norm(detection.position - entry[0])
                    if index not in used_entries
                    else math.inf
                    for index, entry in enumerate(self.entries)
                ]
            )
            index = int(np.argmin(distances))
            if distances[index] > self.match_distance:
                self.entries.append((detection.position.copy(), 1, self.frame_count))
                continue
            position, observations, last_frame = self.entries[index]
            if last_frame == self.frame_count:
                continue
            # ゆっくり更新して計測ノイズには追従させるが、
            # 移動人物によって静的候補の中心が大きく動かないようにする。
            updated_position = 0.8 * position + 0.2 * detection.position
            self.entries[index] = (updated_position, observations + 1, self.frame_count)
            used_entries.add(index)

    def _freeze(self) -> None:
        threshold = self.min_presence * self.learning_frames
        self.prototypes = [
            position
            for position, observations, _ in self.entries
            if observations >= threshold
        ]

    def update(self, detections: Sequence[Detection]) -> list[Detection]:
        """候補の滞在位置を更新し、静的でない候補だけを返す。"""

        # 学習期間中は静的候補のプロトタイプを作るため、検出結果を返さない。
        if not self.enabled:
            self.frame_count += 1
            return list(detections)

        if self.frame_count < self.learning_frames:
            self._observe(detections)
            self.frame_count += 1
            return []
        if self.frame_count == self.learning_frames:
            self._freeze()
        self.frame_count += 1
        if not self.prototypes:
            return list(detections)
        return [
            detection
            for detection in detections
            if min(
                np.linalg.norm(detection.position - prototype)
                for prototype in self.prototypes
            )
            > self.match_distance
        ]


def _downsample(points: np.ndarray, voxel_size: float) -> np.ndarray:
    """同じボクセル内の点を1点に間引き、クラスタ処理を軽量化する。"""

    if len(points) == 0:
        return points.reshape(0, 3).astype(np.float32, copy=False)
    keys = np.floor(points / voxel_size).astype(np.int64)
    _, first_indices = np.unique(keys, axis=0, return_index=True)
    return points[np.sort(first_indices)]


class _UnionFind:
    """連結成分の統合を効率よく管理する素集合データ構造。"""

    def __init__(self, size: int) -> None:
        self.parent = np.arange(size, dtype=np.int64)
        self.rank = np.zeros(size, dtype=np.int8)

    def find(self, value: int) -> int:
        root = value
        while self.parent[root] != root:
            root = int(self.parent[root])
        while self.parent[value] != value:
            next_value = int(self.parent[value])
            self.parent[value] = root
            value = next_value
        return root

    def union(self, left: int, right: int) -> None:
        left_root = self.find(left)
        right_root = self.find(right)
        if left_root == right_root:
            return
        if self.rank[left_root] < self.rank[right_root]:
            left_root, right_root = right_root, left_root
        self.parent[right_root] = left_root
        if self.rank[left_root] == self.rank[right_root]:
            self.rank[left_root] += 1


def _cluster_labels(
    points: np.ndarray,
    tolerance: float,
    *,
    range_start: float = math.inf,
    tolerance_growth_per_meter: float = 0.0,
    max_tolerance: Optional[float] = None,
) -> np.ndarray:
    """距離に応じた近傍条件で点を連結成分へまとめる。

    遠距離では同じ人体表面の点間隔が広がるため、設定した上限内で
    水平距離に応じてクラスタ接続距離を少しずつ広げる。
    """

    if len(points) == 0:
        return np.empty(0, dtype=np.int64)
    union_find = _UnionFind(len(points))
    point_ranges = np.linalg.norm(points, axis=1)
    maximum_pair_tolerance = max(tolerance, max_tolerance or tolerance)

    def connect(indices: np.ndarray, cell_size: float, adaptive: bool) -> None:
        cells: dict[tuple[int, int, int], list[int]] = {}
        cell_keys = np.floor(points[indices] / cell_size).astype(np.int64)
        for local_index, key_array in enumerate(cell_keys):
            key = (int(key_array[0]), int(key_array[1]), int(key_array[2]))
            cells.setdefault(key, []).append(int(indices[local_index]))
        for index in indices:
            base = tuple(np.floor(points[index] / cell_size).astype(np.int64))
            point = points[index]
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    for dz in (-1, 0, 1):
                        candidates = cells.get((base[0] + dx, base[1] + dy, base[2] + dz))
                        if not candidates:
                            continue
                        candidate_indices = np.asarray(candidates, dtype=np.int64)
                        distances_squared = np.sum((points[candidate_indices] - point) ** 2, axis=1)
                        if adaptive:
                            pair_ranges = 0.5 * (point_ranges[index] + point_ranges[candidate_indices])
                            local_tolerances = np.minimum(
                                maximum_pair_tolerance,
                                tolerance
                                + tolerance_growth_per_meter
                                * np.maximum(0.0, pair_ranges - range_start),
                            )
                        else:
                            local_tolerances = tolerance
                        nearby = (
                            (candidate_indices > index)
                            & (distances_squared <= local_tolerances * local_tolerances)
                        )
                        for candidate in candidate_indices[nearby]:
                            union_find.union(int(index), int(candidate))

    all_indices = np.arange(len(points), dtype=np.int64)
    connect(all_indices, tolerance, adaptive=False)
    if maximum_pair_tolerance > tolerance and tolerance_growth_per_meter > 0:
        # The expanded grid only contains the far field and its boundary band;
        # close-range cells keep their finer grid and processing cost.
        far_indices = all_indices[point_ranges >= range_start - maximum_pair_tolerance]
        if len(far_indices):
            connect(far_indices, maximum_pair_tolerance, adaptive=True)

    return np.fromiter(
        (union_find.find(index) for index in range(len(points))),
        dtype=np.int64,
        count=len(points),
    )


def detect_clusters(points: np.ndarray, settings: TrackerSettings) -> list[Detection]:
    """大きさ・高さ・形状が人物らしいクラスタだけを検出結果として返す。"""

    if len(points) == 0:
        return []
    # 近接点を連結成分に分け、各成分を人物候補として形状判定する。
    labels = _cluster_labels(
        points,
        settings.cluster_tolerance,
        range_start=settings.far_detection_start,
        tolerance_growth_per_meter=settings.far_cluster_tolerance_growth_per_meter,
        max_tolerance=settings.max_cluster_tolerance,
    )
    unique_labels, counts = np.unique(labels, return_counts=True)
    detections: list[Detection] = []
    for label, count in zip(unique_labels, counts):
        count = int(count)
        cluster = points[labels == label]
        position = np.median(cluster, axis=0).astype(np.float64)
        horizontal_range = float(np.linalg.norm(position[:2]))
        # 遠距離では点密度が下がるため、検出条件を少し緩和する。
        far = horizontal_range >= settings.far_detection_start
        min_points = (
            settings.far_min_cluster_points if far else settings.min_cluster_points
        )
        min_width = settings.far_min_cluster_width if far else settings.min_cluster_width
        min_depth = settings.far_min_cluster_depth if far else settings.min_cluster_depth
        if count < min_points or count > settings.max_cluster_points:
            continue
        # クラスタの大きさをX/Y/Z方向ごとに調べ、人物らしくない形を除く。
        extent = np.ptp(cluster, axis=0)
        if extent[2] < settings.min_cluster_height:
            continue
        if settings.ground_mode == "fixed":
            if float(np.min(cluster[:, 2])) > settings.ground_z + settings.max_cluster_base_height:
                continue
        if extent[0] < min_width or extent[1] < min_depth:
            # 斜めから見た人物は疎な点群では細長く見えることがある。
            # 高く密なクラスタは許可し、細いポールや壁の縁のような小クラスタは除外する。
            if count < min_points * 2 or extent[2] < 0.5:
                continue
        if extent[2] > settings.max_cluster_height:
            continue
        if extent[0] > settings.max_cluster_width or extent[1] > settings.max_cluster_depth:
            continue

        # 平均ではなく中央値を使い、壁の一部や伸ばした腕の影響を抑える。
        # Zの中央値は人物マーカーの高さとして使う。
        confidence = min(1.0, count / max(1.0, min_points * 4.0))
        detections.append(Detection(position, extent, count, confidence))
    detections.sort(key=lambda detection: (float(detection.position[0]), float(detection.position[1])))
    return detections


def _assignment(
    tracks: Sequence[_Track], detections: Sequence[Detection], gate: float
) -> list[tuple[int, int]]:
    """距離ゲート内で、トラックと検出結果を1対1に対応付ける。

    通常の少人数では動的計画法で最小距離の組み合わせを求め、
    大人数の場合は決定的な貪欲法へ切り替える。
    """

    if not tracks or not detections:
        return []
    # 対応付けの距離ゲートには直前に観測した位置を使う。
    # 一時的なノイズで速度が大きくなっても、ID対応付けが飛ばないようにする。
    # 速度は見失いフレームの予測には使い、対応付け後に更新する。
    predictions = np.array([track.position[:2] for track in tracks])
    measurements = np.array([detection.position[:2] for detection in detections])
    distances = np.linalg.norm(predictions[:, None, :] - measurements[None, :, :], axis=2)
    allowed = distances <= gate
    track_count = len(tracks)
    detection_count = len(detections)

    if detection_count > 20 or track_count > 20:
        candidates = sorted(
            (float(distances[i, j]), i, j)
            for i in range(track_count)
            for j in range(detection_count)
            if allowed[i, j]
        )
        used_tracks: set[int] = set()
        used_detections: set[int] = set()
        matches: list[tuple[int, int]] = []
        for _, track_index, detection_index in candidates:
            if track_index not in used_tracks and detection_index not in used_detections:
                used_tracks.add(track_index)
                used_detections.add(detection_index)
                matches.append((track_index, detection_index))
        return matches

    unmatched_cost = gate * gate * 1.000001
    costs = distances * distances
    memo: dict[tuple[int, int], tuple[float, tuple[Optional[int], ...]]] = {}

    def solve(track_index: int, used_mask: int) -> tuple[float, tuple[Optional[int], ...]]:
        key = (track_index, used_mask)
        if key in memo:
            return memo[key]
        if track_index == track_count:
            return 0.0, ()
        best_cost, best_tail = solve(track_index + 1, used_mask)
        best_cost += unmatched_cost
        best_assignment: tuple[Optional[int], ...] = (None,) + best_tail
        for detection_index in range(detection_count):
            if not allowed[track_index, detection_index] or used_mask & (1 << detection_index):
                continue
            tail_cost, tail_assignment = solve(
                track_index + 1, used_mask | (1 << detection_index)
            )
            candidate_cost = float(costs[track_index, detection_index]) + tail_cost
            if candidate_cost < best_cost:
                best_cost = candidate_cost
                best_assignment = (detection_index,) + tail_assignment
        memo[key] = (best_cost, best_assignment)
        return memo[key]

    _, choices = solve(0, 0)
    return [
        (track_index, detection_index)
        for track_index, detection_index in enumerate(choices)
        if detection_index is not None
    ]


class PeopleTracker:
    """点群検出と等速モデルによるマルチオブジェクト追跡を管理する。"""

    def __init__(self, settings: TrackerSettings) -> None:
        self.settings = settings
        self.background = BackgroundModel(settings)
        self.cluster_background = PersistentClusterModel(settings)
        self.tracks: list[_Track] = []
        self.next_id = 1
        self.frame_index = 0
        self.first_timestamp_ns: Optional[int] = None

    def preprocess(self, points: np.ndarray) -> np.ndarray:
        """ROI、床面、ボクセル、時間背景の順に点群を前処理する。"""

        # ①入力をN x 3へそろえ、NaN/InfとROI外の点を除く。
        if points.size == 0:
            return np.empty((0, 3), dtype=np.float32)
        points = np.asarray(points, dtype=np.float32).reshape(-1, 3)
        if self.settings.gravity_alignment_enabled:
            # 固定傾斜したセンサー座標を重力に沿った水平座標へ変換してから高さ判定する。
            points = (
                points @ self.settings.gravity_alignment_rotation.T
                + self.settings.gravity_alignment_translation
            ).astype(np.float32, copy=False)
        finite = np.isfinite(points).all(axis=1)
        roi = self.settings.roi
        inside = (
            (points[:, 0] >= roi.x_min)
            & (points[:, 0] <= roi.x_max)
            & (points[:, 1] >= roi.y_min)
            & (points[:, 1] <= roi.y_max)
            & (points[:, 2] >= roi.z_min)
            & (points[:, 2] <= roi.z_max)
        )
        points = points[finite & inside]
        if len(points) == 0:
            self.background.frame_count += 1
            return points

        # ②床面の高さを固定値、点群からの自動推定、無効のいずれかで決める。
        if self.settings.ground_mode == "fixed":
            ground_z = self.settings.ground_z
        elif self.settings.ground_mode == "auto":
            # 低い高さ帯で最も点が多いビンを床面とみなす。
            # 1点だけの外れ値に引きずられにくい方法である。
            low = float(np.percentile(points[:, 2], 5))
            high = float(np.percentile(points[:, 2], 35))
            if high <= low:
                ground_z = low
            else:
                bin_size = max(0.03, self.settings.voxel_size)
                edges = np.arange(low, high + bin_size, bin_size)
                histogram, _ = np.histogram(points[:, 2], bins=edges)
                ground_z = float(edges[int(np.argmax(histogram))] + bin_size / 2.0)
        else:
            ground_z = -math.inf

        # 床面より低い点と、人物として高すぎる点を除去する。
        if math.isfinite(ground_z):
            points = points[
                (points[:, 2] >= ground_z + self.settings.ground_clearance)
                & (points[:, 2] <= ground_z + self.settings.max_height_above_ground)
            ]
        if len(points) == 0:
            self.background.frame_count += 1
            return points
        # ③空間ボクセルで間引き、同じ場所に残る静的点を時間背景で除去する。
        points = _downsample(points, self.settings.voxel_size)
        return self.background.filter(points)

    def update(self, points: np.ndarray, timestamp_ns: int) -> list[TrackSnapshot]:
        """前処理、検出、ID対応、状態更新の順に1フレームを処理する。"""

        processed = self.preprocess(points)
        # 点群の背景と候補中心の背景を順に除き、設備の誤検出を抑える。
        detections = self.cluster_background.update(
            detect_clusters(processed, self.settings)
        )
        active_before = list(self.tracks)
        matches = _assignment(active_before, detections, self.settings.max_match_distance)
        matched_tracks = {track_index for track_index, _ in matches}
        matched_detections = {detection_index for _, detection_index in matches}

        # 対応した人物は観測値で更新し、見失った人物は短時間だけ予測する。
        for track_index, detection_index in matches:
            self._update_matched_track(
                active_before[track_index], detections[detection_index], timestamp_ns
            )
        for index, track in enumerate(active_before):
            if index not in matched_tracks:
                self._predict_missing_track(track, timestamp_ns)

        # 新しい検出からIDを発行し、見失い期限を過ぎたIDを終了する。
        for detection_index, detection in enumerate(detections):
            if detection_index not in matched_detections:
                self._start_track(detection, timestamp_ns)
        self.tracks = [
            track
            for track in self.tracks
            if track.missed_frames <= self.settings.max_missed_frames
        ]
        self.frame_index += 1
        return self._snapshots()

    def _update_matched_track(
        self, track: _Track, detection: Detection, timestamp_ns: int
    ) -> None:
        """対応した候補で位置・速度を更新し、連続歩行なら確定状態へ進める。"""

        smoothing = self.settings.velocity_smoothing
        previous_missed_frames = track.missed_frames
        if previous_missed_frames == 0:
            track.consecutive_hits += 1
            track.recent_positions.append(detection.position.copy())
        else:
            track.consecutive_hits = 1
            track.recent_positions = [detection.position.copy()]
        max_recent_positions = max(
            self.settings.motion_consistency_window,
            self.settings.min_confirmed_consecutive_hits,
        )
        if len(track.recent_positions) > max_recent_positions:
            del track.recent_positions[:-max_recent_positions]
        step_distance = float(
            np.linalg.norm(detection.position[:2] - track.position[:2])
        )
        # 静的物は計測中心が少し揺れることがあるが、
        # 複数フレームにわたって同じ方向へ歩き続けることはない。
        # 連続して妥当な移動が見えた場合だけ人物の根拠として加算する。
        motion_step_limit = min(self.settings.max_match_distance, 0.6)
        if 0.05 <= step_distance <= motion_step_limit:
            track.motion_observations += 1
            track.motion_streak += 1
        else:
            track.motion_streak = 0
        delta_seconds = max(
            1e-3, (timestamp_ns - track.last_timestamp_ns) / 1_000_000_000.0
        )
        measured_velocity = (detection.position - track.position) / delta_seconds
        track.velocity = (
            (1.0 - smoothing) * track.velocity + smoothing * measured_velocity
        )
        horizontal_speed = float(np.linalg.norm(track.velocity[:2]))
        if horizontal_speed > self.settings.max_track_speed:
            track.velocity[:2] *= self.settings.max_track_speed / horizontal_speed
        track.velocity[2] = 0.0
        track.position = detection.position.copy()
        displacement = float(np.linalg.norm(track.position[:2] - track.initial_position[:2]))
        track.max_displacement = max(track.max_displacement, displacement)
        track.last_timestamp_ns = timestamp_ns
        track.hits += 1
        track.missed_frames = 0
        track.point_count = detection.point_count
        track.point_count_total += detection.point_count
        track.detected_observations += 1
        track.extent = detection.extent.copy()
        track.confidence = detection.confidence
        track.detected = True
        if not track.confirmed:
            recent = np.asarray(
                [position[:2] for position in track.recent_positions],
                dtype=np.float64,
            )
            if len(recent) >= 2:
                recent_steps = np.diff(recent, axis=0)
                recent_path = float(
                    np.linalg.norm(recent_steps, axis=1).sum()
                )
                recent_displacement = float(
                    np.linalg.norm(recent[-1] - recent[0])
                )
                motion_consistency = (
                    recent_displacement / recent_path
                    if recent_path > 1e-6
                    else 0.0
                )
            else:
                motion_consistency = 0.0
            enough_motion = (
                self.settings.min_track_displacement <= 0
                or (
                    len(track.recent_positions)
                    >= self.settings.motion_consistency_window
                    and track.max_displacement >= self.settings.min_track_displacement
                    and track.motion_observations >= 4
                    and motion_consistency >= self.settings.min_motion_consistency
                    and (
                        track.point_count_total / max(1, track.detected_observations)
                        >= _confirmation_point_threshold(
                            float(np.linalg.norm(track.position[:2])), self.settings
                        )
                    )
                )
            )
            track.confirmed = (
                track.hits >= self.settings.min_hits
                and track.consecutive_hits
                >= self.settings.min_confirmed_consecutive_hits
                and enough_motion
            )

    def _predict_missing_track(self, track: _Track, timestamp_ns: int) -> None:
        """遮蔽中のIDを等速で予測し、次フレームで再対応できるようにする。"""

        delta_seconds = max(
            0.0, (timestamp_ns - track.last_timestamp_ns) / 1_000_000_000.0
        )
        track.position = track.position + track.velocity * min(delta_seconds, 1.0)
        track.last_timestamp_ns = timestamp_ns
        track.missed_frames += 1
        track.detected = False
        track.confidence *= 0.92

    def _start_track(self, detection: Detection, timestamp_ns: int) -> None:
        """未対応の人物候補に新しいIDを割り当てる。"""

        self.tracks.append(
            _Track(
                track_id=self.next_id,
                position=detection.position.copy(),
                velocity=np.zeros(3, dtype=np.float64),
                last_timestamp_ns=timestamp_ns,
                point_count=detection.point_count,
                extent=detection.extent.copy(),
                confidence=detection.confidence,
                initial_position=detection.position.copy(),
                point_count_total=detection.point_count,
                recent_positions=[detection.position.copy()],
            )
        )
        self.next_id += 1

    def _snapshots(self) -> list[TrackSnapshot]:
        """外部へ返す値をコピーし、追跡内部の状態を変更されないようにする。"""

        snapshots: list[TrackSnapshot] = []
        for track in sorted(self.tracks, key=lambda item: item.track_id):
            if track.hits < self.settings.min_hits and track.missed_frames > 0:
                continue
            snapshots.append(
                TrackSnapshot(
                    track_id=track.track_id,
                    position=track.position.copy(),
                    velocity=track.velocity.copy(),
                    point_count=track.point_count,
                    extent=track.extent.copy(),
                    confidence=track.confidence,
                    hits=track.hits,
                    missed_frames=track.missed_frames,
                    detected=track.detected,
                    confirmed=track.confirmed,
                )
            )
        return snapshots


def _pointcloud2_to_numpy(message: object) -> np.ndarray:
    try:
        from sensor_msgs_py import point_cloud2
    except ImportError as exc:
        raise TrackingError(
            "sensor_msgs_py is unavailable; source the ROS 2 setup.bash first."
        ) from exc
    try:
        values = point_cloud2.read_points(
            message, field_names=("x", "y", "z"), skip_nans=True
        )
    except (AttributeError, RuntimeError, ValueError) as exc:
        raise TrackingError(f"PointCloud2 does not contain readable x/y/z fields: {exc}") from exc
    if isinstance(values, np.ndarray) and values.dtype.names:
        try:
            return np.column_stack((values["x"], values["y"], values["z"])).astype(
                np.float32, copy=False
            )
        except (ValueError, TypeError) as exc:
            raise TrackingError(f"Could not decode PointCloud2 x/y/z fields: {exc}") from exc
    return np.asarray(list(values), dtype=np.float32).reshape(-1, 3)


def _custom_msg_to_numpy(message: object) -> np.ndarray:
    points = getattr(message, "points", None)
    if points is None:
        raise TrackingError("Livox CustomMsg does not contain a points field.")
    return np.asarray(
        [(float(point.x), float(point.y), float(point.z)) for point in points],
        dtype=np.float32,
    ).reshape(-1, 3)


class CsvWriter:
    """トラック1件ごとにCSVを書き出し、利用可能な座標系を併記する。"""

    # x/y/zとvx/vyは従来どおりlivox_frame、level_*は重力水平化座標。
    # corridor_*は校正JSONが読み込めた場合だけ値を持つ。
    # zed_*はLiDAR→ZED外部変換JSONが読み込めた場合だけ値を持つ。
    FIELDNAMES = [
        "frame",
        "time_sec",
        "timestamp_ns",
        "track_id",
        "status",
        "detected",
        "hits",
        "missed_frames",
        "x",
        "y",
        "z",
        "vx",
        "vy",
        "speed_xy",
        "level_x",
        "level_y",
        "level_z",
        "level_vx",
        "level_vy",
        "level_vz",
        "level_speed_xy",
        "zed_x",
        "zed_y",
        "zed_z",
        "zed_vx",
        "zed_vy",
        "zed_vz",
        "corridor_x",
        "corridor_y",
        "corridor_z",
        "corridor_vx",
        "corridor_vy",
        "corridor_vz",
        "corridor_speed_xy",
        "point_count",
        "width",
        "depth",
        "height",
        "confidence",
    ]

    def __init__(
        self,
        path: Path,
        corridor_frame: Optional[CorridorFrame] = None,
        lidar_to_zed_frame: Optional[SensorTransform] = None,
        gravity_alignment_rotation: Optional[np.ndarray] = None,
        gravity_alignment_translation: Optional[np.ndarray] = None,
    ) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        self.corridor_frame = corridor_frame
        self.lidar_to_zed_frame = lidar_to_zed_frame
        self.gravity_alignment_rotation = (
            np.asarray(gravity_alignment_rotation, dtype=np.float64).reshape(3, 3)
            if gravity_alignment_rotation is not None
            else np.eye(3, dtype=np.float64)
        )
        self.gravity_alignment_translation = (
            np.asarray(gravity_alignment_translation, dtype=np.float64).reshape(3)
            if gravity_alignment_translation is not None
            else np.zeros(3, dtype=np.float64)
        )
        self.stream = path.open("w", encoding="utf-8", newline="")
        self.writer = csv.DictWriter(self.stream, fieldnames=self.FIELDNAMES)
        self.writer.writeheader()

    def write(self, frame: int, timestamp_ns: int, snapshots: Sequence[TrackSnapshot], start_ns: int) -> None:
        time_sec = (timestamp_ns - start_ns) / 1_000_000_000.0
        for snapshot in snapshots:
            level_position = snapshot.position
            level_velocity = snapshot.velocity
            # x/y/zはPointCloud2の元フレームへ戻し、既存のCSV/RViz用途を保つ。
            raw_position = (
                (level_position - self.gravity_alignment_translation)
                @ self.gravity_alignment_rotation
            )
            raw_velocity = level_velocity @ self.gravity_alignment_rotation
            x, y, z = raw_position
            vx, vy = raw_velocity[:2]
            width, depth, height = snapshot.extent
            # 位置は原点移動、速度は軸の回転だけを行う。
            corridor_position = (
                self.corridor_frame.transform_position(snapshot.position)
                if self.corridor_frame is not None
                else None
            )
            corridor_velocity = (
                self.corridor_frame.transform_velocity(snapshot.velocity)
                if self.corridor_frame is not None
                else None
            )
            zed_position = (
                self.lidar_to_zed_frame.transform_position(snapshot.position)
                if self.lidar_to_zed_frame is not None
                else None
            )
            zed_velocity = (
                self.lidar_to_zed_frame.transform_vector(snapshot.velocity)
                if self.lidar_to_zed_frame is not None
                else None
            )
            self.writer.writerow(
                {
                    "frame": frame,
                    "time_sec": f"{time_sec:.6f}",
                    "timestamp_ns": timestamp_ns,
                    "track_id": snapshot.track_id,
                    "status": snapshot.status,
                    "detected": int(snapshot.detected),
                    "hits": snapshot.hits,
                    "missed_frames": snapshot.missed_frames,
                    "x": f"{x:.5f}",
                    "y": f"{y:.5f}",
                    "z": f"{z:.5f}",
                    "vx": f"{vx:.5f}",
                    "vy": f"{vy:.5f}",
                    "speed_xy": f"{math.hypot(vx, vy):.5f}",
                    "level_x": f"{level_position[0]:.5f}",
                    "level_y": f"{level_position[1]:.5f}",
                    "level_z": f"{level_position[2]:.5f}",
                    "level_vx": f"{level_velocity[0]:.5f}",
                    "level_vy": f"{level_velocity[1]:.5f}",
                    "level_vz": f"{level_velocity[2]:.5f}",
                    "level_speed_xy": (
                        f"{math.hypot(level_velocity[0], level_velocity[1]):.5f}"
                    ),
                    "zed_x": f"{zed_position[0]:.5f}" if zed_position is not None else "",
                    "zed_y": f"{zed_position[1]:.5f}" if zed_position is not None else "",
                    "zed_z": f"{zed_position[2]:.5f}" if zed_position is not None else "",
                    "zed_vx": f"{zed_velocity[0]:.5f}" if zed_velocity is not None else "",
                    "zed_vy": f"{zed_velocity[1]:.5f}" if zed_velocity is not None else "",
                    "zed_vz": f"{zed_velocity[2]:.5f}" if zed_velocity is not None else "",
                    "corridor_x": (
                        f"{corridor_position[0]:.5f}" if corridor_position is not None else ""
                    ),
                    "corridor_y": (
                        f"{corridor_position[1]:.5f}" if corridor_position is not None else ""
                    ),
                    "corridor_z": (
                        f"{corridor_position[2]:.5f}" if corridor_position is not None else ""
                    ),
                    "corridor_vx": (
                        f"{corridor_velocity[0]:.5f}" if corridor_velocity is not None else ""
                    ),
                    "corridor_vy": (
                        f"{corridor_velocity[1]:.5f}" if corridor_velocity is not None else ""
                    ),
                    "corridor_vz": (
                        f"{corridor_velocity[2]:.5f}" if corridor_velocity is not None else ""
                    ),
                    "corridor_speed_xy": (
                        f"{math.hypot(corridor_velocity[0], corridor_velocity[1]):.5f}"
                        if corridor_velocity is not None
                        else ""
                    ),
                    "point_count": snapshot.point_count,
                    "width": f"{width:.5f}",
                    "depth": f"{depth:.5f}",
                    "height": f"{height:.5f}",
                    "confidence": f"{snapshot.confidence:.5f}",
                }
            )
        self.stream.flush()

    def close(self) -> None:
        self.stream.close()


def _message_timestamp_ns(message: object, fallback_ns: int) -> int:
    """ROSメッセージのheader時刻をナノ秒で返し、無ければfallbackを使う。"""

    header = getattr(message, "header", None)
    stamp = getattr(header, "stamp", None)
    if stamp is None:
        return fallback_ns
    timestamp_ns = int(getattr(stamp, "sec", 0)) * 1_000_000_000 + int(
        getattr(stamp, "nanosec", 0)
    )
    return timestamp_ns or fallback_ns


def _import_rosbag_types(message_type: str):
    try:
        import rosbag2_py
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message
    except ImportError as exc:
        raise TrackingError(
            "rosbag2_py/rclpy is unavailable; source the ROS 2 setup.bash first."
        ) from exc
    try:
        message_class = get_message(message_type)
    except (AttributeError, ModuleNotFoundError, ValueError) as exc:
        raise TrackingError(f"Could not load ROS message type {message_type}: {exc}") from exc
    return rosbag2_py, deserialize_message, message_class


def _iter_bag_clouds(
    bag_path: Path, topic: str, requested_type: str, storage_id: str
) -> Iterator[tuple[object, int, str]]:
    """rosbag2から指定トピックの点群メッセージを順番に読み出す。"""

    if not bag_path.is_dir():
        raise TrackingError(f"rosbag2 directory does not exist: {bag_path}")
    if not (bag_path / "metadata.yaml").is_file():
        raise TrackingError(f"rosbag2 metadata.yaml is missing: {bag_path}")
    rosbag2_py, deserialize_message, _ = _import_rosbag_types(
        "sensor_msgs/msg/PointCloud2"
    )
    reader = rosbag2_py.SequentialReader()
    try:
        reader.open(
            rosbag2_py.StorageOptions(uri=os.fspath(bag_path), storage_id=storage_id),
            rosbag2_py.ConverterOptions("", ""),
        )
    except (RuntimeError, TypeError, ValueError) as exc:
        raise TrackingError(f"Could not open rosbag2: {exc}") from exc

    topic_types = {item.name: item.type for item in reader.get_all_topics_and_types()}
    if topic not in topic_types:
        available = ", ".join(sorted(topic_types)) or "(none)"
        raise TrackingError(f"Topic {topic} was not found in the bag. Available: {available}")
    actual_type = topic_types[topic]
    if requested_type == "auto":
        if actual_type == "sensor_msgs/msg/PointCloud2":
            requested_type = "pointcloud2"
        elif actual_type.endswith("/CustomMsg"):
            requested_type = "custom"
        else:
            raise TrackingError(f"Unsupported point cloud type on {topic}: {actual_type}")
    expected_type = (
        "sensor_msgs/msg/PointCloud2"
        if requested_type == "pointcloud2"
        else actual_type
    )
    if requested_type not in {"pointcloud2", "custom"}:
        raise TrackingError("Message type must be auto, pointcloud2, or custom.")
    if requested_type == "pointcloud2" and actual_type != expected_type:
        raise TrackingError(f"Topic {topic} contains {actual_type}, not PointCloud2.")
    if requested_type == "custom" and not actual_type.endswith("/CustomMsg"):
        raise TrackingError(f"Topic {topic} contains {actual_type}, not a CustomMsg.")
    _, _, message_class = _import_rosbag_types(expected_type)

    # 他トピックを読み飛ばし、指定された点群トピックだけをデシリアライズする。
    while reader.has_next():
        current_topic, serialized, bag_timestamp_ns = reader.read_next()
        if current_topic != topic:
            continue
        message = deserialize_message(serialized, message_class)
        yield message, int(bag_timestamp_ns), requested_type


def _default_output_path(bag_path: Optional[Path]) -> Path:
    name = "live" if bag_path is None else (
        bag_path.parent.name if bag_path.name == "lidar" else bag_path.name
    )
    return DEFAULT_OUTPUT_DIR / f"{name}_tracks.csv"


def _print_progress(frame: int, snapshots: Sequence[TrackSnapshot], started: float) -> None:
    if frame == 1 or frame % 10 == 0:
        confirmed = sum(snapshot.detected for snapshot in snapshots)
        elapsed = max(0.001, time.monotonic() - started)
        print(
            f"\rframes={frame} tracks={len(snapshots)} detections={confirmed} "
            f"processing={frame / elapsed:.1f} fps",
            end="",
            flush=True,
        )


def process_offline(
    bag_path: Path,
    settings: TrackerSettings,
    output_path: Path,
    message_type: str,
    storage_id: str,
    max_frames: Optional[int],
    stride: int,
) -> int:
    # trackerは検出、writerは元フレーム・水平化・廊下座標をCSVへ出力する。
    tracker = PeopleTracker(settings)
    writer = CsvWriter(
        output_path,
        settings.corridor_frame,
        settings.lidar_to_zed_frame,
        settings.gravity_alignment_rotation,
        settings.gravity_alignment_translation,
    )
    frame = 0
    cloud_count = 0
    start_timestamp_ns: Optional[int] = None
    started = time.monotonic()
    try:
        # bagの点群を録画順に処理し、stride指定時は一定間隔のフレームだけ使う。
        for message, bag_timestamp_ns, actual_type in _iter_bag_clouds(
            bag_path, settings.topic, message_type, storage_id
        ):
            cloud_count += 1
            if cloud_count % stride != 0:
                continue
            if actual_type == "custom":
                points = _custom_msg_to_numpy(message)
            else:
                points = _pointcloud2_to_numpy(message)
            timestamp_ns = _message_timestamp_ns(message, bag_timestamp_ns)
            if start_timestamp_ns is None:
                start_timestamp_ns = timestamp_ns
            snapshots = tracker.update(points, timestamp_ns)
            frame += 1
            writer.write(frame, timestamp_ns, snapshots, start_timestamp_ns)
            _print_progress(frame, snapshots, started)
            if max_frames is not None and frame >= max_frames:
                break
    finally:
        writer.close()
    print()
    print(f"Processed {frame} point-cloud frames from {bag_path}.")
    print(f"Track CSV: {output_path}")
    print(f"Unique IDs created: {tracker.next_id - 1}")
    return 0


class MarkerPublisher:
    """RVizへ人物中心・軌跡・速度矢印・座標ラベルを配信する。"""

    TRAJECTORY_HISTORY_LENGTH = 60
    VELOCITY_ARROW_HORIZON_SECONDS = 0.8
    MARKER_NAMESPACES = (
        "people",
        "people_labels",
        "people_velocity",
        "people_trajectory",
    )

    def __init__(
        self,
        node: object,
        topic: str,
        *,
        include_tentative: bool = False,
        corridor_frame: Optional[CorridorFrame] = None,
        gravity_alignment_rotation: Optional[np.ndarray] = None,
        gravity_alignment_translation: Optional[np.ndarray] = None,
    ) -> None:
        try:
            from visualization_msgs.msg import MarkerArray
        except ImportError as exc:
            raise TrackingError("visualization_msgs is unavailable in this ROS 2 environment.") from exc
        self._marker_array_type = MarkerArray
        self.publisher = node.create_publisher(MarkerArray, topic, 10)
        self.include_tentative = include_tentative
        self.corridor_frame = corridor_frame
        self.gravity_alignment_rotation = (
            np.asarray(gravity_alignment_rotation, dtype=np.float64).reshape(3, 3)
            if gravity_alignment_rotation is not None
            else np.eye(3, dtype=np.float64)
        )
        self.gravity_alignment_translation = (
            np.asarray(gravity_alignment_translation, dtype=np.float64).reshape(3)
            if gravity_alignment_translation is not None
            else np.zeros(3, dtype=np.float64)
        )
        self.previous_ids: set[int] = set()
        self.trajectory_points: dict[int, list[tuple[float, float, float]]] = {}
        self.last_frame_id = ""
        self.last_stamp: object = None

    def _position_in_message_frame(self, position: np.ndarray) -> np.ndarray:
        return (
            (np.asarray(position, dtype=np.float64) - self.gravity_alignment_translation)
            @ self.gravity_alignment_rotation
        )

    def _vector_in_message_frame(self, vector: np.ndarray) -> np.ndarray:
        return np.asarray(vector, dtype=np.float64) @ self.gravity_alignment_rotation

    def reset_sequence(self) -> None:
        """録画bagを切り替える前に、古いマーカーと軌跡を消去する。"""

        if self.previous_ids and self.last_stamp is not None:
            from visualization_msgs.msg import Marker

            array = self._marker_array_type()
            for track_id in self.previous_ids:
                for namespace in self.MARKER_NAMESPACES:
                    marker = Marker()
                    marker.header.frame_id = self.last_frame_id
                    marker.header.stamp = self.last_stamp
                    marker.ns = namespace
                    marker.id = track_id
                    marker.action = Marker.DELETE
                    array.markers.append(marker)
            self.publisher.publish(array)
        self.previous_ids.clear()
        self.trajectory_points.clear()

    def publish(self, snapshots: Sequence[TrackSnapshot], frame_id: str, stamp: object) -> None:
        from visualization_msgs.msg import Marker
        from geometry_msgs.msg import Point

        # 確定前の観測も履歴に残し、確定した瞬間から軌跡が始まるようにする。
        active_ids = {snapshot.track_id for snapshot in snapshots}
        for track_id in set(self.trajectory_points) - active_ids:
            del self.trajectory_points[track_id]
        for snapshot in snapshots:
            if not snapshot.detected:
                continue
            history = self.trajectory_points.setdefault(snapshot.track_id, [])
            message_position = self._position_in_message_frame(snapshot.position)
            history.append(tuple(float(value) for value in message_position))
            if len(history) > self.TRAJECTORY_HISTORY_LENGTH:
                del history[:-self.TRAJECTORY_HISTORY_LENGTH]

        # 静止物候補はCSVには残すが、RVizでは既定で未確定トラックを隠す。
        if not self.include_tentative:
            snapshots = [snapshot for snapshot in snapshots if snapshot.confirmed]
        array = self._marker_array_type()
        current_ids = {snapshot.track_id for snapshot in snapshots}
        for deleted_id in self.previous_ids - current_ids:
            for namespace in self.MARKER_NAMESPACES:
                marker = Marker()
                marker.header.frame_id = frame_id
                marker.header.stamp = stamp
                marker.ns = namespace
                marker.id = deleted_id
                marker.action = Marker.DELETE
                array.markers.append(marker)

        for snapshot in snapshots:
            message_position = self._position_in_message_frame(snapshot.position)
            message_velocity = self._vector_in_message_frame(snapshot.velocity)
            color = (
                float((snapshot.track_id * 0.37) % 1.0),
                float((snapshot.track_id * 0.61) % 1.0),
                float((snapshot.track_id * 0.83) % 1.0),
            )
            alpha = (0.9 if snapshot.detected else 0.45) * (
                1.0 if snapshot.confirmed else 0.55
            )

            # 球は点群クラスタの3次元中心を示す。
            center = Marker()
            center.header.frame_id = frame_id
            center.header.stamp = stamp
            center.ns = "people"
            center.id = snapshot.track_id
            center.type = Marker.SPHERE
            center.action = Marker.ADD
            center.pose.position.x = float(message_position[0])
            center.pose.position.y = float(message_position[1])
            center.pose.position.z = float(message_position[2])
            center.pose.orientation.w = 1.0
            center.scale.x = 0.35
            center.scale.y = 0.35
            center.scale.z = 0.35
            center.color.a = alpha
            center.color.r, center.color.g, center.color.b = color
            array.markers.append(center)

            # 矢印は歩行方向を示し、長さは0.8秒先までの移動量に相当する。
            velocity = Marker()
            velocity.header = center.header
            velocity.ns = "people_velocity"
            velocity.id = snapshot.track_id
            velocity.type = Marker.ARROW
            velocity.action = Marker.ADD
            start = Point()
            start.x = float(message_position[0])
            start.y = float(message_position[1])
            start.z = float(message_position[2])
            end = Point()
            end.x = start.x + float(message_velocity[0]) * self.VELOCITY_ARROW_HORIZON_SECONDS
            end.y = start.y + float(message_velocity[1]) * self.VELOCITY_ARROW_HORIZON_SECONDS
            end.z = start.z + float(message_velocity[2]) * self.VELOCITY_ARROW_HORIZON_SECONDS
            velocity.points = [start, end]
            velocity.scale.x = 0.045
            velocity.scale.y = 0.11
            velocity.scale.z = 0.14
            velocity.color.a = alpha
            velocity.color.r, velocity.color.g, velocity.color.b = color
            array.markers.append(velocity)

            # 直近の中心位置を線でつなぎ、短い軌跡として表示する。
            trajectory = Marker()
            trajectory.header = center.header
            trajectory.ns = "people_trajectory"
            trajectory.id = snapshot.track_id
            trajectory.type = Marker.LINE_STRIP
            trajectory.action = Marker.ADD
            trajectory.scale.x = 0.045
            trajectory.color.a = 0.8
            trajectory.color.r, trajectory.color.g, trajectory.color.b = color
            for x, y, z in self.trajectory_points.get(snapshot.track_id, []):
                point = Point()
                point.x, point.y, point.z = x, y, z
                trajectory.points.append(point)
            array.markers.append(trajectory)

            # ラベルには生のLiDAR座標と水平速度を毎フレーム表示する。
            text = Marker()
            text.header = center.header
            text.ns = "people_labels"
            text.id = snapshot.track_id
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            label_offset = max(0.35, float(snapshot.extent[2]) * 0.5) + 0.25
            message_up = self._vector_in_message_frame(
                np.array([0.0, 0.0, label_offset], dtype=np.float64)
            )
            text.pose.position.x = float(message_position[0] + message_up[0])
            text.pose.position.y = float(message_position[1] + message_up[1])
            text.pose.position.z = float(message_position[2] + message_up[2])
            text.pose.orientation.w = 1.0
            text.scale.z = 0.20
            text.color.a = alpha
            text.color.r = text.color.g = text.color.b = 1.0
            x, y, z = (float(value) for value in message_position)
            vx, vy = (float(value) for value in snapshot.velocity[:2])
            speed = math.hypot(float(snapshot.velocity[0]), float(snapshot.velocity[1]))
            text.text = (
                f"ID {snapshot.track_id}\n"
                f"LiDAR中心 x={x:.2f}  y={y:.2f}  z={z:.2f} m\n"
                f"水平速度 {speed:.2f} m/s  (level vx={vx:+.2f}, vy={vy:+.2f})"
            )
            # マーカーは元PointCloud2のframe_idへ戻して重ね、廊下座標もラベルへ追加する。
            if self.corridor_frame is not None:
                corridor_position = self.corridor_frame.transform_position(snapshot.position)
                corridor_velocity = self.corridor_frame.transform_velocity(snapshot.velocity)
                corridor_speed = math.hypot(
                    float(corridor_velocity[0]), float(corridor_velocity[1])
                )
                text.text += (
                    f"\n廊下 X={corridor_position[0]:+.2f}  Y={corridor_position[1]:+.2f}  "
                    f"Z={corridor_position[2]:+.2f} m"
                    f"\n廊下速度 {corridor_speed:.2f} m/s"
                )
            array.markers.append(text)

        self.previous_ids = current_ids
        self.last_frame_id = frame_id
        self.last_stamp = stamp
        self.publisher.publish(array)


def run_live(
    settings: TrackerSettings,
    output_path: Path,
    publish_markers: bool,
) -> int:
    try:
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import PointCloud2
    except ImportError as exc:
        raise TrackingError("rclpy/sensor_msgs is unavailable; source ROS 2 setup.bash first.") from exc

    class LiveNode(Node):
        def __init__(self) -> None:
            super().__init__("people_point_cloud_tracker")
            self.tracker = PeopleTracker(settings)
            self.writer = CsvWriter(
                output_path,
                settings.corridor_frame,
                settings.lidar_to_zed_frame,
                settings.gravity_alignment_rotation,
                settings.gravity_alignment_translation,
            )
            self.start_timestamp_ns: Optional[int] = None
            self.frame = 0
            self.publisher_adapter = (
                MarkerPublisher(
                    self,
                    settings.markers_topic,
                    corridor_frame=settings.corridor_frame,
                    gravity_alignment_rotation=settings.gravity_alignment_rotation,
                    gravity_alignment_translation=settings.gravity_alignment_translation,
                )
                if publish_markers
                else None
            )
            self.subscription = self.create_subscription(
                PointCloud2,
                settings.topic,
                self.on_cloud,
                qos_profile_sensor_data,
            )
            self.get_logger().info(
                f"Tracking PointCloud2 {settings.topic}; CSV: {output_path}"
            )

        def on_cloud(self, message: object) -> None:
            now_ns = self.get_clock().now().nanoseconds
            timestamp_ns = _message_timestamp_ns(message, now_ns)
            if self.start_timestamp_ns is None:
                self.start_timestamp_ns = timestamp_ns
            snapshots = self.tracker.update(_pointcloud2_to_numpy(message), timestamp_ns)
            self.frame += 1
            self.writer.write(self.frame, timestamp_ns, snapshots, self.start_timestamp_ns)
            if self.publisher_adapter is not None:
                frame_id = getattr(getattr(message, "header", None), "frame_id", "")
                self.publisher_adapter.publish(snapshots, frame_id, message.header.stamp)
            if self.frame % 10 == 0:
                self.get_logger().info(f"processed={self.frame}, tracks={len(snapshots)}")

        def close(self) -> None:
            self.writer.close()

    rclpy.init(args=None)
    node = LiveNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        rclpy.shutdown()
    print(f"Track CSV: {output_path}")
    return 0


def _parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    """コマンドライン引数を定義する。"""

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "bag",
        nargs="?",
        type=Path,
        help="処理するrosbag2ディレクトリ（--live使用時は省略）",
    )
    parser.add_argument("--live", action="store_true", help="ライブのPointCloud2トピックを購読する")
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG_PATH)
    parser.add_argument("--topic", default=None, help=f"点群トピック（既定値: {DEFAULT_TOPIC}）")
    parser.add_argument("--output", type=Path, default=None, help="追跡CSVの出力先")
    parser.add_argument(
        "--message-type",
        choices=("auto", "pointcloud2", "custom"),
        default="auto",
        help="録画メッセージの型（ライブ時はPointCloud2のみ）",
    )
    parser.add_argument("--storage-id", default="mcap", help="rosbag2のストレージプラグイン（既定値: mcap）")
    parser.add_argument("--max-frames", type=int, default=None)
    parser.add_argument("--stride", type=int, default=1, help="Nフレームごとに1フレームを処理する")
    parser.add_argument("--ground-mode", choices=("fixed", "auto", "none"), default=None)
    parser.add_argument("--ground-z", type=float, default=None)
    parser.add_argument("--no-background", action="store_true", help="時間背景除去を無効にする")
    parser.add_argument("--publish-markers", action="store_true", help="ライブ時にRViz MarkerArrayを配信する")
    return parser.parse_args(argv)


def main(argv: Optional[Sequence[str]] = None) -> int:
    """設定を読み込み、オフラインまたはライブ追跡を開始する。"""

    args = _parse_args(argv)
    if args.live == (args.bag is not None):
        print("Specify exactly one of a rosbag2 directory or --live.", file=sys.stderr)
        return 2
    if args.stride < 1:
        print("--stride must be positive.", file=sys.stderr)
        return 2
    if args.max_frames is not None and args.max_frames < 1:
        print("--max-frames must be positive.", file=sys.stderr)
        return 2
    try:
        topic = args.topic.strip() if isinstance(args.topic, str) else None
        settings = load_settings(args.config, topic)
        if args.ground_mode is not None:
            settings.ground_mode = args.ground_mode
        if args.ground_z is not None:
            if not math.isfinite(args.ground_z):
                raise TrackingError("--ground-z must be finite.")
            settings.ground_z = args.ground_z
        if args.no_background:
            settings.background_mode = "none"
        output_path = (
            args.output.expanduser().resolve()
            if args.output is not None
            else _default_output_path(args.bag)
        )
        if args.live:
            return run_live(settings, output_path, args.publish_markers or settings.publish_markers)
        bag_path = args.bag.expanduser().resolve()
        return process_offline(
            bag_path,
            settings,
            output_path,
            args.message_type,
            args.storage_id,
            args.max_frames,
            args.stride,
        )
    except (TrackingError, OSError, ValueError) as exc:
        print(f"Error: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
