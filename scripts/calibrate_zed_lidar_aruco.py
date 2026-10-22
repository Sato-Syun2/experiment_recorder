#!/usr/bin/env python3
"""両センサーが観測したArUcoパネルの平面からLiDAR→ZED外部校正候補を求める。

既知サイズのマーカーからカメラ側の平面を求め、対応するLiDAR点群にも平面を当てはめる。
複数姿勢の平面法線・距離とIMUの重力方向から回転・並進を推定する。
並進を求めるには、パネル法線が十分異なる方向を向く観測が必要になる。
候補は未使用の独立データで確認してから適用する。
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import sys
from pathlib import Path
from typing import Any

import numpy as np


def _json(path: Path) -> dict[str, Any]:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"Expected a JSON object in {path}")
    return value


def _unit(value: np.ndarray) -> np.ndarray:
    norm = float(np.linalg.norm(value))
    if not math.isfinite(norm) or norm < 1e-9:
        raise ValueError("Cannot normalize a zero or non-finite vector")
    return value / norm


def _rotation_distance_deg(a: np.ndarray, b: np.ndarray) -> float:
    relative = a @ b.T
    cosine = float(np.clip((np.trace(relative) - 1.0) / 2.0, -1.0, 1.0))
    return math.degrees(math.acos(cosine))


def _vector_angle_deg(a: np.ndarray, b: np.ndarray) -> float:
    cosine = float(np.clip(_unit(a) @ _unit(b), -1.0, 1.0))
    return math.degrees(math.acos(cosine))


def _mean_rotation(rotations: list[np.ndarray]) -> np.ndarray:
    u, _, vh = np.linalg.svd(np.sum(rotations, axis=0))
    correction = np.eye(3)
    correction[-1, -1] = np.linalg.det(u @ vh)
    return u @ correction @ vh


def _quaternion_xyzw_to_rotation(value: list[float] | np.ndarray) -> np.ndarray:
    x, y, z, w = _unit(np.asarray(value, dtype=np.float64).reshape(4))
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def _euler_zyx_deg(rotation: np.ndarray) -> dict[str, float]:
    pitch = math.asin(float(np.clip(-rotation[2, 0], -1.0, 1.0)))
    if abs(math.cos(pitch)) > 1e-7:
        roll = math.atan2(float(rotation[2, 1]), float(rotation[2, 2]))
        yaw = math.atan2(float(rotation[1, 0]), float(rotation[0, 0]))
    else:
        roll = 0.0
        yaw = math.atan2(float(-rotation[0, 1]), float(rotation[1, 1]))
    scale = 180.0 / math.pi
    return {"yaw": yaw * scale, "pitch": pitch * scale, "roll": roll * scale}


def _detect_marker_poses(
    review_dir: Path,
    marker_id: int,
    marker_size_m: float,
    dictionary_name: str,
) -> list[dict[str, Any]]:
    """補正済み左画像で既知サイズのArUcoを検出し、パネル姿勢を推定する。"""

    try:
        import cv2
    except ImportError as exc:
        raise RuntimeError(
            "OpenCV is required. Install opencv-contrib-python in the review Python environment."
        ) from exc
    if not hasattr(cv2, "aruco"):
        raise RuntimeError(
            "cv2.aruco is unavailable. Install opencv-contrib-python, not opencv-python."
        )

    camera = _json(review_dir / "camera.json")
    imu_path = review_dir / "imu.csv"
    if not imu_path.is_file():
        raise ValueError(
            "ZED IMU data is missing; re-export the SVO with export_svo_review.py --include-imu"
        )
    imu_rows = {
        int(row["image_timestamp_ns"]): row
        for row in csv.DictReader(imu_path.open(encoding="utf-8-sig"))
    }
    fx, fy, cx, cy = (float(camera[key]) for key in ("fx", "fy", "cx", "cy"))
    matrix = np.array([[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]], dtype=np.float64)
    # 書き出し済みの補正左画像を使うため、歪み係数は0とする。
    distortion = np.zeros((5, 1), dtype=np.float64)
    aruco = cv2.aruco
    dictionary_id = getattr(aruco, dictionary_name, None)
    if dictionary_id is None:
        raise ValueError(f"Unknown OpenCV ArUco dictionary: {dictionary_name}")
    dictionary = aruco.getPredefinedDictionary(dictionary_id)
    if hasattr(aruco, "DetectorParameters"):
        parameters = aruco.DetectorParameters()
    else:  # OpenCV versions before the constructor was renamed
        parameters = aruco.DetectorParameters_create()
    detector = (
        aruco.ArucoDetector(dictionary, parameters) if hasattr(aruco, "ArucoDetector") else None
    )

    # IPPE_SQUAREが要求する順序で、左上・右上・右下・左下の角を定義する。
    half = marker_size_m / 2.0
    object_points = np.array(
        [[-half, half, 0.0], [half, half, 0.0], [half, -half, 0.0], [-half, -half, 0.0]],
        dtype=np.float64,
    )
    frame_rows = list(csv.DictReader((review_dir / "frames.csv").open(encoding="utf-8-sig")))
    observations: list[dict[str, Any]] = []
    for row in frame_rows:
        image_path = review_dir / row["image_file"]
        image = cv2.imread(str(image_path), cv2.IMREAD_GRAYSCALE)
        if image is None:
            continue
        if detector is not None:
            corners, ids, _ = detector.detectMarkers(image)
        else:  # ArucoDetector導入前のOpenCVでも同じマーカー検出を使う。
            corners, ids, _ = aruco.detectMarkers(image, dictionary, parameters=parameters)
        if ids is None:
            continue
        matching = [
            index
            for index, detected_id in enumerate(ids.reshape(-1))
            if int(detected_id) == marker_id
        ]
        if len(matching) != 1:
            continue
        image_points = np.asarray(corners[matching[0]], dtype=np.float64).reshape(4, 2)
        success, rvec, tvec = cv2.solvePnP(
            object_points,
            image_points,
            matrix,
            distortion,
            flags=cv2.SOLVEPNP_IPPE_SQUARE,
        )
        if not success:
            continue
        rotation, _ = cv2.Rodrigues(rvec)
        center = tvec.reshape(3)
        if center[2] <= 0 or not np.isfinite(center).all():
            continue
        normal = rotation[:, 2]
        if float(normal @ -center) < 0:
            normal = -normal
        observations.append(
            {
                "timestamp_ns": int(row["image_timestamp_ns"]),
                "elapsed_s": float(row["elapsed_s"]),
                "center_cam_marker_m": center,
                "rotation_cam_marker": rotation,
                "normal_cam": _unit(normal),
                "zed_acceleration_imu_m_s2": (
                    np.asarray(
                        [
                            float(imu_rows[int(row["image_timestamp_ns"])][f"accel_{axis}_m_s2"])
                            for axis in ("x", "y", "z")
                        ],
                        dtype=np.float64,
                    )
                    if int(row["image_timestamp_ns"]) in imu_rows
                    else None
                ),
                "zed_gyro_imu_deg_s": (
                    np.asarray(
                        [
                            float(imu_rows[int(row["image_timestamp_ns"])][f"gyro_{axis}_deg_s"])
                            for axis in ("x", "y", "z")
                        ],
                        dtype=np.float64,
                    )
                    if int(row["image_timestamp_ns"]) in imu_rows
                    else None
                ),
                "frame": row["image_file"],
            }
        )
    if not observations:
        raise ValueError(
            f"No ArUco ID {marker_id} detections. Check the marker dictionary, ID, image sharpness, and lighting."
        )
    return observations


def _stable_pose_groups(
    poses: list[dict[str, Any]],
    translation_tolerance_m: float,
    rotation_tolerance_deg: float,
    min_duration_s: float,
    max_observations: int,
) -> list[dict[str, Any]]:
    """連続したマーカー観測から、姿勢が安定した区間をまとめる。"""

    groups: list[list[dict[str, Any]]] = []
    current: list[dict[str, Any]] = []
    for pose in poses:
        if current:
            center = np.mean([item["center_cam_marker_m"] for item in current], axis=0)
            rotation = _mean_rotation([item["rotation_cam_marker"] for item in current])
            moved = np.linalg.norm(pose["center_cam_marker_m"] - center) > translation_tolerance_m
            turned = (
                _rotation_distance_deg(pose["rotation_cam_marker"], rotation)
                > rotation_tolerance_deg
            )
            if moved or turned:
                groups.append(current)
                current = []
        current.append(pose)
    if current:
        groups.append(current)

    stable: list[dict[str, Any]] = []
    for group in groups:
        duration = group[-1]["elapsed_s"] - group[0]["elapsed_s"]
        if len(group) < 3 or duration < min_duration_s:
            continue
        zed_accelerations = [
            item["zed_acceleration_imu_m_s2"]
            for item in group
            if item.get("zed_acceleration_imu_m_s2") is not None
            and item.get("zed_gyro_imu_deg_s") is not None
        ]
        zed_gyros = [
            item["zed_gyro_imu_deg_s"]
            for item in group
            if item.get("zed_acceleration_imu_m_s2") is not None
            and item.get("zed_gyro_imu_deg_s") is not None
        ]
        static_imu = [
            acceleration
            for acceleration, gyro in zip(zed_accelerations, zed_gyros)
            if 8.0 <= float(np.linalg.norm(acceleration)) <= 11.5
            and float(np.linalg.norm(gyro)) <= 2.0
        ]
        if len(static_imu) < 2:
            continue
        rotation = _mean_rotation([item["rotation_cam_marker"] for item in group])
        center_marker = np.mean([item["center_cam_marker_m"] for item in group], axis=0)
        normal_cam = _unit(rotation[:, 2])
        if float(normal_cam @ -center_marker) < 0:
            normal_cam = -normal_cam
        stable.append(
            {
                "timestamp_ns": int(np.median([item["timestamp_ns"] for item in group])),
                "first_timestamp_ns": group[0]["timestamp_ns"],
                "last_timestamp_ns": group[-1]["timestamp_ns"],
                "elapsed_s": float(np.median([item["elapsed_s"] for item in group])),
                "duration_s": float(duration),
                "frame_count": len(group),
                "center_cam_marker_m": center_marker,
                "rotation_cam_marker": rotation,
                "normal_cam": normal_cam,
                "zed_gravity_imu_m_s2": np.mean(static_imu, axis=0),
                "zed_imu_acceleration_std_m_s2": np.std(static_imu, axis=0),
                "zed_imu_samples": len(static_imu),
                "frames": [item["frame"] for item in group],
            }
        )
    if len(stable) < 6:
        raise ValueError(
            f"Only {len(stable)} stable marker poses with usable ZED IMU samples were found; "
            "record at least 6, preferably 10, holding the panel still for about one second at each pose."
        )
    return stable[:max_observations]


def _read_lidar_observation_points(
    bag_path: Path,
    topic: str,
    observations: list[dict[str, Any]],
    time_offset_s: float,
    window_s: float,
    storage: str,
) -> tuple[list[np.ndarray], list[int]]:
    """パネルの安定区間に対応するLiDAR点群を収集する。"""

    sys.path.insert(0, str(Path(__file__).resolve().parent))
    from track_people import (  # pylint: disable=import-outside-toplevel
        _iter_bag_clouds,
        _message_timestamp_ns,
        _pointcloud2_to_numpy,
    )

    offset_ns = int(time_offset_s * 1e9)
    window_ns = int(window_s * 1e9)
    starts = [int(item["first_timestamp_ns"]) + offset_ns - window_ns for item in observations]
    ends = [int(item["last_timestamp_ns"]) + offset_ns + window_ns for item in observations]
    chunks: list[list[np.ndarray]] = [[] for _ in observations]
    counts = [0 for _ in observations]
    for message, fallback_ns, _ in _iter_bag_clouds(bag_path, topic, "pointcloud2", storage):
        stamp = _message_timestamp_ns(message, fallback_ns)
        points: np.ndarray | None = None
        for index, (start, end) in enumerate(zip(starts, ends)):
            if start <= stamp <= end:
                if points is None:
                    points = _pointcloud2_to_numpy(message).astype(np.float32, copy=False)
                    points = points[np.isfinite(points).all(axis=1)]
                    ranges = np.linalg.norm(points, axis=1)
                    points = points[(ranges >= 0.35) & (ranges <= 30.0)]
                if len(points):
                    chunks[index].append(points)
                    counts[index] += 1
    return [
        np.concatenate(value).astype(np.float64) if value else np.empty((0, 3)) for value in chunks
    ], counts


def _read_lidar_imu_observations(
    bag_path: Path,
    topic: str,
    observations: list[dict[str, Any]],
    time_offset_s: float,
    window_s: float,
    storage: str,
    acceleration_unit: str,
    max_gyro_rad_s: float,
) -> tuple[list[dict[str, Any] | None], list[str]]:
    """対応区間のIMU加速度・角速度を集計し、静止状態を確認する。"""

    sys.path.insert(0, str(Path(__file__).resolve().parent))
    from track_people import (  # pylint: disable=import-outside-toplevel
        _import_rosbag_types,
        _message_timestamp_ns,
    )

    rosbag2_py, deserialize_message, message_class = _import_rosbag_types("sensor_msgs/msg/Imu")
    reader = rosbag2_py.SequentialReader()
    try:
        reader.open(
            rosbag2_py.StorageOptions(uri=str(bag_path), storage_id=storage),
            rosbag2_py.ConverterOptions("", ""),
        )
    except (RuntimeError, TypeError, ValueError) as exc:
        raise ValueError(f"Could not open rosbag2 for IMU data: {exc}") from exc
    topic_types = {item.name: item.type for item in reader.get_all_topics_and_types()}
    if topic not in topic_types:
        available = ", ".join(sorted(topic_types)) or "(none)"
        raise ValueError(f"LiDAR IMU topic {topic} was not found. Available: {available}")
    if topic_types[topic] != "sensor_msgs/msg/Imu":
        raise ValueError(
            f"LiDAR IMU topic {topic} has type {topic_types[topic]}, expected sensor_msgs/msg/Imu"
        )

    offset_ns = int(time_offset_s * 1e9)
    window_ns = int(window_s * 1e9)
    starts = [int(item["first_timestamp_ns"]) + offset_ns - window_ns for item in observations]
    ends = [int(item["last_timestamp_ns"]) + offset_ns + window_ns for item in observations]
    samples: list[list[tuple[np.ndarray, np.ndarray]]] = [[] for _ in observations]
    frame_ids: set[str] = set()
    accel_scale = 9.80665 if acceleration_unit == "g" else 1.0
    while reader.has_next():
        current_topic, serialized, bag_timestamp_ns = reader.read_next()
        if current_topic != topic:
            continue
        message = deserialize_message(serialized, message_class)
        stamp = _message_timestamp_ns(message, int(bag_timestamp_ns))
        frame_id = str(getattr(getattr(message, "header", None), "frame_id", ""))
        if frame_id:
            frame_ids.add(frame_id)
        acceleration = (
            np.array(
                [
                    message.linear_acceleration.x,
                    message.linear_acceleration.y,
                    message.linear_acceleration.z,
                ],
                dtype=np.float64,
            )
            * accel_scale
        )
        gyro = np.array(
            [message.angular_velocity.x, message.angular_velocity.y, message.angular_velocity.z],
            dtype=np.float64,
        )
        if not np.isfinite(acceleration).all() or not np.isfinite(gyro).all():
            continue
        acceleration_norm = float(np.linalg.norm(acceleration))
        if not (0.75 * 9.80665 <= acceleration_norm <= 1.25 * 9.80665):
            continue
        if float(np.linalg.norm(gyro)) > max_gyro_rad_s:
            continue
        for index, (start, end) in enumerate(zip(starts, ends)):
            if start <= stamp <= end:
                samples[index].append((acceleration, gyro))

    summaries: list[dict[str, Any] | None] = []
    for group_samples in samples:
        if len(group_samples) < 3:
            summaries.append(None)
            continue
        accelerations = np.asarray([item[0] for item in group_samples])
        gyros = np.asarray([item[1] for item in group_samples])
        summaries.append(
            {
                "acceleration_lidar_m_s2": np.mean(accelerations, axis=0),
                "acceleration_std_m_s2": np.std(accelerations, axis=0),
                "gyro_rms_rad_s": float(np.sqrt(np.mean(np.sum(gyros**2, axis=1)))),
                "sample_count": len(group_samples),
            }
        )
    return summaries, sorted(frame_ids)


def _fit_target_plane(
    points: np.ndarray,
    center_guess: np.ndarray,
    board_rotation_lidar: np.ndarray,
    width_m: float,
    height_m: float,
    roi_margin_m: float,
    plane_search_m: float,
    normal_gate_deg: float,
    seed: int,
) -> dict[str, Any]:
    """予測パネル周囲の点群から平面を推定し、形状・支持点を確認する。"""

    if not len(points):
        raise ValueError("No LiDAR frames overlap this stable marker pose")
    axis_x = _unit(board_rotation_lidar[:, 0])
    axis_y = _unit(board_rotation_lidar[:, 1])
    expected_normal = _unit(board_rotation_lidar[:, 2])
    if float(expected_normal @ -center_guess) < 0:
        expected_normal = -expected_normal
    delta = points - center_guess
    local_x, local_y, local_n = delta @ axis_x, delta @ axis_y, delta @ expected_normal
    roi = (
        (np.abs(local_x) <= width_m / 2 + roi_margin_m)
        & (np.abs(local_y) <= height_m / 2 + roi_margin_m)
        & (np.abs(local_n) <= plane_search_m)
    )
    candidates = points[roi]
    if len(candidates) < 20:
        raise ValueError(f"Only {len(candidates)} LiDAR points in the predicted panel region")

    expected_d = -float(expected_normal @ center_guess)
    rng = np.random.default_rng(seed)
    cosine_min = math.cos(math.radians(normal_gate_deg))
    best_count = 0
    best_normal: np.ndarray | None = None
    best_d = 0.0
    iterations = min(6000, max(1200, len(candidates) * 8))
    for _ in range(iterations):
        triangle = candidates[rng.choice(len(candidates), 3, replace=False)]
        normal = np.cross(triangle[1] - triangle[0], triangle[2] - triangle[0])
        norm = float(np.linalg.norm(normal))
        if norm < 1e-7:
            continue
        normal /= norm
        if float(normal @ expected_normal) < 0:
            normal *= -1
        if float(normal @ expected_normal) < cosine_min:
            continue
        offset = -float(normal @ triangle[0])
        if abs(offset - expected_d) > plane_search_m:
            continue
        inliers = np.abs(candidates @ normal + offset) < 0.018
        count = int(inliers.sum())
        if count > best_count:
            best_count = count
            best_normal = normal.copy()
            best_d = offset
    if best_normal is None or best_count < 20:
        raise ValueError(f"Could not fit the panel plane (best inlier count {best_count})")

    normal, offset = best_normal, best_d
    for _ in range(6):
        residual = np.abs(candidates @ normal + offset)
        inliers = candidates[residual < 0.018]
        if len(inliers) < 3:
            break
        center = np.mean(inliers, axis=0)
        _, _, vh = np.linalg.svd(inliers - center, full_matrices=False)
        refined = vh[-1]
        refined_d = -float(refined @ center)
        if float(refined @ expected_normal) < 0:
            refined *= -1
            refined_d *= -1
        normal, offset = refined, refined_d
    residuals = candidates @ normal + offset
    inliers = candidates[np.abs(residuals) < 0.018]
    if len(inliers) < 20:
        raise ValueError("Panel plane refinement produced fewer than 20 inliers")
    # 広い探索範囲には大きな平行壁も入る。
    # 点数の多さだけでパネルと誤認しないよう、支持領域の寸法も確認する。
    panel_spans = np.ptp((inliers - center_guess) @ np.column_stack((axis_x, axis_y)), axis=0)
    if np.any(panel_spans > np.array([width_m, height_m]) + 0.08):
        raise ValueError(
            "Fitted plane exceeds panel dimensions; likely wall/background contamination"
        )
    spread = np.linalg.svd(inliers - np.mean(inliers, axis=0), compute_uv=False) / math.sqrt(
        len(inliers)
    )
    if spread[1] < 0.015:
        raise ValueError("Panel returns are nearly collinear; collect more spatial coverage")
    if float(normal @ expected_normal) < cosine_min:
        raise ValueError("Refined panel normal is outside the CAD orientation gate")
    return {
        "normal_lidar": _unit(normal),
        "d_lidar_m": float(offset),
        "inliers": int(len(inliers)),
        "candidate_points": int(len(candidates)),
        "median_abs_residual_m": float(np.median(np.abs(residuals[np.abs(residuals) < 0.018]))),
        "span_xyz_m": np.ptp(inliers, axis=0).tolist(),
        "panel_axis_spans_m": panel_spans.tolist(),
        "principal_spread_m": spread.tolist(),
    }


def _kabsch_normals(
    source: np.ndarray, target: np.ndarray, weights: np.ndarray | None = None
) -> np.ndarray:
    """対応法線の重み付きSVDから、反射を含まない回転を求める。"""

    if weights is None:
        weights = np.ones(len(source), dtype=np.float64)
    u, _, vh = np.linalg.svd(target.T @ (source * weights[:, None]))
    correction = np.eye(3)
    correction[-1, -1] = np.linalg.det(u @ vh)
    return u @ correction @ vh


def _solve_transform(
    observations: list[dict[str, Any]], imu_rotation_weight: float
) -> tuple[np.ndarray, np.ndarray, dict[str, Any]]:
    """法線と重力から回転、平面距離から並進を求め、外れ値と退化を確認する。"""

    source_normals = np.asarray([item["plane_lidar"]["normal_lidar"] for item in observations])
    target_normals = np.asarray([item["normal_cam"] for item in observations])
    gravity_lidar = _unit(np.mean([item["gravity_lidar"] for item in observations], axis=0))
    gravity_cam = _unit(np.mean([item["gravity_cam"] for item in observations], axis=0))
    # パネル法線が3方向を十分に張らないと、並進の一部が決まらない。
    source_rank = np.linalg.svd(source_normals, compute_uv=False)
    if source_rank[-1] / source_rank[0] < 0.06:
        raise ValueError(
            "Panel normals do not span 3D well enough to solve translation. Tilt the panel toward "
            "several different directions and collect new poses."
        )

    def fit_rotation(keep_mask: np.ndarray) -> np.ndarray:
        source = np.vstack((source_normals[keep_mask], gravity_lidar))
        target = np.vstack((target_normals[keep_mask], gravity_cam))
        weights = np.concatenate(
            (np.ones(int(keep_mask.sum()), dtype=np.float64), [imu_rotation_weight])
        )
        return _kabsch_normals(source, target, weights)

    board_only_rotation = _kabsch_normals(source_normals, target_normals)
    rotation = fit_rotation(np.ones(len(observations), dtype=bool))
    normal_errors = np.array(
        [
            math.degrees(math.acos(float(np.clip((rotation @ src) @ dst, -1.0, 1.0))))
            for src, dst in zip(source_normals, target_normals)
        ]
    )
    keep = normal_errors <= 12.0
    if int(keep.sum()) < 6:
        raise ValueError(
            f"Only {int(keep.sum())} observations have consistent plane normals; check target association and pose diversity."
        )
    if not keep.all():
        rotation = fit_rotation(keep)
        normal_errors = np.array(
            [
                math.degrees(math.acos(float(np.clip((rotation @ src) @ dst, -1.0, 1.0))))
                for src, dst in zip(source_normals, target_normals)
            ]
        )

    normals_cam = (rotation @ source_normals.T).T
    target_offsets = np.asarray(
        [item["plane_lidar"]["d_lidar_m"] - item["d_cam_m"] for item in observations],
        dtype=np.float64,
    )
    keep = normal_errors <= 12.0
    if int(keep.sum()) < 6:
        raise ValueError("Fewer than six consistent normals remain after refinement")
    matrix = normals_cam[keep]
    rhs = target_offsets[keep]
    singular = np.linalg.svd(matrix, compute_uv=False)
    if len(singular) < 3 or singular[-1] / singular[0] < 0.06:
        raise ValueError(
            "The measured plane normals leave translation poorly constrained. Collect poses with "
            "panel normals spanning three directions (not just different panel positions)."
        )
    translation, _, _, _ = np.linalg.lstsq(matrix, rhs, rcond=None)
    residuals = normals_cam @ translation - target_offsets
    # 中央値とMADで平面距離の外れ値を除き、残る法線の条件数も再確認する。
    median = float(np.median(residuals[keep]))
    mad = float(np.median(np.abs(residuals[keep] - median)))
    translation_keep = keep & (np.abs(residuals - median) <= max(0.04, 3.5 * 1.4826 * mad))
    if not np.array_equal(translation_keep, keep):
        if int(translation_keep.sum()) < 6:
            raise ValueError("Fewer than six plane offsets survive robust rejection")
        matrix = normals_cam[translation_keep]
        rhs = target_offsets[translation_keep]
        singular = np.linalg.svd(matrix, compute_uv=False)
        if singular[-1] / singular[0] < 0.06:
            raise ValueError("Translation becomes unobservable after outlier rejection")
        translation, _, _, _ = np.linalg.lstsq(matrix, rhs, rcond=None)
    final_residuals = normals_cam @ translation - target_offsets
    # 法線・回転を固定した条件付き共分散を計算する。
    # ターゲットや系統誤差を含む、外部校正全体の不確かさではない。
    variance = float(np.sum(final_residuals[translation_keep] ** 2) / (translation_keep.sum() - 3))
    covariance = variance * np.linalg.inv(matrix.T @ matrix)
    leave_one_out = []
    for index in range(len(matrix)):
        subset = np.arange(len(matrix)) != index
        if np.linalg.matrix_rank(matrix[subset]) == 3:
            trial = np.linalg.lstsq(matrix[subset], rhs[subset], rcond=None)[0]
            leave_one_out.append(float(np.linalg.norm(trial - translation)))
    report = {
        "rotation_normal_errors_deg": normal_errors.tolist(),
        "plane_offset_residuals_m": final_residuals.tolist(),
        "used_observations": int(translation_keep.sum()),
        "rotation_inlier_mask": keep.tolist(),
        "translation_inlier_mask": translation_keep.tolist(),
        "translation_conditional_covariance_m2": covariance.tolist(),
        "translation_conditional_std_m": np.sqrt(np.diag(covariance)).tolist(),
        "uncertainty_note": "Conditional on fitted rotation/normals; excludes systematic and rotation errors. Not independent accuracy.",
        "translation_leave_one_out_max_change_m": max(leave_one_out) if leave_one_out else None,
        "translation_leave_one_out_solvable_count": len(leave_one_out),
        "gravity_direction_lidar": gravity_lidar.tolist(),
        "gravity_direction_zed": gravity_cam.tolist(),
        "gravity_alignment_error_deg": math.degrees(
            math.acos(float(np.clip((rotation @ gravity_lidar) @ gravity_cam, -1.0, 1.0)))
        ),
        "board_only_to_imu_fused_rotation_deg": _rotation_distance_deg(
            rotation, board_only_rotation
        ),
        "imu_rotation_weight": imu_rotation_weight,
        "translation_singular_values": singular.tolist(),
        "translation_condition_number": float(singular[0] / singular[-1]),
        "plane_offset_rmse_m": float(np.sqrt(np.mean(final_residuals[translation_keep] ** 2))),
        "plane_offset_all_rmse_m": float(np.sqrt(np.mean(final_residuals**2))),
    }
    return rotation, translation, report


def _markdown(result: dict[str, Any]) -> str:
    rows = "\n".join(
        f"| {item['index']} | {item['elapsed_s']:.2f} | {item['lidar_plane']['inliers']} | "
        f"{item['zed_imu_samples']} | {np.linalg.norm(item['zed_imu_acceleration_std_m_s2']):.3f} | "
        f"{item['lidar_imu_samples']} | {np.linalg.norm(item['lidar_imu_acceleration_std_m_s2']):.3f} | "
        f"{item['normal_error_deg']:.2f} | {item['plane_offset_residual_m']:+.3f} |"
        for item in result["observations"]
    )
    candidate = result["candidate"]
    return f"""# ArUcoパネルによるZED–LiDAR校正

- 対象: `{result['review_dir']}` と `{result['bag']}`
- ArUco: ID {result['marker_id']}、辞書 `{result['dictionary']}`、マーカー辺 {result['marker_size_m']:.3f} m
- パネル: {result['board_width_m']:.3f} × {result['board_height_m']:.3f} m
- LiDAR時刻オフセット: `{result['lidar_time_offset_s']:+.6f} s`（LiDAR時刻 = ZED時刻 + offset）
- 使用姿勢: {result['used_observations']} / {result['detected_stable_poses']}

## 推定値

`p_zed = R × p_lidar + t`（入力はLivox元座標、出力はZED左カメラIMAGE座標、単位 m）

- 回転行列: `{json.dumps(candidate['rotation_source_to_target'])}`
- 並進: `{np.round(candidate['translation_m'], 5).tolist()} m`
- 平面法線の角度残差 RMS: {result['normal_error_rms_deg']:.2f}°
- 平面オフセット残差 RMS: {result['plane_offset_rmse_m']:.3f} m
- 並進の条件数: {result['translation_condition_number']:.1f}
- IMU重力方向の角度残差: {result['gravity_alignment_error_deg']:.2f}°
- ZED重力入力: `{result['zed_gravity_acceleration_override_m_s2'] or 'SVO静止サンプル平均'}`
- 入力値とSVO静止平均の方向差: {result['zed_gravity_override_difference_from_svo_deg'] if result['zed_gravity_override_difference_from_svo_deg'] is not None else '—'}°
- IMU統合による回転補正量（板面のみの解から）: {result['board_only_to_imu_fused_rotation_deg']:.2f}°
- 相対角度 ZYX: yaw {result['relative_orientation_zyx_deg']['yaw']:.2f}°、pitch {result['relative_orientation_zyx_deg']['pitch']:.2f}°、roll {result['relative_orientation_zyx_deg']['roll']:.2f}°

| # | ZED経過時間 (s) | LiDAR面 inliers | ZED IMU n | ZED加速度σ (m/s²) | LiDAR IMU n | LiDAR加速度σ (m/s²) | 法線残差 (°) | 面オフセット残差 (m) |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|
{rows}

## 解釈

この方法は各センサーで同じパネル面を観測して、複数姿勢の面法線と面オフセットを合わせます。加えて、静止区間のZED・Livox加速度から得た重力方向を回転推定へ重み付きで加えます。重力ベクトルは相対ピッチ・ロールを拘束し、板面の複数方向の法線は重力軸周りの回転と位置を補います。パネルの中心位置をLiDAR点群から直接推定しないため、面内の点密度の偏りに影響されにくい方法です。マーカーはパネル中央に貼り、マーカー中心とパネル中心を一致させてください。

平面法線の向きが似通うと並進が不安定になります。校正値を採用する前に、法線残差・オフセット残差・条件数と各姿勢の点数を確認してください。ここでの残差は校正に使ったターゲット面への適合度で、独立な測位精度の保証ではありません。

## 成果物

- 候補変換: `candidate.json`
- 詳細: `calibration.json`
- 適用先: `{result['applied_to'] or '未適用（--apply なし）'}`
"""


def main() -> int:
    """複数姿勢の観測から校正候補と残差・条件数のレポートを作る。"""

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--review-dir",
        type=Path,
        required=True,
        help="export_svo_review.py output with camera.json, frames.csv, and frames/",
    )
    parser.add_argument("--bag", type=Path, required=True, help="LiDAR rosbag2 directory")
    parser.add_argument(
        "--output-dir",
        type=Path,
        required=True,
        help="new output directory for the calibration candidate",
    )
    parser.add_argument(
        "--calibration", type=Path, default=Path("tracking/lidar_to_zed_left_eye.json")
    )
    parser.add_argument("--topic", default="/livox/lidar")
    parser.add_argument("--imu-topic", default="/livox/imu")
    parser.add_argument("--storage", choices=("mcap", "sqlite3"), default="mcap")
    parser.add_argument("--marker-id", type=int, default=1)
    parser.add_argument(
        "--dictionary",
        default="DICT_4X4_50",
        help="OpenCV ArUco dictionary; default assumes DICT_4X4_50",
    )
    parser.add_argument("--marker-size-m", type=float, default=0.150)
    parser.add_argument("--board-width-m", type=float, default=0.250)
    parser.add_argument("--board-height-m", type=float, default=0.180)
    parser.add_argument(
        "--marker-offset-x-m",
        type=float,
        default=0.0,
        help="marker center relative to panel center in marker x",
    )
    parser.add_argument(
        "--marker-offset-y-m",
        type=float,
        default=0.0,
        help="marker center relative to panel center in marker y",
    )
    parser.add_argument(
        "--lidar-time-offset-seconds",
        type=float,
        required=True,
        help="LiDAR timestamp = ZED timestamp + this explicit, verified offset",
    )
    parser.add_argument("--lidar-window-seconds", type=float, default=0.30)
    parser.add_argument(
        "--zed-gravity-acceleration-m-s2",
        type=float,
        nargs=3,
        metavar=("X", "Y", "Z"),
        help="use a measured static ZED IMU acceleration vector (in IMU axes, m/s^2) for gravity alignment",
    )
    parser.add_argument(
        "--livox-acceleration-unit",
        choices=("g", "m/s2"),
        default="g",
        help="MID-360 driver2 publishes acceleration values in g",
    )
    parser.add_argument("--max-static-gyro-rad-s", type=float, default=0.15)
    parser.add_argument(
        "--imu-rotation-weight",
        type=float,
        default=4.0,
        help="gravity-vector weight relative to one panel normal",
    )
    parser.add_argument("--translation-stability-m", type=float, default=0.05)
    parser.add_argument("--rotation-stability-deg", type=float, default=5.0)
    parser.add_argument("--min-stable-seconds", type=float, default=0.40)
    parser.add_argument("--max-observations", type=int, default=10)
    parser.add_argument("--roi-margin-m", type=float, default=0.60)
    parser.add_argument("--plane-search-m", type=float, default=0.80)
    parser.add_argument("--normal-gate-deg", type=float, default=25.0)
    parser.add_argument("--seed", type=int, default=34103)
    parser.add_argument(
        "--apply", action="store_true", help="write candidate to the active calibration JSON"
    )
    args = parser.parse_args()

    if not math.isfinite(args.lidar_time_offset_seconds):
        parser.error("LiDAR time offset must be finite")
    output_dir = args.output_dir.expanduser().resolve()
    if output_dir.exists() and any(output_dir.iterdir()):
        parser.error(f"Output directory is not empty: {output_dir}")
    if min(args.marker_size_m, args.board_width_m, args.board_height_m, args.max_observations) <= 0:
        parser.error("Marker and panel dimensions and --max-observations must be positive")
    if args.board_width_m < args.marker_size_m or args.board_height_m < args.marker_size_m:
        parser.error("The marker must fit inside the panel")
    if args.lidar_window_seconds < 0 or args.min_stable_seconds < 0:
        parser.error("Time windows must be non-negative")
    if args.translation_stability_m <= 0 or args.rotation_stability_deg <= 0:
        parser.error("Pose stability thresholds must be positive")
    if args.roi_margin_m < 0 or args.plane_search_m <= 0:
        parser.error("LiDAR ROI margin must be non-negative and plane search must be positive")
    if args.max_static_gyro_rad_s <= 0 or args.imu_rotation_weight <= 0:
        parser.error("IMU gyro threshold and rotation weight must be positive")
    zed_gravity_override = None
    if args.zed_gravity_acceleration_m_s2 is not None:
        zed_gravity_override = np.asarray(args.zed_gravity_acceleration_m_s2, dtype=np.float64)
        gravity_norm = float(np.linalg.norm(zed_gravity_override))
        if not np.isfinite(zed_gravity_override).all() or not (
            0.75 * 9.80665 <= gravity_norm <= 1.25 * 9.80665
        ):
            parser.error(
                "--zed-gravity-acceleration-m-s2 must be a finite static acceleration vector near 1 g"
            )

    review_dir = args.review_dir.expanduser().resolve()
    bag_path = args.bag.expanduser().resolve()
    calibration_path = args.calibration.expanduser().resolve()
    poses = _detect_marker_poses(review_dir, args.marker_id, args.marker_size_m, args.dictionary)
    groups = _stable_pose_groups(
        poses,
        args.translation_stability_m,
        args.rotation_stability_deg,
        args.min_stable_seconds,
        args.max_observations,
    )
    lidar_chunks, lidar_frame_counts = _read_lidar_observation_points(
        bag_path,
        args.topic,
        groups,
        args.lidar_time_offset_seconds,
        args.lidar_window_seconds,
        args.storage,
    )
    lidar_imu, lidar_imu_frame_ids = _read_lidar_imu_observations(
        bag_path,
        args.imu_topic,
        groups,
        args.lidar_time_offset_seconds,
        args.lidar_window_seconds,
        args.storage,
        args.livox_acceleration_unit,
        args.max_static_gyro_rad_s,
    )
    camera_metadata = _json(review_dir / "camera.json")
    if not str(camera_metadata.get("coordinate_system", "")).startswith("IMAGE"):
        raise ValueError(
            "camera.json must declare the ZED IMAGE coordinate system; re-export the SVO with the updated exporter"
        )
    if camera_metadata.get("camera_imu_transform_direction") != "imu_to_left_camera":
        raise ValueError(
            "camera.json is missing the ZED IMU-to-left-camera rotation; re-export with --include-imu"
        )
    camera_imu_rotation = _quaternion_xyzw_to_rotation(
        camera_metadata["camera_imu_rotation_imu_to_left_camera_xyzw"]
    )

    initial = _json(calibration_path)
    source_frame = initial.get("source_frame", "livox_frame")
    if lidar_imu_frame_ids and any(frame_id != source_frame for frame_id in lidar_imu_frame_ids):
        raise ValueError(
            f"Livox IMU frame IDs {lidar_imu_frame_ids} do not match calibration source frame {source_frame!r}"
        )
    rotation0 = np.asarray(initial["rotation_source_to_target"], dtype=np.float64)
    translation0 = np.asarray(initial["translation_m"], dtype=np.float64)
    if rotation0.shape != (3, 3) or translation0.shape != (3,):
        raise ValueError("Initial calibration must contain a 3x3 rotation and 3-vector translation")

    observations: list[dict[str, Any]] = []
    failures: list[dict[str, Any]] = []
    for index, (group, points, frame_count, imu_summary) in enumerate(
        zip(groups, lidar_chunks, lidar_frame_counts, lidar_imu), start=1
    ):
        if imu_summary is None:
            failures.append(
                {
                    "index": index,
                    "reason": "too few static Livox IMU samples near this panel pose",
                    "elapsed_s": group["elapsed_s"],
                }
            )
            continue
        board_rotation_cam = group["rotation_cam_marker"]
        center_cam = group["center_cam_marker_m"] - board_rotation_cam @ np.array(
            [args.marker_offset_x_m, args.marker_offset_y_m, 0.0]
        )
        center_lidar_guess = rotation0.T @ (center_cam - translation0)
        board_rotation_lidar = rotation0.T @ board_rotation_cam
        try:
            plane = _fit_target_plane(
                points,
                center_lidar_guess,
                board_rotation_lidar,
                args.board_width_m,
                args.board_height_m,
                args.roi_margin_m,
                args.plane_search_m,
                args.normal_gate_deg,
                args.seed + index,
            )
        except ValueError as exc:
            failures.append({"index": index, "reason": str(exc), "elapsed_s": group["elapsed_s"]})
            continue
        normal_cam = plane["normal_lidar"] @ rotation0.T
        normal_cam = _unit(normal_cam)
        marker_normal_cam = group["normal_cam"]
        if float(normal_cam @ marker_normal_cam) < 0:
            normal_cam *= -1
            plane["normal_lidar"] *= -1
            plane["d_lidar_m"] *= -1
        center_marker_cam = center_cam
        d_cam = -float(marker_normal_cam @ center_marker_cam)
        zed_acceleration_imu = (
            zed_gravity_override
            if zed_gravity_override is not None
            else group["zed_gravity_imu_m_s2"]
        )
        observations.append(
            {
                "elapsed_s": group["elapsed_s"],
                "duration_s": group["duration_s"],
                "camera_frames": group["frame_count"],
                "lidar_frames": frame_count,
                "normal_cam": marker_normal_cam,
                "d_cam_m": d_cam,
                "gravity_cam": _unit(camera_imu_rotation @ zed_acceleration_imu),
                "gravity_lidar": _unit(imu_summary["acceleration_lidar_m_s2"]),
                "zed_acceleration_imu_m_s2": zed_acceleration_imu,
                "zed_gravity_from_svo_imu_m_s2": group["zed_gravity_imu_m_s2"],
                "zed_imu_samples": group["zed_imu_samples"],
                "lidar_imu_samples": imu_summary["sample_count"],
                "lidar_imu_acceleration_std_m_s2": imu_summary["acceleration_std_m_s2"].tolist(),
                "lidar_imu_gyro_rms_rad_s": imu_summary["gyro_rms_rad_s"],
                "plane_lidar": plane,
            }
        )
    if len(observations) < 6:
        details = "; ".join(
            f"#{item['index']} at {item['elapsed_s']:.2f}s: {item['reason']}" for item in failures
        )
        raise ValueError(
            f"Only {len(observations)} LiDAR panel planes were usable; need at least 6. {details}"
        )

    rotation, translation, solve_report = _solve_transform(observations, args.imu_rotation_weight)
    normal_errors = np.asarray(solve_report["rotation_normal_errors_deg"], dtype=np.float64)
    offset_residuals = np.asarray(solve_report["plane_offset_residuals_m"], dtype=np.float64)
    candidate = {
        "version": int(initial.get("version", 1)),
        "source_frame": initial.get("source_frame", "livox_frame"),
        "target_frame": initial.get("target_frame", "zed_left_eye"),
        "coordinate_system": initial.get("coordinate_system", "IMAGE"),
        "units": "m",
        "calibration_status": "aruco_panel_plane_calibrated",
        "translation_m": translation.tolist(),
        "rotation_source_to_target": rotation.tolist(),
        "relative_orientation_zyx_deg": _euler_zyx_deg(rotation),
        "zed_gravity_acceleration_imu_m_s2": (
            zed_gravity_override.tolist() if zed_gravity_override is not None else None
        ),
        "zed_gravity_source": (
            "user_supplied_static_measurement"
            if zed_gravity_override is not None
            else "static_svo_samples"
        ),
        "method": "Multi-pose ArUco camera plane and LiDAR point-cloud plane alignment",
        "observable_translation_directions": ["three-dimensional when panel plane normals span 3D"],
        "notes": "Rotation is fitted jointly to ArUco/LiDAR panel-plane normals and the two IMUs' gravity directions. Translation is fitted from the paired panel plane offsets. Verify residuals and condition number before applying; this is a target-fit result, not independent position accuracy.",
    }
    result_observations = []
    for index, item in enumerate(observations, start=1):
        result_observations.append(
            {
                "index": index,
                "elapsed_s": item["elapsed_s"],
                "duration_s": item["duration_s"],
                "camera_frames": item["camera_frames"],
                "lidar_frames": item["lidar_frames"],
                "zed_imu_samples": item["zed_imu_samples"],
                "gravity_camera": item["gravity_cam"].tolist(),
                "gravity_lidar": item["gravity_lidar"].tolist(),
                "zed_acceleration_imu_used_m_s2": item["zed_acceleration_imu_m_s2"].tolist(),
                "zed_gravity_from_svo_imu_m_s2": item["zed_gravity_from_svo_imu_m_s2"].tolist(),
                "zed_imu_acceleration_std_m_s2": item["zed_imu_acceleration_std_m_s2"].tolist(),
                "lidar_imu_samples": item["lidar_imu_samples"],
                "lidar_plane": item["plane_lidar"],
                "normal_error_deg": float(normal_errors[index - 1]),
                "plane_offset_residual_m": float(offset_residuals[index - 1]),
            }
        )
    zed_override_difference_from_svo = None
    if zed_gravity_override is not None:
        zed_svo_static_mean = np.mean([item["zed_gravity_imu_m_s2"] for item in groups], axis=0)
        zed_override_difference_from_svo = _vector_angle_deg(
            zed_gravity_override, zed_svo_static_mean
        )
    result = {
        "review_dir": str(review_dir),
        "bag": str(bag_path),
        "topic": args.topic,
        "imu_topic": args.imu_topic,
        "storage": args.storage,
        "initial_calibration": initial,
        "marker_id": args.marker_id,
        "dictionary": args.dictionary,
        "marker_size_m": args.marker_size_m,
        "board_width_m": args.board_width_m,
        "board_height_m": args.board_height_m,
        "marker_offset_m": [args.marker_offset_x_m, args.marker_offset_y_m],
        "lidar_time_offset_s": args.lidar_time_offset_seconds,
        "livox_acceleration_unit": args.livox_acceleration_unit,
        "zed_gravity_acceleration_override_m_s2": (
            zed_gravity_override.tolist() if zed_gravity_override is not None else None
        ),
        "zed_gravity_override_difference_from_svo_deg": zed_override_difference_from_svo,
        "lidar_imu_frame_ids": lidar_imu_frame_ids,
        "detected_marker_frames": len(poses),
        "detected_stable_poses": len(groups),
        "used_observations": solve_report["used_observations"],
        "failed_observations": failures,
        "candidate": candidate,
        "observations": result_observations,
        "normal_error_rms_deg": float(np.sqrt(np.mean(normal_errors**2))),
        "plane_offset_rmse_m": solve_report["plane_offset_rmse_m"],
        "translation_condition_number": solve_report["translation_condition_number"],
        "gravity_direction_lidar": solve_report["gravity_direction_lidar"],
        "gravity_direction_zed": solve_report["gravity_direction_zed"],
        "gravity_alignment_error_deg": solve_report["gravity_alignment_error_deg"],
        "board_only_to_imu_fused_rotation_deg": solve_report[
            "board_only_to_imu_fused_rotation_deg"
        ],
        "relative_orientation_zyx_deg": candidate["relative_orientation_zyx_deg"],
        "solver": solve_report,
        "applied_to": str(calibration_path) if args.apply else None,
    }
    output_dir.mkdir(parents=True, exist_ok=True)
    (output_dir / "candidate.json").write_text(
        json.dumps(candidate, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    (output_dir / "calibration.json").write_text(
        json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    (output_dir / "README.md").write_text(_markdown(result), encoding="utf-8")
    if args.apply:
        calibration_path.write_text(
            json.dumps(candidate, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
        )
    print(f"Candidate: {output_dir / 'candidate.json'}")
    print(f"Report: {output_dir / 'README.md'}")
    print(f"Used panel poses: {len(observations)} / {len(groups)}")
    print(f"Normal residual RMS: {result['normal_error_rms_deg']:.2f} deg")
    print(f"Plane offset RMS: {result['plane_offset_rmse_m']:.3f} m")
    print("Applied to active calibration." if args.apply else "Candidate was not applied.")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (OSError, RuntimeError, ValueError) as exc:
        print(f"Error: {exc}", file=sys.stderr)
        raise SystemExit(1) from exc
