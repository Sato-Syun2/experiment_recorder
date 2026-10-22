#!/usr/bin/env python3
"""Associate per-frame ZED body observations with confirmed LiDAR tracks.

The ZED SDK export contains absolute image timestamps, person tracks, skeletons,
and per-person depth summaries. This program aligns those observations with the
existing LiDAR track CSV, transforms positions into the calibrated corridor
frame, rejects isolated motion jumps, and writes reviewable CSV/JSON/HTML.
"""

from __future__ import annotations

import argparse
import csv
import html
import json
import math
import statistics
import sys
from collections import defaultdict
from pathlib import Path
from typing import Any, Optional, Sequence

import numpy as np

from track_people import load_settings


PROJECT_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CONFIG = PROJECT_ROOT / "config.json"
DEFAULT_TRACKS = PROJECT_ROOT / "tracking/20261013_group001_recording_001_gravity_aligned_tracks.csv"
DEFAULT_BODY_DIR = Path("/data/experiment_data/recording_001_zed_body_tracking")
DEFAULT_OUTPUT = PROJECT_ROOT / "tracking/20261013_group001_recording_001_zed_lidar_fusion"


def read_csv(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8-sig") as stream:
        return list(csv.DictReader(stream))


def finite_float(value: Any) -> Optional[float]:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if math.isfinite(number) else None


def parse_json_cell(row: dict[str, str], name: str, default: Any) -> Any:
    value = row.get(name, "")
    if not value:
        return default
    try:
        return json.loads(value)
    except json.JSONDecodeError:
        return default


def vector3(value: Any) -> Optional[np.ndarray]:
    try:
        point = np.asarray(value, dtype=float).reshape(-1)
    except (TypeError, ValueError):
        return None
    if point.size < 3 or not np.isfinite(point[:3]).all():
        return None
    return point[:3]


def interpolate(values: list[dict[str, Any]], timestamp_ns: int, max_gap_ns: int) -> Optional[dict[str, Any]]:
    """Interpolate only inside a short, directly observed confirmed segment."""

    if not values:
        return None
    lo, hi = 0, len(values)
    while lo < hi:
        mid = (lo + hi) // 2
        if values[mid]["timestamp_ns"] < timestamp_ns:
            lo = mid + 1
        else:
            hi = mid
    right_index = lo
    if right_index < len(values) and values[right_index]["timestamp_ns"] == timestamp_ns:
        result = dict(values[right_index])
        result["source_before_ns"] = timestamp_ns
        result["source_after_ns"] = timestamp_ns
        result["nearest_source_timestamp_ns"] = timestamp_ns
        return result
    if right_index == 0 or right_index >= len(values):
        return None
    before, after = values[right_index - 1], values[right_index]
    gap = after["timestamp_ns"] - before["timestamp_ns"]
    if gap <= 0 or gap > max_gap_ns:
        return None
    fraction = (timestamp_ns - before["timestamp_ns"]) / gap
    result: dict[str, Any] = {
        "timestamp_ns": timestamp_ns,
        "track_id": before["track_id"],
        "status": "confirmed",
        "detected": "1",
        "interpolated": True,
        "source_before_ns": before["timestamp_ns"],
        "source_after_ns": after["timestamp_ns"],
        "nearest_source_timestamp_ns": (
            before["timestamp_ns"]
            if timestamp_ns - before["timestamp_ns"] <= after["timestamp_ns"] - timestamp_ns
            else after["timestamp_ns"]
        ),
    }
    for field in (
        "corridor_x", "corridor_y", "corridor_z", "corridor_vx", "corridor_vy",
        "corridor_vz", "zed_x", "zed_y", "zed_z", "confidence", "point_count",
        "width", "depth", "height",
    ):
        a, b = finite_float(before.get(field)), finite_float(after.get(field))
        result[field] = a + fraction * (b - a) if a is not None and b is not None else None
    return result


def camera_to_corridor(point_camera: np.ndarray, settings: Any) -> np.ndarray:
    transform = settings.lidar_to_zed_frame
    corridor = settings.corridor_frame
    if transform is None or corridor is None:
        raise RuntimeError("Config must provide the LiDAR-to-ZED and corridor calibrations.")
    # track_people composes this transform with gravity alignment, so its source
    # is livox_level_frame and its target is the ZED IMAGE coordinate frame.
    point_level = transform.rotation.T @ (point_camera - transform.translation)
    return corridor.transform_position(point_level)


def keypoint_map(raw: Any, names: Sequence[str]) -> dict[str, np.ndarray]:
    if not isinstance(raw, list):
        return {}
    result: dict[str, np.ndarray] = {}
    for index, item in enumerate(raw):
        if index >= len(names):
            break
        point = vector3(item)
        if point is not None:
            result[str(names[index]).upper()] = point
    return result


def keypoint_confidence_map(raw: Any, names: Sequence[str]) -> dict[str, float]:
    if not isinstance(raw, list):
        return {}
    result: dict[str, float] = {}
    for index, item in enumerate(raw):
        if index >= len(names):
            break
        number = finite_float(item)
        if number is not None:
            result[str(names[index]).upper()] = number
    return result


def angle_at(a: Optional[np.ndarray], joint: Optional[np.ndarray], b: Optional[np.ndarray]) -> Optional[float]:
    if a is None or joint is None or b is None:
        return None
    left = a - joint
    right = b - joint
    norm = float(np.linalg.norm(left) * np.linalg.norm(right))
    if norm < 1e-9:
        return None
    cosine = float(np.clip(np.dot(left, right) / norm, -1.0, 1.0))
    return math.degrees(math.acos(cosine))


def joint_metrics(
    points: dict[str, np.ndarray],
    confidence: dict[str, float],
    names: Sequence[str],
    settings: Any,
) -> tuple[dict[str, Optional[float]], list[Any]]:
    transformed: list[Any] = []
    lookup: dict[str, np.ndarray] = {}
    for name in names:
        joint_name = str(name).upper()
        point = points.get(joint_name)
        # Avoid angles built from SDK joints marked as low-confidence. If a
        # build omits per-joint confidence, keep the available 3D point.
        if point is not None and joint_name in confidence and confidence[joint_name] < 25.0:
            point = None
        if point is None:
            transformed.append(None)
        else:
            corridor_point = camera_to_corridor(point, settings)
            transformed.append(corridor_point.tolist())
            lookup[str(name).upper()] = corridor_point

    metrics: dict[str, Optional[float]] = {}
    for side in ("LEFT", "RIGHT"):
        hip = lookup.get(f"{side}_HIP")
        knee = lookup.get(f"{side}_KNEE")
        ankle = lookup.get(f"{side}_ANKLE")
        foot = lookup.get(f"{side}_FOOT")
        metrics[f"{side.lower()}_knee_angle_deg"] = angle_at(hip, knee, ankle)
        metrics[f"{side.lower()}_ankle_angle_deg"] = angle_at(knee, ankle, foot)
        pelvis = lookup.get("PELVIS")
        neck = lookup.get("NECK")
        if pelvis is not None and neck is not None and hip is not None and knee is not None:
            # Approximate hip flexion relative to the torso; this is an SDK
            # skeleton angle, not a calibrated anatomical joint-center angle.
            torso = neck - pelvis
            thigh = knee - hip
            norm = float(np.linalg.norm(torso) * np.linalg.norm(thigh))
            metrics[f"{side.lower()}_hip_torso_angle_deg"] = (
                math.degrees(math.acos(float(np.clip(np.dot(torso, thigh) / norm, -1, 1))))
                if norm > 1e-9 else None
            )
        else:
            metrics[f"{side.lower()}_hip_torso_angle_deg"] = None

    pelvis, neck = lookup.get("PELVIS"), lookup.get("NECK")
    if pelvis is not None and neck is not None:
        trunk = neck - pelvis
        norm = float(np.linalg.norm(trunk))
        metrics["trunk_lean_from_vertical_deg"] = (
            math.degrees(math.acos(float(np.clip(abs(trunk[2]) / norm, -1, 1))))
            if norm > 1e-9 else None
        )
        metrics["trunk_forward_lean_signed_deg"] = (
            math.degrees(math.atan2(float(trunk[0]), abs(float(trunk[2]))))
            if norm > 1e-9 else None
        )
    else:
        metrics["trunk_lean_from_vertical_deg"] = None
        metrics["trunk_forward_lean_signed_deg"] = None
    return metrics, transformed


def bbox_distance(u: float, v: float, bbox: Sequence[float]) -> float:
    if len(bbox) != 4:
        return 0.0
    x0, y0, x1, y1 = [float(value) for value in bbox]
    dx = max(x0 - u, 0.0, u - x1)
    dy = max(y0 - v, 0.0, v - y1)
    return math.hypot(dx, dy)


def associate_tracks(
    observations: list[dict[str, Any]],
    lidar_by_id: dict[str, list[dict[str, Any]]],
    intrinsics: dict[str, float],
    *,
    max_lidar_gap_ns: int,
    timestamp_offset_ns: int,
    bbox_gate_px: float,
    center_gate_m: float,
    forced: dict[str, str],
) -> tuple[dict[str, str], dict[str, list[dict[str, Any]]]]:
    """Find a stable one-to-one ZED ID to LiDAR ID map from projection and 3D."""

    costs: dict[tuple[str, str], list[tuple[float, float, float]]] = defaultdict(list)
    candidate_cache: dict[str, list[dict[str, Any]]] = {}
    fx, fy, cx, cy = (intrinsics[key] for key in ("fx", "fy", "cx", "cy"))
    for obs in observations:
        body_id = str(obs["body_id"])
        target_ns = int(obs["timestamp_ns"]) + timestamp_offset_ns
        candidates: list[dict[str, Any]] = []
        body_camera = obs.get("body_position_camera")
        bbox = obs.get("bbox", [])
        for track_id, timeline in lidar_by_id.items():
            sample = interpolate(timeline, target_ns, max_lidar_gap_ns)
            if sample is None:
                continue
            zed_point = vector3([sample.get("zed_x"), sample.get("zed_y"), sample.get("zed_z")])
            if zed_point is None or zed_point[2] <= 0:
                continue
            u = fx * zed_point[0] / zed_point[2] + cx
            v = fy * zed_point[1] / zed_point[2] + cy
            pixel_error = bbox_distance(u, v, bbox)
            center_error = float(np.linalg.norm(zed_point - body_camera)) if body_camera is not None else 0.0
            if pixel_error > bbox_gate_px or center_error > center_gate_m:
                continue
            score = center_error + pixel_error / max(bbox_gate_px, 1.0)
            candidate = {
                "track_id": track_id, "sample": sample, "projected_u": u,
                "projected_v": v, "pixel_error": pixel_error,
                "center_error": center_error, "score": score,
            }
            candidates.append(candidate)
            costs[(body_id, track_id)].append((score, center_error, pixel_error))
        candidate_cache[f"{obs['observation_index']}"] = candidates

    mapping = dict(forced)
    pair_rows: list[tuple[float, int, str, str]] = []
    for (body_id, track_id), values in costs.items():
        if body_id in mapping or track_id in mapping.values():
            continue
        pair_rows.append((statistics.median(value[0] for value in values), len(values), body_id, track_id))
    used_bodies, used_tracks = set(mapping), set(mapping.values())
    for _score, count, body_id, track_id in sorted(pair_rows, key=lambda item: (item[0], -item[1])):
        if count < 3 or body_id in used_bodies or track_id in used_tracks:
            continue
        mapping[body_id] = track_id
        used_bodies.add(body_id)
        used_tracks.add(track_id)

    selected: dict[str, list[dict[str, Any]]] = {}
    for obs in observations:
        body_id = str(obs["body_id"])
        track_id = mapping.get(body_id)
        if not track_id:
            selected[str(obs["observation_index"])] = []
            continue
        selected[str(obs["observation_index"])] = [
            candidate for candidate in candidate_cache.get(str(obs["observation_index"]), [])
            if candidate["track_id"] == track_id
        ]
    return mapping, selected


def _clamp01(value: Optional[float], fallback: float) -> float:
    if value is None:
        return fallback
    return min(1.0, max(0.0, float(value)))


def build_observations(
    body_rows: list[dict[str, str]],
    names: Sequence[str],
    settings: Any,
    min_confidence: float,
) -> list[dict[str, Any]]:
    observations: list[dict[str, Any]] = []
    for row in body_rows:
        confidence = finite_float(row.get("confidence")) or 0.0
        if confidence < min_confidence:
            continue
        timestamp_ns = int(row["image_timestamp_ns"])
        body_id = str(row.get("body_id", "-1"))
        points = keypoint_map(parse_json_cell(row, "keypoints_3d_camera_json", []), names)
        kp_conf = keypoint_confidence_map(parse_json_cell(row, "keypoint_confidence_json", []), names)
        sdk_velocity_camera = vector3([
            row.get("velocity_camera_x_mps"), row.get("velocity_camera_y_mps"),
            row.get("velocity_camera_z_mps"),
        ])
        sdk_velocity_corridor = None
        if sdk_velocity_camera is not None:
            velocity_level = settings.lidar_to_zed_frame.rotation.T @ sdk_velocity_camera
            sdk_velocity_corridor = settings.corridor_frame.transform_velocity(velocity_level)
        pelvis = points.get("PELVIS")
        body_position = vector3([
            row.get("position_camera_x_m"), row.get("position_camera_y_m"),
            row.get("position_camera_z_m"),
        ])
        skeleton_base_camera = pelvis if pelvis is not None else body_position
        pelvis_depth_camera = vector3([
            row.get("pelvis_depth_x_m"), row.get("pelvis_depth_y_m"),
            row.get("pelvis_depth_z_m"),
        ])
        # The SDK's metric depth sampled around the pelvis pixel is the preferred
        # position measurement. Keep the fitted 3D skeleton pelvis as a separate
        # value so the two ZED estimates remain reviewable.
        fusion_camera = pelvis_depth_camera if pelvis_depth_camera is not None else skeleton_base_camera
        zed_corridor = camera_to_corridor(fusion_camera, settings) if fusion_camera is not None else None
        skeleton_corridor = camera_to_corridor(skeleton_base_camera, settings) if skeleton_base_camera is not None else None
        pelvis_depth_corridor = camera_to_corridor(pelvis_depth_camera, settings) if pelvis_depth_camera is not None else None
        metrics, points_corridor = joint_metrics(points, kp_conf, names, settings)
        bbox = parse_json_cell(row, "bounding_box_2d_json", [])
        if not isinstance(bbox, list) or len(bbox) != 4:
            bbox = []
        depth_median = finite_float(row.get("depth_median_m"))
        depth_p10 = finite_float(row.get("depth_p10_m"))
        depth_p90 = finite_float(row.get("depth_p90_m"))
        pelvis_conf = kp_conf.get("PELVIS", 70.0)
        zed_quality = _clamp01(confidence / 100.0, 0.35) * _clamp01(pelvis_conf / 100.0, 0.7)
        pelvis_depth_pixels = int(finite_float(row.get("pelvis_depth_valid_pixels")) or 0)
        if pelvis_depth_camera is not None:
            zed_quality *= min(1.0, math.sqrt(pelvis_depth_pixels / 15.0))
        if depth_median is not None and depth_p90 is not None and depth_p10 is not None:
            spread = max(0.0, depth_p90 - depth_p10)
            zed_quality *= 1.0 / (1.0 + min(spread, 5.0) / 3.0)
        observations.append({
            "observation_index": len(observations),
            "timestamp_ns": timestamp_ns,
            "elapsed_s": finite_float(row.get("elapsed_s")) or 0.0,
            "body_id": body_id,
            "tracking_state": row.get("tracking_state", ""),
            "confidence": confidence,
            "body_position_camera": skeleton_base_camera,
            "zed_sdk_velocity_corridor": sdk_velocity_corridor,
            "zed_position_corridor": zed_corridor,
            "zed_skeleton_pelvis_corridor": skeleton_corridor,
            "zed_depth_pelvis_corridor": pelvis_depth_corridor,
            "zed_position_source": "pelvis_depth" if pelvis_depth_camera is not None else "skeleton_pelvis_or_body_position",
            "bbox": bbox,
            "points_camera": points,
            "points_corridor": points_corridor,
            "metrics": metrics,
            "keypoint_confidence": kp_conf,
            "depth_median_m": depth_median,
            "depth_p10_m": depth_p10,
            "depth_p90_m": depth_p90,
            "depth_valid_pixels": int(finite_float(row.get("depth_valid_pixels")) or 0),
            "pelvis_depth_m": finite_float(row.get("pelvis_depth_m")),
            "pelvis_depth_valid_pixels": pelvis_depth_pixels,
            "zed_quality": zed_quality,
        })
    return observations


def load_lidar(path: Path) -> dict[str, list[dict[str, Any]]]:
    grouped: dict[str, list[dict[str, Any]]] = defaultdict(list)
    for row in read_csv(path):
        if row.get("status") != "confirmed" or row.get("detected") != "1":
            continue
        try:
            timestamp_ns = int(row["timestamp_ns"])
        except (KeyError, ValueError):
            continue
        item: dict[str, Any] = dict(row)
        item["timestamp_ns"] = timestamp_ns
        item["track_id"] = str(row.get("track_id", ""))
        required = [finite_float(row.get(key)) for key in ("corridor_x", "corridor_y", "corridor_z", "zed_x", "zed_y", "zed_z")]
        if any(value is None for value in required):
            continue
        grouped[item["track_id"]].append(item)
    for timeline in grouped.values():
        timeline.sort(key=lambda item: item["timestamp_ns"])
    return dict(grouped)


def load_forced_associations(path: Optional[Path]) -> dict[str, str]:
    if path is None:
        return {}
    value = json.loads(path.read_text(encoding="utf-8"))
    mapping = value.get("zed_to_lidar", value) if isinstance(value, dict) else None
    if not isinstance(mapping, dict):
        raise ValueError("Association JSON must be an object mapping ZED body IDs to LiDAR track IDs.")
    return {str(key): str(item) for key, item in mapping.items()}


def make_output_row(
    obs: dict[str, Any],
    lidar: Optional[dict[str, Any]],
    *,
    max_fusion_residual_m: float,
    minimum_lidar_longitudinal_share: float,
) -> dict[str, Any]:
    zed_pos = obs["zed_position_corridor"]
    lidar_pos = None
    lidar_cam = None
    if lidar is not None:
        lidar_pos = vector3([lidar.get("corridor_x"), lidar.get("corridor_y"), lidar.get("corridor_z")])
        lidar_cam = vector3([lidar.get("zed_x"), lidar.get("zed_y"), lidar.get("zed_z")])
    zed_lidar_residual = (
        float(np.linalg.norm(zed_pos - lidar_pos))
        if zed_pos is not None and lidar_pos is not None else None
    )

    lidar_quality = 0.0
    lidar_point_count = 0
    lidar_confidence = None
    if lidar is not None:
        lidar_confidence = finite_float(lidar.get("confidence"))
        lidar_point_count = int(finite_float(lidar.get("point_count")) or 0)
        lidar_quality = _clamp01(lidar_confidence, 0.5) * min(1.0, math.sqrt(max(0, lidar_point_count) / 30.0))

    zed_quality = float(obs["zed_quality"])
    fused: Optional[np.ndarray] = None
    fusion_status = "zed_only"
    zed_weight = zed_quality if zed_pos is not None else 0.0
    lidar_weight = lidar_quality if lidar_pos is not None else 0.0
    longitudinal_lidar_weight = lidar_weight
    longitudinal_lidar_share_used: Optional[float] = None
    if zed_pos is not None and lidar_pos is not None:
        if zed_lidar_residual is not None and zed_lidar_residual <= max_fusion_residual_m:
            share_weight = zed_weight * minimum_lidar_longitudinal_share / max(1e-9, 1.0 - minimum_lidar_longitudinal_share)
            longitudinal_lidar_weight = max(lidar_weight, share_weight)
            longitudinal_total = zed_weight + longitudinal_lidar_weight
            lateral_total = zed_weight + lidar_weight
            if longitudinal_total > 1e-9 and lateral_total > 1e-9:
                fused = np.array([
                    (zed_weight * zed_pos[0] + longitudinal_lidar_weight * lidar_pos[0]) / longitudinal_total,
                    (zed_weight * zed_pos[1] + lidar_weight * lidar_pos[1]) / lateral_total,
                    (zed_weight * zed_pos[2] + lidar_weight * lidar_pos[2]) / lateral_total,
                ])
                fusion_status = "weighted_fusion"
                longitudinal_lidar_share_used = longitudinal_lidar_weight / longitudinal_total
        else:
            fusion_status = "sensor_disagreement"
    elif lidar_pos is not None:
        fused = lidar_pos
        fusion_status = "lidar_only"
        longitudinal_lidar_share_used = 1.0
    elif zed_pos is not None:
        fused = zed_pos
        longitudinal_lidar_share_used = 0.0

    metrics = obs["metrics"]
    row: dict[str, Any] = {
        "timestamp_ns": obs["timestamp_ns"],
        "elapsed_s": obs["elapsed_s"],
        "zed_body_id": obs["body_id"],
        "zed_tracking_state": obs["tracking_state"],
        "zed_confidence": obs["confidence"],
        "zed_sdk_velocity_corridor_x_mps": None if obs["zed_sdk_velocity_corridor"] is None else obs["zed_sdk_velocity_corridor"][0],
        "zed_sdk_velocity_corridor_y_mps": None if obs["zed_sdk_velocity_corridor"] is None else obs["zed_sdk_velocity_corridor"][1],
        "zed_sdk_velocity_corridor_z_mps": None if obs["zed_sdk_velocity_corridor"] is None else obs["zed_sdk_velocity_corridor"][2],
        "zed_sdk_speed_corridor_mps": None if obs["zed_sdk_velocity_corridor"] is None else float(np.linalg.norm(obs["zed_sdk_velocity_corridor"][:2])),
        "lidar_track_id": "" if lidar is None else lidar.get("track_id", ""),
        "lidar_interpolated": "" if lidar is None else int(bool(lidar.get("interpolated", False))),
        "lidar_confidence": lidar_confidence,
        "lidar_point_count": lidar_point_count,
        "lidar_position_corridor_x_m": None if lidar_pos is None else lidar_pos[0],
        "lidar_position_corridor_y_m": None if lidar_pos is None else lidar_pos[1],
        "lidar_position_corridor_z_m": None if lidar_pos is None else lidar_pos[2],
        "lidar_velocity_corridor_x_mps": None if lidar is None else finite_float(lidar.get("corridor_vx")),
        "lidar_velocity_corridor_y_mps": None if lidar is None else finite_float(lidar.get("corridor_vy")),
        "lidar_velocity_corridor_z_mps": None if lidar is None else finite_float(lidar.get("corridor_vz")),
        "zed_position_corridor_x_m": None if zed_pos is None else zed_pos[0],
        "zed_position_corridor_y_m": None if zed_pos is None else zed_pos[1],
        "zed_position_corridor_z_m": None if zed_pos is None else zed_pos[2],
        "zed_position_source": obs["zed_position_source"],
        "zed_skeleton_pelvis_corridor_x_m": None if obs["zed_skeleton_pelvis_corridor"] is None else obs["zed_skeleton_pelvis_corridor"][0],
        "zed_skeleton_pelvis_corridor_y_m": None if obs["zed_skeleton_pelvis_corridor"] is None else obs["zed_skeleton_pelvis_corridor"][1],
        "zed_skeleton_pelvis_corridor_z_m": None if obs["zed_skeleton_pelvis_corridor"] is None else obs["zed_skeleton_pelvis_corridor"][2],
        "zed_depth_pelvis_corridor_x_m": None if obs["zed_depth_pelvis_corridor"] is None else obs["zed_depth_pelvis_corridor"][0],
        "zed_depth_pelvis_corridor_y_m": None if obs["zed_depth_pelvis_corridor"] is None else obs["zed_depth_pelvis_corridor"][1],
        "zed_depth_pelvis_corridor_z_m": None if obs["zed_depth_pelvis_corridor"] is None else obs["zed_depth_pelvis_corridor"][2],
        "zed_depth_median_m": obs["depth_median_m"],
        "zed_pelvis_depth_m": obs["pelvis_depth_m"],
        "zed_pelvis_depth_valid_pixels": obs["pelvis_depth_valid_pixels"],
        "zed_depth_p10_m": obs["depth_p10_m"],
        "zed_depth_p90_m": obs["depth_p90_m"],
        "zed_depth_valid_pixels": obs["depth_valid_pixels"],
        "zed_lidar_position_residual_m": zed_lidar_residual,
        "zed_reliability_weight": zed_weight,
        "lidar_reliability_weight": lidar_weight,
        "lidar_longitudinal_share_used": longitudinal_lidar_share_used,
        "fusion_status": fusion_status,
        "fused_position_corridor_x_m": None if fused is None else fused[0],
        "fused_position_corridor_y_m": None if fused is None else fused[1],
        "fused_position_corridor_z_m": None if fused is None else fused[2],
        "fused_velocity_corridor_x_mps": None,
        "fused_velocity_corridor_y_mps": None,
        "fused_velocity_corridor_z_mps": None,
        "motion_outlier": 0,
        "motion_valid": 0,
        "motion_segment_id": None,
        "fused_speed_corridor_mps": None,
        "keypoints_corridor_json": json.dumps(obs["points_corridor"], ensure_ascii=False, allow_nan=False),
        **metrics,
    }
    return row


def smooth_and_velocity(rows: list[dict[str, Any]], max_speed_mps: float, max_gap_s: float) -> None:
    by_body: dict[str, list[dict[str, Any]]] = defaultdict(list)
    for row in rows:
        by_body[str(row["zed_body_id"])].append(row)
    for sequence in by_body.values():
        sequence.sort(key=lambda row: int(row["timestamp_ns"]))
        previous: Optional[tuple[int, np.ndarray]] = None
        for row in sequence:
            point = vector3([
                row.get("fused_position_corridor_x_m"), row.get("fused_position_corridor_y_m"),
                row.get("fused_position_corridor_z_m"),
            ])
            timestamp_ns = int(row["timestamp_ns"])
            row["_point"] = point
            if point is None:
                continue
            if previous is not None:
                dt = (timestamp_ns - previous[0]) / 1e9
                speed = float(np.linalg.norm(point - previous[1]) / dt) if dt > 0 else math.inf
                if dt <= max_gap_s and speed > max_speed_mps:
                    row["motion_outlier"] = 1
                    row["_point"] = None
                    # Compare the next frame to the last accepted point so one
                    # isolated jump does not invalidate the return to the path.
                    continue
            previous = (timestamp_ns, point)

        valid = [row for row in sequence if row["_point"] is not None and not row["motion_outlier"]]
        segment_id = -1
        previous_valid: Optional[dict[str, Any]] = None
        break_after_outlier = False
        for row in sequence:
            if row["motion_outlier"]:
                break_after_outlier = True
                continue
            if row["_point"] is None:
                continue
            timestamp_ns = int(row["timestamp_ns"])
            if previous_valid is None or (
                timestamp_ns - int(previous_valid["timestamp_ns"])
            ) / 1e9 > max_gap_s or break_after_outlier:
                segment_id += 1
            row["motion_segment_id"] = segment_id
            previous_valid = row
            break_after_outlier = False

        smoothed: dict[int, np.ndarray] = {}
        for index, row in enumerate(valid):
            timestamp_ns = int(row["timestamp_ns"])
            neighborhood = [row]
            for direction in (-1, 1):
                cursor = index + direction
                while 0 <= cursor < len(valid) and len(neighborhood) < 5:
                    candidate = valid[cursor]
                    if candidate["motion_segment_id"] != row["motion_segment_id"]:
                        break
                    dt = abs(int(candidate["timestamp_ns"]) - timestamp_ns) / 1e9
                    if dt > max_gap_s:
                        break
                    neighborhood.append(candidate)
                    cursor += direction
            smoothed[id(row)] = np.median(np.stack([item["_point"] for item in neighborhood]), axis=0)
            row["motion_valid"] = 1
            row["fused_position_corridor_x_m"] = float(smoothed[id(row)][0])
            row["fused_position_corridor_y_m"] = float(smoothed[id(row)][1])
            row["fused_position_corridor_z_m"] = float(smoothed[id(row)][2])

        for index, row in enumerate(valid):
            if not row["motion_valid"]:
                continue
            previous_row = valid[index - 1] if index > 0 else None
            next_row = valid[index + 1] if index + 1 < len(valid) else None
            if previous_row is not None and previous_row["motion_segment_id"] != row["motion_segment_id"]:
                previous_row = None
            if next_row is not None and next_row["motion_segment_id"] != row["motion_segment_id"]:
                next_row = None
            if previous_row is not None and next_row is not None:
                dt = (int(next_row["timestamp_ns"]) - int(previous_row["timestamp_ns"])) / 1e9
                if dt <= 0 or dt > 2 * max_gap_s:
                    previous_row = None
                    next_row = None
            if previous_row is None and next_row is None:
                continue
            if previous_row is not None and next_row is not None:
                velocity = (smoothed[id(next_row)] - smoothed[id(previous_row)]) / dt
            elif next_row is not None:
                dt = (int(next_row["timestamp_ns"]) - int(row["timestamp_ns"])) / 1e9
                if dt <= 0 or dt > max_gap_s:
                    continue
                velocity = (smoothed[id(next_row)] - smoothed[id(row)]) / dt
            else:
                dt = (int(row["timestamp_ns"]) - int(previous_row["timestamp_ns"])) / 1e9
                if dt <= 0 or dt > max_gap_s:
                    continue
                velocity = (smoothed[id(row)] - smoothed[id(previous_row)]) / dt
            row["fused_velocity_corridor_x_mps"] = float(velocity[0])
            row["fused_velocity_corridor_y_mps"] = float(velocity[1])
            row["fused_velocity_corridor_z_mps"] = float(velocity[2])
            row["fused_speed_corridor_mps"] = float(np.linalg.norm(velocity[:2]))


def _clean_csv_value(value: Any) -> Any:
    if isinstance(value, np.generic):
        value = value.item()
    if isinstance(value, float) and not math.isfinite(value):
        return ""
    if value is None:
        return ""
    if isinstance(value, (list, dict)):
        return json.dumps(value, ensure_ascii=False, allow_nan=False)
    return value


def write_html(path: Path, rows: list[dict[str, Any]], associations: dict[str, str], metrics: dict[str, Any]) -> None:
    chart_data = []
    for row in rows:
        chart_data.append({
            "t": row["elapsed_s"],
            "x": row.get("fused_position_corridor_x_m"),
            "y": row.get("fused_position_corridor_y_m"),
            "speed": row.get("fused_speed_corridor_mps"),
            "body": row["zed_body_id"],
            "status": row["fusion_status"],
            "motion_valid": row.get("motion_valid", 0),
            "motion_outlier": row.get("motion_outlier", 0),
            "segment": row.get("motion_segment_id"),
        })
    details = html.escape(json.dumps({"associations": associations, "metrics": metrics}, ensure_ascii=False, indent=2))
    data_json = json.dumps(chart_data, ensure_ascii=False, allow_nan=False)
    document = f"""<!doctype html>
<html lang="ja"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>ZED・LiDAR 軌道統合レビュー</title>
<style>body{{font-family:system-ui,sans-serif;margin:24px;color:#18212b}}canvas{{width:100%;height:340px;border:1px solid #ccd3dc;margin:10px 0 24px}}pre{{background:#f4f6f8;padding:14px;overflow:auto}}.note{{color:#596979}}</style>
<h1>ZED・LiDAR 軌道統合レビュー</h1>
<p class="note">横軸はZED先頭フレームからの経過秒、位置は廊下座標系です。センサー間で位置が合わないフレームは統合位置に含めていません。</p>
<h2>廊下平面の軌跡（corridor_x / corridor_y）</h2><canvas id="path" width="1100" height="340"></canvas>
<h2>水平速度</h2><canvas id="speed" width="1100" height="340"></canvas>
<h2>対応付け・集計</h2><pre>{details}</pre>
<script>
const data={data_json};
function plot(canvasId, key, xKey){{
 const c=document.getElementById(canvasId),ctx=c.getContext('2d'),w=c.width,h=c.height;
 ctx.clearRect(0,0,w,h);ctx.strokeStyle='#d8dee6';ctx.lineWidth=1;
 for(let i=0;i<=5;i++){{let y=24+(h-54)*i/5;ctx.beginPath();ctx.moveTo(55,y);ctx.lineTo(w-20,y);ctx.stroke();}}
 const vals=data.map(d=>d[key]).filter(v=>v!==null&&Number.isFinite(v));
 if(!vals.length){{ctx.fillStyle='#555';ctx.fillText('有効な統合値がありません',60,50);return;}}
 let ymin=Math.min(...vals),ymax=Math.max(...vals);if(ymax-ymin<1e-6){{ymin-=1;ymax+=1;}}
 const times=data.map(d=>d.t),tmin=Math.min(...times),tmax=Math.max(...times);
 ctx.fillStyle='#374151';ctx.font='13px sans-serif';ctx.fillText(`${{ymin.toFixed(2)}}`,8,h-24);ctx.fillText(`${{ymax.toFixed(2)}}`,8,30);ctx.fillText(`${{tmin.toFixed(1)}} s`,55,h-8);ctx.fillText(`${{tmax.toFixed(1)}} s`,w-70,h-8);
 const colors=['#087e8b','#e07a5f','#6a4c93','#2a9d8f'];
 for(const body of [...new Set(data.map(d=>d.body))]){{ctx.strokeStyle=colors[Math.abs(Number(body)||0)%colors.length];ctx.lineWidth=2;ctx.beginPath();let started=false;
  for(const d of data.filter(v=>v.body===body)){{const v=d[key];if(v===null||!Number.isFinite(v)){{started=false;continue;}}const x=55+(d.t-tmin)/Math.max(1e-9,tmax-tmin)*(w-75),y=24+(ymax-v)/Math.max(1e-9,ymax-ymin)*(h-54);if(!started){{ctx.moveTo(x,y);started=true;}}else ctx.lineTo(x,y);}}ctx.stroke();}}
}}
function plotPath(){{
 const c=document.getElementById('path'),ctx=c.getContext('2d'),w=c.width,h=c.height;
 ctx.clearRect(0,0,w,h);const pts=data.filter(d=>d.motion_valid===1&&d.x!==null&&d.y!==null&&Number.isFinite(d.x)&&Number.isFinite(d.y));
 if(!pts.length){{ctx.fillText('有効な統合位置がありません',60,50);return;}}
 let xmin=Math.min(...pts.map(d=>d.x)),xmax=Math.max(...pts.map(d=>d.x)),ymin=Math.min(...pts.map(d=>d.y)),ymax=Math.max(...pts.map(d=>d.y));
 if(xmax-xmin<1e-6){{xmin-=1;xmax+=1;}}if(ymax-ymin<1e-6){{ymin-=1;ymax+=1;}}
 const px=v=>55+(v-xmin)/(xmax-xmin)*(w-80),py=v=>24+(ymax-v)/(ymax-ymin)*(h-58);
 ctx.strokeStyle='#d8dee6';ctx.lineWidth=1;for(let i=0;i<=5;i++){{let x=55+(w-80)*i/5;ctx.beginPath();ctx.moveTo(x,24);ctx.lineTo(x,h-34);ctx.stroke();let y=24+(h-58)*i/5;ctx.beginPath();ctx.moveTo(55,y);ctx.lineTo(w-25,y);ctx.stroke();}}
 ctx.fillStyle='#374151';ctx.font='13px sans-serif';ctx.fillText(`corridor_y ${{ymax.toFixed(2)}}`,8,18);ctx.fillText(`corridor_x ${{xmin.toFixed(2)}}–${{xmax.toFixed(2)}} m`,55,h-10);
 for(const body of [...new Set(data.map(d=>d.body))]){{ctx.strokeStyle='#087e8b';ctx.lineWidth=2;ctx.beginPath();let started=false,lastSegment=null;for(const d of data.filter(v=>v.body===body)){{if(d.motion_valid!==1||d.x===null||d.y===null){{started=false;lastSegment=null;continue;}}if(!started||d.segment!==lastSegment){{ctx.moveTo(px(d.x),py(d.y));started=true;lastSegment=d.segment;}}else ctx.lineTo(px(d.x),py(d.y));}}ctx.stroke();}}
}}
plotPath();
// Speed is shown independently because it has a different scale from position.
plot('speed','speed');
</script></html>"""
    path.write_text(document, encoding="utf-8")


def fuse(args: argparse.Namespace) -> None:
    body_dir = args.body_dir.expanduser().resolve()
    body_csv = (args.body_csv or body_dir / "bodies.csv").expanduser().resolve()
    camera_json_path = (args.camera_json or body_dir / "camera.json").expanduser().resolve()
    tracks_path = args.tracks.expanduser().resolve()
    config_path = args.config.expanduser().resolve()
    for required in (body_csv, camera_json_path, tracks_path, config_path):
        if not required.is_file():
            raise FileNotFoundError(f"Required input file does not exist: {required}")

    camera = json.loads(camera_json_path.read_text(encoding="utf-8"))
    intrinsics = {key: float(camera[key]) for key in ("fx", "fy", "cx", "cy")}
    names = camera.get("keypoint_names", [])
    if not isinstance(names, list) or not names:
        raise ValueError("camera.json must contain the exported keypoint_names.")
    settings = load_settings(config_path)
    if settings.corridor_frame is None or settings.lidar_to_zed_frame is None:
        raise RuntimeError("Tracking config is missing the corridor or LiDAR-to-ZED calibration.")

    observations = build_observations(read_csv(body_csv), names, settings, args.min_body_confidence)
    lidar_by_id = load_lidar(tracks_path)
    if not observations:
        raise RuntimeError("No ZED body observations passed the confidence threshold.")
    if not lidar_by_id:
        print("Warning: no confirmed LiDAR detections found; output will contain ZED-only observations.")

    output_dir = args.output_dir.expanduser().resolve()
    if output_dir.exists() and any(output_dir.iterdir()):
        raise RuntimeError(f"Output directory is not empty; choose a new folder: {output_dir}")
    output_dir.mkdir(parents=True, exist_ok=True)

    offset_ns = int(round(args.timestamp_offset_ms * 1_000_000))
    max_lidar_gap_ns = int(args.max_lidar_gap_s * 1_000_000_000)
    forced = load_forced_associations(args.association_json)
    mapping, selected = associate_tracks(
        observations, lidar_by_id, intrinsics, max_lidar_gap_ns=max_lidar_gap_ns,
        timestamp_offset_ns=offset_ns, bbox_gate_px=args.association_bbox_gate_px,
        center_gate_m=args.association_center_gate_m, forced=forced,
    )

    rows: list[dict[str, Any]] = []
    residuals: list[float] = []
    matched = 0
    for obs in observations:
        candidates = selected.get(str(obs["observation_index"]), [])
        candidate = min(candidates, key=lambda item: item["score"]) if candidates else None
        lidar = None if candidate is None else candidate["sample"]
        row = make_output_row(obs, lidar, max_fusion_residual_m=args.max_fusion_residual_m,
                              minimum_lidar_longitudinal_share=args.minimum_lidar_longitudinal_share)
        if lidar is not None:
            matched += 1
            row["sync_residual_ms"] = (int(lidar["nearest_source_timestamp_ns"]) - (int(obs["timestamp_ns"]) + offset_ns)) / 1_000_000.0
            row["lidar_bracket_span_ms"] = (int(lidar["source_after_ns"]) - int(lidar["source_before_ns"])) / 1_000_000.0
            if row["zed_lidar_position_residual_m"] is not None:
                residuals.append(float(row["zed_lidar_position_residual_m"]))
            row["association_pixel_error_px"] = candidate["pixel_error"]
            row["association_center_error_m"] = candidate["center_error"]
        else:
            row["sync_residual_ms"] = ""
            row["lidar_bracket_span_ms"] = ""
            row["association_pixel_error_px"] = ""
            row["association_center_error_m"] = ""
        rows.append(row)

    smooth_and_velocity(rows, args.max_speed_mps, args.max_motion_gap_s)
    output_csv = output_dir / "fused_trajectory.csv"
    fields = [key for key in rows[0].keys() if not key.startswith("_")]
    with output_csv.open("w", newline="", encoding="utf-8-sig") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for row in rows:
            writer.writerow({key: _clean_csv_value(value) for key, value in row.items() if not key.startswith("_")})

    fused_positions = [
        vector3([row.get("fused_position_corridor_x_m"), row.get("fused_position_corridor_y_m"), row.get("fused_position_corridor_z_m")])
        for row in rows if row.get("motion_valid") == 1
    ]
    fused_positions = [point for point in fused_positions if point is not None]
    summary = {
        "source_script": "fuse_zed_lidar.py",
        "source_body_csv": str(body_csv),
        "source_camera_json": str(camera_json_path),
        "source_lidar_tracks_csv": str(tracks_path),
        "source_config": str(config_path),
        "corridor_frame_file": str(settings.corridor_frame_file),
        "lidar_to_zed_frame_file": str(settings.lidar_to_zed_frame_file),
        "timestamp_matching": "absolute nanosecond timestamps from ZED image timestamp and LiDAR message header stamp",
        "timestamp_offset_applied_ms": args.timestamp_offset_ms,
        "zed_body_observations": len(observations),
        "observations_with_associated_lidar": matched,
        "association_coverage": matched / len(observations) if observations else 0.0,
        "zed_to_lidar_track_mapping": mapping,
        "position_residual_median_m": statistics.median(residuals) if residuals else None,
        "position_residual_p90_m": float(np.percentile(residuals, 90)) if residuals else None,
        "valid_motion_frames": len(fused_positions),
        "motion_outlier_frames": sum(int(row["motion_outlier"]) for row in rows),
        "fused_position_range_corridor_x_m": [
            float(min(point[0] for point in fused_positions)),
            float(max(point[0] for point in fused_positions)),
        ] if fused_positions else None,
        "fusion_method": "confidence, keypoint support, person-depth spread, LiDAR confidence and point count; the longitudinal corridor coordinate has the configured minimum LiDAR share; weighted positions are retained only within the configured sensor agreement gate",
        "joint_angle_note": "Angles are computed from the SDK BODY_34 skeleton and are approximate model keypoint angles, not calibrated anatomical joint centers.",
        "thresholds": {
            "minimum_body_confidence": args.min_body_confidence,
            "max_lidar_interpolation_gap_s": args.max_lidar_gap_s,
            "association_bbox_gate_px": args.association_bbox_gate_px,
            "association_center_gate_m": args.association_center_gate_m,
            "max_fusion_residual_m": args.max_fusion_residual_m,
            "minimum_lidar_longitudinal_share": args.minimum_lidar_longitudinal_share,
            "max_human_speed_mps": args.max_speed_mps,
            "max_motion_gap_s": args.max_motion_gap_s,
        },
    }
    (output_dir / "summary.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    (output_dir / "associations.json").write_text(json.dumps({"zed_to_lidar": mapping}, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    write_html(output_dir / "fused_trajectory.html", rows, mapping, summary)
    print(f"Fused trajectory: {output_csv}")
    print(f"Review page: {output_dir / 'fused_trajectory.html'}")
    print(f"Associated ZED frames: {matched}/{len(observations)}")


def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--body-dir", type=Path, default=DEFAULT_BODY_DIR,
                        help="folder created by analyze_svo_body_tracking.py on the SDK PC")
    parser.add_argument("--body-csv", type=Path, default=None, help="override bodies.csv")
    parser.add_argument("--camera-json", type=Path, default=None, help="override camera.json")
    parser.add_argument("--tracks", type=Path, default=DEFAULT_TRACKS, help="LiDAR tracks CSV")
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG, help="project config.json")
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT,
                        help="new output folder; existing non-empty folders are not overwritten")
    parser.add_argument("--timestamp-offset-ms", type=float, default=0.0,
                        help="optional timestamp correction; current recording uses direct raw timestamps")
    parser.add_argument("--max-lidar-gap-s", type=float, default=0.25,
                        help="only interpolate between confirmed LiDAR detections within this gap")
    parser.add_argument("--association-bbox-gate-px", type=float, default=250.0,
                        help="allow for the current initial camera/LiDAR calibration residual")
    parser.add_argument("--association-center-gate-m", type=float, default=4.0)
    parser.add_argument("--association-json", type=Path, default=None,
                        help="optional manual mapping, e.g. {\"zed_to_lidar\": {\"1\": \"45\"}}")
    parser.add_argument("--min-body-confidence", type=float, default=35.0)
    parser.add_argument("--max-fusion-residual-m", type=float, default=2.0,
                        help="skip weighted fusion when camera/LiDAR positions disagree more")
    parser.add_argument("--minimum-lidar-longitudinal-share", type=float, default=0.70,
                        help="minimum normalized LiDAR share for corridor_x when measurements agree")
    parser.add_argument("--max-speed-mps", type=float, default=4.0,
                        help="reject isolated position jumps faster than this")
    parser.add_argument("--max-motion-gap-s", type=float, default=0.25,
                        help="maximum adjacent time gap for smoothing and velocity")
    args = parser.parse_args(argv)
    if (args.max_lidar_gap_s <= 0 or args.max_motion_gap_s <= 0 or args.max_speed_mps <= 0
            or not 0.0 <= args.minimum_lidar_longitudinal_share < 1.0):
        parser.error("gap and speed thresholds must be positive; LiDAR share must be within [0, 1)")
    return args


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = parse_args(argv)
    try:
        fuse(args)
    except (OSError, ValueError, RuntimeError, KeyError, json.JSONDecodeError) as exc:
        print(f"Error: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
