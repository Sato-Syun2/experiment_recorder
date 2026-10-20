#!/usr/bin/env python3
"""Calibrate ZED↔LiDAR rotation and observable translation from a static hallway.

This program uses already exported, timestamped ZED metric-depth frames and a
rosbag2 PointCloud2 topic. It fits the floor and two parallel hallway walls in
both sensors, estimates a rigid transform, and preserves the existing mount
estimate along the hallway direction, which parallel planes cannot observe.
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


def _read_lidar_points(
    bag_path: Path, topic: str, first_zed_ns: int, max_elapsed_s: float, seed: int
) -> tuple[np.ndarray, int, int]:
    # Reuse the project rosbag2 decoder; this keeps PointCloud2 handling in one place.
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    from track_people import (  # pylint: disable=import-outside-toplevel
        _iter_bag_clouds,
        _message_timestamp_ns,
        _pointcloud2_to_numpy,
    )

    rng = np.random.default_rng(seed)
    chunks: list[np.ndarray] = []
    stamp_count = 0
    last_stamp = first_zed_ns
    for message, fallback_ns, _ in _iter_bag_clouds(
        bag_path, topic, "pointcloud2", "mcap"
    ):
        stamp_ns = _message_timestamp_ns(message, fallback_ns)
        if stamp_ns < first_zed_ns or stamp_ns > first_zed_ns + int(max_elapsed_s * 1e9):
            continue
        points = _pointcloud2_to_numpy(message).astype(np.float32, copy=False)
        if not len(points):
            continue
        points = points[np.isfinite(points).all(axis=1)]
        ranges = np.linalg.norm(points, axis=1)
        points = points[(ranges >= 0.4) & (ranges <= 25.0)]
        if len(points) > 12_000:
            points = points[rng.choice(len(points), 12_000, replace=False)]
        if len(points):
            chunks.append(points)
            stamp_count += 1
            last_stamp = max(last_stamp, stamp_ns)
    if not chunks:
        raise ValueError("No LiDAR frames overlap the requested static interval")
    return np.concatenate(chunks).astype(np.float64), stamp_count, last_stamp


def _read_zed_points(review_dir: Path, max_elapsed_s: float, stride: int) -> tuple[np.ndarray, int]:
    camera = _json(review_dir / "camera.json")
    fx, fy, cx, cy = (float(camera[key]) for key in ("fx", "fy", "cx", "cy"))
    frames: list[dict[str, str]] = list(csv.DictReader((review_dir / "frames.csv").open(encoding="utf-8-sig")))
    chunks: list[np.ndarray] = []
    frame_count = 0
    for frame in frames:
        if float(frame["elapsed_s"]) > max_elapsed_s:
            continue
        depth_path = review_dir / frame["depth_file"]
        with np.load(depth_path) as archive:
            depth = archive["depth_m"].astype(np.float32, copy=False)
        rows = np.arange(8, depth.shape[0], stride)
        cols = np.arange(8, depth.shape[1], stride)
        vv, uu = np.meshgrid(rows, cols, indexing="ij")
        z = depth[vv, uu]
        valid = np.isfinite(z) & (z >= 0.4) & (z <= 25.0)
        if not valid.any():
            continue
        u = uu[valid].astype(np.float32)
        v = vv[valid].astype(np.float32)
        z = z[valid]
        chunks.append(
            np.column_stack(((u - cx) * z / fx, (v - cy) * z / fy, z)).astype(
                np.float64
            )
        )
        frame_count += 1
    if not chunks:
        raise ValueError("No valid ZED depth frames in the requested static interval")
    return np.concatenate(chunks), frame_count


def _fit_plane(
    points: np.ndarray,
    expected_normal: np.ndarray,
    expected_d: float,
    d_tolerance: float,
    *,
    angle_tolerance_deg: float,
    point_tolerance_m: float,
    iterations: int,
    seed: int,
) -> dict[str, Any]:
    """RANSAC plane with normal and offset priors, followed by total least squares."""
    rng = np.random.default_rng(seed)
    expected_normal = _unit(np.asarray(expected_normal, dtype=np.float64))
    sample = points
    if len(points) > 60_000:
        sample = points[rng.choice(len(points), 60_000, replace=False)]
    # Draw hypotheses from a broad slab around the prior plane. This makes sparse
    # floor returns likely to form a triplet without excluding a modest angle error.
    slab_width = d_tolerance + min(2.0, max(0.35, math.sin(math.radians(angle_tolerance_deg)) * 8.0))
    hypothesis_points = points[
        np.abs(points @ expected_normal + expected_d) <= slab_width
    ]
    if len(hypothesis_points) < 100:
        hypothesis_points = points
    cosine_min = math.cos(math.radians(angle_tolerance_deg))
    best_count = 0
    best_normal: np.ndarray | None = None
    best_d = 0.0
    for start in range(0, iterations, 100):
        count = min(100, iterations - start)
        triangles = hypothesis_points[
            rng.integers(0, len(hypothesis_points), size=(count, 3))
        ]
        normals = np.cross(triangles[:, 1] - triangles[:, 0], triangles[:, 2] - triangles[:, 0])
        norms = np.linalg.norm(normals, axis=1)
        valid = norms > 1e-7
        normals[valid] /= norms[valid, None]
        dots = normals @ expected_normal
        flip = dots < 0
        normals[flip] *= -1
        offsets = -np.einsum("ij,ij->i", normals, triangles[:, 0])
        valid &= (np.abs(dots) >= cosine_min) & (
            np.abs(offsets - expected_d) <= d_tolerance
        )
        normals = normals[valid]
        offsets = offsets[valid]
        for batch_start in range(0, len(offsets), 20):
            batch_n = normals[batch_start : batch_start + 20]
            batch_d = offsets[batch_start : batch_start + 20]
            inliers = np.abs(sample @ batch_n.T + batch_d) < point_tolerance_m
            counts = inliers.sum(axis=0)
            local_index = int(np.argmax(counts))
            if int(counts[local_index]) > best_count:
                best_count = int(counts[local_index])
                best_normal = batch_n[local_index].copy()
                best_d = float(batch_d[local_index])
    if best_normal is None or best_count < 500:
        raise ValueError(
            f"Could not fit a plane near normal {expected_normal.tolist()} and d={expected_d:.3f}"
        )

    normal = best_normal
    offset = best_d
    for _ in range(5):
        residual = np.abs(points @ normal + offset)
        inlier_points = points[residual < point_tolerance_m]
        if len(inlier_points) < 3:
            break
        center = inlier_points.mean(axis=0)
        _, _, vh = np.linalg.svd(inlier_points - center, full_matrices=False)
        refined_normal = vh[-1]
        refined_d = -float(refined_normal @ center)
        if refined_normal @ expected_normal < 0:
            refined_normal *= -1
            refined_d *= -1
        normal, offset = refined_normal, refined_d

    residual = np.abs(points @ normal + offset)
    inlier_points = points[residual < point_tolerance_m]
    residual = residual[residual < point_tolerance_m]
    if len(inlier_points) < 500:
        raise ValueError("Plane refinement produced too few inliers")
    return {
        "normal": normal.tolist(),
        "d_m": float(offset),
        "inliers": int(len(inlier_points)),
        "inlier_fraction": float(len(inlier_points) / len(points)),
        "median_residual_m": float(np.median(residual)),
        "p90_residual_m": float(np.quantile(residual, 0.9)),
        "span_xyz_m": np.ptp(inlier_points, axis=0).tolist(),
        "normal_angle_from_prior_deg": float(
            math.degrees(math.acos(np.clip(normal @ expected_normal, -1.0, 1.0)))
        ),
    }


def _wall_modes(points: np.ndarray, lateral: np.ndarray) -> list[float]:
    projection = points @ lateral
    in_range = projection[(projection >= -4.0) & (projection <= 4.0)]
    counts, edges = np.histogram(in_range, bins=np.arange(-4.0, 4.05, 0.05))
    # Smooth adjacent 5 cm bins and find the two strongest spatially separate bands.
    smooth = np.convolve(counts, np.ones(3, dtype=np.int64), mode="same")
    candidates = np.argsort(smooth)[::-1]
    peaks: list[float] = []
    for index in candidates:
        center = float((edges[index] + edges[index + 1]) / 2)
        if all(abs(center - previous) >= 1.5 for previous in peaks):
            peaks.append(center)
        if len(peaks) == 2:
            break
    if len(peaks) != 2:
        raise ValueError("Could not identify two separated LiDAR side-wall bands")
    return sorted(peaks)


def _kabsch_two_normals(
    source_floor: np.ndarray,
    source_wall: np.ndarray,
    target_floor: np.ndarray,
    target_wall: np.ndarray,
) -> np.ndarray:
    source = np.column_stack((_unit(source_floor), _unit(source_wall)))
    target = np.column_stack((_unit(target_floor), _unit(target_wall)))
    u, _, vh = np.linalg.svd(target @ source.T)
    correction = np.eye(3)
    correction[-1, -1] = np.linalg.det(u @ vh)
    return u @ correction @ vh


def _angle_between(a: np.ndarray, b: np.ndarray) -> float:
    return math.degrees(math.acos(float(np.clip(_unit(a) @ _unit(b), -1.0, 1.0))))


def _validate_person_alignment(
    body_csv: Path,
    tracks_csv: Path,
    body_id: str,
    lidar_track_id: str,
    rotation0: np.ndarray,
    translation0: np.ndarray,
    rotation: np.ndarray,
    translation: np.ndarray,
) -> dict[str, Any]:
    bodies = [
        row
        for row in csv.DictReader(body_csv.open(encoding="utf-8-sig"))
        if row.get("body_id") == body_id
        and all(
            row.get(key)
            for key in ("pelvis_depth_x_m", "pelvis_depth_y_m", "pelvis_depth_z_m")
        )
    ]
    tracks = [
        row
        for row in csv.DictReader(tracks_csv.open(encoding="utf-8-sig"))
        if row.get("track_id") == lidar_track_id
        and row.get("status") == "confirmed"
        and row.get("detected") == "1"
    ]
    if not bodies or not tracks:
        raise ValueError("Person validation found no matching ZED bodies or confirmed LiDAR detections")
    body_times = np.array([int(row["image_timestamp_ns"]) for row in bodies], dtype=np.int64)
    order = np.argsort(body_times)
    body_times = body_times[order]
    bodies = [bodies[index] for index in order]
    old_residuals: list[np.ndarray] = []
    new_residuals: list[np.ndarray] = []
    for track in tracks:
        stamp = int(track["timestamp_ns"])
        position = int(np.searchsorted(body_times, stamp))
        candidates = [index for index in (position - 1, position) if 0 <= index < len(body_times)]
        if not candidates:
            continue
        nearest = min(candidates, key=lambda index: abs(int(body_times[index]) - stamp))
        if abs(int(body_times[nearest]) - stamp) > 80_000_000:
            continue
        body = bodies[nearest]
        pelvis = np.array(
            [float(body[key]) for key in ("pelvis_depth_x_m", "pelvis_depth_y_m", "pelvis_depth_z_m")]
        )
        lidar = np.array([float(track[key]) for key in ("x", "y", "z")])
        old_residuals.append(rotation0 @ lidar + translation0 - pelvis)
        new_residuals.append(rotation @ lidar + translation - pelvis)
    if not old_residuals:
        raise ValueError("No timestamp-matched person observations were available for validation")
    old = np.asarray(old_residuals)
    new = np.asarray(new_residuals)
    return {
        "body_id": body_id,
        "lidar_track_id": lidar_track_id,
        "matched_observations": int(len(old)),
        "old_median_3d_residual_m": float(np.median(np.linalg.norm(old, axis=1))),
        "new_median_3d_residual_m": float(np.median(np.linalg.norm(new, axis=1))),
        "old_median_abs_camera_xyz_m": np.median(np.abs(old), axis=0).tolist(),
        "new_median_abs_camera_xyz_m": np.median(np.abs(new), axis=0).tolist(),
        "description": "Nearest timestamp pairing of confirmed LiDAR cluster center and ZED pelvis depth; these are different body reference points, not ground-truth accuracy.",
    }


def _markdown_report(result: dict[str, Any]) -> str:
    planes = result["planes"]
    candidate = result["candidate"]
    old = result["initial_transform"]
    new = candidate
    validation = result.get("person_validation")
    validation_section = ""
    if validation:
        old_axis = np.round(validation["old_median_abs_camera_xyz_m"], 3).tolist()
        new_axis = np.round(validation["new_median_abs_camera_xyz_m"], 3).tolist()
        validation_section = f"""
## 人体データによる独立確認

確定LiDAR人物トラックとZED骨盤深度点を絶対時刻で最近傍対応させた {validation['matched_observations']} 組では、3D残差中央値が {validation['old_median_3d_residual_m']:.3f} m から {validation['new_median_3d_residual_m']:.3f} m になりました。カメラ座標の各軸の絶対残差中央値は、初期変換で `{old_axis}` m、候補変換で `{new_axis}` m です。比較対象は異なる人体基準点（LiDARクラスタ中心とZED骨盤深度）であり、正解位置誤差ではありません。
"""
    return f"""# ZED–LiDAR 床・壁面校正

- 対象: `{result['recording']}`
- 使用データ: 空の廊下区間 {result['static_interval_s']:.2f} s、LiDAR {result['lidar_frames']} frames、ZED深度 {result['zed_frames']} frames
- ZEDとLiDARの対応: 絶対タイムスタンプ。同期オフセットは追加していない。
- 外部PCでの追加処理: 不要。既存のZED SDK書き出し深度を使用。

## 推定

- 回転の初期値からの変化: **{result['rotation_change_deg']:.2f}°**
- 平面で観測できる並進補正: **{np.linalg.norm(np.asarray(new['translation_m']) - np.asarray(old['translation_m'])):.3f} m**
- 変換: `p_zed = R × p_lidar + t`、単位 m
- 校正状態: `{new['calibration_status']}`

### 平面の対応

| 面 | LiDAR d (m) | ZED d (m) | LiDAR inliers | ZED inliers | 法線角度差 |
|---|---:|---:|---:|---:|---:|
| 床 | {planes['source_floor']['d_m']:.3f} | {planes['target_floor']['d_m']:.3f} | {planes['source_floor']['inliers']} | {planes['target_floor']['inliers']} | {result['normal_angle_residuals_deg']['floor']:.2f}° |
| 側壁 A | {planes['source_walls'][0]['d_m']:.3f} | {planes['target_walls'][0]['d_m']:.3f} | {planes['source_walls'][0]['inliers']} | {planes['target_walls'][0]['inliers']} | {result['normal_angle_residuals_deg']['wall_a']:.2f}° |
| 側壁 B | {planes['source_walls'][1]['d_m']:.3f} | {planes['target_walls'][1]['d_m']:.3f} | {planes['source_walls'][1]['inliers']} | {planes['target_walls'][1]['inliers']} | {result['normal_angle_residuals_deg']['wall_b']:.2f}° |

床と側壁の法線は回転を、面までの距離は床高と横位置を拘束する。LiDAR側の壁間隔は {result['source_wall_gap_m']:.3f} m、ZED深度側は {result['target_wall_gap_m']:.3f} m で、その差は {result['wall_gap_difference_m']:.3f} m。ZED深度の壁面形状や面の対応に起因する差が残るため、cm単位の測位精度を保証する校正ではない。

## 未観測の自由度

床と平行な左右壁は廊下方向に一様なので、廊下方向の並進はこのデータだけでは決まりません。候補ファイルでは既存の取り付け測定値の廊下方向成分を保っています。完全な6自由度校正には、両センサーに共通して写る廊下端の横断壁、または既知位置の平面ターゲットを追加で撮影する必要があります。録画後はZED SDKで深度を同じ形式に書き出せばよく、このPCで現在のZED深度出力を再解析する追加作業は不要です。
{validation_section}

## 成果物

- 候補変換: `candidate.json`
- 詳細数値: `calibration.json`
- 適用先: `{result['applied_to'] or '未適用'}`
"""


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--review-dir", type=Path, required=True, help="SVO review export with frames.csv, camera.json, and depth/")
    parser.add_argument("--bag", type=Path, required=True, help="rosbag2 directory")
    parser.add_argument("--config", type=Path, default=Path("config.json"))
    parser.add_argument("--calibration", type=Path, default=Path("tracking/lidar_to_zed_left_eye.json"))
    parser.add_argument("--output-dir", type=Path, required=True, help="new calibration output directory")
    parser.add_argument("--topic", default="/livox/lidar")
    parser.add_argument("--static-seconds", type=float, default=8.5)
    parser.add_argument("--depth-stride", type=int, default=4)
    parser.add_argument("--seed", type=int, default=34013)
    parser.add_argument("--validation-body-csv", type=Path, help="optional ZED bodies.csv for a person-position comparison")
    parser.add_argument("--validation-tracks", type=Path, help="optional confirmed LiDAR tracks CSV")
    parser.add_argument("--validation-body-id", default="0")
    parser.add_argument("--validation-track-id", help="LiDAR track ID paired with --validation-body-id")
    parser.add_argument("--apply", action="store_true", help="replace the active calibration JSON with the candidate")
    args = parser.parse_args()

    if args.output_dir.exists() and any(args.output_dir.iterdir()):
        parser.error(f"Output directory is not empty: {args.output_dir}")
    if args.static_seconds <= 0 or args.depth_stride < 1:
        parser.error("--static-seconds must be positive and --depth-stride must be at least 1")
    validation_args = (args.validation_body_csv, args.validation_tracks, args.validation_track_id)
    if any(validation_args) and not all(validation_args):
        parser.error("Use --validation-body-csv, --validation-tracks, and --validation-track-id together")

    review_dir = args.review_dir.expanduser().resolve()
    calibration_path = args.calibration.expanduser().resolve()
    config_path = args.config.expanduser().resolve()
    config = _json(config_path)
    tracking = config["tracking"]
    gravity = _unit(np.asarray(tracking["gravity_alignment"]["up_vector"], dtype=np.float64))
    # The level-frame rotation has gravity up as its third row; invert it for axes in livox_frame.
    from track_people import _rotation_from_up_vector  # pylint: disable=import-outside-toplevel

    raw_to_level = _rotation_from_up_vector(gravity)
    corridor_file = Path(tracking["corridor_frame_file"])
    if not corridor_file.is_absolute():
        corridor_file = config_path.parent / corridor_file
    corridor = _json(corridor_file)
    lateral_level = np.array([*corridor["lateral_xy"], 0.0], dtype=np.float64)
    lateral = _unit(raw_to_level.T @ lateral_level)
    initial = _json(calibration_path)
    rotation0 = np.asarray(initial["rotation_source_to_target"], dtype=np.float64)
    translation0 = np.asarray(initial["translation_m"], dtype=np.float64)
    camera = _json(review_dir / "camera.json")
    frames = list(csv.DictReader((review_dir / "frames.csv").open(encoding="utf-8-sig")))
    if not frames:
        raise ValueError("frames.csv is empty")
    first_zed_ns = int(frames[0]["image_timestamp_ns"])

    lidar_points, lidar_frames, last_stamp = _read_lidar_points(
        args.bag.expanduser().resolve(), args.topic, first_zed_ns, args.static_seconds, args.seed
    )
    zed_points, zed_frames = _read_zed_points(review_dir, args.static_seconds, args.depth_stride)

    ground_height = float(tracking["gravity_alignment"]["ground_height_along_up_m"])
    source_floor = _fit_plane(
        lidar_points,
        gravity,
        -ground_height,
        0.55,
        angle_tolerance_deg=12.0,
        point_tolerance_m=0.035,
        iterations=3000,
        seed=args.seed + 1,
    )
    wall_modes = _wall_modes(lidar_points, lateral)
    source_walls = [
        _fit_plane(
            lidar_points,
            lateral,
            -mode,
            0.35,
            angle_tolerance_deg=10.0,
            point_tolerance_m=0.035,
            iterations=3000,
            seed=args.seed + 10 + index,
        )
        for index, mode in enumerate(wall_modes)
    ]
    source_walls.sort(key=lambda plane: plane["d_m"])

    target_floor_prior_n = _unit(rotation0 @ np.asarray(source_floor["normal"]))
    target_floor_prior_d = source_floor["d_m"] - float(target_floor_prior_n @ translation0)
    target_floor = _fit_plane(
        zed_points,
        target_floor_prior_n,
        target_floor_prior_d,
        0.50,
        angle_tolerance_deg=12.0,
        point_tolerance_m=0.04,
        iterations=3500,
        seed=args.seed + 20,
    )
    target_walls: list[dict[str, Any]] = []
    for index, source_wall in enumerate(source_walls):
        prior_n = _unit(rotation0 @ np.asarray(source_wall["normal"]))
        prior_d = source_wall["d_m"] - float(prior_n @ translation0)
        target_walls.append(
            _fit_plane(
                zed_points,
                prior_n,
                prior_d,
                0.40,
                angle_tolerance_deg=3.0,
                point_tolerance_m=0.04,
                iterations=3500,
                seed=args.seed + 30 + index,
            )
        )
    target_walls.sort(key=lambda plane: plane["d_m"])

    source_floor_n = np.asarray(source_floor["normal"], dtype=np.float64)
    source_wall_n = _unit(np.mean([np.asarray(plane["normal"]) for plane in source_walls], axis=0))
    target_floor_n = np.asarray(target_floor["normal"], dtype=np.float64)
    target_wall_n = _unit(np.mean([np.asarray(plane["normal"]) for plane in target_walls], axis=0))
    rotation = _kabsch_two_normals(source_floor_n, source_wall_n, target_floor_n, target_wall_n)

    # Plane constraints determine height and lateral translation, but not the hallway axis.
    source_wall_d_mean = float(np.mean([plane["d_m"] for plane in source_walls]))
    target_wall_d_mean = float(np.mean([plane["d_m"] for plane in target_walls]))
    constraint_normals = np.vstack((rotation @ source_floor_n, rotation @ source_wall_n))
    constraint_offsets = np.array(
        [source_floor["d_m"] - target_floor["d_m"], source_wall_d_mean - target_wall_d_mean]
    )
    translation = translation0 + np.linalg.pinv(constraint_normals) @ (
        constraint_offsets - constraint_normals @ translation0
    )
    null_axis = _unit(np.cross(constraint_normals[0], constraint_normals[1]))
    candidate = {
        "version": int(initial.get("version", 1)),
        "source_frame": initial.get("source_frame", "livox_frame"),
        "target_frame": initial.get("target_frame", "zed_left_eye"),
        "coordinate_system": initial.get("coordinate_system", "IMAGE"),
        "units": "m",
        "calibration_status": "plane_calibrated_partial_translation",
        "translation_m": translation.tolist(),
        "rotation_source_to_target": rotation.tolist(),
        "method": "RANSAC floor and parallel side-wall plane alignment using timestamped LiDAR and ZED depth",
        "observable_translation_directions": ["floor normal", "hallway lateral"],
        "unobservable_translation_direction_target": null_axis.tolist(),
        "unobservable_translation_kept_from_initial_m": float(null_axis @ translation0),
        "notes": "Parallel floor and hallway walls do not observe translation along the hallway. That component is retained from the initial mount measurement; obtain a shared transverse wall or known target for full translation calibration.",
    }
    normal_errors = {
        "floor": _angle_between(rotation @ source_floor_n, target_floor_n),
        "wall_a": _angle_between(rotation @ np.asarray(source_walls[0]["normal"]), np.asarray(target_walls[0]["normal"])),
        "wall_b": _angle_between(rotation @ np.asarray(source_walls[1]["normal"]), np.asarray(target_walls[1]["normal"])),
    }
    source_gap = abs(source_walls[0]["d_m"] - source_walls[1]["d_m"])
    target_gap = abs(target_walls[0]["d_m"] - target_walls[1]["d_m"])
    result: dict[str, Any] = {
        "recording": "20261013_group001/recording_001",
        "review_dir": str(review_dir),
        "bag": str(args.bag.expanduser().resolve()),
        "topic": args.topic,
        "first_zed_timestamp_ns": first_zed_ns,
        "last_lidar_timestamp_ns": last_stamp,
        "static_interval_s": args.static_seconds,
        "lidar_frames": lidar_frames,
        "zed_frames": zed_frames,
        "initial_transform": initial,
        "rotation_change_deg": math.degrees(
            math.acos(float(np.clip((np.trace(rotation @ rotation0.T) - 1) / 2, -1.0, 1.0)))
        ),
        "candidate": candidate,
        "planes": {"source_floor": source_floor, "target_floor": target_floor, "source_walls": source_walls, "target_walls": target_walls},
        "normal_angle_residuals_deg": normal_errors,
        "source_wall_gap_m": source_gap,
        "target_wall_gap_m": target_gap,
        "wall_gap_difference_m": target_gap - source_gap,
        "applied_to": str(calibration_path) if args.apply else None,
    }
    if args.validation_body_csv:
        result["person_validation"] = _validate_person_alignment(
            args.validation_body_csv.expanduser().resolve(),
            args.validation_tracks.expanduser().resolve(),
            args.validation_body_id,
            args.validation_track_id,
            rotation0,
            translation0,
            rotation,
            translation,
        )
    args.output_dir.expanduser().resolve().mkdir(parents=True, exist_ok=True)
    output_dir = args.output_dir.expanduser().resolve()
    (output_dir / "candidate.json").write_text(json.dumps(candidate, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    (output_dir / "calibration.json").write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    (output_dir / "README.md").write_text(_markdown_report(result), encoding="utf-8")
    if args.apply:
        calibration_path.write_text(json.dumps(candidate, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(f"Candidate: {output_dir / 'candidate.json'}")
    print(f"Report: {output_dir / 'README.md'}")
    print(f"Rotation change: {result['rotation_change_deg']:.2f} deg")
    print(f"Translation: {translation.tolist()} m")
    print(f"Wall gap source/target: {source_gap:.3f}/{target_gap:.3f} m")
    print("Applied to active calibration." if args.apply else "Candidate was not applied.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
