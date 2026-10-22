#!/usr/bin/env python3
"""Exploratory, held-out LiDAR↔ZED registration using the 2026-10-13 doors.

The first four seconds of empty-hallway data fit a point-to-point ICP correction;
the next four seconds are held out for a static-scene consistency check. This
does not write to the active calibration or source recordings.
"""

from __future__ import annotations

import csv
import json
import math
import sys
from pathlib import Path
from typing import Any

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "scripts"))
from track_people import _iter_bag_clouds, _message_timestamp_ns, _pointcloud2_to_numpy  # noqa: E402

REVIEW = Path("/data/experiment_data/recording_001_svo_review")
BAG = Path("/data/experiment_data/recordings/20261013_group001/recording_001/lidar")
ACTIVE = ROOT / "tracking/lidar_to_zed_left_eye.json"
OUT = Path(__file__).resolve().parent
TOPIC = "/livox/lidar"
STATIC_SECONDS = 8.0
SPLIT_SECONDS = 4.0
RNG_SEED = 20261013


def load_json(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def read_depth_cloud(rows: list[dict[str, str]], camera: dict[str, Any]) -> np.ndarray:
    fx, fy, cx, cy = (float(camera[k]) for k in ("fx", "fy", "cx", "cy"))
    chunks: list[np.ndarray] = []
    for row in rows:
        with np.load(REVIEW / row["depth_file"]) as archive:
            depth = archive["depth_m"].astype(np.float32, copy=False)
        vv, uu = np.mgrid[8:712:3, 8:1272:3]
        z = depth[vv, uu]
        good = np.isfinite(z) & (z >= 0.45) & (z <= 16.0)
        chunks.append(
            np.column_stack(
                ((uu[good] - cx) * z[good] / fx, (vv[good] - cy) * z[good] / fy, z[good])
            ).astype(np.float32)
        )
    return np.concatenate(chunks) if chunks else np.empty((0, 3), dtype=np.float32)


def voxel_first(points: np.ndarray, size_m: float) -> np.ndarray:
    keys = np.floor(points / size_m).astype(np.int32)
    _, keep = np.unique(keys, axis=0, return_index=True)
    return points[np.sort(keep)]


def read_lidar_halves(first_zed_ns: int) -> tuple[np.ndarray, np.ndarray, int, int]:
    rng = np.random.default_rng(RNG_SEED)
    early: list[np.ndarray] = []
    late: list[np.ndarray] = []
    early_count = 0
    late_count = 0
    first_ns = first_zed_ns
    split_ns = first_ns + int(SPLIT_SECONDS * 1e9)
    stop_ns = first_ns + int(STATIC_SECONDS * 1e9)
    for message, fallback_ns, _ in _iter_bag_clouds(BAG, TOPIC, "pointcloud2", "mcap"):
        stamp_ns = _message_timestamp_ns(message, fallback_ns)
        if stamp_ns < first_ns or stamp_ns > stop_ns:
            continue
        points = _pointcloud2_to_numpy(message).astype(np.float32, copy=False)
        if not len(points):
            continue
        points = points[np.isfinite(points).all(axis=1)]
        ranges = np.linalg.norm(points, axis=1)
        points = points[(ranges >= 0.45) & (ranges <= 16.0)]
        if len(points) > 1200:
            points = points[rng.choice(len(points), 1200, replace=False)]
        if not len(points):
            continue
        if stamp_ns <= split_ns:
            early.append(points)
            early_count += 1
        else:
            late.append(points)
            late_count += 1
    if not early or not late:
        raise RuntimeError("The selected static interval did not contain LiDAR frames in both halves")
    return np.concatenate(early), np.concatenate(late), early_count, late_count


def nearest(index: cv2.flann_Index, points: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    ids, squared = index.knnSearch(
        np.ascontiguousarray(points, dtype=np.float32), 1, params={"checks": 64}
    )
    return np.sqrt(squared[:, 0]), ids[:, 0]


def kabsch(source: np.ndarray, target: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    source_center = source.mean(axis=0)
    target_center = target.mean(axis=0)
    u, _, vh = np.linalg.svd((source - source_center).T @ (target - target_center))
    reflection = np.eye(3)
    reflection[2, 2] = np.linalg.det(vh.T @ u.T)
    rotation = vh.T @ reflection @ u.T
    translation = target_center - rotation @ source_center
    return rotation, translation


def fit_icp(source: np.ndarray, target: np.ndarray) -> tuple[np.ndarray, np.ndarray, dict[str, Any]]:
    target = voxel_first(target, 0.02)
    source = voxel_first(source, 0.025)
    source = source[
        (source[:, 2] > 0.4)
        & (source[:, 2] < 15.0)
        & (np.abs(source[:, 0]) < 5.0)
        & (np.abs(source[:, 1]) < 4.0)
    ]
    source_masks = door_masks(source)
    source_door_mask = (
        source_masks["near_left_door_region"] | source_masks["near_right_recess_region"]
    ) & (source[:, 1] > -2.5) & (source[:, 1] < 1.1)
    source = source[source_door_mask]
    target_masks = door_masks(target)
    target_door_mask = (
        target_masks["near_left_door_region"] | target_masks["near_right_recess_region"]
    ) & (target[:, 1] > -2.5) & (target[:, 1] < 1.1)
    target = target[target_door_mask]
    index = cv2.flann_Index(target, dict(algorithm=1, trees=6))
    current = source.copy()
    rotation_total = np.eye(3)
    translation_total = np.zeros(3)
    history: list[dict[str, float | int]] = []
    for threshold in (0.35, 0.25, 0.18, 0.14):
        for iteration in range(12):
            distances, target_ids = nearest(index, current)
            mask = distances < threshold
            if int(mask.sum()) < 300:
                break
            delta_rotation, delta_translation = kabsch(current[mask], target[target_ids[mask]])
            current = current @ delta_rotation.T + delta_translation
            rotation_total = delta_rotation @ rotation_total
            translation_total = delta_rotation @ translation_total + delta_translation
            angle = math.degrees(
                math.acos(float(np.clip((np.trace(delta_rotation) - 1.0) / 2.0, -1.0, 1.0)))
            )
            history.append(
                {
                    "distance_gate_m": threshold,
                    "iteration": iteration,
                    "inliers": int(mask.sum()),
                    "median_inlier_distance_m": float(np.median(distances[mask])),
                    "translation_step_m": float(np.linalg.norm(delta_translation)),
                    "rotation_step_deg": angle,
                }
            )
            if np.linalg.norm(delta_translation) < 0.0005 and angle < 0.003:
                break
    return rotation_total, translation_total, {
        "source_points_after_voxel": int(len(source)),
        "target_points_after_voxel": int(len(target)),
        "fit_regions": ["near_left_door_region", "near_right_recess_region"],
        "vertical_camera_y_range_m": [-2.5, 1.1],
        "iterations": history,
    }


def door_masks(points: np.ndarray) -> dict[str, np.ndarray]:
    near = (points[:, 2] > 0.6) & (points[:, 2] < 4.5)
    return {
        "all_visible_static_surfaces": (points[:, 2] > 0.4) & (points[:, 2] < 15.0),
        "near_left_door_region": near & (points[:, 0] < -0.65) & (points[:, 0] > -2.5),
        "near_right_recess_region": near & (points[:, 0] > 0.7) & (points[:, 0] < 3.2),
    }


def summarize_registration(
    lidar_points: np.ndarray,
    target: np.ndarray,
    rotation: np.ndarray,
    translation: np.ndarray,
) -> dict[str, Any]:
    transformed = lidar_points @ rotation.T + translation
    transformed = voxel_first(transformed, 0.03)
    transformed = transformed[
        (transformed[:, 2] > 0.4)
        & (transformed[:, 2] < 15.0)
        & (np.abs(transformed[:, 0]) < 5.0)
        & (np.abs(transformed[:, 1]) < 4.0)
    ]
    target = voxel_first(target, 0.02)
    index = cv2.flann_Index(target, dict(algorithm=1, trees=6))
    result: dict[str, Any] = {}
    for region, mask in door_masks(transformed).items():
        points = transformed[mask]
        if not len(points):
            result[region] = {"points": 0}
            continue
        distances, _ = nearest(index, points)
        result[region] = {
            "points": int(len(points)),
            "median_m": float(np.median(distances)),
            "p90_m": float(np.quantile(distances, 0.90)),
            "fraction_within_10cm": float(np.mean(distances < 0.10)),
        }
    return result


def render_overlay(
    row: dict[str, str],
    camera: dict[str, Any],
    lidar_points: np.ndarray,
    initial_rotation: np.ndarray,
    initial_translation: np.ndarray,
    candidate_rotation: np.ndarray,
    candidate_translation: np.ndarray,
) -> None:
    image = cv2.imread(str(REVIEW / row["image_file"]))
    if image is None:
        return
    fx, fy, cx, cy = (float(camera[k]) for k in ("fx", "fy", "cx", "cy"))
    panels = []
    for rotation, translation, title in (
        (initial_rotation, initial_translation, "Active transform"),
        (candidate_rotation, candidate_translation, "Door-only ICP candidate"),
    ):
        canvas = image.copy()
        cloud = voxel_first(lidar_points @ rotation.T + translation, 0.035)
        door = door_masks(cloud)
        mask = (
            door["near_left_door_region"] | door["near_right_recess_region"]
        ) & (cloud[:, 1] > -2.5) & (cloud[:, 1] < 1.1)
        cloud = cloud[mask]
        z = cloud[:, 2]
        u = np.rint(fx * cloud[:, 0] / z + cx).astype(int)
        v = np.rint(fy * cloud[:, 1] / z + cy).astype(int)
        good = (
            (z > 0.4)
            & (z < 15.0)
            & (u >= 0)
            & (u < image.shape[1])
            & (v >= 0)
            & (v < image.shape[0])
        )
        for px, py in zip(u[good], v[good]):
            cv2.circle(canvas, (int(px), int(py)), 2, (0, 230, 255), -1, lineType=cv2.LINE_AA)
        cv2.rectangle(canvas, (8, 8), (400, 52), (0, 0, 0), -1)
        cv2.putText(canvas, title, (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2, cv2.LINE_AA)
        panels.append(canvas)
    cv2.imwrite(str(OUT / "heldout_door_projection.png"), np.hstack(panels))


def main() -> None:
    camera = load_json(REVIEW / "camera.json")
    rows = list(csv.DictReader((REVIEW / "frames.csv").open(encoding="utf-8-sig")))
    rows = [row for row in rows if float(row["elapsed_s"]) <= STATIC_SECONDS]
    first_zed_ns = int(rows[0]["image_timestamp_ns"])
    train_rows = [row for row in rows if float(row["elapsed_s"]) <= SPLIT_SECONDS]
    test_rows = [row for row in rows if float(row["elapsed_s"]) > SPLIT_SECONDS]
    lidar_train, lidar_test, train_lidar_frames, test_lidar_frames = read_lidar_halves(first_zed_ns)
    target_train = read_depth_cloud(train_rows, camera)
    target_test = read_depth_cloud(test_rows, camera)
    active = load_json(ACTIVE)
    initial_rotation = np.asarray(active["rotation_source_to_target"], dtype=np.float64)
    initial_translation = np.asarray(active["translation_m"], dtype=np.float64)

    source_train = lidar_train @ initial_rotation.T + initial_translation
    source_test = lidar_test @ initial_rotation.T + initial_translation
    delta_rotation, delta_translation, fit = fit_icp(source_train, target_train)
    candidate_rotation = delta_rotation @ initial_rotation
    candidate_translation = delta_rotation @ initial_translation + delta_translation
    candidate = dict(active)
    candidate.update(
        {
            "calibration_status": "exploratory_door_static_scene_icp_candidate",
            "rotation_source_to_target": candidate_rotation.tolist(),
            "translation_m": candidate_translation.tolist(),
            "method": "Point-to-point ICP on empty-hallway LiDAR and ZED metric-depth clouds, initialized from the existing plane calibration; held-out later half not used to fit.",
            "notes": "Exploratory candidate only. Static-scene ICP can be biased by ZED depth, sparse LiDAR sampling, and repeated parallel hallway surfaces. It has not been accepted as a metric calibration and was not applied to the active transform.",
        }
    )
    test_initial = summarize_registration(lidar_test, target_test, initial_rotation, initial_translation)
    test_candidate = summarize_registration(lidar_test, target_test, candidate_rotation, candidate_translation)
    train_initial = summarize_registration(lidar_train, target_train, initial_rotation, initial_translation)
    train_candidate = summarize_registration(lidar_train, target_train, candidate_rotation, candidate_translation)
    angle_change = math.degrees(
        math.acos(float(np.clip((np.trace(candidate_rotation @ initial_rotation.T) - 1.0) / 2.0, -1.0, 1.0)))
    )
    result = {
        "recording": "20261013_group001/recording_001",
        "review_dir": str(REVIEW),
        "bag": str(BAG),
        "topic": TOPIC,
        "static_interval_zed_elapsed_s": [0.0, STATIC_SECONDS],
        "fit_interval_zed_elapsed_s": [0.0, SPLIT_SECONDS],
        "heldout_interval_zed_elapsed_s": [SPLIT_SECONDS, STATIC_SECONDS],
        "first_zed_timestamp_ns": first_zed_ns,
        "lidar_frames_fit": train_lidar_frames,
        "lidar_frames_heldout": test_lidar_frames,
        "zed_frames_fit": len(train_rows),
        "zed_frames_heldout": len(test_rows),
        "lidar_points_fit_raw_sampled": int(len(lidar_train)),
        "lidar_points_heldout_raw_sampled": int(len(lidar_test)),
        "zed_points_fit_raw": int(len(target_train)),
        "zed_points_heldout_raw": int(len(target_test)),
        "fit": fit,
        "rotation_change_from_active_deg": angle_change,
        "translation_change_from_active_m": float(np.linalg.norm(candidate_translation - initial_translation)),
        "candidate": candidate,
        "training_residuals_active": train_initial,
        "training_residuals_candidate": train_candidate,
        "heldout_residuals_active": test_initial,
        "heldout_residuals_candidate": test_candidate,
        "correspondence_definition": "nearest 3D metric-depth surface to LiDAR points in ZED-left-eye coordinates; this measures cross-sensor surface consistency, not ground-truth object-position error.",
        "door_region_definition": "approximate camera-space ROIs around the two foreground door areas: z=0.6–4.5 m; left x=-2.5 to -0.65 m and right x=0.7 to 3.2 m. These ROIs include adjacent wall/floor points and are not manually annotated door corners.",
        "applied": False,
    }
    (OUT / "candidate.json").write_text(json.dumps(candidate, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    (OUT / "results.json").write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    render_overlay(test_rows[-1], camera, lidar_test, initial_rotation, initial_translation, candidate_rotation, candidate_translation)
    print(json.dumps({
        "train_frames": [train_lidar_frames, len(train_rows)],
        "heldout_frames": [test_lidar_frames, len(test_rows)],
        "rotation_change_deg": angle_change,
        "translation_change_m": result["translation_change_from_active_m"],
        "heldout_active": test_initial,
        "heldout_candidate": test_candidate,
    }, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
