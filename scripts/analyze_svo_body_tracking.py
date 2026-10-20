#!/usr/bin/env python3
"""Export per-frame ZED SDK body tracking, keypoints, and person depth from SVO2.

This script is intended for a PC with ZED SDK 5.4, pyzed, CUDA, NumPy, and
OpenCV installed. It only reads the source SVO and writes into a new output
directory.
"""

from __future__ import annotations

import argparse
import csv
import importlib.metadata
import json
import math
import sys
from pathlib import Path
from typing import Any, Optional, Sequence

import cv2
import numpy as np


BODY_34_NAMES = [
    "PELVIS", "NAVAL_SPINE", "CHEST_SPINE", "NECK", "LEFT_CLAVICLE",
    "LEFT_SHOULDER", "LEFT_ELBOW", "LEFT_WRIST", "LEFT_HAND", "LEFT_HANDTIP",
    "LEFT_THUMB", "RIGHT_CLAVICLE", "RIGHT_SHOULDER", "RIGHT_ELBOW",
    "RIGHT_WRIST", "RIGHT_HAND", "RIGHT_HANDTIP", "RIGHT_THUMB", "LEFT_HIP",
    "LEFT_KNEE", "LEFT_ANKLE", "LEFT_FOOT", "RIGHT_HIP", "RIGHT_KNEE",
    "RIGHT_ANKLE", "RIGHT_FOOT", "HEAD", "NOSE", "LEFT_EYE", "LEFT_EAR",
    "RIGHT_EYE", "RIGHT_EAR", "LEFT_HEEL", "RIGHT_HEEL",
]

BODY_34_EDGES = [
    (0, 1), (1, 2), (2, 4), (4, 5), (5, 6), (6, 7), (7, 8), (8, 9),
    (7, 10), (2, 11), (11, 12), (12, 13), (13, 14), (14, 15), (15, 16),
    (14, 17), (0, 18), (18, 19), (19, 20), (20, 21), (0, 22), (22, 23),
    (23, 24), (24, 25), (2, 3), (3, 26), (26, 27), (27, 28), (28, 29),
    (27, 30), (30, 31), (20, 32), (24, 33), (32, 21), (33, 25),
]


def _json_value(value: Any) -> Any:
    """Convert SDK and NumPy values to finite JSON-compatible values."""

    if isinstance(value, np.ndarray):
        return _json_value(value.tolist())
    if isinstance(value, (list, tuple)):
        return [_json_value(item) for item in value]
    if isinstance(value, (np.integer,)):
        return int(value)
    if isinstance(value, (np.floating, float)):
        number = float(value)
        return number if math.isfinite(number) else None
    if isinstance(value, (np.bool_, bool)):
        return bool(value)
    try:
        number = float(value)
    except (TypeError, ValueError):
        return str(value)
    return number if math.isfinite(number) else None


def _timestamp_ns(zed: Any, sl: Any) -> int:
    stamp = zed.get_timestamp(sl.TIME_REFERENCE.IMAGE)
    if hasattr(stamp, "get_nanoseconds"):
        return int(stamp.get_nanoseconds())
    if hasattr(stamp, "data_ns"):
        return int(stamp.data_ns)
    raise RuntimeError("ZED SDK did not return the image timestamp in nanoseconds.")


def _enum_name(value: Any) -> str:
    name = getattr(value, "name", None)
    return str(name if name is not None else value)


def _bbox_xyxy(body: Any) -> list[float]:
    raw = getattr(body, "bounding_box_2d", None)
    if raw is None:
        return []
    points = np.asarray(raw, dtype=float)
    if points.size < 8:
        return []
    points = points.reshape(-1, 2)
    finite = points[np.isfinite(points).all(axis=1)]
    if len(finite) < 2:
        return []
    return [
        float(finite[:, 0].min()), float(finite[:, 1].min()),
        float(finite[:, 0].max()), float(finite[:, 1].max()),
    ]


def _body_mask(body: Any, shape: tuple[int, int]) -> Optional[np.ndarray]:
    raw = getattr(body, "mask", None)
    if raw is None:
        return None
    try:
        mask = np.asarray(raw)
        if mask.ndim == 3:
            mask = np.squeeze(mask)
        if mask.ndim != 2:
            return None
        if mask.shape != shape:
            mask = cv2.resize(mask.astype(np.uint8), (shape[1], shape[0]), interpolation=cv2.INTER_NEAREST)
        return mask > 0
    except (TypeError, ValueError, cv2.error):
        return None


def _depth_summary(
    depth_m: np.ndarray,
    bbox: Sequence[float],
    intrinsics: dict[str, float],
    body_mask: Optional[np.ndarray],
    pelvis_pixel: Optional[Sequence[float]],
) -> dict[str, Any]:
    height, width = depth_m.shape
    if len(bbox) != 4 and body_mask is None:
        summary = {
            "depth_valid_pixels": 0, "depth_median_m": None,
            "depth_p10_m": None, "depth_p90_m": None,
            "depth_x_m": None, "depth_y_m": None, "depth_z_m": None,
            "depth_mask_used": False,
        }
        summary.update(_pelvis_depth(depth_m, intrinsics, body_mask, pelvis_pixel))
        return summary
    if len(bbox) == 4:
        x0, y0, x1, y1 = bbox
        left = max(0, min(width - 1, int(math.floor(x0))))
        top = max(0, min(height - 1, int(math.floor(y0))))
        right = max(left + 1, min(width, int(math.ceil(x1))))
        bottom = max(top + 1, min(height, int(math.ceil(y1))))
    else:
        left, top, right, bottom = 0, 0, width, height

    roi = depth_m[top:bottom, left:right]
    valid = np.isfinite(roi) & (roi > 0.2) & (roi < 100.0)
    mask_used = False
    if body_mask is not None:
        valid &= body_mask[top:bottom, left:right]
        mask_used = True
    # A segmentation mask can be incomplete in SDK versions/builds. Fall back
    # to the person bounding box rather than losing every depth sample.
    if int(valid.sum()) < 8 and body_mask is not None:
        valid = np.isfinite(roi) & (roi > 0.2) & (roi < 100.0)
        mask_used = False
    if int(valid.sum()) < 8:
        summary = {
            "depth_valid_pixels": int(valid.sum()), "depth_median_m": None,
            "depth_p10_m": None, "depth_p90_m": None,
            "depth_x_m": None, "depth_y_m": None, "depth_z_m": None,
            "depth_mask_used": mask_used,
        }
        summary.update(_pelvis_depth(depth_m, intrinsics, body_mask, pelvis_pixel))
        return summary

    yy, xx = np.where(valid)
    z_values = roi[yy, xx].astype(np.float64)
    z_m = float(np.median(z_values))
    u = float(np.median(xx + left))
    v = float(np.median(yy + top))
    summary = {
        "depth_valid_pixels": int(len(z_values)),
        "depth_median_m": z_m,
        "depth_p10_m": float(np.percentile(z_values, 10)),
        "depth_p90_m": float(np.percentile(z_values, 90)),
        "depth_x_m": (u - intrinsics["cx"]) * z_m / intrinsics["fx"],
        "depth_y_m": (v - intrinsics["cy"]) * z_m / intrinsics["fy"],
        "depth_z_m": z_m,
        "depth_mask_used": mask_used,
    }
    summary.update(_pelvis_depth(depth_m, intrinsics, body_mask, pelvis_pixel))
    return summary


def _pelvis_depth(
    depth_m: np.ndarray,
    intrinsics: dict[str, float],
    body_mask: Optional[np.ndarray],
    pelvis_pixel: Optional[Sequence[float]],
) -> dict[str, Any]:
    """Read robust depth near the pelvis pixel and back-project that image ray."""

    empty = {
        "pelvis_depth_valid_pixels": 0, "pelvis_depth_m": None,
        "pelvis_depth_x_m": None, "pelvis_depth_y_m": None, "pelvis_depth_z_m": None,
    }
    if pelvis_pixel is None or len(pelvis_pixel) < 2:
        return empty
    u, v = float(pelvis_pixel[0]), float(pelvis_pixel[1])
    height, width = depth_m.shape
    if not math.isfinite(u) or not math.isfinite(v) or not (0 <= u < width and 0 <= v < height):
        return empty
    x, y, radius = int(round(u)), int(round(v)), 5
    x0, x1 = max(0, x - radius), min(width, x + radius + 1)
    y0, y1 = max(0, y - radius), min(height, y + radius + 1)
    roi = depth_m[y0:y1, x0:x1]
    valid = np.isfinite(roi) & (roi > 0.2) & (roi < 100.0)
    if body_mask is not None:
        masked = valid & body_mask[y0:y1, x0:x1]
        if int(masked.sum()) >= 3:
            valid = masked
    if int(valid.sum()) < 3:
        empty["pelvis_depth_valid_pixels"] = int(valid.sum())
        return empty
    z_m = float(np.median(roi[valid]))
    return {
        "pelvis_depth_valid_pixels": int(valid.sum()),
        "pelvis_depth_m": z_m,
        "pelvis_depth_x_m": (u - intrinsics["cx"]) * z_m / intrinsics["fx"],
        "pelvis_depth_y_m": (v - intrinsics["cy"]) * z_m / intrinsics["fy"],
        "pelvis_depth_z_m": z_m,
    }


def _array_attr(body: Any, name: str) -> Any:
    value = getattr(body, name, None)
    if value is None:
        return []
    try:
        return _json_value(np.asarray(value))
    except (TypeError, ValueError):
        return _json_value(value)


def _draw_preview(image: np.ndarray, bodies: Sequence[Any], body_names: Sequence[str]) -> np.ndarray:
    canvas = image.copy()
    for body in bodies:
        bbox = _bbox_xyxy(body)
        body_id = getattr(body, "id", -1)
        confidence = getattr(body, "confidence", 0.0)
        if bbox:
            x0, y0, x1, y1 = [int(round(value)) for value in bbox]
            cv2.rectangle(canvas, (x0, y0), (x1, y1), (0, 220, 60), 2)
            cv2.putText(canvas, f"ID {body_id}  {float(confidence):.0f}%", (x0, max(20, y0 - 8)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 220, 60), 2, cv2.LINE_AA)
        points = np.asarray(getattr(body, "keypoint_2d", []), dtype=float)
        if points.ndim != 2 or points.shape[1] < 2:
            continue
        points = points[:, :2]
        for first, second in BODY_34_EDGES:
            if first >= len(points) or second >= len(points):
                continue
            a, b = points[first], points[second]
            if np.isfinite(a).all() and np.isfinite(b).all() and min(a[0], b[0], a[1], b[1]) >= 0:
                cv2.line(canvas, tuple(np.round(a).astype(int)), tuple(np.round(b).astype(int)),
                         (255, 170, 0), 2, cv2.LINE_AA)
        for index, point in enumerate(points):
            if np.isfinite(point).all() and min(point) >= 0:
                cv2.circle(canvas, tuple(np.round(point).astype(int)), 3, (0, 40, 255), -1, cv2.LINE_AA)
    return canvas


def _contact_sheet(preview_paths: list[Path], output: Path) -> None:
    if not preview_paths:
        return
    tile_width, tile_height, label_height, columns = 320, 180, 22, 4
    rows = math.ceil(len(preview_paths) / columns)
    canvas = np.zeros((rows * (tile_height + label_height), columns * tile_width, 3), dtype=np.uint8)
    for index, path in enumerate(preview_paths):
        image = cv2.imread(str(path), cv2.IMREAD_COLOR)
        if image is None:
            continue
        image = cv2.resize(image, (tile_width, tile_height), interpolation=cv2.INTER_AREA)
        x = (index % columns) * tile_width
        y = (index // columns) * (tile_height + label_height)
        canvas[y:y + tile_height, x:x + tile_width] = image
        cv2.rectangle(canvas, (x, y + tile_height), (x + tile_width - 1, y + tile_height + label_height - 1), (20, 20, 20), -1)
        cv2.putText(canvas, path.stem.replace("preview_", "t="), (x + 8, y + tile_height + 16),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.48, (255, 255, 255), 1, cv2.LINE_AA)
    if not cv2.imwrite(str(output), canvas, [cv2.IMWRITE_JPEG_QUALITY, 88]):
        raise RuntimeError(f"Could not write contact sheet: {output}")


def _sdk_body_names(sl: Any, count: int) -> list[str]:
    enum = getattr(sl, "BODY_PARTS_POSE_34", None)
    if enum is not None:
        names: list[tuple[int, str]] = []
        for name in dir(enum):
            if name.startswith("_") or name == "LAST":
                continue
            item = getattr(enum, name, None)
            try:
                index = int(sl.get_idx_34(item)) if hasattr(sl, "get_idx_34") else int(item)
            except (TypeError, ValueError, AttributeError):
                continue
            if 0 <= index < count:
                names.append((index, name))
        if len(names) >= min(count, 30):
            result = [f"JOINT_{index}" for index in range(count)]
            for index, name in names:
                result[index] = name
            return result
    return BODY_34_NAMES[:count] + [f"JOINT_{i}" for i in range(len(BODY_34_NAMES), count)]


def export_svo(
    svo_path: Path,
    output_dir: Path,
    *,
    max_depth_m: float,
    confidence_threshold: int,
    preview_period_s: float,
    max_frames: Optional[int],
) -> None:
    try:
        import pyzed.sl as sl
    except Exception as exc:
        raise RuntimeError(f"Could not import pyzed.sl: {exc}") from exc

    svo_path = svo_path.expanduser().resolve()
    if not svo_path.is_file():
        raise RuntimeError(f"SVO file not found: {svo_path}")
    if not math.isfinite(max_depth_m) or max_depth_m <= 0:
        raise ValueError("--max-depth-m must be positive.")
    if not math.isfinite(preview_period_s) or preview_period_s <= 0:
        raise ValueError("--preview-period-s must be positive.")
    if max_frames is not None and max_frames <= 0:
        raise ValueError("--max-frames must be positive when specified.")

    output_dir = output_dir.expanduser().resolve()
    if output_dir.exists() and any(output_dir.iterdir()):
        raise RuntimeError(f"Output directory is not empty; choose a new folder: {output_dir}")
    output_dir.mkdir(parents=True, exist_ok=True)
    preview_dir = output_dir / "preview_frames"

    zed = sl.Camera()
    init = sl.InitParameters()
    init.set_from_svo_file(str(svo_path))
    init.svo_real_time_mode = False
    init.coordinate_units = sl.UNIT.METER
    # Matches tracking/lidar_to_zed_left_eye.json: right, down, forward.
    init.coordinate_system = sl.COORDINATE_SYSTEM.IMAGE
    init.depth_mode = sl.DEPTH_MODE.NEURAL
    init.depth_maximum_distance = float(max_depth_m)

    status = zed.open(init)
    if status != sl.ERROR_CODE.SUCCESS:
        zed.close()
        raise RuntimeError(f"Could not open SVO2 with ZED SDK: {status}")

    body_enabled = False
    positional_enabled = False
    rows_frames: list[dict[str, Any]] = []
    preview_paths: list[Path] = []
    first_timestamp_ns: Optional[int] = None
    frame_index = 0
    image_mat, depth_mat = sl.Mat(), sl.Mat()
    bodies = sl.Bodies()
    runtime = sl.RuntimeParameters()
    runtime.measure3D_reference_frame = sl.REFERENCE_FRAME.CAMERA
    next_preview_s = 0.0
    body_names = BODY_34_NAMES
    try:
        camera_info = zed.get_camera_information()
        camera_config = camera_info.camera_configuration
        resolution = camera_config.resolution
        left = camera_config.calibration_parameters.left_cam
        intrinsics = {key: float(getattr(left, key)) for key in ("fx", "fy", "cx", "cy")}
        if intrinsics["fx"] <= 0 or intrinsics["fy"] <= 0:
            raise RuntimeError("ZED returned invalid camera intrinsics.")

        positional_params = sl.PositionalTrackingParameters()
        positional_params.set_as_static = True
        status = zed.enable_positional_tracking(positional_params)
        if status != sl.ERROR_CODE.SUCCESS:
            raise RuntimeError(f"Could not enable ZED positional tracking: {status}")
        positional_enabled = True

        body_params = sl.BodyTrackingParameters()
        body_params.enable_tracking = True
        body_params.enable_body_fitting = True
        body_params.enable_segmentation = True
        body_params.detection_model = sl.BODY_TRACKING_MODEL.HUMAN_BODY_ACCURATE
        body_params.body_format = sl.BODY_FORMAT.BODY_34
        body_params.max_range = float(max_depth_m)
        status = zed.enable_body_tracking(body_params)
        if status != sl.ERROR_CODE.SUCCESS:
            raise RuntimeError(f"Could not enable ZED body tracking: {status}")
        body_enabled = True
        preview_dir.mkdir()

        body_runtime = sl.BodyTrackingRuntimeParameters()
        body_runtime.detection_confidence_threshold = int(confidence_threshold)
        body_names = _sdk_body_names(sl, len(BODY_34_NAMES))
        fps = float(camera_config.fps)
        camera_json = {
            "source_svo": str(svo_path),
            "sdk_version": _sdk_version(),
            "camera_model": str(getattr(camera_info, "camera_model", "unknown")),
            "serial_number": int(getattr(camera_info, "serial_number", 0)),
            "fps": fps,
            "width": int(resolution.width), "height": int(resolution.height),
            **intrinsics,
            "coordinate_system": "IMAGE (x right, y down, z forward)",
            "coordinate_units": "meter",
            "body_format": "BODY_34",
            "keypoint_names": body_names,
            "body_fitting": True,
            "body_segmentation": True,
            "body_tracking_model": "HUMAN_BODY_ACCURATE",
            "depth_mode": "NEURAL",
            "max_depth_m": max_depth_m,
            "detection_confidence_threshold": confidence_threshold,
            "preview_period_s": preview_period_s,
        }

        csv_path = output_dir / "frames.csv"
        body_path = output_dir / "bodies.csv"
        frame_fields = ["frame_index", "image_timestamp_ns", "elapsed_s", "body_count"]
        body_fields = [
            "frame_index", "image_timestamp_ns", "elapsed_s", "body_id", "tracking_state",
            "action_state", "confidence", "position_camera_x_m", "position_camera_y_m",
            "position_camera_z_m", "velocity_camera_x_mps", "velocity_camera_y_mps",
            "velocity_camera_z_mps", "bounding_box_2d_json", "keypoints_2d_json",
            "keypoints_3d_camera_json", "keypoint_confidence_json",
            "global_root_orientation_xyzw_json", "local_position_per_joint_json",
            "local_orientation_per_joint_xyzw_json", "depth_valid_pixels", "depth_median_m",
            "depth_p10_m", "depth_p90_m", "depth_x_m", "depth_y_m", "depth_z_m",
            "depth_mask_used", "pelvis_depth_valid_pixels", "pelvis_depth_m",
            "pelvis_depth_x_m", "pelvis_depth_y_m", "pelvis_depth_z_m",
        ]
        with csv_path.open("w", newline="", encoding="utf-8-sig") as frame_stream, \
                body_path.open("w", newline="", encoding="utf-8-sig") as body_stream:
            frame_writer = csv.DictWriter(frame_stream, fieldnames=frame_fields)
            body_writer = csv.DictWriter(body_stream, fieldnames=body_fields)
            frame_writer.writeheader()
            body_writer.writeheader()

            while max_frames is None or frame_index < max_frames:
                grab_status = zed.grab(runtime)
                if grab_status == sl.ERROR_CODE.END_OF_SVOFILE_REACHED:
                    break
                if grab_status != sl.ERROR_CODE.SUCCESS:
                    raise RuntimeError(f"Failed reading SVO frame {frame_index}: {grab_status}")
                timestamp_ns = _timestamp_ns(zed, sl)
                if first_timestamp_ns is None:
                    first_timestamp_ns = timestamp_ns
                elapsed_s = (timestamp_ns - first_timestamp_ns) / 1_000_000_000.0
                status = zed.retrieve_bodies(bodies, body_runtime)
                if status != sl.ERROR_CODE.SUCCESS:
                    raise RuntimeError(f"Could not retrieve bodies at frame {frame_index}: {status}")
                body_list = list(getattr(bodies, "body_list", []))

                depth_m: Optional[np.ndarray] = None
                if body_list:
                    depth_mat.free(sl.MEM.CPU)
                    depth_status = zed.retrieve_measure(depth_mat, sl.MEASURE.DEPTH, sl.MEM.CPU)
                    if depth_status == sl.ERROR_CODE.SUCCESS:
                        depth_m = np.asarray(depth_mat.get_data(), dtype=np.float32).squeeze()
                        if depth_m.ndim != 2:
                            depth_m = None

                frame_writer.writerow({
                    "frame_index": frame_index,
                    "image_timestamp_ns": timestamp_ns,
                    "elapsed_s": f"{elapsed_s:.9f}",
                    "body_count": len(body_list),
                })
                for body in body_list:
                    bbox = _bbox_xyxy(body)
                    keypoints_2d = np.asarray(getattr(body, "keypoint_2d", []), dtype=float)
                    pelvis_pixel = (
                        keypoints_2d[0, :2]
                        if keypoints_2d.ndim == 2 and keypoints_2d.shape[0] > 0
                        and keypoints_2d.shape[1] >= 2 and np.isfinite(keypoints_2d[0, :2]).all()
                        else None
                    )
                    mask = _body_mask(body, depth_m.shape) if depth_m is not None else None
                    depth_summary = (
                        _depth_summary(depth_m, bbox, intrinsics, mask, pelvis_pixel)
                        if depth_m is not None else {
                            "depth_valid_pixels": 0, "depth_median_m": None,
                            "depth_p10_m": None, "depth_p90_m": None,
                            "depth_x_m": None, "depth_y_m": None, "depth_z_m": None,
                            "depth_mask_used": False,
                            "pelvis_depth_valid_pixels": 0, "pelvis_depth_m": None,
                            "pelvis_depth_x_m": None, "pelvis_depth_y_m": None,
                            "pelvis_depth_z_m": None,
                        }
                    )
                    position = np.asarray(getattr(body, "position", []), dtype=float).reshape(-1)
                    velocity = np.asarray(getattr(body, "velocity", []), dtype=float).reshape(-1)
                    position = np.pad(position[:3], (0, max(0, 3 - len(position[:3]))), constant_values=np.nan)
                    velocity = np.pad(velocity[:3], (0, max(0, 3 - len(velocity[:3]))), constant_values=np.nan)
                    row: dict[str, Any] = {
                        "frame_index": frame_index,
                        "image_timestamp_ns": timestamp_ns,
                        "elapsed_s": f"{elapsed_s:.9f}",
                        "body_id": getattr(body, "id", -1),
                        "tracking_state": _enum_name(getattr(body, "tracking_state", "unknown")),
                        "action_state": _enum_name(getattr(body, "action_state", "unknown")),
                        "confidence": _json_value(getattr(body, "confidence", None)),
                        "position_camera_x_m": _json_value(position[0]),
                        "position_camera_y_m": _json_value(position[1]),
                        "position_camera_z_m": _json_value(position[2]),
                        "velocity_camera_x_mps": _json_value(velocity[0]),
                        "velocity_camera_y_mps": _json_value(velocity[1]),
                        "velocity_camera_z_mps": _json_value(velocity[2]),
                        "bounding_box_2d_json": json.dumps(bbox, allow_nan=False),
                        "keypoints_2d_json": json.dumps(_array_attr(body, "keypoint_2d"), allow_nan=False),
                        "keypoints_3d_camera_json": json.dumps(_array_attr(body, "keypoint"), allow_nan=False),
                        "keypoint_confidence_json": json.dumps(_array_attr(body, "keypoint_confidence"), allow_nan=False),
                        "global_root_orientation_xyzw_json": json.dumps(_array_attr(body, "global_root_orientation"), allow_nan=False),
                        "local_position_per_joint_json": json.dumps(_array_attr(body, "local_position_per_joint"), allow_nan=False),
                        "local_orientation_per_joint_xyzw_json": json.dumps(_array_attr(body, "local_orientation_per_joint"), allow_nan=False),
                        **depth_summary,
                    }
                    body_writer.writerow(row)

                if elapsed_s + 1e-9 >= next_preview_s:
                    image_mat.free(sl.MEM.CPU)
                    image_status = zed.retrieve_image(image_mat, sl.VIEW.LEFT, sl.MEM.CPU)
                    if image_status == sl.ERROR_CODE.SUCCESS:
                        pixels = np.asarray(image_mat.get_data())
                        if pixels.ndim == 3 and pixels.shape[2] == 4:
                            preview = cv2.cvtColor(pixels, cv2.COLOR_BGRA2BGR)
                        elif pixels.ndim == 3 and pixels.shape[2] == 3:
                            preview = pixels.copy()
                        else:
                            preview = None
                        if preview is not None:
                            preview = _draw_preview(preview, body_list, body_names)
                            cv2.putText(preview, f"t={elapsed_s:.2f}s  f={frame_index}", (16, 28),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2, cv2.LINE_AA)
                            preview_path = preview_dir / f"preview_{elapsed_s:08.3f}s.jpg"
                            if not cv2.imwrite(str(preview_path), preview, [cv2.IMWRITE_JPEG_QUALITY, 88]):
                                raise RuntimeError(f"Could not write preview image: {preview_path}")
                            preview_paths.append(preview_path)
                    next_preview_s += preview_period_s

                frame_index += 1
                if frame_index % max(1, int(round(fps * 5))) == 0:
                    print(f"Read {frame_index} SVO frames; exported {len(preview_paths)} preview images.")

        if frame_index == 0:
            raise RuntimeError("No SVO frames were read.")
        _contact_sheet(preview_paths, output_dir / "contact_sheet.jpg")
        camera_json.update({
            "total_svo_frames_read": frame_index,
            "preview_frames": len(preview_paths),
            "first_image_timestamp_ns": first_timestamp_ns,
            "last_image_timestamp_ns": timestamp_ns,
            "duration_s": (timestamp_ns - int(first_timestamp_ns)) / 1_000_000_000.0,
            "frames_csv": csv_path.name,
            "bodies_csv": body_path.name,
            "contact_sheet": "contact_sheet.jpg",
        })
        (output_dir / "camera.json").write_text(
            json.dumps(camera_json, ensure_ascii=False, indent=2, allow_nan=False) + "\n",
            encoding="utf-8",
        )
        print(f"Exported {frame_index} frames and body observations to {output_dir}")
    finally:
        if body_enabled:
            zed.disable_body_tracking()
        if positional_enabled:
            zed.disable_positional_tracking()
        image_mat.free(sl.MEM.CPU)
        depth_mat.free(sl.MEM.CPU)
        zed.close()


def _sdk_version() -> str:
    try:
        return importlib.metadata.version("pyzed")
    except importlib.metadata.PackageNotFoundError:
        return "unknown"


def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("svo", type=Path, help="SVO/SVO2 file; use the external PC's local path")
    parser.add_argument("--output", type=Path, required=True, help="new, separate output directory")
    parser.add_argument("--max-depth-m", type=float, default=30.0,
                        help="maximum SDK depth/body range in meters (default: 30)")
    parser.add_argument("--confidence", type=int, default=35,
                        help="minimum SDK body detection confidence, 1-100 (default: 35)")
    parser.add_argument("--preview-period-s", type=float, default=0.5,
                        help="interval between annotated preview images (default: 0.5 s)")
    parser.add_argument("--max-frames", type=int, default=None,
                        help="optional frame limit for a short manual run")
    args = parser.parse_args(argv)
    if not 1 <= args.confidence <= 100:
        parser.error("--confidence must be between 1 and 100")
    return args


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = parse_args(argv)
    try:
        export_svo(args.svo, args.output, max_depth_m=args.max_depth_m,
                   confidence_threshold=args.confidence, preview_period_s=args.preview_period_s,
                   max_frames=args.max_frames)
    except (OSError, RuntimeError, ValueError, cv2.error) as exc:
        print(f"Error: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
