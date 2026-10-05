"""Still-image and ZED SVO2 camera input adapters."""

from __future__ import annotations

from dataclasses import dataclass
import json
from pathlib import Path
from typing import Any

import cv2
import numpy as np
import yaml

from ..projection import CameraModel


@dataclass(frozen=True)
class CameraFrame:
    image_rgb: np.ndarray
    camera_model: CameraModel
    source: str
    frame_index: int = 0
    timestamp_ns: int | None = None


def load_rgb_image(path: Path) -> np.ndarray:
    image = cv2.imread(str(path.expanduser()), cv2.IMREAD_COLOR)
    if image is None:
        raise ValueError(f"Could not read image: {path}")
    return cv2.cvtColor(image, cv2.COLOR_BGR2RGB)


def _number(value: Any, name: str) -> float:
    if isinstance(value, (list, tuple, dict)):
        raise ValueError(f"Camera intrinsics field {name!r} must be numeric.")
    result = float(value)
    if not np.isfinite(result):
        raise ValueError(f"Camera intrinsics field {name!r} must be finite.")
    return result


def load_camera_model(path: Path, width: int, height: int) -> CameraModel:
    """Load intrinsics from a small YAML/JSON file; scale K to the image size."""

    raw = path.expanduser().read_text(encoding="utf-8")
    if path.suffix.lower() == ".json":
        value = json.loads(raw)
    else:
        # OpenCV FileStorage emits a version directive and a custom matrix tag;
        # its matrix payload is still ordinary YAML once those markers are removed.
        normalized_yaml = "\n".join(
            line for line in raw.splitlines() if not line.strip().startswith("%YAML:")
        ).replace("!!opencv-matrix", "")
        value = yaml.safe_load(normalized_yaml)
    if not isinstance(value, dict):
        raise ValueError("Camera intrinsics file must contain a mapping.")
    source_width = int(value.get("width", value.get("image_width", width)))
    source_height = int(value.get("height", value.get("image_height", height)))
    matrix_key = next((key for key in ("camera_matrix", "K", "k") if key in value), None)
    if matrix_key:
        matrix = value[matrix_key]
        if isinstance(matrix, dict) and "data" in matrix:
            matrix = matrix["data"]
        matrix = np.asarray(matrix, dtype=float).reshape(3, 3)
        fx, fy, cx, cy = matrix[0, 0], matrix[1, 1], matrix[0, 2], matrix[1, 2]
    else:
        fx, fy, cx, cy = (_number(value[k], k) for k in ("fx", "fy", "cx", "cy"))
    sx, sy = width / source_width, height / source_height
    distortion = value.get(
        "distortion",
        value.get("distortion_coefficients", value.get("D", value.get("d", []))),
    )
    if isinstance(distortion, dict) and "data" in distortion:
        distortion = distortion["data"]
    distortion = np.asarray(distortion, dtype=float).reshape(-1)
    rectified = bool(value.get("rectified", not len(distortion) or not np.any(distortion)))
    if rectified:
        distortion = None
    return CameraModel(
        width=width,
        height=height,
        fx=float(fx) * sx,
        fy=float(fy) * sy,
        cx=float(cx) * sx,
        cy=float(cy) * sy,
        distortion=distortion,
        rectified=rectified,
        distortion_model=(
            "FISHEYE"
            if str(value.get("distortion_model", "RAD_TAN")).lower() in {"equidistant", "fisheye"}
            else "RAD_TAN"
        ),
    )


def _zed_intrinsics(camera: object, image: np.ndarray) -> CameraModel:
    info = camera.get_camera_information()
    configuration = info.camera_configuration
    calibration = configuration.calibration_parameters
    left = calibration.left_cam
    height, width = image.shape[:2]
    size = getattr(left, "image_size", None)
    if size is None:
        size = getattr(configuration, "resolution", None)
    sdk_width = int(getattr(size, "width", width))
    sdk_height = int(getattr(size, "height", height))
    sx, sy = width / max(1, sdk_width), height / max(1, sdk_height)
    # VIEW.LEFT is rectified; these matching SDK parameters have zero distortion.
    return CameraModel(
        width=width,
        height=height,
        fx=float(left.fx) * sx,
        fy=float(left.fy) * sy,
        cx=float(left.cx) * sx,
        cy=float(left.cy) * sy,
        distortion=None,
        rectified=True,
        distortion_model="PINHOLE",
    )


def load_svo_frame(path: Path, frame_index: int = 0) -> CameraFrame:
    """Read one rectified left image and its matching rectified K from SVO/SVO2."""

    if frame_index < 0:
        raise ValueError("SVO frame index must be non-negative.")
    try:
        import pyzed.sl as sl
    except Exception as exc:
        raise RuntimeError(
            "The ZED Python SDK could not be imported. Install/source ZED SDK 5.4.0 "
            "and verify its NVIDIA runtime libraries before loading SVO files. "
            f"Import error: {exc}"
        ) from exc

    camera = sl.Camera()
    init = sl.InitParameters()
    if hasattr(init, "set_from_svo_file"):
        init.set_from_svo_file(str(path.expanduser().resolve()))
    elif hasattr(init, "input") and hasattr(init.input, "set_from_svo_file"):
        init.input.set_from_svo_file(str(path.expanduser().resolve()))
    else:
        raise RuntimeError("This ZED SDK Python binding does not expose SVO playback setup.")
    status = camera.open(init)
    if status != sl.ERROR_CODE.SUCCESS:
        raise RuntimeError(f"ZED SDK could not open SVO: {status}")
    try:
        image_mat = sl.Mat()
        for current in range(frame_index + 1):
            status = camera.grab()
            if status != sl.ERROR_CODE.SUCCESS:
                raise RuntimeError(f"Could not read SVO frame {current}: {status}")
        camera.retrieve_image(image_mat, sl.VIEW.LEFT, sl.MEM.CPU)
        pixels = np.asarray(image_mat.get_data()).copy()
        if pixels.ndim == 3 and pixels.shape[2] == 4:
            image_rgb = cv2.cvtColor(pixels, cv2.COLOR_BGRA2RGB)
        elif pixels.ndim == 3 and pixels.shape[2] == 3:
            image_rgb = cv2.cvtColor(pixels, cv2.COLOR_BGR2RGB)
        else:
            raise RuntimeError(f"Unexpected ZED image buffer shape: {pixels.shape}")
        camera_model = _zed_intrinsics(camera, image_rgb)
        timestamp_ns = None
        try:
            timestamp_ns = int(camera.get_timestamp(sl.TIME_REFERENCE.IMAGE).get_nanoseconds())
        except (AttributeError, TypeError, ValueError):
            pass
        return CameraFrame(image_rgb, camera_model, str(path.resolve()), frame_index, timestamp_ns)
    finally:
        camera.close()


def load_image_and_intrinsics(image_path: Path, intrinsics_path: Path) -> CameraFrame:
    image = load_rgb_image(image_path)
    camera = load_camera_model(intrinsics_path, image.shape[1], image.shape[0])
    return CameraFrame(image, camera, str(image_path.expanduser().resolve()))
