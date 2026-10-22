"""Pinhole camera projection, point visibility, image overlay, and RGB sampling."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import cv2
import numpy as np

from .transform import Extrinsic


@dataclass(frozen=True)
class CameraModel:
    width: int
    height: int
    fx: float
    fy: float
    cx: float
    cy: float
    distortion: Optional[np.ndarray] = None
    rectified: bool = True
    distortion_model: str = "RAD_TAN"

    def __post_init__(self) -> None:
        if self.width <= 0 or self.height <= 0 or self.fx <= 0 or self.fy <= 0:
            raise ValueError("Camera size and focal lengths must be positive.")
        if not np.isfinite([self.fx, self.fy, self.cx, self.cy]).all():
            raise ValueError("Camera intrinsics must be finite.")
        if self.rectified and self.distortion is not None and np.any(self.distortion):
            raise ValueError("Rectified camera parameters must not have non-zero distortion.")

    @property
    def matrix(self) -> np.ndarray:
        return np.array(
            [[self.fx, 0.0, self.cx], [0.0, self.fy, self.cy], [0.0, 0.0, 1.0]],
            dtype=float,
        )


@dataclass(frozen=True)
class Projection:
    uv: np.ndarray
    depth_m: np.ndarray
    valid: np.ndarray
    camera_points: np.ndarray


def project_camera_points(
    points_camera: np.ndarray,
    camera: CameraModel,
    *,
    min_distance_m: float = 0.0,
    max_distance_m: float = float("inf"),
    image_bounds: bool = True,
) -> Projection:
    """Project optical-frame XYZ points; discard points behind camera and invalids."""

    points = np.asarray(points_camera, dtype=float)
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError("Points must be an N x 3 array.")
    n = len(points)
    uv = np.full((n, 2), np.nan, dtype=float)
    depth = points[:, 2].copy()
    finite = np.isfinite(points).all(axis=1)
    valid = finite & (depth > 1e-6) & (depth >= min_distance_m) & (depth <= max_distance_m)
    indexes = np.flatnonzero(valid)
    if len(indexes):
        if camera.rectified or camera.distortion is None:
            z = points[indexes, 2]
            uv[indexes, 0] = camera.fx * points[indexes, 0] / z + camera.cx
            uv[indexes, 1] = camera.fy * points[indexes, 1] / z + camera.cy
        else:
            if camera.distortion_model.upper() == "FISHEYE":
                obj = points[indexes].reshape(-1, 1, 3).astype(np.float64)
                image, _ = cv2.fisheye.projectPoints(
                    obj,
                    np.zeros(3),
                    np.zeros(3),
                    camera.matrix,
                    np.asarray(camera.distortion, dtype=float).reshape(-1, 1),
                )
            else:
                image, _ = cv2.projectPoints(
                    points[indexes], np.zeros(3), np.zeros(3), camera.matrix,
                    np.asarray(camera.distortion, dtype=float).reshape(-1, 1),
                )
            uv[indexes] = image.reshape(-1, 2)
    valid &= np.isfinite(uv).all(axis=1)
    if image_bounds:
        valid &= (
            (uv[:, 0] >= 0.0)
            & (uv[:, 0] < camera.width)
            & (uv[:, 1] >= 0.0)
            & (uv[:, 1] < camera.height)
        )
    return Projection(uv, depth, valid, points)


def project_lidar_points(
    points_lidar: np.ndarray,
    extrinsic: Extrinsic,
    camera: CameraModel,
    *,
    min_distance_m: float = 0.0,
    max_distance_m: float = float("inf"),
) -> Projection:
    camera_points = extrinsic.transform_points(points_lidar)
    return project_camera_points(
        camera_points,
        camera,
        min_distance_m=min_distance_m,
        max_distance_m=max_distance_m,
    )


def zbuffer_visible_mask(projection: Projection, image_width: int, image_height: int) -> np.ndarray:
    """Mark one nearest LiDAR return per image pixel as colorizable/visible."""

    visible = np.zeros(len(projection.valid), dtype=bool)
    indexes = np.flatnonzero(projection.valid)
    if not len(indexes):
        return visible
    pixels = np.rint(projection.uv[indexes]).astype(np.int64)
    inside = (
        (pixels[:, 0] >= 0) & (pixels[:, 0] < image_width)
        & (pixels[:, 1] >= 0) & (pixels[:, 1] < image_height)
    )
    indexes, pixels = indexes[inside], pixels[inside]
    if not len(indexes):
        return visible
    flat = pixels[:, 1] * image_width + pixels[:, 0]
    order = np.lexsort((projection.depth_m[indexes], flat))
    sorted_flat = flat[order]
    first = np.r_[True, sorted_flat[1:] != sorted_flat[:-1]]
    visible[indexes[order[first]]] = True
    return visible


def depth_colors(depth_m: np.ndarray, valid: np.ndarray) -> np.ndarray:
    """Return an RGB Turbo colormap with near points warm and far points cool."""

    depth = np.asarray(depth_m, dtype=float)
    mask = np.asarray(valid, dtype=bool)
    colors = np.full((len(depth), 3), 0.35, dtype=float)
    if not mask.any():
        return colors
    low, high = np.percentile(depth[mask], [2.0, 98.0])
    if high <= low:
        high = low + 1e-6
    gray = np.clip((depth - low) / (high - low), 0.0, 1.0)
    mapped = cv2.applyColorMap(np.rint(gray * 255).astype(np.uint8).reshape(-1, 1), cv2.COLORMAP_TURBO)
    colors[mask] = mapped[mask, 0, ::-1].astype(float) / 255.0
    return colors


def intensity_colors(intensity: Optional[np.ndarray], count: int) -> np.ndarray:
    """Return normalized intensity colors or a neutral gray when unavailable."""

    colors = np.full((count, 3), 0.35, dtype=float)
    if intensity is None:
        return colors
    values = np.asarray(intensity, dtype=float).reshape(-1)
    finite = np.isfinite(values)
    if len(values) != count or not finite.any():
        return colors
    low, high = np.percentile(values[finite], [2.0, 98.0])
    if high <= low:
        high = low + 1e-6
    normalized = np.clip((values - low) / (high - low), 0.0, 1.0)
    mapped = cv2.applyColorMap(np.rint(normalized * 255).astype(np.uint8).reshape(-1, 1), cv2.COLORMAP_TURBO)
    colors[finite] = mapped[finite, 0, ::-1].astype(float) / 255.0
    return colors


def sample_rgb_colors(
    image_rgb: np.ndarray, projection: Projection, *, zbuffer: bool = True
) -> tuple[np.ndarray, np.ndarray]:
    """Sample camera RGB for visible returns. Out-of-view/occluded points stay gray."""

    image = np.asarray(image_rgb)
    if image.ndim != 3 or image.shape[2] < 3:
        raise ValueError("Camera image must be RGB or RGBA.")
    height, width = image.shape[:2]
    colors = np.full((len(projection.valid), 3), 0.35, dtype=float)
    visible = (
        zbuffer_visible_mask(projection, width, height)
        if zbuffer
        else projection.valid.copy()
    )
    indexes = np.flatnonzero(visible)
    if len(indexes):
        pixels = np.rint(projection.uv[indexes]).astype(np.int64)
        pixels[:, 0] = np.clip(pixels[:, 0], 0, width - 1)
        pixels[:, 1] = np.clip(pixels[:, 1], 0, height - 1)
        colors[indexes] = image[pixels[:, 1], pixels[:, 0], :3].astype(float) / 255.0
    return colors, visible


def overlay_projection(
    image_rgb: np.ndarray,
    projection: Projection,
    colors_rgb: np.ndarray,
    *,
    point_size: int = 3,
    zbuffer: bool = True,
) -> np.ndarray:
    """Paint projected points over an RGB image for the GUI's 2D panel."""

    image = np.asarray(image_rgb)
    output = image[:, :, :3].copy()
    visible = (
        zbuffer_visible_mask(projection, image.shape[1], image.shape[0])
        if zbuffer
        else projection.valid
    )
    for index in np.flatnonzero(visible):
        u, v = np.rint(projection.uv[index]).astype(int)
        rgb = np.clip(np.rint(np.asarray(colors_rgb[index]) * 255), 0, 255).astype(np.uint8)
        cv2.circle(output, (u, v), max(1, int(point_size)), tuple(int(c) for c in rgb), -1, cv2.LINE_AA)
    return output
