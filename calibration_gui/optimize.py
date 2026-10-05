"""Robust 2D-3D reprojection refinement from explicit correspondences."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from scipy.optimize import least_squares

from .projection import CameraModel, project_camera_points
from .transform import Extrinsic


@dataclass(frozen=True)
class ReprojectionResult:
    extrinsic: Extrinsic
    rms_px: float
    median_px: float
    errors_px: np.ndarray
    optimizer_message: str


def reprojection_errors(
    points_lidar: np.ndarray,
    pixels: np.ndarray,
    extrinsic: Extrinsic,
    camera: CameraModel,
) -> np.ndarray:
    points = np.asarray(points_lidar, dtype=float).reshape(-1, 3)
    observed = np.asarray(pixels, dtype=float).reshape(-1, 2)
    if len(points) != len(observed):
        raise ValueError("3D point and 2D pixel correspondence counts must match.")
    projection = project_camera_points(extrinsic.transform_points(points), camera, image_bounds=False)
    predicted = projection.uv.copy()
    invalid = ~np.isfinite(predicted).all(axis=1)
    predicted[invalid] = np.array([camera.width * 2.0, camera.height * 2.0])
    return np.linalg.norm(predicted - observed, axis=1)


def refine_extrinsic(
    points_lidar: np.ndarray,
    pixels: np.ndarray,
    initial: Extrinsic,
    camera: CameraModel,
    *,
    min_correspondences: int = 4,
    max_nfev: int = 300,
) -> ReprojectionResult:
    """Optimize translation and RPY to minimize robust pixel reprojection residuals.

    Correspondences must be manually identified or otherwise unambiguous. The
    objective is reprojection error; color similarity is never treated as a metric.
    """

    points = np.asarray(points_lidar, dtype=float).reshape(-1, 3)
    observed = np.asarray(pixels, dtype=float).reshape(-1, 2)
    if len(points) != len(observed):
        raise ValueError("3D point and 2D pixel correspondence counts must match.")
    if len(points) < min_correspondences:
        raise ValueError(f"At least {min_correspondences} correspondences are required.")
    if not np.isfinite(points).all() or not np.isfinite(observed).all():
        raise ValueError("Correspondences must be finite.")

    initial_parameters = np.r_[initial.translation_m, initial.rpy_rad]

    def residuals(parameters: np.ndarray) -> np.ndarray:
        candidate = Extrinsic(
            translation_m=parameters[:3],
            rpy_rad=parameters[3:],
            source_frame=initial.source_frame,
            target_frame=initial.target_frame,
        )
        camera_points = candidate.transform_points(points)
        projection = project_camera_points(camera_points, camera, image_bounds=False)
        residual = projection.uv - observed
        invalid = ~projection.valid
        residual[invalid] = np.array([camera.width * 2.0, camera.height * 2.0])
        return residual.reshape(-1)

    optimized = least_squares(
        residuals,
        initial_parameters,
        loss="soft_l1",
        f_scale=3.0,
        max_nfev=max_nfev,
        x_scale=np.array([0.1, 0.1, 0.1, 0.1, 0.1, 0.1]),
    )
    result_extrinsic = Extrinsic(
        translation_m=optimized.x[:3],
        rpy_rad=optimized.x[3:],
        source_frame=initial.source_frame,
        target_frame=initial.target_frame,
    )
    errors = reprojection_errors(points, observed, result_extrinsic, camera)
    return ReprojectionResult(
        extrinsic=result_extrinsic,
        rms_px=float(np.sqrt(np.mean(np.square(errors)))),
        median_px=float(np.median(errors)),
        errors_px=errors,
        optimizer_message=str(optimized.message),
    )
