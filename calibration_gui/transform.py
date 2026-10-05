"""Coordinate-frame conventions and rigid transforms for the calibration app."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Iterable

import numpy as np


OPTICAL_FRAME = "zed_left_optical_frame"


def rotation_matrix_rpy(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """Return active Rz(yaw) @ Ry(pitch) @ Rx(roll), mapping source axes to target."""

    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rx = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]])
    ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]])
    rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])
    return rz @ ry @ rx


def matrix_to_rpy(rotation: np.ndarray) -> np.ndarray:
    """Invert Rz(yaw) @ Ry(pitch) @ Rx(roll), returning radians."""

    r = np.asarray(rotation, dtype=float).reshape(3, 3)
    pitch = math.asin(float(np.clip(-r[2, 0], -1.0, 1.0)))
    if abs(math.cos(pitch)) > 1e-8:
        roll = math.atan2(float(r[2, 1]), float(r[2, 2]))
        yaw = math.atan2(float(r[1, 0]), float(r[0, 0]))
    else:
        roll = math.atan2(float(-r[1, 2]), float(r[1, 1]))
        yaw = 0.0
    return np.array([roll, pitch, yaw], dtype=float)


@dataclass(frozen=True)
class Extrinsic:
    """Rigid transform ``target_from_source``; translation is expressed in target axes.

    The app uses metres and radians internally. Rotation is active, right-handed,
    and composed as Rz(yaw) @ Ry(pitch) @ Rx(roll).
    """

    translation_m: np.ndarray
    rpy_rad: np.ndarray
    source_frame: str = "livox_frame"
    target_frame: str = OPTICAL_FRAME

    def __post_init__(self) -> None:
        t = np.asarray(self.translation_m, dtype=float).reshape(-1)
        rpy = np.asarray(self.rpy_rad, dtype=float).reshape(-1)
        if t.size != 3 or rpy.size != 3:
            raise ValueError("Translation and RPY must each have three values.")
        if not np.isfinite(t).all() or not np.isfinite(rpy).all():
            raise ValueError("Transform values must be finite.")
        object.__setattr__(self, "translation_m", t)
        object.__setattr__(self, "rpy_rad", rpy)

    @property
    def rotation(self) -> np.ndarray:
        return rotation_matrix_rpy(*self.rpy_rad)

    @property
    def matrix(self) -> np.ndarray:
        matrix = np.eye(4, dtype=float)
        matrix[:3, :3] = self.rotation
        matrix[:3, 3] = self.translation_m
        return matrix

    def transform_points(self, points: np.ndarray) -> np.ndarray:
        values = np.asarray(points, dtype=float)
        if values.ndim != 2 or values.shape[1] != 3:
            raise ValueError("Points must be an N x 3 array.")
        return values @ self.rotation.T + self.translation_m

    def transform_vectors(self, vectors: np.ndarray) -> np.ndarray:
        values = np.asarray(vectors, dtype=float)
        if values.ndim != 2 or values.shape[1] != 3:
            raise ValueError("Vectors must be an N x 3 array.")
        return values @ self.rotation.T

    @classmethod
    def from_matrix(cls, matrix: np.ndarray, source_frame: str, target_frame: str) -> "Extrinsic":
        value = np.asarray(matrix, dtype=float).reshape(4, 4)
        if not np.allclose(value[3], [0.0, 0.0, 0.0, 1.0], atol=1e-8):
            raise ValueError("Homogeneous transform last row must be [0, 0, 0, 1].")
        rotation = value[:3, :3]
        if not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-5):
            raise ValueError("Transform rotation must be orthonormal.")
        if not math.isclose(float(np.linalg.det(rotation)), 1.0, abs_tol=1e-5):
            raise ValueError("Transform rotation determinant must be +1.")
        return cls(value[:3, 3], matrix_to_rpy(rotation), source_frame, target_frame)


def sdk_to_optical_matrix(coordinate_system: str) -> np.ndarray:
    """Map a ZED SDK camera coordinate vector into optical x-right/y-down/z-forward.

    Supported ZED SDK conventions are IMAGE, RIGHT_HANDED_Y_UP,
    LEFT_HANDED_Y_UP, RIGHT_HANDED_Z_UP_X_FORWARD, LEFT_HANDED_Z_UP,
    and RIGHT_HANDED_Z_UP. IMAGE is the SDK default.
    """

    name = coordinate_system.strip().upper()
    matrices = {
        "IMAGE": np.eye(3),
        "RIGHT_HANDED_Y_UP": np.diag([1.0, -1.0, -1.0]),
        "LEFT_HANDED_Y_UP": np.diag([1.0, -1.0, 1.0]),
        "RIGHT_HANDED_Z_UP_X_FORWARD": np.array(
            [[0.0, -1.0, 0.0], [0.0, 0.0, -1.0], [1.0, 0.0, 0.0]]
        ),
        "LEFT_HANDED_Z_UP": np.array(
            [[0.0, 1.0, 0.0], [0.0, 0.0, -1.0], [1.0, 0.0, 0.0]]
        ),
        "RIGHT_HANDED_Z_UP": np.array(
            [[1.0, 0.0, 0.0], [0.0, 0.0, -1.0], [0.0, 1.0, 0.0]]
        ),
    }
    if name not in matrices:
        raise ValueError(f"Unsupported ZED SDK coordinate system: {coordinate_system}")
    return matrices[name]


def transform_sdk_pose_to_optical(
    position: Iterable[float], rotation: np.ndarray, coordinate_system: str
) -> tuple[np.ndarray, np.ndarray]:
    """Convert a camera pose's position and orientation axes to optical convention."""

    sdk_to_optical = sdk_to_optical_matrix(coordinate_system)
    return (
        sdk_to_optical @ np.asarray(position, dtype=float).reshape(3),
        sdk_to_optical @ np.asarray(rotation, dtype=float).reshape(3, 3),
    )
