"""センサー間の剛体座標変換を読み込み、位置とベクトルへ適用する。"""

from __future__ import annotations

import dataclasses
import json
import math
from pathlib import Path
from typing import Sequence

import numpy as np


class SensorTransformError(ValueError):
    """センサー間変換の読込・検証に失敗した場合の例外。"""


@dataclasses.dataclass(frozen=True)
class SensorTransform:
    """source座標の値をtarget座標へ写す剛体変換。"""

    rotation: np.ndarray
    translation: np.ndarray
    source_frame: str = "livox_frame"
    target_frame: str = "zed_left_eye"

    def __post_init__(self) -> None:
        rotation = np.asarray(self.rotation, dtype=float)
        translation = np.asarray(self.translation, dtype=float).reshape(-1)
        if rotation.shape != (3, 3) or translation.size != 3:
            raise SensorTransformError(
                "Rotation must be 3x3 and translation must have three values."
            )
        if not np.isfinite(rotation).all() or not np.isfinite(translation).all():
            raise SensorTransformError("Sensor transform values must be finite.")
        if not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-5):
            raise SensorTransformError("Sensor transform rotation must be orthonormal.")
        if not math.isclose(float(np.linalg.det(rotation)), 1.0, abs_tol=1e-5):
            raise SensorTransformError("Sensor transform rotation must have determinant +1.")
        object.__setattr__(self, "rotation", rotation)
        object.__setattr__(self, "translation", translation)

    def transform_position(self, position: Sequence[float]) -> np.ndarray:
        value = np.asarray(position, dtype=float).reshape(-1)
        if value.size != 3:
            raise SensorTransformError("A position must contain three values.")
        # 位置には回転と原点移動を適用する。速度等にはtransform_vectorを使う。
        return self.rotation @ value + self.translation

    def transform_vector(self, vector: Sequence[float]) -> np.ndarray:
        """速度など原点を持たないベクトルを回転する。"""

        value = np.asarray(vector, dtype=float).reshape(-1)
        if value.size != 3:
            raise SensorTransformError("A vector must contain three values.")
        return self.rotation @ value

    @classmethod
    def from_json(cls, path: Path) -> "SensorTransform":
        try:
            value = json.loads(path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError) as exc:
            raise SensorTransformError(f"Could not read sensor transform {path}: {exc}") from exc
        if not isinstance(value, dict):
            raise SensorTransformError("Sensor transform JSON must contain an object.")
        if value.get("units", "m") != "m":
            raise SensorTransformError("Sensor transform translation must be in meters (units=m).")
        if value.get("coordinate_system", "IMAGE") != "IMAGE":
            raise SensorTransformError("ZED transform must use the IMAGE coordinate system.")
        try:
            rotation = np.asarray(value["rotation_source_to_target"], dtype=float)
            translation = np.asarray(value["translation_m"], dtype=float)
            return cls(
                rotation=rotation,
                translation=translation,
                source_frame=str(value.get("source_frame", "livox_frame")),
                target_frame=str(value.get("target_frame", "zed_left_eye")),
            )
        except (KeyError, TypeError, ValueError) as exc:
            raise SensorTransformError("Sensor transform JSON has invalid fields.") from exc
