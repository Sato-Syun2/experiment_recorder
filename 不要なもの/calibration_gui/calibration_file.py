"""Human-readable calibration YAML and ROS 2 static-transform export."""

from __future__ import annotations

from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Optional

import numpy as np
import yaml
from scipy.spatial.transform import Rotation

from .transform import Extrinsic


def _extrinsic_dict(extrinsic: Extrinsic) -> dict[str, Any]:
    rotation = extrinsic.rotation
    quaternion_xyzw = Rotation.from_matrix(rotation).as_quat().tolist()
    return {
        "translation_m": extrinsic.translation_m.tolist(),
        "translation_mm": (extrinsic.translation_m * 1000.0).tolist(),
        "rotation_rpy_deg": np.rad2deg(extrinsic.rpy_rad).tolist(),
        "rotation_quaternion_xyzw": quaternion_xyzw,
        "rotation_matrix": rotation.tolist(),
        "homogeneous_matrix": extrinsic.matrix.tolist(),
    }


def save_calibration_yaml(
    path: Path,
    *,
    initial: Extrinsic,
    calibrated: Extrinsic,
    camera_intrinsics: dict[str, Any],
    input_files: dict[str, str],
    reprojection: Optional[dict[str, Any]] = None,
) -> None:
    """Save all requested frame definitions and transform representations."""

    document = {
        "schema_version": 1,
        "calibrated_at_utc": datetime.now(timezone.utc).isoformat(),
        "frames": {
            "source": calibrated.source_frame,
            "target": calibrated.target_frame,
            "target_definition": "ZED left optical frame: +X right, +Y down, +Z forward",
            "rotation_convention": "active source-to-target, Rz(yaw) @ Ry(pitch) @ Rx(roll)",
            "translation_convention": "translation of source origin expressed in target axes",
            "internal_units": "m and rad",
        },
        "initial_cad": _extrinsic_dict(initial),
        "calibrated": _extrinsic_dict(calibrated),
        "camera_intrinsics": camera_intrinsics,
        "input_files": input_files,
        "reprojection_metrics": reprojection or {},
        "ros2_static_transform": {
            "parent_frame_id": calibrated.target_frame,
            "child_frame_id": calibrated.source_frame,
            "translation_m": calibrated.translation_m.tolist(),
            "rotation_quaternion_xyzw": Rotation.from_matrix(calibrated.rotation).as_quat().tolist(),
        },
    }
    path = path.expanduser()
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(yaml.safe_dump(document, sort_keys=False, allow_unicode=True), encoding="utf-8")


def load_calibration_yaml(path: Path) -> tuple[Extrinsic, Extrinsic, dict[str, Any]]:
    value = yaml.safe_load(path.expanduser().read_text(encoding="utf-8"))
    if not isinstance(value, dict) or "calibrated" not in value:
        raise ValueError("Calibration YAML is missing the calibrated transform.")

    def parse(entry: dict[str, Any], defaults: tuple[str, str]) -> Extrinsic:
        frames = value.get("frames", {})
        if "homogeneous_matrix" in entry:
            return Extrinsic.from_matrix(
                entry["homogeneous_matrix"],
                str(frames.get("source", defaults[0])),
                str(frames.get("target", defaults[1])),
            )
        return Extrinsic(
            translation_m=entry["translation_m"],
            rpy_rad=np.deg2rad(entry["rotation_rpy_deg"]),
            source_frame=str(frames.get("source", defaults[0])),
            target_frame=str(frames.get("target", defaults[1])),
        )

    calibrated = parse(value["calibrated"], ("livox_frame", "zed_left_optical_frame"))
    initial_entry = value.get("initial_cad", value["calibrated"])
    initial = parse(initial_entry, (calibrated.source_frame, calibrated.target_frame))
    return initial, calibrated, value
