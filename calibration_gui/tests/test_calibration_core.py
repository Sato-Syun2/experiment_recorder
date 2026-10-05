from __future__ import annotations

import json
from pathlib import Path

import numpy as np
import pytest

from calibration_gui.calibration_file import load_calibration_yaml, save_calibration_yaml
from calibration_gui.io.camera import load_camera_model
from calibration_gui.io.pointcloud import load_point_cloud
from calibration_gui.optimize import refine_extrinsic
from calibration_gui.projection import (
    CameraModel,
    depth_colors,
    intensity_colors,
    project_lidar_points,
    sample_rgb_colors,
)
from calibration_gui.transform import (
    OPTICAL_FRAME,
    Extrinsic,
    sdk_to_optical_matrix,
)


def test_reported_mount_transform_maps_axes_and_translation() -> None:
    initial_path = Path(__file__).parents[1] / "default_extrinsic.json"
    value = json.loads(initial_path.read_text())
    matrix = np.eye(4)
    matrix[:3, :3] = value["rotation_source_to_target"]
    matrix[:3, 3] = value["translation_m"]
    extrinsic = Extrinsic.from_matrix(matrix, "livox_frame", OPTICAL_FRAME)

    assert np.allclose(extrinsic.translation_m, [0.060, -0.080, 0.020])
    assert np.allclose(extrinsic.transform_vectors([[0, 1, 0]]), [[-1, 0, 0]], atol=1e-8)
    lidar_x = extrinsic.rotation[:, 0]
    lidar_z = extrinsic.rotation[:, 2]
    assert np.isclose(np.rad2deg(np.arccos(lidar_x[2])), 25.0, atol=1e-5)
    assert np.isclose(np.rad2deg(np.arccos(lidar_z[2])), 65.0, atol=1e-5)
    assert np.allclose(Extrinsic.from_matrix(extrinsic.matrix, "livox_frame", OPTICAL_FRAME).matrix, matrix)


def test_sdk_coordinate_system_conversion_is_explicit() -> None:
    assert np.array_equal(sdk_to_optical_matrix("IMAGE"), np.eye(3))
    assert np.array_equal(sdk_to_optical_matrix("RIGHT_HANDED_Y_UP"), np.diag([1.0, -1.0, -1.0]))
    with pytest.raises(ValueError, match="Unsupported"):
        sdk_to_optical_matrix("UNKNOWN")


def test_projection_rejects_behind_invalid_out_of_view_and_distance() -> None:
    camera = CameraModel(100, 80, 50, 50, 50, 40)
    extrinsic = Extrinsic(np.zeros(3), np.zeros(3))
    points = np.array([[0, 0, 2], [2, 0, 2], [0, 0, -1], [np.nan, 0, 2], [0, 0, 8]])
    result = project_lidar_points(points, extrinsic, camera, max_distance_m=5)
    assert result.valid.tolist() == [True, False, False, False, False]
    assert np.allclose(result.uv[0], [50, 40])


def test_zbuffer_rgb_sampling_and_colormaps() -> None:
    camera = CameraModel(20, 20, 10, 10, 10, 10)
    camera_points = np.array([[0, 0, 2], [0, 0, 4], [0, 0, -1], [5, 0, 2]], dtype=float)
    from calibration_gui.projection import project_camera_points

    projection = project_camera_points(camera_points, camera)
    image = np.zeros((20, 20, 3), dtype=np.uint8)
    image[10, 10] = [17, 103, 211]
    colors, visible = sample_rgb_colors(image, projection)
    assert visible.tolist() == [True, False, False, False]
    assert np.allclose(colors[0], np.array([17, 103, 211]) / 255.0)
    assert np.allclose(colors[1:], 0.35)
    assert depth_colors(np.array([1.0, 2.0]), np.array([True, True])).shape == (2, 3)
    assert intensity_colors(np.array([3.0, 8.0]), 2).shape == (2, 3)
    assert intensity_colors(None, 4).shape == (4, 3)


def test_refine_extrinsic_from_explicit_3d_2d_correspondences() -> None:
    rng = np.random.default_rng(12)
    camera = CameraModel(1280, 720, 850, 840, 640, 360)
    true = Extrinsic(np.array([0.06, -0.08, 0.02]), np.deg2rad([2.0, -16.0, 87.0]))
    points = np.column_stack(
        [rng.uniform(-2.0, 2.0, 80), rng.uniform(-1.0, 1.0, 80), rng.uniform(3.0, 12.0, 80)]
    )
    from calibration_gui.projection import project_camera_points

    target_pixels = project_camera_points(true.transform_points(points), camera, image_bounds=False).uv
    start = Extrinsic(
        true.translation_m + np.array([0.012, -0.009, 0.015]),
        true.rpy_rad + np.deg2rad([0.7, -0.5, 0.8]),
    )
    result = refine_extrinsic(points, target_pixels, start, camera)
    assert result.rms_px < 1e-4
    assert np.allclose(result.extrinsic.matrix, true.matrix, atol=1e-5)


def test_yaml_round_trip_records_ros_transform(tmp_path: Path) -> None:
    initial = Extrinsic(np.array([0.06, -0.08, 0.02]), np.deg2rad([0, -65, 90]))
    calibrated = Extrinsic(np.array([0.061, -0.079, 0.019]), np.deg2rad([1, -64, 89]))
    path = tmp_path / "calibration.yaml"
    save_calibration_yaml(
        path,
        initial=initial,
        calibrated=calibrated,
        camera_intrinsics={"width": 640},
        input_files={"camera": "hallway.svo2", "point_cloud": "scan.pcd"},
        reprojection={"rms_px": 1.2},
    )
    loaded_initial, loaded, value = load_calibration_yaml(path)
    assert np.allclose(loaded_initial.matrix, initial.matrix)
    assert np.allclose(loaded.matrix, calibrated.matrix)
    assert value["ros2_static_transform"]["parent_frame_id"] == OPTICAL_FRAME
    assert value["ros2_static_transform"]["child_frame_id"] == "livox_frame"
    assert value["reprojection_metrics"]["rms_px"] == 1.2


def test_ascii_pcd_and_ply_preserve_xyz_and_intensity(tmp_path: Path) -> None:
    pcd = tmp_path / "scan.pcd"
    pcd.write_text(
        "VERSION .7\nFIELDS intensity z x y\nSIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\n"
        "WIDTH 2\nHEIGHT 1\nPOINTS 2\nDATA ascii\n10 3 1 2\n20 6 4 5\n"
    )
    cloud = load_point_cloud(pcd)
    assert np.allclose(cloud.points_m, [[1, 2, 3], [4, 5, 6]])
    assert np.allclose(cloud.intensity, [10, 20])

    ply = tmp_path / "scan.ply"
    ply.write_text(
        "ply\nformat ascii 1.0\nelement vertex 2\nproperty uchar red\nproperty float x\n"
        "property float intensity\nproperty float y\nproperty float z\nproperty uchar green\n"
        "property uchar blue\nend_header\n255 1 11 2 3 0 0\n0 4 22 5 6 128 255\n"
    )
    cloud = load_point_cloud(ply)
    assert np.allclose(cloud.points_m, [[1, 2, 3], [4, 5, 6]])
    assert np.allclose(cloud.intensity, [11, 22])
    assert np.allclose(cloud.colors_rgb, [[1, 0, 0], [0, 128 / 255, 1]])


def test_binary_pcd_and_ply_preserve_intensity(tmp_path: Path) -> None:
    pcd = tmp_path / "scan_binary.pcd"
    dtype = np.dtype([("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("intensity", "<u2")])
    records = np.array([(1, 2, 3, 7), (4, 5, 6, 9)], dtype=dtype)
    pcd.write_bytes(
        b"VERSION .7\nFIELDS x y z intensity\nSIZE 4 4 4 2\nTYPE F F F U\nCOUNT 1 1 1 1\n"
        b"WIDTH 2\nHEIGHT 1\nPOINTS 2\nDATA binary\n" + records.tobytes()
    )
    cloud = load_point_cloud(pcd)
    assert np.allclose(cloud.points_m, [[1, 2, 3], [4, 5, 6]])
    assert np.allclose(cloud.intensity, [7, 9])

    ply = tmp_path / "scan_binary.ply"
    ply_dtype = np.dtype([("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("intensity", "<f4")])
    ply_records = np.array([(1, 2, 3, 7), (4, 5, 6, 9)], dtype=ply_dtype)
    ply.write_bytes(
        b"ply\nformat binary_little_endian 1.0\nelement vertex 2\nproperty float x\n"
        b"property float y\nproperty float z\nproperty float intensity\nend_header\n" + ply_records.tobytes()
    )
    cloud = load_point_cloud(ply)
    assert np.allclose(cloud.points_m, [[1, 2, 3], [4, 5, 6]])
    assert np.allclose(cloud.intensity, [7, 9])


def test_compressed_pcd_never_silently_drops_intensity(tmp_path: Path) -> None:
    path = tmp_path / "compressed.pcd"
    path.write_bytes(
        b"VERSION .7\nFIELDS x y z intensity\nSIZE 4 4 4 1\nTYPE F F F U\nCOUNT 1 1 1 1\n"
        b"WIDTH 1\nHEIGHT 1\nPOINTS 1\nDATA binary_compressed\n" + b"\x00" * 16
    )
    with pytest.raises(ValueError, match="would drop it"):
        load_point_cloud(path)


@pytest.mark.parametrize(
    "text, expected_distortion",
    [
        (
            "image_width: 640\nimage_height: 480\ncamera_matrix: {rows: 3, cols: 3, data: [500,0,320,0,510,240,0,0,1]}\n"
            "distortion_model: plumb_bob\nD: [0.1, -0.02, 0.0, 0.0, 0.0]\n",
            True,
        ),
        (
            "%YAML:1.0\n---\ncamera_matrix: !!opencv-matrix\n   rows: 3\n   cols: 3\n   dt: d\n   data: [500, 0, 320, 0, 510, 240, 0, 0, 1]\ndistortion_coefficients: !!opencv-matrix\n   rows: 1\n   cols: 5\n   dt: d\n   data: [0, 0, 0, 0, 0]\n",
            False,
        ),
    ],
)
def test_camera_intrinsics_formats(tmp_path: Path, text: str, expected_distortion: bool) -> None:
    path = tmp_path / "intrinsics.yaml"
    path.write_text(text)
    camera = load_camera_model(path, 640, 480)
    assert (camera.distortion is not None) is expected_distortion
    assert camera.fx == 500
    assert camera.fy == 510
