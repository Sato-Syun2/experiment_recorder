"""Deterministic synthetic camera scene and LiDAR cloud for offline verification."""

from __future__ import annotations

import numpy as np

from .io.camera import CameraFrame
from .io.pointcloud import PointCloud
from .projection import CameraModel
from .transform import Extrinsic


def make_demo_data(extrinsic: Extrinsic | None = None) -> tuple[CameraFrame, PointCloud]:
    width, height = 640, 480
    yy, xx = np.mgrid[0:height, 0:width]
    image = np.stack(
        [
            np.clip(xx * 255 / (width - 1), 0, 255),
            np.clip(yy * 255 / (height - 1), 0, 255),
            np.clip((0.5 + 0.5 * np.sin(xx / 35) * np.cos(yy / 29)) * 255, 0, 255),
        ],
        axis=-1,
    ).astype(np.uint8)
    image[80:160, 100:200] = [235, 80, 60]
    image[210:320, 370:520] = [45, 190, 100]
    camera = CameraModel(width, height, 520.0, 520.0, width / 2.0, height / 2.0)

    # Two planar walls and a box, sampled deterministically in optical coordinates.
    wall_x = np.linspace(-2.1, 2.1, 95)
    wall_y = np.linspace(-1.35, 1.35, 60)
    wx, wy = np.meshgrid(wall_x, wall_y)
    front_wall = np.column_stack([wx.ravel(), wy.ravel(), np.full(wx.size, 4.5)])
    side_y = np.array([-1.25, 1.25])
    side_x, side_z = np.meshgrid(np.linspace(-2.0, 2.0, 75), np.linspace(-1.0, 1.3, 46))
    side_walls = np.concatenate(
        [
            np.column_stack([side_x.ravel(), np.full(side_x.size, side_y[0]), side_z.ravel()]),
            np.column_stack([side_x.ravel(), np.full(side_x.size, side_y[1]), side_z.ravel()]),
        ],
        axis=0,
    )
    cube_faces = []
    edge = np.linspace(-0.35, 0.35, 34)
    ex, ey = np.meshgrid(edge, edge)
    for z in (2.0, 2.7):
        cube_faces.append(np.column_stack([0.45 + ex.ravel(), -0.05 + ey.ravel(), np.full(ex.size, z)]))
    for x in (0.10, 0.80):
        cube_faces.append(np.column_stack([np.full(ex.size, x), -0.05 + ex.ravel(), 2.35 + ey.ravel()]))
    optical_points = np.concatenate([front_wall, side_walls, *cube_faces], axis=0)
    if extrinsic is None:
        lidar_points = optical_points
    else:
        lidar_points = (optical_points - extrinsic.translation_m) @ extrinsic.rotation
    intensity = np.linspace(0.0, 255.0, len(lidar_points), dtype=float)
    frame = CameraFrame(image, camera, "synthetic demo", 0, None)
    cloud = PointCloud(lidar_points, intensity, source="synthetic demo")
    return frame, cloud
