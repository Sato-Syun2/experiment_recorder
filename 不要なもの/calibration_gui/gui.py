"""Qt desktop interface for simultaneous image overlay and 3D extrinsic tuning."""

from __future__ import annotations

import math
from pathlib import Path
from typing import Optional

import numpy as np

try:
    from PySide6.QtCore import Qt, QTimer, Signal
    from PySide6.QtGui import QImage, QPainter, QPixmap
    from PySide6.QtWidgets import (
        QApplication,
        QCheckBox,
        QComboBox,
        QDoubleSpinBox,
        QFileDialog,
        QFormLayout,
        QGridLayout,
        QGroupBox,
        QHBoxLayout,
        QLabel,
        QMainWindow,
        QMessageBox,
        QPushButton,
        QScrollArea,
        QSlider,
        QSpinBox,
        QSplitter,
        QTableWidget,
        QTableWidgetItem,
        QVBoxLayout,
        QWidget,
        QInputDialog,
    )
    QT_API = "PySide6"
except ImportError:
    try:
        from PyQt5.QtCore import Qt, QTimer, pyqtSignal as Signal
        from PyQt5.QtGui import QImage, QPainter, QPixmap
        from PyQt5.QtWidgets import (
            QApplication,
            QCheckBox,
            QComboBox,
            QDoubleSpinBox,
            QFileDialog,
            QFormLayout,
            QGridLayout,
            QGroupBox,
            QHBoxLayout,
            QLabel,
            QMainWindow,
            QMessageBox,
            QPushButton,
            QScrollArea,
            QSlider,
            QSpinBox,
            QSplitter,
            QTableWidget,
            QTableWidgetItem,
            QVBoxLayout,
            QWidget,
            QInputDialog,
        )
        QT_API = "PyQt5"
    except ImportError as exc:
        raise RuntimeError("Install PySide6 (recommended) or PyQt5 to run the calibration GUI.") from exc

from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg
from matplotlib.figure import Figure

from .calibration_file import load_calibration_yaml, save_calibration_yaml
from .demo import make_demo_data
from .io.camera import CameraFrame, load_image_and_intrinsics, load_svo_frame
from .io.pointcloud import PointCloud, load_point_cloud
from .io.rosbag2 import read_bag_pointcloud
from .optimize import refine_extrinsic, reprojection_errors
from .projection import (
    depth_colors,
    intensity_colors,
    overlay_projection,
    project_lidar_points,
    sample_rgb_colors,
)
from .transform import Extrinsic, OPTICAL_FRAME


_QT_HORIZONTAL = getattr(Qt, "Horizontal", None)
if _QT_HORIZONTAL is None:
    _QT_HORIZONTAL = Qt.Orientation.Horizontal
_QT_ALIGN_CENTER = getattr(Qt, "AlignCenter", None)
if _QT_ALIGN_CENTER is None:
    _QT_ALIGN_CENTER = Qt.AlignmentFlag.AlignCenter
_QT_LEFT_BUTTON = getattr(Qt, "LeftButton", None)
if _QT_LEFT_BUTTON is None:
    _QT_LEFT_BUTTON = Qt.MouseButton.LeftButton
_QIMAGE_RGB888 = getattr(QImage, "Format_RGB888", None)
if _QIMAGE_RGB888 is None:
    _QIMAGE_RGB888 = QImage.Format.Format_RGB888


class ImageView(QWidget):
    clicked = Signal(float, float)

    def __init__(self) -> None:
        super().__init__()
        self.image_rgb: Optional[np.ndarray] = None
        self.zoom = 1.0
        self._draw_rect = (0.0, 0.0, 1.0, 1.0)
        self.setMinimumSize(420, 320)
        self.setMouseTracking(True)

    def set_image(self, image_rgb: np.ndarray) -> None:
        self.image_rgb = np.ascontiguousarray(image_rgb[:, :, :3], dtype=np.uint8)
        self.update()

    def paintEvent(self, event: object) -> None:
        painter = QPainter(self)
        painter.fillRect(self.rect(), Qt.black)
        if self.image_rgb is None:
            painter.setPen(Qt.white)
            painter.drawText(self.rect(), _QT_ALIGN_CENTER, "Load a ZED image/SVO or launch --demo")
            painter.end()
            return
        height, width = self.image_rgb.shape[:2]
        image = QImage(self.image_rgb.data, width, height, 3 * width, _QIMAGE_RGB888).copy()
        pixmap = QPixmap.fromImage(image)
        scale = min(self.width() / width, self.height() / height) * self.zoom
        draw_width, draw_height = width * scale, height * scale
        left, top = (self.width() - draw_width) / 2.0, (self.height() - draw_height) / 2.0
        self._draw_rect = (left, top, draw_width, draw_height)
        painter.drawPixmap(int(left), int(top), int(draw_width), int(draw_height), pixmap)
        painter.end()

    def mousePressEvent(self, event: object) -> None:
        if self.image_rgb is None or event.button() != _QT_LEFT_BUTTON:
            return
        left, top, width, height = self._draw_rect
        px = (event.position().x() if hasattr(event, "position") else event.x())
        py = (event.position().y() if hasattr(event, "position") else event.y())
        if width <= 0 or height <= 0 or not (left <= px < left + width and top <= py < top + height):
            return
        image_h, image_w = self.image_rgb.shape[:2]
        self.clicked.emit((px - left) * image_w / width, (py - top) * image_h / height)

    def wheelEvent(self, event: object) -> None:
        delta = event.angleDelta().y()
        self.zoom = float(np.clip(self.zoom * (1.12 if delta > 0 else 1 / 1.12), 0.5, 5.0))
        self.update()


class CalibrationWindow(QMainWindow):
    def __init__(
        self,
        camera_frame: Optional[CameraFrame],
        cloud: Optional[PointCloud],
        initial: Extrinsic,
        *,
        camera_file: str = "",
        cloud_file: str = "",
    ) -> None:
        super().__init__()
        self.setWindowTitle("ZED 2i × Livox Mid-360S Extrinsic Calibration")
        self.resize(1600, 980)
        self.camera_frame = camera_frame
        self.cloud = cloud
        self.initial = initial
        self.current = initial
        self.camera_file = camera_file or (camera_frame.source if camera_frame else "")
        self.cloud_file = cloud_file or (cloud.source if cloud else "")
        self.correspondences: list[tuple[np.ndarray, np.ndarray]] = []
        self.reprojection_metrics: dict[str, float | int] = {}
        self.selected_lidar_point: Optional[np.ndarray] = None
        self._history: list[Extrinsic] = []
        self._history_group_open = False
        self._display_indices = np.empty(0, dtype=int)
        self._last_projection = None
        self._last_colors = np.empty((0, 3), dtype=float)
        self._open3d_bridge = None

        self._refresh_timer = QTimer(self)
        self._refresh_timer.setSingleShot(True)
        self._refresh_timer.timeout.connect(self.refresh_views)
        self._history_timer = QTimer(self)
        self._history_timer.setSingleShot(True)
        self._history_timer.timeout.connect(self._close_history_group)

        self._build_ui()
        if self.camera_frame and self.cloud:
            self.refresh_views()
        self._update_parameter_widgets(initial)

    def _build_ui(self) -> None:
        central = QWidget()
        root = QVBoxLayout(central)
        toolbar = QHBoxLayout()
        for label, handler in (
            ("Open SVO2…", self.open_svo),
            ("Open Image + K…", self.open_image),
            ("Open PCD/PLY…", self.open_cloud),
            ("Open rosbag2…", self.open_bag),
            ("Save YAML…", self.save_yaml),
            ("Load YAML…", self.load_yaml),
            ("Screenshot…", self.save_screenshot),
        ):
            button = QPushButton(label)
            button.clicked.connect(handler)
            toolbar.addWidget(button)
        root.addLayout(toolbar)

        splitter = QSplitter(_QT_HORIZONTAL)
        view_area = QWidget()
        views = QHBoxLayout(view_area)
        image_group = QGroupBox("2D — ZED left image + LiDAR projection")
        image_layout = QVBoxLayout(image_group)
        self.image_view = ImageView()
        self.image_view.clicked.connect(self._image_clicked)
        image_layout.addWidget(self.image_view)
        self.overlay_check = QCheckBox("Show projected LiDAR points")
        self.overlay_check.setChecked(True)
        self.overlay_check.stateChanged.connect(lambda *_: self.schedule_refresh())
        image_layout.addWidget(self.overlay_check)
        views.addWidget(image_group, 1)

        cloud_group = QGroupBox("3D — colored LiDAR cloud in ZED optical frame")
        cloud_layout = QVBoxLayout(cloud_group)
        self.figure = Figure(figsize=(6, 5), tight_layout=True)
        self.canvas = FigureCanvasQTAgg(self.figure)
        self.axes = self.figure.add_subplot(111, projection="3d")
        self._scatter = None
        self.canvas.mpl_connect("pick_event", self._point_picked)
        cloud_layout.addWidget(self.canvas)
        cloud_controls = QHBoxLayout()
        self.color_mode = QComboBox()
        self.color_mode.addItems(["RGB Color", "Intensity", "Depth"])
        self.color_mode.currentIndexChanged.connect(lambda *_: self.schedule_refresh())
        cloud_controls.addWidget(QLabel("Color:"))
        cloud_controls.addWidget(self.color_mode)
        self.open3d_button = QPushButton("Open3D window")
        self.open3d_button.clicked.connect(self.open3d_window)
        try:
            import open3d  # noqa: F401
        except ImportError:
            self.open3d_button.setEnabled(False)
            self.open3d_button.setToolTip("Install Open3D; the embedded 3D view remains available.")
        cloud_controls.addWidget(self.open3d_button)
        cloud_layout.addLayout(cloud_controls)
        views.addWidget(cloud_group, 1)
        splitter.addWidget(view_area)

        controls = QWidget()
        controls_layout = QVBoxLayout(controls)
        controls_layout.addWidget(self._build_transform_controls())
        controls_layout.addWidget(self._build_display_controls())
        controls_layout.addWidget(self._build_correspondence_controls())
        self.status_label = QLabel("Ready. Use paired 3D/2D features for reprojection refinement.")
        self.status_label.setWordWrap(True)
        controls_layout.addWidget(self.status_label)
        controls_layout.addStretch(1)
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setWidget(controls)
        scroll.setMinimumWidth(400)
        splitter.addWidget(scroll)
        splitter.setSizes([1150, 450])
        root.addWidget(splitter, 1)
        self.setCentralWidget(central)

    def _build_transform_controls(self) -> QWidget:
        group = QGroupBox("Extrinsic: p_camera = R_CL p_lidar + t_CL")
        outer = QVBoxLayout(group)
        label = QLabel(
            "Translation is expressed in the ZED target axes. Rotation is active "
            "Rz(yaw)·Ry(pitch)·Rx(roll), source Livox → target optical frame."
        )
        label.setWordWrap(True)
        outer.addWidget(label)
        grid = QGridLayout()
        grid.addWidget(QLabel("Parameter"), 0, 0)
        grid.addWidget(QLabel("Slider"), 0, 1)
        grid.addWidget(QLabel("Value"), 0, 2)
        self.parameter_sliders = []
        self.parameter_spins = []
        specs = [
            ("Translation X (mm)", -1000.0, 1000.0, 0.1, 10.0),
            ("Translation Y (mm)", -1000.0, 1000.0, 0.1, 10.0),
            ("Translation Z (mm)", -1000.0, 1000.0, 0.1, 10.0),
            ("Roll (deg)", -180.0, 180.0, 0.01, 100.0),
            ("Pitch (deg)", -180.0, 180.0, 0.01, 100.0),
            ("Yaw (deg)", -180.0, 180.0, 0.01, 100.0),
        ]
        for index, (name, low, high, decimals, scale) in enumerate(specs, start=1):
            grid.addWidget(QLabel(name), index, 0)
            slider = QSlider(_QT_HORIZONTAL)
            slider.setRange(round(low * scale), round(high * scale))
            spin = QDoubleSpinBox()
            spin.setRange(low, high)
            spin.setDecimals(1 if index <= 3 else 2)
            spin.setSingleStep(decimals)
            grid.addWidget(slider, index, 1)
            grid.addWidget(spin, index, 2)
            slider.valueChanged.connect(lambda value, s=spin, k=scale: s.setValue(value / k))
            spin.valueChanged.connect(lambda value, s=slider, k=scale: s.setValue(round(value * k)))
            spin.valueChanged.connect(self._parameter_changed)
            self.parameter_sliders.append((slider, scale))
            self.parameter_spins.append(spin)
        outer.addLayout(grid)
        buttons = QHBoxLayout()
        for label, handler in (("Reset to initial CAD", self.reset_initial), ("Undo", self.undo)):
            button = QPushButton(label)
            button.clicked.connect(handler)
            buttons.addWidget(button)
        outer.addLayout(buttons)
        self.delta_label = QLabel()
        outer.addWidget(self.delta_label)
        self._update_delta_label()
        return group

    def _build_display_controls(self) -> QWidget:
        group = QGroupBox("Projection and display")
        form = QFormLayout(group)
        self.min_distance = QDoubleSpinBox()
        self.min_distance.setRange(0.0, 1000.0)
        self.min_distance.setValue(0.1)
        self.min_distance.setSuffix(" m")
        self.max_distance = QDoubleSpinBox()
        self.max_distance.setRange(0.1, 1000.0)
        self.max_distance.setValue(30.0)
        self.max_distance.setSuffix(" m")
        self.point_size = QSpinBox()
        self.point_size.setRange(1, 15)
        self.point_size.setValue(3)
        self.draw_3d_check = QCheckBox("Show 3D cloud")
        self.draw_3d_check.setChecked(True)
        self.image_plane_check = QCheckBox("Show image plane in 3D")
        for widget in (self.min_distance, self.max_distance, self.point_size):
            widget.valueChanged.connect(lambda *_: self.schedule_refresh())
        self.draw_3d_check.stateChanged.connect(lambda *_: self.schedule_refresh())
        self.image_plane_check.stateChanged.connect(lambda *_: self.schedule_refresh())
        form.addRow("Minimum distance", self.min_distance)
        form.addRow("Maximum distance", self.max_distance)
        form.addRow("2D point radius", self.point_size)
        form.addRow(self.draw_3d_check)
        form.addRow(self.image_plane_check)
        return group

    def _build_correspondence_controls(self) -> QWidget:
        group = QGroupBox("Explicit 3D ↔ 2D correspondences")
        layout = QVBoxLayout(group)
        hint = QLabel(
            "Click a LiDAR return in the 3D view and its matching feature in the image. "
            "Add each pair, then refine by pixel reprojection error. Color similarity is not a metric."
        )
        hint.setWordWrap(True)
        layout.addWidget(hint)
        fields = QGridLayout()
        self.correspondence_spins = []
        for i, name in enumerate(("Lidar X (m)", "Lidar Y (m)", "Lidar Z (m)", "Pixel U", "Pixel V")):
            fields.addWidget(QLabel(name), i // 2, (i % 2) * 2)
            spin = QDoubleSpinBox()
            spin.setRange(-10000.0, 10000.0)
            spin.setDecimals(5)
            fields.addWidget(spin, i // 2, (i % 2) * 2 + 1)
            self.correspondence_spins.append(spin)
        layout.addLayout(fields)
        buttons = QHBoxLayout()
        add_button = QPushButton("Add correspondence")
        add_button.clicked.connect(self.add_correspondence)
        remove_button = QPushButton("Remove selected")
        remove_button.clicked.connect(self.remove_correspondence)
        fit_button = QPushButton("Refine from correspondences")
        fit_button.clicked.connect(self.refine_from_correspondences)
        buttons.addWidget(add_button)
        buttons.addWidget(remove_button)
        buttons.addWidget(fit_button)
        layout.addLayout(buttons)
        self.correspondence_table = QTableWidget(0, 5)
        self.correspondence_table.setHorizontalHeaderLabels(["X m", "Y m", "Z m", "u px", "v px"])
        self.correspondence_table.setMaximumHeight(170)
        layout.addWidget(self.correspondence_table)
        self.reprojection_label = QLabel("Reprojection RMS: not measured")
        layout.addWidget(self.reprojection_label)
        return group

    def _update_parameter_widgets(self, transform: Extrinsic) -> None:
        values = np.r_[transform.translation_m * 1000.0, np.rad2deg(transform.rpy_rad)]
        for spin, (slider, scale), value in zip(self.parameter_spins, self.parameter_sliders, values):
            spin.blockSignals(True)
            slider.blockSignals(True)
            spin.setValue(float(value))
            slider.setValue(round(float(value) * scale))
            spin.blockSignals(False)
            slider.blockSignals(False)
        self._update_delta_label()

    def _update_delta_label(self) -> None:
        translation_delta = (self.current.translation_m - self.initial.translation_m) * 1000.0
        rotation_delta = np.rad2deg(self.current.rpy_rad - self.initial.rpy_rad)
        self.delta_label.setText(
            "Δ from initial: translation ["
            + ", ".join(f"{value:+.1f}" for value in translation_delta)
            + "] mm; RPY ["
            + ", ".join(f"{value:+.2f}" for value in rotation_delta)
            + "] deg"
        )

    def _parameter_changed(self, *_: object) -> None:
        values = [spin.value() for spin in self.parameter_spins]
        candidate = Extrinsic(
            translation_m=np.asarray(values[:3], dtype=float) / 1000.0,
            rpy_rad=np.deg2rad(values[3:]),
            source_frame=self.current.source_frame,
            target_frame=self.current.target_frame,
        )
        if not np.allclose(candidate.matrix, self.current.matrix, atol=1e-12):
            if not self._history_group_open:
                self._history.append(self.current)
                self._history_group_open = True
            self._history_timer.start(500)
            self.current = candidate
            self._update_delta_label()
            self.schedule_refresh()

    def _close_history_group(self) -> None:
        self._history_group_open = False

    def reset_initial(self) -> None:
        self._history.append(self.current)
        self.current = self.initial
        self._update_parameter_widgets(self.current)
        self.schedule_refresh()

    def undo(self) -> None:
        if self._history:
            self.current = self._history.pop()
            self._history_group_open = False
            self._update_parameter_widgets(self.current)
            self.schedule_refresh()

    def schedule_refresh(self) -> None:
        self._refresh_timer.start(80)

    def _work_cloud(self) -> tuple[np.ndarray, np.ndarray, Optional[np.ndarray]]:
        if self.cloud is None:
            return np.empty((0, 3)), np.empty(0, dtype=int), None
        points = self.cloud.points_m
        limit = 100_000
        if len(points) > limit:
            indexes = np.linspace(0, len(points) - 1, limit, dtype=int)
        else:
            indexes = np.arange(len(points))
        intensity = self.cloud.intensity[indexes] if self.cloud.intensity is not None else None
        return points[indexes], indexes, intensity

    def refresh_views(self) -> None:
        if self.camera_frame is None:
            return
        source_image = self.camera_frame.image_rgb
        if self.cloud is None:
            self.image_view.set_image(source_image)
            self.axes.clear()
            self.axes.set_title("Load a PCD/PLY or rosbag2 point cloud")
            self.canvas.draw_idle()
            return
        points, indexes, intensity = self._work_cloud()
        self._display_indices = indexes
        projection = project_lidar_points(
            points,
            self.current,
            self.camera_frame.camera_model,
            min_distance_m=self.min_distance.value(),
            max_distance_m=self.max_distance.value(),
        )
        rgb_colors, visible = sample_rgb_colors(source_image, projection, zbuffer=True)
        mode = self.color_mode.currentText()
        if mode == "Depth":
            colors = depth_colors(projection.depth_m, projection.valid)
        elif mode == "Intensity":
            colors = intensity_colors(intensity, len(points))
        else:
            colors = rgb_colors
        colors[~visible] = 0.35
        self._last_projection = projection
        self._last_colors = colors
        if self.overlay_check.isChecked():
            overlay = overlay_projection(
                source_image,
                projection,
                colors,
                point_size=self.point_size.value(),
                zbuffer=True,
            )
            self.image_view.set_image(overlay)
        else:
            self.image_view.set_image(source_image)
        self._refresh_3d(points, indexes, projection, colors)
        in_view_count = int(visible.sum())
        intensity_note = "" if self.cloud.intensity is not None else " | no intensity field"
        self.status_label.setText(
            f"{len(self.cloud.points_m):,} cloud points ({len(points):,} displayed); "
            f"{in_view_count:,} nearest projected returns{intensity_note}. "
            f"Camera image: {self.camera_frame.source}; cloud: {self.cloud.source}"
        )
        if self._open3d_bridge is not None:
            display_indices = self._depth_limited_indices(projection)
            self._open3d_bridge.update(
                projection.camera_points[display_indices], colors[display_indices], self.current
            )

    def _depth_limited_indices(self, projection: object) -> np.ndarray:
        depth = projection.depth_m
        finite = np.isfinite(projection.camera_points).all(axis=1) & np.isfinite(depth)
        return np.flatnonzero(
            finite
            & (depth > 1e-6)
            & (depth >= self.min_distance.value())
            & (depth <= self.max_distance.value())
        )

    def _refresh_3d(
        self,
        points_lidar: np.ndarray,
        indexes: np.ndarray,
        projection: object,
        colors: np.ndarray,
    ) -> None:
        if not self.draw_3d_check.isChecked():
            self.axes.clear()
            self.canvas.draw_idle()
            return
        old_elev, old_azim = self.axes.elev, self.axes.azim
        self.axes.clear()
        camera_points = projection.camera_points
        in_range = self._depth_limited_indices(projection)
        if len(in_range) > 15_000:
            display = in_range[np.linspace(0, len(in_range) - 1, 15_000, dtype=int)]
        else:
            display = in_range
        if len(display):
            self._scatter = self.axes.scatter(
                camera_points[display, 0],
                camera_points[display, 1],
                camera_points[display, 2],
                c=colors[display],
                s=3,
                depthshade=False,
                picker=6,
            )
            self._display_to_work = display
        else:
            self._display_to_work = np.empty(0, dtype=int)
        self.axes.scatter([0], [0], [0], c=[[1, 0.1, 0.1]], marker="x", s=55, label="ZED left origin")
        origin = self.current.translation_m
        self.axes.scatter([origin[0]], [origin[1]], [origin[2]], c=[[1, 0.8, 0]], marker="o", s=35, label="Livox O")
        axis_length = 0.20
        rotation = self.current.rotation
        for axis, color in enumerate(("r", "g", "b")):
            vector = rotation[:, axis] * axis_length
            self.axes.plot(
                [origin[0], origin[0] + vector[0]],
                [origin[1], origin[1] + vector[1]],
                [origin[2], origin[2] + vector[2]],
                color=color,
                linewidth=2,
            )
        self.axes.plot([0, axis_length], [0, 0], [0, 0], color="r", linewidth=1.5)
        self.axes.plot([0, 0], [0, axis_length], [0, 0], color="g", linewidth=1.5)
        self.axes.plot([0, 0], [0, 0], [0, axis_length], color="b", linewidth=1.5)
        if self.image_plane_check.isChecked():
            z = 0.5
            model = self.camera_frame.camera_model
            corners = np.array(
                [
                    [0, 0], [model.width - 1, 0],
                    [model.width - 1, model.height - 1], [0, model.height - 1], [0, 0],
                ], dtype=float
            )
            xy = np.column_stack(
                [(corners[:, 0] - model.cx) * z / model.fx,
                 (corners[:, 1] - model.cy) * z / model.fy]
            )
            self.axes.plot(xy[:, 0], xy[:, 1], np.full(len(xy), z), color="orange", linewidth=1)
        self.axes.set_xlabel("X right (m)")
        self.axes.set_ylabel("Y down (m)")
        self.axes.set_zlabel("Z forward (m)")
        self.axes.set_title("ZED optical frame")
        if len(camera_points):
            sample = camera_points[display] if len(display) else camera_points
            spans = np.ptp(sample, axis=0)
            centers = np.mean(sample, axis=0)
            radius = max(float(np.max(spans)) * 0.55, 0.25)
            self.axes.set_xlim(centers[0] - radius, centers[0] + radius)
            self.axes.set_ylim(centers[1] - radius, centers[1] + radius)
            self.axes.set_zlim(max(0.0, centers[2] - radius), centers[2] + radius)
        try:
            self.axes.set_box_aspect((1, 1, 1))
        except AttributeError:
            pass
        self.axes.view_init(elev=old_elev, azim=old_azim)
        self.axes.legend(loc="upper right", fontsize=7)
        self.canvas.draw_idle()

    def _point_picked(self, event: object) -> None:
        if event.artist is not self._scatter or not len(event.ind):
            return
        display_index = int(event.ind[0])
        work_index = int(self._display_to_work[display_index])
        source_index = int(self._display_indices[work_index])
        self.selected_lidar_point = self.cloud.points_m[source_index]
        for spin, value in zip(self.correspondence_spins[:3], self.selected_lidar_point):
            spin.setValue(float(value))
        self.status_label.setText(
            "Selected LiDAR point ["
            + ", ".join(f"{value:.4f}" for value in self.selected_lidar_point)
            + "] m. Click its image feature and add the correspondence."
        )

    def _image_clicked(self, u: float, v: float) -> None:
        self.correspondence_spins[3].setValue(u)
        self.correspondence_spins[4].setValue(v)

    def add_correspondence(self) -> None:
        values = np.array([spin.value() for spin in self.correspondence_spins], dtype=float)
        point, pixel = values[:3], values[3:]
        if self.camera_frame is None:
            QMessageBox.warning(self, "No camera", "Load a camera image and intrinsics first.")
            return
        if not (0 <= pixel[0] < self.camera_frame.camera_model.width and 0 <= pixel[1] < self.camera_frame.camera_model.height):
            QMessageBox.warning(self, "Pixel outside image", "Select a pixel inside the ZED image.")
            return
        self.correspondences.append((point, pixel))
        row = self.correspondence_table.rowCount()
        self.correspondence_table.insertRow(row)
        for column, value in enumerate(values):
            self.correspondence_table.setItem(row, column, QTableWidgetItem(f"{value:.5f}"))
        self.selected_lidar_point = None

    def remove_correspondence(self) -> None:
        row = self.correspondence_table.currentRow()
        if row < 0:
            return
        self.correspondence_table.removeRow(row)
        del self.correspondences[row]

    def refine_from_correspondences(self) -> None:
        if self.camera_frame is None:
            QMessageBox.warning(self, "No camera", "Load camera image and intrinsics first.")
            return
        if len(self.correspondences) < 6:
            QMessageBox.warning(self, "More correspondences required", "Add at least six clear 3D-to-2D correspondences distributed across different positions and depths.")
            return
        points = np.asarray([entry[0] for entry in self.correspondences])
        pixels = np.asarray([entry[1] for entry in self.correspondences])
        self._history.append(self.current)
        try:
            result = refine_extrinsic(points, pixels, self.current, self.camera_frame.camera_model)
        except Exception as exc:
            self._history.pop()
            QMessageBox.critical(self, "Optimization failed", str(exc))
            return
        self.current = result.extrinsic
        self.reprojection_metrics = {
            "correspondence_count": len(points),
            "rms_px": result.rms_px,
            "median_px": result.median_px,
        }
        self.reprojection_label.setText(
            f"Reprojection RMS: {result.rms_px:.2f} px | median: {result.median_px:.2f} px"
        )
        self._update_parameter_widgets(self.current)
        self.schedule_refresh()

    def _intrinsics_meta(self) -> dict[str, object]:
        if self.camera_frame is None:
            return {}
        model = self.camera_frame.camera_model
        return {
            "width": model.width,
            "height": model.height,
            "fx": model.fx,
            "fy": model.fy,
            "cx": model.cx,
            "cy": model.cy,
            "rectified": model.rectified,
            "distortion_model": model.distortion_model,
            "distortion": None if model.distortion is None else model.distortion.tolist(),
        }

    def save_yaml(self) -> None:
        path, _ = QFileDialog.getSaveFileName(self, "Save calibration", "calibration.yaml", "YAML (*.yaml *.yml)")
        if not path:
            return
        try:
            save_calibration_yaml(
                Path(path),
                initial=self.initial,
                calibrated=self.current,
                camera_intrinsics=self._intrinsics_meta(),
                input_files={"camera": self.camera_file, "point_cloud": self.cloud_file},
                reprojection=self.reprojection_metrics,
            )
        except Exception as exc:
            QMessageBox.critical(self, "Save failed", str(exc))
            return
        self.status_label.setText(f"Saved calibration: {path}")

    def load_yaml(self) -> None:
        path, _ = QFileDialog.getOpenFileName(self, "Load calibration", "", "YAML (*.yaml *.yml)")
        if not path:
            return
        try:
            initial, calibrated, _ = load_calibration_yaml(Path(path))
        except Exception as exc:
            QMessageBox.critical(self, "Load failed", str(exc))
            return
        self.initial = initial
        self.current = calibrated
        self._history.clear()
        self._update_parameter_widgets(calibrated)
        self.schedule_refresh()

    def save_screenshot(self) -> None:
        path, _ = QFileDialog.getSaveFileName(self, "Save screenshot", "calibration_view.png", "PNG (*.png)")
        if path:
            self.grab().save(path)

    def open_svo(self) -> None:
        path, _ = QFileDialog.getOpenFileName(self, "Open ZED SVO/SVO2", "", "ZED recording (*.svo *.svo2)")
        if not path:
            return
        index, accepted = QInputDialog.getInt(self, "SVO frame", "Frame index", 0, 0, 10_000_000)
        if not accepted:
            return
        self._load_camera_action(lambda: load_svo_frame(Path(path), index), path)

    def open_image(self) -> None:
        image_path, _ = QFileDialog.getOpenFileName(self, "Open left RGB image", "", "Images (*.png *.jpg *.jpeg *.bmp)")
        if not image_path:
            return
        intrinsics_path, _ = QFileDialog.getOpenFileName(self, "Open camera intrinsics", "", "YAML/JSON (*.yaml *.yml *.json)")
        if not intrinsics_path:
            return
        self._load_camera_action(
            lambda: load_image_and_intrinsics(Path(image_path), Path(intrinsics_path)),
            f"{image_path} | {intrinsics_path}",
        )

    def _load_camera_action(self, loader: object, description: str) -> None:
        try:
            self.camera_frame = loader()
        except Exception as exc:
            QMessageBox.critical(self, "Camera input failed", str(exc))
            return
        self.camera_file = description
        self.schedule_refresh()

    def open_cloud(self) -> None:
        path, _ = QFileDialog.getOpenFileName(self, "Open point cloud", "", "Point clouds (*.pcd *.ply *.npy *.npz)")
        if not path:
            return
        try:
            self.cloud = load_point_cloud(Path(path))
        except Exception as exc:
            QMessageBox.critical(self, "Point cloud input failed", str(exc))
            return
        self.cloud_file = path
        self.schedule_refresh()

    def open_bag(self) -> None:
        path = QFileDialog.getExistingDirectory(self, "Open rosbag2 directory")
        if not path:
            return
        topic, accepted = QInputDialog.getText(self, "LiDAR topic", "Topic", text="/livox/lidar")
        if not accepted or not topic:
            return
        index, accepted = QInputDialog.getInt(self, "Point-cloud frame", "Frame index", 0, 0, 10_000_000)
        if not accepted:
            return
        try:
            frame = read_bag_pointcloud(Path(path), topic=topic, frame_index=index)
            self.cloud = frame.cloud
            self.cloud_file = f"{path} [{topic} frame {index}, {frame.message_type}]"
            if frame.frame_id and frame.frame_id != self.current.source_frame:
                QMessageBox.warning(
                    self,
                    "Point cloud frame differs",
                    f"Bag frame_id is {frame.frame_id!r}, but calibration source is {self.current.source_frame!r}. Confirm their axes match.",
                )
        except Exception as exc:
            QMessageBox.critical(self, "rosbag2 input failed", str(exc))
            return
        self.schedule_refresh()

    def open3d_window(self) -> None:
        try:
            if self._open3d_bridge is None or self._open3d_bridge.closed:
                self._open3d_bridge = Open3DBridge()
            if self._last_projection is not None:
                self._open3d_bridge.update(
                    self._last_projection.camera_points, self._last_colors, self.current
                )
        except Exception as exc:
            QMessageBox.warning(self, "Open3D unavailable", str(exc))
            self._open3d_bridge = None


class Open3DBridge:
    """Optional non-blocking Open3D Visualizer updated by the Qt timer."""

    def __init__(self) -> None:
        try:
            import open3d as o3d
        except ImportError as exc:
            raise RuntimeError("Open3D is not installed. Install the optional open3d package.") from exc
        self.o3d = o3d
        self.visualizer = o3d.visualization.Visualizer()
        if not self.visualizer.create_window(window_name="LiDAR RGB Colorization", width=900, height=700):
            raise RuntimeError("Could not create an Open3D window.")
        self.cloud = o3d.geometry.PointCloud()
        self.visualizer.add_geometry(self.cloud)
        self.axes = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.2)
        self.visualizer.add_geometry(self.axes)
        self.lidar_axes = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.2)
        self.visualizer.add_geometry(self.lidar_axes)
        self._lidar_pose_key = None
        self.closed = False
        self._points = np.empty((0, 3))
        self._colors = np.empty((0, 3))
        self._extrinsic = None
        self.timer = QTimer()
        self.timer.timeout.connect(self._tick)
        self.timer.start(30)

    def update(self, points_camera: np.ndarray, colors: np.ndarray, extrinsic: Extrinsic) -> None:
        if len(points_camera) > 100_000:
            indexes = np.linspace(0, len(points_camera) - 1, 100_000, dtype=int)
            self._points, self._colors = points_camera[indexes], colors[indexes]
        else:
            self._points, self._colors = points_camera, colors
        self._extrinsic = extrinsic

    def _tick(self) -> None:
        self.cloud.points = self.o3d.utility.Vector3dVector(self._points)
        self.cloud.colors = self.o3d.utility.Vector3dVector(np.clip(self._colors, 0.0, 1.0))
        self.visualizer.update_geometry(self.cloud)
        if self._extrinsic is not None:
            pose_key = self._extrinsic.matrix.tobytes()
            if pose_key != self._lidar_pose_key:
                self.visualizer.remove_geometry(self.lidar_axes, reset_bounding_box=False)
                self.lidar_axes = self.o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.2)
                self.lidar_axes.transform(self._extrinsic.matrix)
                self.visualizer.add_geometry(self.lidar_axes, reset_bounding_box=False)
                self._lidar_pose_key = pose_key
        if not self.visualizer.poll_events():
            self.timer.stop()
            self.visualizer.destroy_window()
            self.closed = True
        else:
            self.visualizer.update_renderer()


def load_initial_extrinsic(path: Path) -> Extrinsic:
    """Read the project's nominal R/t JSON and reinterpret IMAGE as optical."""

    import json

    value = json.loads(path.expanduser().read_text(encoding="utf-8"))
    matrix = np.eye(4)
    matrix[:3, :3] = np.asarray(value["rotation_source_to_target"], dtype=float).reshape(3, 3)
    matrix[:3, 3] = np.asarray(value["translation_m"], dtype=float).reshape(3)
    return Extrinsic.from_matrix(
        matrix,
        str(value.get("source_frame", "livox_frame")),
        OPTICAL_FRAME,
    )


def launch_gui(
    camera_frame: Optional[CameraFrame],
    cloud: Optional[PointCloud],
    initial: Extrinsic,
    *,
    camera_file: str = "",
    cloud_file: str = "",
) -> int:
    app = QApplication.instance() or QApplication([])
    window = CalibrationWindow(
        camera_frame,
        cloud,
        initial,
        camera_file=camera_file,
        cloud_file=cloud_file,
    )
    window.show()
    return app.exec()
