"""Known trajectories/geometry; no sensors, SDK, or network required."""
import itertools
import json
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from assignment import gated_assignment
from track_people import Detection, PeopleTracker, TrackerSettings, TrackingError, load_settings
from sensor_transform import SensorTransform, SensorTransformError
from fuse_zed_lidar import smooth_and_velocity, camera_to_corridor, parse_args
from calibrate_zed_lidar_aruco import _solve_transform, _fit_target_plane
from evaluate_tracking import evaluate_reference, kinematics


def detection(x, y=0.0):
    return Detection(np.array([x, y, 1.0]), np.array([0.4, 0.4, 1.7]), 30, 0.9)


def run_detections(sequences, times, module=None):
    if module is None:
        import track_people as module
    settings = module.TrackerSettings(
        background_mode="none", min_hits=1, min_confirmed_consecutive_hits=2,
        min_track_displacement=0, velocity_smoothing=1,
    )
    tracker = module.PeopleTracker(settings)
    tracker.preprocess = lambda points: points
    with patch.object(module, "detect_clusters", side_effect=sequences):
        return [tracker.update(np.empty((0, 3)), int(round(t * 1e9))) for t in times]


class MeasurementTests(unittest.TestCase):
    def test_kinematics_nonuniform_quadratic_and_missing_samples(self):
        times = [0, .07, .18, .29, .4]
        rows = [dict(timestamp_ns=round(t*1e9), track_id="1", status="confirmed",
                     detected="1", corridor_x=t*t, corridor_y=0) for t in times]
        result = kinematics(rows)
        for row, t in zip(result[1:-1], times[1:-1]):
            self.assertAlmostEqual(row["raw_derivative_vx_mps"], 2*t)
            self.assertAlmostEqual(row["raw_derivative_ax_mps2"], 2)
        rows[2]["status"] = "predicted"
        self.assertTrue(all(row["derivative_valid"] == 0 for row in kinematics(rows)))

    def test_optional_smoothing_retains_raw_columns(self):
        rows = [dict(timestamp_ns=round(t*1e9), zed_body_id="1", fusion_status="fused",
                     fused_position_corridor_x_m=t, fused_position_corridor_y_m=0,
                     fused_position_corridor_z_m=1, motion_outlier=0, motion_valid=0)
                for t in np.arange(9)*.1]
        smooth_and_velocity(rows, 4, .25, .21)
        for i, row in enumerate(rows):
            self.assertAlmostEqual(row["raw_fused_position_corridor_x_m"], i*.1)
        self.assertAlmostEqual(rows[4]["fused_position_corridor_x_m"], .4)

    def test_evaluator_counts_missing_false_positive_and_id_switch(self):
        def row(t, id, x, status="confirmed"):
            return dict(timestamp_ns=t, track_id=id, corridor_x=x, corridor_y=0,
                        corridor_vx=1, corridor_vy=0, status=status, detected="1")
        rows = [row(0, "1", 0), row(100000000, "2", .1),
                row(100000000, "9", 5), row(200000000, "2", .2, "predicted")]
        labels = [dict(timestamp_ns=t, person_id="A", x_m=x, y_m=0, vx_mps=1, vy_mps=0)
                  for t, x in ((0, 0), (100000000, .1), (200000000, .2), (300000000, .3))]
        metrics, _ = evaluate_reference(rows, [0, 100000000, 200000000, 300000000], labels, .5, .01, 1)
        self.assertEqual(metrics["matched_samples"], 2)
        self.assertEqual(metrics["missed_samples"], 2)
        self.assertEqual(metrics["false_positive_samples"], 1)
        self.assertEqual(metrics["id_switches"], 1)
        self.assertEqual(metrics["velocity_rmse_mps"], 0)

    def test_evaluator_empty_scene_and_no_frame(self):
        metric, _ = evaluate_reference([], [], [dict(timestamp_ns=0, person_id="")], .5, .01, 1)
        self.assertEqual(metric["evaluated_reference_frames"], 1)
        self.assertIsNone(metric["position_rmse_m"])

    def test_assignment_matches_bruteforce_with_forbidden_edges(self):
        rng = np.random.default_rng(73)
        for n, m in ((2, 3), (4, 3), (3, 4)):
            for _ in range(10):
                cost = rng.uniform(0, 3, (n, m))
                allowed = rng.random((n, m)) > .4
                candidates = []
                for choices in itertools.product(range(-1, m), repeat=n):
                    used = [j for j in choices if j >= 0]
                    if len(set(used)) != len(used):
                        continue
                    pairs = [(i, j) for i, j in enumerate(choices) if j >= 0]
                    if all(allowed[i, j] for i, j in pairs):
                        candidates.append((-len(pairs), sum(cost[i, j] for i, j in pairs)))
                result = gated_assignment(cost, allowed)
                self.assertEqual(-len(result), min(candidates)[0])
                self.assertAlmostEqual(sum(cost[i, j] for i, j in result), min(candidates)[1])

    def test_crossing_keeps_motion_identity(self):
        times = np.arange(20) * .1
        seq = [[detection(-.95 + t), detection(.95 - t)] for t in times]
        frames = run_detections(seq, times)
        self.assertAlmostEqual(frames[-1][0].position[0], .95)
        self.assertAlmostEqual(frames[-1][1].position[0], -.95)

    def test_reacquisition_velocity_uses_observations(self):
        frames = run_detections([[detection(0)], [detection(.1)], [], [detection(.3)]],
                                [0, .1, .2, .3])
        self.assertEqual(frames[-1][0].track_id, 1)
        self.assertAlmostEqual(frames[-1][0].velocity[0], 1)

    def test_elapsed_time_expires_id_without_intermediate_frames(self):
        frames = run_detections([[detection(0)], [detection(0)]], [0, 3])
        self.assertEqual(frames[-1][0].track_id, 2)

    def test_duplicate_and_reversed_timestamps_rejected(self):
        for times in ([1, 1], [1, .9]):
            with self.assertRaises(TrackingError):
                run_detections([[detection(0)], [detection(0)]], times)

    def test_coordinate_chain_roundtrip(self):
        root = Path(__file__).resolve().parents[1]
        settings = load_settings(root / "config.json")
        raw_transform = SensorTransform.from_json(root / "tracking/lidar_to_zed_left_eye.json")
        raw = np.array([4, 1, .3])
        level = settings.gravity_alignment_rotation @ raw + settings.gravity_alignment_translation
        np.testing.assert_allclose(settings.lidar_to_zed_frame.transform_position(level),
                                   raw_transform.transform_position(raw), atol=1e-12)
        np.testing.assert_allclose(camera_to_corridor(raw_transform.transform_position(raw), settings),
                                   settings.corridor_frame.transform_position(level), atol=1e-12)

    def test_reject_millimeter_transform(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "t.json"
            path.write_text(json.dumps({"units": "mm", "rotation_source_to_target": np.eye(3).tolist(),
                                        "translation_m": [0, 0, 0]}))
            with self.assertRaises(SensorTransformError):
                SensorTransform.from_json(path)

    def test_unsmoothed_acceleration_nonuniform_times(self):
        times = [0, .07, .18, .29, .4]
        rows = [dict(timestamp_ns=int(t * 1e9), zed_body_id="1", fusion_status="fused",
                     fused_position_corridor_x_m=t*t, fused_position_corridor_y_m=0,
                     fused_position_corridor_z_m=1, motion_outlier=0, motion_valid=0)
                for t in times]
        smooth_and_velocity(rows, 4, .2)
        for row, t in zip(rows, times):
            self.assertAlmostEqual(row["fused_position_corridor_x_m"], t*t)
        for row, t in zip(rows[1:-1], times[1:-1]):
            self.assertAlmostEqual(row["fused_velocity_corridor_x_mps"], 2*t)

    def test_sync_offset_cannot_be_implicitly_zero(self):
        with self.assertRaises(SystemExit):
            parse_args([])

    def test_plane_solution_and_degeneracy(self):
        rng = np.random.default_rng(9)
        normals = rng.normal(size=(12, 3))
        normals /= np.linalg.norm(normals, axis=1)[:, None]
        translation = np.array([.1, -.2, .3])
        observations = [
            dict(normal_cam=n, d_cam_m=2 - n @ translation,
                 plane_lidar=dict(normal_lidar=n, d_lidar_m=2),
                 gravity_cam=np.array([0, 0, 1]), gravity_lidar=np.array([0, 0, 1]))
            for n in normals
        ]
        r, t, report = _solve_transform(observations, 4)
        np.testing.assert_allclose(r, np.eye(3), atol=1e-12)
        np.testing.assert_allclose(t, translation, atol=1e-12)
        self.assertEqual(report["used_observations"], 12)
        for obs in observations:
            obs["normal_cam"] = obs["plane_lidar"]["normal_lidar"] = np.array([0, 0, 1])
        with self.assertRaises(ValueError):
            _solve_transform(observations, 4)

    def test_large_wall_not_accepted_as_small_panel(self):
        x, y = np.meshgrid(np.linspace(-.5, .5, 15), np.linspace(-.4, .4, 15))
        points = np.column_stack((x.ravel(), y.ravel(), np.ones(x.size) * 2))
        with self.assertRaisesRegex(ValueError, "panel dimensions"):
            _fit_target_plane(points, np.array([0, 0, 2]), np.eye(3),
                              .25, .18, .6, .8, 25, 1)


if __name__ == "__main__":
    unittest.main()
