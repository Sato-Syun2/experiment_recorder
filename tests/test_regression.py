"""実機なしで録画名、再生時刻、CSV、追跡状態の主要動作を確認する。"""

from __future__ import annotations

import datetime as dt
import json
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))

from corridor_frame import CorridorFrame
from playback_timeline import inspect_playback_timeline
from record_zed import generate_filename, generate_livox_bag_path
from replay_tracks_csv import HTML_PAGE, load_tracking_csv
from track_people import Detection, PeopleTracker, TrackerSettings


class RefactorRegressionTests(unittest.TestCase):
    def test_recording_names_keep_trial_and_avoid_overwrite(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            started = dt.datetime(2026, 9, 24, 10, 30, 0)
            first = generate_filename("walk test", 2, root, started)
            self.assertEqual(first.name, "walk_test_002_20260924_103000.svo2")
            first.touch()
            self.assertEqual(
                generate_filename("walk test", 2, root, started).name,
                "walk_test_002_20260924_103000_01.svo2",
            )
            bag = generate_livox_bag_path("walk test", 2, root, started)
            self.assertEqual(bag.name, "walk_test_002_20260924_103000")

    def test_playback_uses_recorded_timestamp_offset(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            svo = root / "sample.svo2"
            bag = root / "bag"
            bag.mkdir()
            magic = b"\x89MCAP0\r\n"
            chunk = (2_000_000_000).to_bytes(8, "little") + (5_000_000_000).to_bytes(8, "little") + bytes(8)
            svo.write_bytes(magic + bytes([6]) + len(chunk).to_bytes(8, "little") + chunk + bytes([2]) + bytes(8) + magic)
            (bag / "metadata.yaml").write_text(
                "starting_time:\n  nanoseconds_since_epoch: 1000000000\n"
                "duration:\n  nanoseconds: 5000000000\n", encoding="utf-8"
            )
            timeline = inspect_playback_timeline(svo, bag)
            self.assertIsNotNone(timeline)
            self.assertEqual(timeline.svo_start_delay_seconds, 1.0)
            self.assertEqual(timeline.bag_start_delay_seconds, 0.0)
            self.assertEqual(timeline.svo_duration_seconds, 3.0)
            self.assertEqual(timeline.bag_duration_seconds, 5.0)

    def test_csv_replay_interpolates_missing_frame(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            csv_path = Path(directory) / "tracks.csv"
            csv_path.write_text(
                "frame,time_sec,track_id,x,y,vx,vy,status\n"
                "1,0.0,1,0.0,1.0,0.2,0.0,confirmed\n"
                "3,0.4,1,0.4,1.0,0.2,0.0,confirmed\n", encoding="utf-8"
            )
            result = load_tracking_csv(csv_path, 10)
            self.assertEqual([frame["time_sec"] for frame in result["frames"]], [0.0, 0.2, 0.4])
            self.assertEqual(len(result["tracks"][0]["samples"]), 2)
            self.assertIn("__DATA__", HTML_PAGE)

    def test_tracker_keeps_id_when_detection_is_temporarily_missing(self) -> None:
        settings = TrackerSettings(
            max_match_distance=1.0,
            min_hits=2,
            min_confirmed_consecutive_hits=2,
            min_track_displacement=0.0,
        )
        tracker = PeopleTracker(settings)
        tracker.preprocess = lambda points: points
        tracker.cluster_background.update = lambda detections: detections

        def detection(x: float) -> Detection:
            return Detection(np.array([x, 0.0, 1.0]), np.array([0.3, 0.3, 1.7]), 30, 0.9)

        with patch("track_people.detect_clusters", side_effect=[[detection(0.0)], [detection(0.2)], []]):
            first = tracker.update(np.empty((0, 3)), 1_000_000_000)
            second = tracker.update(np.empty((0, 3)), 2_000_000_000)
            missing = tracker.update(np.empty((0, 3)), 3_000_000_000)
        self.assertEqual([item.track_id for item in first], [1])
        self.assertEqual([item.track_id for item in second], [1])
        self.assertTrue(second[0].confirmed)
        self.assertEqual(missing[0].status, "predicted")
        self.assertEqual(missing[0].missed_frames, 1)
        self.assertGreater(missing[0].position[0], second[0].position[0])

    def test_corridor_coordinates_survive_json_round_trip(self) -> None:
        frame = CorridorFrame(
            origin_xy=np.array([2.0, 3.0]),
            longitudinal_xy=np.array([1.0, 0.0]),
            lateral_xy=np.array([0.0, 1.0]),
            wall_offsets=(-1.0, 1.0),
            wall_gap=2.0,
            wall_inlier_counts=(10, 10),
        )
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "corridor.json"
            frame.write_json(path)
            restored = CorridorFrame.from_json(path)
            self.assertEqual(json.loads(path.read_text())["wall_gap"], 2.0)
            np.testing.assert_allclose(restored.transform_position([4.0, 4.0, 1.5]), [2.0, 1.0, 1.5])
            np.testing.assert_allclose(restored.transform_velocity([0.2, -0.1, 0.0]), [0.2, -0.1, 0.0])


if __name__ == "__main__":
    unittest.main()
