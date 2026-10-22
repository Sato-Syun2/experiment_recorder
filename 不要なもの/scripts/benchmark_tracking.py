#!/usr/bin/env python3
"""Reproducible synthetic association regression, not real-world accuracy.

Injects known detections downstream of clustering so identity/velocity defects
can be isolated. Does not validate merged-cloud detection or sensor noise models.
"""
import argparse
import importlib.util
import json
import sys
from pathlib import Path
from unittest.mock import patch

import numpy as np

import track_people
from evaluate_tracking import evaluate_reference, quality_summary, write_csv, write_dashboard


def load_module(path):
    spec = importlib.util.spec_from_file_location("baseline_tracker", path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def case_data(name):
    times = np.arange(41) * .1
    hidden = set()
    if name == "crossing":
        positions = np.array([[[-1.95+t, -.05], [1.95-t, .05]] for t in times])
    elif name == "collinear_association":
        positions = np.array([[[-1.95+t, 0], [1.95-t, 0]] for t in times])
    elif name.startswith("overtaking"):
        sign = 1 if name.endswith("left") else -1
        positions = np.array([[[2+.5*t, 0], [1+1.1*t, sign*.14]] for t in times])
        hidden = {(i, 0) for i in range(15, 19)}
    elif name == "irregular_reacquisition":
        times = np.cumsum(np.resize([.07, .13, .09, .11], 41)) - .07
        positions = np.array([[[t, .12*np.sin(t)]] for t in times])
        hidden = {(i, 0) for i in range(12, 16)}
    elif name == "sparse_far":
        positions = np.array([[[10+.4*t, .08*np.sin(t)]] for t in times])
        hidden = {(i, 0) for i in range(10, 35, 3)}
    elif name == "accelerating_occlusion":
        positions = np.array([[[t+.1*t*t, 0]] for t in times])
        hidden = {(i, 0) for i in range(12, 18)}
    else:
        raise ValueError(name)
    return times, positions, hidden


def benchmark(module, name):
    times, positions, hidden = case_data(name)
    settings = module.TrackerSettings(background_mode="none", min_hits=1,
                                     min_confirmed_consecutive_hits=2, min_track_displacement=0,
                                     velocity_smoothing=.65)
    tracker = module.PeopleTracker(settings)
    tracker.preprocess = lambda points: points
    sequences = [[module.Detection(np.r_[point, 1.0], np.array([.4, .4, 1.7]),
                                   9 if name == "sparse_far" else 30, .8)
                  for j, point in enumerate(frame) if (i, j) not in hidden]
                 for i, frame in enumerate(positions)]
    stamps = [1_000_000_000 + int(round(t * 1e9)) for t in times]
    velocities = np.gradient(positions, times, axis=0)
    rows, labels = [], []
    with patch.object(module, "detect_clusters", side_effect=sequences):
        for i, stamp in enumerate(stamps):
            for s in tracker.update(np.empty((0, 3)), stamp):
                rows.append(dict(timestamp_ns=stamp, track_id=str(s.track_id), status=s.status,
                                 detected=str(int(s.detected)), corridor_x=float(s.position[0]),
                                 corridor_y=float(s.position[1]), corridor_vx=float(s.velocity[0]),
                                 corridor_vy=float(s.velocity[1]), confidence=s.confidence))
            for j, point in enumerate(positions[i]):
                labels.append(dict(timestamp_ns=stamp, person_id=f"person_{j+1}",
                                   x_m=point[0], y_m=point[1],
                                   vx_mps=velocities[i, j, 0], vy_mps=velocities[i, j, 1]))
    # All persons remain in reference during simulated occlusion. Predictions
    # are excluded so missing measurements are not presented as observed success.
    metrics, matches = evaluate_reference(rows, stamps, labels, .5, .01, 1.0)
    return rows, stamps, labels, metrics, matches


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--baseline-module", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()
    if args.output_dir.exists():
        parser.error("Choose a new output directory.")
    old = load_module(args.baseline_module.resolve())
    args.output_dir.mkdir(parents=True)
    report = {"note": __doc__, "cases": {}}
    for name in ("crossing", "collinear_association", "overtaking_left", "overtaking_right",
                 "irregular_reacquisition", "sparse_far", "accelerating_occlusion"):
        report["cases"][name] = {}
        for mode, module in (("before", old), ("after", track_people)):
            rows, stamps, labels, metrics, matches = benchmark(module, name)
            output = args.output_dir / name / mode
            output.mkdir(parents=True)
            write_csv(output / "tracks.csv", rows)
            write_csv(output / "frames.csv", [dict(timestamp_ns=t) for t in stamps])
            write_csv(output / "reference.csv", labels)
            write_csv(output / "matches.csv", matches)
            write_dashboard(output / "index.html", rows, stamps, metrics)
            report["cases"][name][mode] = metrics
    (args.output_dir / "metrics.json").write_text(json.dumps(report, indent=2, allow_nan=False) + "\n")
    print(args.output_dir / "metrics.json")


if __name__ == "__main__":
    main()
