#!/usr/bin/env python3
"""保存済みbagの時計の連続性と、IMUから見た設置方向を確認する。

ヘッダー時刻とbag記録時刻を分けて集計する。IMU平均による重力方向は、
装置が静止していることを前提とする。別センサーとの同期精度は検証しない。
"""

import argparse
import json
import math
from pathlib import Path

import numpy as np

from track_people import _import_rosbag_types, _message_timestamp_ns


def audit(bag, up_reference):
    """bagを順に読み、点群の時刻差と最大2000標本のIMU平均を集計する。"""

    ros, deserialize, imu_cls = _import_rosbag_types("sensor_msgs/msg/Imu")
    _, _, cloud_cls = _import_rosbag_types("sensor_msgs/msg/PointCloud2")
    reader = ros.SequentialReader()
    reader.open(ros.StorageOptions(uri=str(bag), storage_id="mcap"), ros.ConverterOptions("", ""))
    stamps, delays, imu = [], [], []
    fallback = 0
    while reader.has_next():
        topic, data, t = reader.read_next()
        if topic == "/livox/lidar":
            msg = deserialize(data, cloud_cls)
            h = _message_timestamp_ns(msg, 0)
            fallback += not bool(h)
            stamps.append(h or t)
            delays.append((t - (h or t)) / 1e6)
        elif topic == "/livox/imu" and len(imu) < 2000:
            msg = deserialize(data, imu_cls)
            a = msg.linear_acceleration
            imu.append([a.x, a.y, a.z])
    if len(stamps) < 2:
        raise ValueError(f"Insufficient PointCloud2 frames: {bag}")
    up, difference, avg = None, None, None
    if imu:
        avg = np.mean(imu, axis=0)
        if np.isfinite(avg).all() and np.linalg.norm(avg) > 1e-9:
            up = avg / np.linalg.norm(avg)
            difference = math.degrees(math.acos(float(np.clip(up @ up_reference, -1, 1))))
    recording = bag.parent / "recording.json"
    meta = json.loads(recording.read_text()) if recording.exists() else {}
    return dict(
        bag=str(bag),
        condition_label=meta.get("condition_category"),
        note="Planned metadata does not verify actual behavior. IMU average assumes a stationary rig.",
        frames=len(stamps),
        first_header_ns=min(stamps),
        last_header_ns=max(stamps),
        duration_s=(max(stamps) - min(stamps)) / 1e9,
        nonmonotonic=int(np.sum(np.diff(stamps) <= 0)),
        header_fallback_count=fallback,
        header_interval_ms_p50_p99=np.percentile(np.diff(stamps) / 1e6, [50, 99]).tolist(),
        bag_minus_header_ms_p50_p99=np.percentile(delays, [50, 99]).tolist(),
        imu_first_2000_mean_native=None if avg is None else avg.tolist(),
        imu_up=None if up is None else up.tolist(),
        up_difference_from_config_deg=difference,
    )


def main():
    """録画群を読み取り、時計・設置状態の確認結果を新規JSONへ保存する。"""

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--recordings", type=Path, required=True)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        parser.error("Choose a new output file.")
    config = json.loads(args.config.read_text())
    up = np.asarray(config["tracking"]["gravity_alignment"]["up_vector"], float)
    up /= np.linalg.norm(up)
    results = [audit(bag, up) for bag in sorted(args.recordings.glob("*/*/lidar"))]
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("x", encoding="utf-8") as stream:
        json.dump(results, stream, indent=2, ensure_ascii=False, allow_nan=False)


if __name__ == "__main__":
    main()
