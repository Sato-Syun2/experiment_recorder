#!/usr/bin/env python3
"""録画済みLiDAR bagから廊下中心座標系を校正する。

最初の数フレームに含まれる左右の壁を検出し、
結果をconfig.jsonで指定した廊下校正ファイルへ保存する。
"""

from __future__ import annotations

import argparse
import dataclasses
import hashlib
import json
import sys
from pathlib import Path
from typing import Optional, Sequence

import numpy as np

from corridor_frame import CorridorFrameError, estimate_corridor_frame
from track_people import (
    TrackingError,
    _custom_msg_to_numpy,
    _iter_bag_clouds,
    _pointcloud2_to_numpy,
    load_settings,
)

PROJECT_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CONFIG_PATH = PROJECT_ROOT / "config.json"


def _parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", type=Path, help="解析するrosbag2ディレクトリ")
    parser.add_argument(
        "--output",
        type=Path,
        default=None,
        help="校正結果のJSON保存先（省略時はconfig.jsonのcorridor_frame_file）",
    )
    parser.add_argument(
        "--config", type=Path, default=DEFAULT_CONFIG_PATH, help="トピックとROIを読む設定ファイル"
    )
    parser.add_argument("--topic", default=None, help="点群トピック（省略時はconfig.json）")
    parser.add_argument("--message-type", choices=("auto", "pointcloud2", "custom"), default="auto")
    parser.add_argument("--storage-id", default="mcap")
    parser.add_argument("--frames", type=int, default=30, help="壁推定に使う点群フレーム数")
    parser.add_argument("--wall-z-min", type=float, default=0.5, help="壁として使う高さの下限[m]")
    parser.add_argument("--wall-z-max", type=float, default=2.2, help="壁として使う高さの上限[m]")
    parser.add_argument("--wall-voxel-size", type=float, default=0.10)
    parser.add_argument("--line-tolerance", type=float, default=0.12)
    parser.add_argument("--min-wall-span", type=float, default=3.0)
    parser.add_argument("--min-wall-gap", type=float, default=0.8)
    parser.add_argument("--max-wall-gap", type=float, default=8.0)
    parser.add_argument("--ransac-iterations", type=int, default=2500)
    return parser.parse_args(argv)


def main(argv: Optional[Sequence[str]] = None) -> int:
    """保存bagの点群を集め、左右壁から廊下座標を推定してJSONへ保存する。"""

    args = _parse_args(argv)
    if args.frames <= 0 or args.ransac_iterations <= 0:
        print("Error: frames and ransac iterations must be positive.", file=sys.stderr)
        return 2
    if args.wall_z_min >= args.wall_z_max:
        print("Error: wall z range is invalid.", file=sys.stderr)
        return 2
    bag_path = args.bag.expanduser().resolve()
    config_path = args.config.expanduser().resolve()
    try:
        # 設定から点群トピックとセンサー座標系のROIを取得する。
        settings = load_settings(config_path, args.topic)
        output_path = (
            args.output.expanduser()
            if args.output is not None
            else (
                settings.corridor_frame_file
                if settings.corridor_frame_file is not None
                else PROJECT_ROOT / "tracking" / "corridor_frame.json"
            )
        )
        if not output_path.is_absolute():
            output_path = (config_path.parent / output_path).resolve()
        else:
            output_path = output_path.resolve()
        if output_path.exists():
            raise TrackingError(f"Choose a new calibration output; already exists: {output_path}")
        points_by_frame: list[np.ndarray] = []
        actual_type = None
        # bagの先頭から指定フレーム数だけ読み込み、壁点をまとめる。
        for frame_number, (message, _, message_kind) in enumerate(
            _iter_bag_clouds(bag_path, settings.topic, args.message_type, args.storage_id),
            start=1,
        ):
            actual_type = message_kind
            points = (
                _custom_msg_to_numpy(message)
                if message_kind == "custom"
                else _pointcloud2_to_numpy(message)
            )
            if settings.gravity_alignment_enabled and len(points):
                points = (
                    points @ settings.gravity_alignment_rotation.T
                    + settings.gravity_alignment_translation
                ).astype(np.float32, copy=False)
            if len(points):
                points_by_frame.append(points)
            if frame_number >= args.frames:
                break
        if not points_by_frame:
            raise TrackingError("The bag contained no non-empty point-cloud frames.")
        # 複数フレームをまとめることで、人物や一時的な欠損の影響を抑える。
        points = np.concatenate(points_by_frame, axis=0)
        roi = settings.roi
        frame = estimate_corridor_frame(
            points,
            z_min=args.wall_z_min,
            z_max=args.wall_z_max,
            x_min=roi.x_min,
            x_max=roi.x_max,
            y_min=roi.y_min,
            y_max=roi.y_max,
            voxel_size=args.wall_voxel_size,
            line_tolerance=args.line_tolerance,
            min_wall_span=args.min_wall_span,
            min_wall_gap=args.min_wall_gap,
            max_wall_gap=args.max_wall_gap,
            ransac_iterations=args.ransac_iterations,
        )
        if settings.gravity_alignment_enabled:
            frame = dataclasses.replace(frame, source_frame="livox_level_frame")
        frame.quality.update(
            {
                "input_bag": str(bag_path),
                "source_script": str(Path(__file__).resolve()),
                "config_sha256": hashlib.sha256(config_path.read_bytes()).hexdigest(),
                "config_snapshot": json.loads(config_path.read_text()),
                "frames": len(points_by_frame),
                "parameters": {
                    key: str(value) if isinstance(value, Path) else value
                    for key, value in vars(args).items()
                },
            }
        )
        # 推定結果は次回の追跡処理から読み込めるJSONとして保存する。
        frame.write_json(output_path)
    except (TrackingError, CorridorFrameError, OSError, ValueError) as exc:
        print(f"Error: {exc}", file=sys.stderr)
        return 1

    print(f"Read {len(points_by_frame)} {actual_type or 'unknown'} frames.")
    print(f"Corridor frame: {output_path}")
    print(
        f"  longitudinal (+X): [{frame.longitudinal_xy[0]:+.5f}, {frame.longitudinal_xy[1]:+.5f}]"
    )
    print(f"  lateral (+Y): [{frame.lateral_xy[0]:+.5f}, {frame.lateral_xy[1]:+.5f}]")
    print(f"  center origin XY: {frame.origin_xy.tolist()}")
    print(f"  wall gap: {frame.wall_gap:.3f} m")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
