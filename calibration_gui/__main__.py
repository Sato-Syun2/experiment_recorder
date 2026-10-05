"""Command-line entry point for the offline ZED–Livox calibration GUI."""

from __future__ import annotations

import argparse
from pathlib import Path
import sys

from .gui import launch_gui, load_initial_extrinsic
from .io.camera import load_image_and_intrinsics, load_svo_frame
from .io.pointcloud import load_point_cloud
from .io.rosbag2 import read_bag_pointcloud


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="python -m calibration_gui",
        description="Inspect and refine ZED 2i ↔ Livox Mid-360S extrinsics from offline data.",
    )
    source = parser.add_mutually_exclusive_group()
    source.add_argument("--svo", type=Path, help="ZED .svo/.svo2 recording")
    source.add_argument("--image", type=Path, help="Rectified left RGB image")
    parser.add_argument("--intrinsics", type=Path, help="K/distortion YAML or JSON, required with --image")
    parser.add_argument("--frame", type=int, default=0, help="SVO frame index (default: 0)")
    lidar = parser.add_mutually_exclusive_group()
    lidar.add_argument("--cloud", type=Path, help="PCD, PLY, NPY, or NPZ point cloud")
    lidar.add_argument("--bag", type=Path, help="rosbag2 directory containing a supported point-cloud topic")
    parser.add_argument("--topic", default="/livox/lidar", help="rosbag2 topic (default: /livox/lidar)")
    parser.add_argument("--bag-frame", type=int, default=0, help="Point-cloud message index (default: 0)")
    parser.add_argument(
        "--initial",
        type=Path,
        default=Path(__file__).with_name("default_extrinsic.json"),
        help="Initial target_from_source JSON (default: packaged nominal mount estimate)",
    )
    parser.add_argument("--demo", action="store_true", help="Start with deterministic synthetic camera/cloud data")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    if args.frame < 0 or args.bag_frame < 0:
        _parser().error("frame indices must be non-negative")
    if args.image and not args.intrinsics:
        _parser().error("--intrinsics is required with --image")
    if args.intrinsics and not args.image:
        _parser().error("--intrinsics can only be used with --image")
    if args.demo and (args.svo or args.image or args.cloud or args.bag):
        _parser().error("--demo cannot be combined with explicit camera or LiDAR input")

    try:
        initial = load_initial_extrinsic(args.initial)
        if args.demo:
            from .demo import make_demo_data

            camera, cloud = make_demo_data(initial)
            camera_file, cloud_file = camera.source, cloud.source
        else:
            camera = None
            cloud = None
            camera_file = cloud_file = ""
            if args.svo:
                camera = load_svo_frame(args.svo, args.frame)
                camera_file = f"{args.svo} [frame {args.frame}]"
            elif args.image:
                camera = load_image_and_intrinsics(args.image, args.intrinsics)
                camera_file = f"{args.image} | {args.intrinsics}"
            if args.cloud:
                cloud = load_point_cloud(args.cloud)
                cloud_file = str(args.cloud)
            elif args.bag:
                frame = read_bag_pointcloud(args.bag, topic=args.topic, frame_index=args.bag_frame)
                cloud = frame.cloud
                cloud_file = f"{args.bag} [{args.topic} frame {args.bag_frame}, {frame.message_type}]"
        return launch_gui(camera, cloud, initial, camera_file=camera_file, cloud_file=cloud_file)
    except Exception as exc:
        print(f"calibration_gui: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
