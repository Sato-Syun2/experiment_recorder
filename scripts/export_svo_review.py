#!/usr/bin/env python3
"""Export timestamped review frames and optional metric depth from a ZED SVO2.

Run this on a machine with the matching ZED SDK Python binding and NVIDIA
runtime. The regular workspace can then inspect camera images alongside a
rosbag without needing to decode SVO2 locally.
"""

from __future__ import annotations

import argparse
import csv
import importlib.metadata
import json
import math
import sys
from pathlib import Path
from typing import Optional, Sequence

import cv2
import numpy as np


def _timestamp_ns(zed: object, sl: object) -> int:
    value = zed.get_timestamp(sl.TIME_REFERENCE.IMAGE)
    if hasattr(value, "get_nanoseconds"):
        return int(value.get_nanoseconds())
    if hasattr(value, "data_ns"):
        return int(value.data_ns)
    raise RuntimeError("ZED SDK did not return an image timestamp in nanoseconds.")


def _write_contact_sheet(frame_rows: list[dict[str, object]], output_path: Path) -> None:
    tile_width, tile_height, label_height, columns = 320, 180, 24, 4
    rows = math.ceil(len(frame_rows) / columns)
    canvas = np.zeros((rows * (tile_height + label_height), columns * tile_width, 3), dtype=np.uint8)
    for index, row in enumerate(frame_rows):
        image = cv2.imread(str(output_path.parent / str(row["image_file"])), cv2.IMREAD_COLOR)
        if image is None:
            continue
        tile = cv2.resize(image, (tile_width, tile_height), interpolation=cv2.INTER_AREA)
        x = (index % columns) * tile_width
        y = (index // columns) * (tile_height + label_height)
        canvas[y : y + tile_height, x : x + tile_width] = tile
        cv2.rectangle(canvas, (x, y + tile_height), (x + tile_width - 1, y + tile_height + label_height - 1), (20, 20, 20), -1)
        label = f"f={row['frame_index']}  t={float(row['elapsed_s']):.2f}s"
        cv2.putText(canvas, label, (x + 8, y + tile_height + 17), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
    if not cv2.imwrite(str(output_path), canvas, [cv2.IMWRITE_JPEG_QUALITY, 88]):
        raise RuntimeError(f"Could not write contact sheet: {output_path}")


def export_review(
    svo_path: Path,
    output_dir: Path,
    sample_period_s: float,
    include_depth: bool,
) -> None:
    try:
        import pyzed.sl as sl
    except Exception as exc:
        raise RuntimeError(
            "Could not import pyzed.sl. Run this on a PC with the matching ZED SDK "
            f"Python binding and NVIDIA runtime. Import error: {exc}"
        ) from exc

    svo_path = svo_path.expanduser().resolve()
    if not svo_path.is_file():
        raise RuntimeError(f"SVO file not found: {svo_path}")
    if not math.isfinite(sample_period_s) or sample_period_s <= 0:
        raise RuntimeError("--every-seconds must be a finite positive number.")

    output_dir = output_dir.expanduser().resolve()
    image_dir = output_dir / "frames"
    depth_dir = output_dir / "depth"
    image_dir.mkdir(parents=True, exist_ok=True)
    if include_depth:
        depth_dir.mkdir(parents=True, exist_ok=True)

    zed = sl.Camera()
    init = sl.InitParameters()
    init.set_from_svo_file(str(svo_path))
    init.svo_real_time_mode = False
    init.coordinate_units = sl.UNIT.METER
    init.depth_mode = sl.DEPTH_MODE.NEURAL if include_depth else sl.DEPTH_MODE.NONE

    status = zed.open(init)
    if status != sl.ERROR_CODE.SUCCESS:
        zed.close()
        raise RuntimeError(f"Could not open SVO2 with ZED SDK: {status}")

    rows: list[dict[str, object]] = []
    first_timestamp_ns: Optional[int] = None
    frame_index = 0
    image_mat, depth_mat = sl.Mat(), sl.Mat()
    runtime = sl.RuntimeParameters()
    try:
        camera_info = zed.get_camera_information()
        camera_config = camera_info.camera_configuration
        fps = float(camera_config.fps)
        stride = max(1, int(round(fps * sample_period_s)))
        left = camera_config.calibration_parameters.left_cam
        resolution = camera_config.resolution
        try:
            sdk_version = importlib.metadata.version("pyzed")
        except importlib.metadata.PackageNotFoundError:
            sdk_version = "unknown"
        metadata = {
            "source_svo": str(svo_path),
            "sdk_version": sdk_version,
            "camera_model": str(getattr(camera_info, "camera_model", "unknown")),
            "serial_number": int(getattr(camera_info, "serial_number", 0)),
            "fps": fps,
            "width": int(resolution.width),
            "height": int(resolution.height),
            "fx": float(left.fx),
            "fy": float(left.fy),
            "cx": float(left.cx),
            "cy": float(left.cy),
            "image_view": "VIEW.LEFT (rectified)",
            "coordinate_units": "meter",
            "depth_mode": "NEURAL" if include_depth else "disabled",
            "sample_period_requested_s": sample_period_s,
            "sample_stride_frames": stride,
        }

        while True:
            status = zed.grab(runtime)
            if status == sl.ERROR_CODE.END_OF_SVOFILE_REACHED:
                break
            if status != sl.ERROR_CODE.SUCCESS:
                raise RuntimeError(f"Failed while reading SVO frame {frame_index}: {status}")

            if frame_index % stride == 0:
                timestamp_ns = _timestamp_ns(zed, sl)
                if first_timestamp_ns is None:
                    first_timestamp_ns = timestamp_ns
                sample_index = len(rows)
                image_name = f"frame_{sample_index:04d}_f{frame_index:06d}.jpg"
                image_mat.free(sl.MEM.CPU)
                zed.retrieve_image(image_mat, sl.VIEW.LEFT, sl.MEM.CPU)
                pixels = np.asarray(image_mat.get_data())
                if pixels.ndim != 3 or pixels.shape[2] not in (3, 4):
                    raise RuntimeError(f"Unexpected ZED image shape: {pixels.shape}")
                bgr = cv2.cvtColor(pixels, cv2.COLOR_BGRA2BGR) if pixels.shape[2] == 4 else pixels
                image_path = image_dir / image_name
                if not cv2.imwrite(str(image_path), bgr, [cv2.IMWRITE_JPEG_QUALITY, 92]):
                    raise RuntimeError(f"Could not write image: {image_path}")

                depth_name = ""
                if include_depth:
                    depth_mat.free(sl.MEM.CPU)
                    zed.retrieve_measure(depth_mat, sl.MEASURE.DEPTH, sl.MEM.CPU)
                    depth_name = f"depth_{sample_index:04d}_f{frame_index:06d}.npz"
                    depth_m = np.asarray(depth_mat.get_data(), dtype=np.float32).squeeze()
                    if depth_m.ndim != 2:
                        raise RuntimeError(f"Unexpected ZED depth shape: {depth_m.shape}")
                    np.savez_compressed(depth_dir / depth_name, depth_m=depth_m.astype(np.float16))

                rows.append(
                    {
                        "sample_index": sample_index,
                        "frame_index": frame_index,
                        "elapsed_s": (timestamp_ns - first_timestamp_ns) / 1_000_000_000.0,
                        "image_timestamp_ns": timestamp_ns,
                        "image_file": f"frames/{image_name}",
                        "depth_file": f"depth/{depth_name}" if depth_name else "",
                    }
                )
            frame_index += 1
            if frame_index % max(1, int(round(fps * 5))) == 0:
                print(f"Read {frame_index} SVO frames; exported {len(rows)} review samples.")

        if not rows:
            raise RuntimeError("No SVO frames were exported.")
        metadata["total_svo_frames"] = frame_index
        metadata["exported_samples"] = len(rows)
        metadata["first_image_timestamp_ns"] = rows[0]["image_timestamp_ns"]
        metadata["last_image_timestamp_ns"] = rows[-1]["image_timestamp_ns"]
        (output_dir / "camera.json").write_text(
            json.dumps(metadata, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
        )
        with (output_dir / "frames.csv").open("w", encoding="utf-8", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
            writer.writeheader()
            writer.writerows(rows)
        _write_contact_sheet(rows, output_dir / "contact_sheet.jpg")
        print(f"Exported {len(rows)} samples from {frame_index} SVO frames to {output_dir}")
    finally:
        zed.close()


def _parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("svo", type=Path, help="ZED .svo or .svo2 file")
    parser.add_argument("--output", type=Path, required=True, help="review output directory")
    parser.add_argument(
        "--every-seconds", type=float, default=0.5,
        help="sample interval in seconds (default: 0.5)",
    )
    parser.add_argument(
        "--include-depth", action="store_true",
        help="also export metric ZED depth maps (requires GPU depth support)",
    )
    return parser.parse_args(argv)


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = _parse_args(argv)
    try:
        export_review(args.svo, args.output, args.every_seconds, args.include_depth)
    except (OSError, RuntimeError, ValueError) as exc:
        print(f"Error: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
