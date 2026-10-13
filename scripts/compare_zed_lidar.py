#!/usr/bin/env python3
"""Create an offline visual comparison of ZED depth and LiDAR tracks.

The ZED export contains registered left images and depth maps, but no semantic
person detections. This script associates a single moving foreground region
with a confirmed LiDAR track using a static empty-hallway background model.
The LiDAR projection is shown as a separate overlay so calibration error is
visible in the review page.
"""

from __future__ import annotations

import argparse
import csv
import html
import json
import math
from pathlib import Path
from typing import Any
import warnings

import cv2
import numpy as np


DEFAULT_ROOT = Path("/data/experiment_data/recording_001_svo_review")
PROJECT_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_TRACKS = PROJECT_ROOT / "tracking/20261013_group001_recording_001_gravity_aligned_tracks.csv"
DEFAULT_OUTPUT = PROJECT_ROOT / "tracking/20261013_group001_recording_001_zed_lidar_comparison.html"


def read_csv(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8-sig") as handle:
        return list(csv.DictReader(handle))


def valid_depth(array: np.ndarray) -> np.ndarray:
    return np.isfinite(array) & (array > 0.2) & (array < 100.0)


def make_background(review_root: Path, frames: list[dict[str, str]], count: int) -> tuple[np.ndarray, np.ndarray]:
    selected = frames[: min(count, len(frames))]
    gray_frames: list[np.ndarray] = []
    depth_frames: list[np.ndarray] = []
    for row in selected:
        image = cv2.imread(str(review_root / row["image_file"]), cv2.IMREAD_COLOR)
        if image is None:
            raise RuntimeError(f"Cannot read image: {row['image_file']}")
        gray_frames.append(cv2.cvtColor(image, cv2.COLOR_BGR2GRAY))
        with np.load(review_root / row["depth_file"]) as archive:
            depth = archive["depth_m"].astype(np.float32)
        depth[~valid_depth(depth)] = np.nan
        depth_frames.append(depth)
    # Early samples are empty in this recording; median suppresses depth noise.
    gray_background = np.median(np.stack(gray_frames), axis=0).astype(np.uint8)
    with warnings.catch_warnings():
        warnings.simplefilter("ignore", RuntimeWarning)
        depth_background = np.nanmedian(np.stack(depth_frames), axis=0).astype(np.float32)
    return gray_background, depth_background


def choose_person_component(
    gray: np.ndarray,
    gray_background: np.ndarray,
    projected_u: float,
    projected_v: float,
    lidar_z: float,
    diff_threshold: int,
) -> tuple[np.ndarray | None, tuple[int, int, int, int] | None, np.ndarray]:
    """Find the moving upright image region nearest the projected track."""
    difference = cv2.absdiff(gray_background, gray)
    foreground = (difference >= diff_threshold).astype(np.uint8)
    roi = np.zeros_like(foreground)
    roi[100:660, 480:820] = 1
    foreground *= roi
    foreground = cv2.morphologyEx(foreground, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    foreground = cv2.morphologyEx(foreground, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))

    count, labels, stats, centers = cv2.connectedComponentsWithStats(foreground, 8)
    expected_height = max(20.0, 1.70 * 523.764221 / max(lidar_z, 0.5))
    candidates: list[tuple[float, int, int, int, int, int]] = []
    for label in range(1, count):
        x, y, width, height, area = map(int, stats[label])
        center_x, center_y = map(float, centers[label])
        if area < 100 or height < max(20.0, 0.38 * expected_height):
            continue
        if width < 5 or height / max(width, 1) < 1.25:
            continue
        if abs(center_x - projected_u) > 120:
            continue
        if y > projected_v + 35 or y + height < projected_v - 1.25 * expected_height:
            continue
        # Favor the projected corridor position while avoiding tiny fragments.
        score = abs(center_x - projected_u) + 0.12 * abs(center_y - (projected_v - 0.2 * expected_height))
        score -= 0.0015 * min(area, 6000)
        candidates.append((score, label, x, y, width, height))
    if not candidates:
        return None, None, labels
    _, label, x, y, width, height = min(candidates)
    return labels == label, (x, y, width, height), labels


def depth_person_position(
    component_mask: np.ndarray,
    bbox: tuple[int, int, int, int],
    depth: np.ndarray,
    background_depth: np.ndarray,
    intrinsics: dict[str, float],
) -> dict[str, Any] | None:
    x, y, width, height = bbox
    valid = valid_depth(depth) & np.isfinite(background_depth)
    # Keep pixels that are closer than the empty-hallway depth to remove shadows.
    foreground_depth = valid & (background_depth - depth > 0.15)
    component_depth = component_mask & foreground_depth
    yy, xx = np.where(component_depth)
    if len(xx) < 30:
        component_depth = component_mask & valid
        yy, xx = np.where(component_depth)
    if len(xx) < 30:
        return None

    z_guess = float(np.median(depth[yy, xx]))
    expected_height = max(18, int(round(intrinsics["fy"] * 1.70 / max(z_guess, 0.5))))
    body_bottom = min(y + height, y + expected_height)
    body_mask = component_depth.copy()
    body_mask[body_bottom:, :] = False
    yy, xx = np.where(body_mask)
    if len(xx) < 30:
        yy, xx = np.where(component_depth)
    if len(xx) < 30:
        return None

    z = float(np.median(depth[yy, xx]))
    u = float(np.median(xx))
    v = float(np.median(yy))
    x_m = (u - intrinsics["cx"]) * z / intrinsics["fx"]
    y_m = (v - intrinsics["cy"]) * z / intrinsics["fy"]
    return {
        "zed_x": x_m,
        "zed_y": y_m,
        "zed_z": z,
        "u": u,
        "v": v,
        "pixels": int(len(xx)),
        "bbox": [x, y, width, max(1, body_bottom - y)],
    }


def colorize_depth(depth: np.ndarray, valid: np.ndarray, max_range: float = 16.0) -> np.ndarray:
    scaled = np.nan_to_num(np.clip(depth / max_range, 0.0, 1.0), nan=0.0, posinf=1.0, neginf=0.0)
    # Near objects are warm; distant objects are cool. Invalid pixels are black.
    image = cv2.applyColorMap((255.0 * scaled).astype(np.uint8), cv2.COLORMAP_TURBO)
    image[~valid] = (0, 0, 0)
    return image


def draw_overlay(
    image: np.ndarray,
    bbox: tuple[int, int, int, int] | None,
    depth_result: dict[str, Any] | None,
    lidar_pixel: tuple[float, float],
    *,
    is_depth: bool,
) -> np.ndarray:
    output = image.copy()
    if bbox is not None:
        x, y, width, height = bbox
        if depth_result:
            _, _, _, body_height = depth_result["bbox"]
            height = min(height, body_height)
        cv2.rectangle(output, (x, y), (x + width, y + height), (60, 255, 80), 3)
    if depth_result:
        cv2.circle(output, (round(depth_result["u"]), round(depth_result["v"])), 9, (0, 230, 255), 3)
    u, v = map(lambda value: int(round(value)), lidar_pixel)
    cv2.drawMarker(output, (u, v), (255, 0, 255), cv2.MARKER_CROSS, 26, 4)
    legend = "Green: RGB foreground ROI   Yellow: ZED depth median   Magenta: LiDAR centroid projection"
    if is_depth:
        legend = "Green: RGB foreground ROI   Yellow: ZED point   Magenta: LiDAR projected point"
    cv2.rectangle(output, (0, 0), (output.shape[1], 42), (10, 10, 10), -1)
    cv2.putText(output, legend, (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.58, (255, 255, 255), 1, cv2.LINE_AA)
    return output


def summarize(records: list[dict[str, Any]]) -> dict[str, Any]:
    valid = [row for row in records if row["zed_depth_m"] is not None]
    if not valid:
        return {"matched_frames": len(records), "depth_samples": 0}
    abs_depth_error = np.asarray([abs(row["depth_error_m"]) for row in valid], dtype=float)
    xyz_error = np.asarray([row["xyz_error_m"] for row in valid], dtype=float)
    pixel_error = np.asarray([row["pixel_error_px"] for row in valid], dtype=float)
    vertical_error = np.asarray([row["pixel_dv_px"] for row in valid], dtype=float)
    far = [row for row in valid if row["lidar_z_m"] >= 10.0]
    return {
        "matched_frames": len(records),
        "depth_samples": len(valid),
        "median_abs_depth_error_m": float(np.median(abs_depth_error)),
        "p90_abs_depth_error_m": float(np.percentile(abs_depth_error, 90)),
        "median_3d_centroid_difference_m": float(np.median(xyz_error)),
        "p90_3d_centroid_difference_m": float(np.percentile(xyz_error, 90)),
        "median_pixel_center_difference_px": float(np.median(pixel_error)),
        "median_vertical_pixel_difference_px": float(np.median(vertical_error)),
        "far_10m_plus_frames": len(far),
        "far_10m_plus_median_abs_depth_error_m": float(np.median([abs(r["depth_error_m"]) for r in far])) if far else None,
        "max_lidar_z_m": max(row["lidar_z_m"] for row in records),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--review-dir", type=Path, default=DEFAULT_ROOT)
    parser.add_argument("--tracks", type=Path, default=DEFAULT_TRACKS)
    parser.add_argument("--camera-json", type=Path, default=None)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--track-id", default="45")
    parser.add_argument("--lidar-time-offset-s", type=float, default=1.675882,
                        help="LiDAR timestamp minus ZED elapsed time for the synchronized recording.")
    parser.add_argument("--background-frames", type=int, default=28)
    parser.add_argument("--diff-threshold", type=int, default=20)
    args = parser.parse_args()

    camera_path = args.camera_json or (args.review_dir / "camera.json")
    camera = json.loads(camera_path.read_text(encoding="utf-8"))
    intrinsics = {key: float(camera[key]) for key in ("fx", "fy", "cx", "cy")}
    frames = read_csv(args.review_dir / "frames.csv")
    tracks = read_csv(args.tracks)
    tracks = [r for r in tracks if r["track_id"] == args.track_id and r["status"] == "confirmed" and r["detected"] == "1"]
    if not tracks:
        raise RuntimeError(f"No confirmed detections for track {args.track_id}")
    gray_background, depth_background = make_background(args.review_dir, frames, args.background_frames)
    track_times = np.asarray([float(row["time_sec"]) for row in tracks])

    output = args.output.resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    assets = output.with_name(output.stem + "_frames")
    assets.mkdir(parents=True, exist_ok=True)
    records: list[dict[str, Any]] = []

    for frame in frames:
        camera_time = float(frame["elapsed_s"])
        target_lidar_time = camera_time + args.lidar_time_offset_s
        closest_index = int(np.argmin(np.abs(track_times - target_lidar_time)))
        lidar = tracks[closest_index]
        lidar_time = float(lidar["time_sec"])
        sync_error = lidar_time - target_lidar_time
        if abs(sync_error) > 0.18:
            continue
        lidar_x, lidar_y, lidar_z = (float(lidar[name]) for name in ("zed_x", "zed_y", "zed_z"))
        projected_u = intrinsics["fx"] * lidar_x / lidar_z + intrinsics["cx"]
        projected_v = intrinsics["fy"] * lidar_y / lidar_z + intrinsics["cy"]
        image_path = args.review_dir / frame["image_file"]
        image = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
        if image is None:
            continue
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        component_mask, component_bbox, _labels = choose_person_component(
            gray, gray_background, projected_u, projected_v, lidar_z, args.diff_threshold
        )
        with np.load(args.review_dir / frame["depth_file"]) as archive:
            depth = archive["depth_m"].astype(np.float32)
        depth_result = None
        if component_mask is not None and component_bbox is not None:
            depth_result = depth_person_position(
                component_mask, component_bbox, depth, depth_background, intrinsics
            )

        record: dict[str, Any] = {
            "sample_index": int(frame["sample_index"]),
            "camera_elapsed_s": camera_time,
            "lidar_time_s": lidar_time,
            "sync_residual_s": sync_error,
            "lidar_z_m": lidar_z,
            "lidar_x_m": lidar_x,
            "lidar_y_m": lidar_y,
            "point_count": int(float(lidar["point_count"])),
            "projected_u_px": projected_u,
            "projected_v_px": projected_v,
            "person_bbox": component_bbox,
            "zed_depth_m": None if depth_result is None else depth_result["zed_z"],
            "zed_x_m": None if depth_result is None else depth_result["zed_x"],
            "zed_y_m": None if depth_result is None else depth_result["zed_y"],
            "zed_u_px": None if depth_result is None else depth_result["u"],
            "zed_v_px": None if depth_result is None else depth_result["v"],
            "zed_depth_pixels": 0 if depth_result is None else depth_result["pixels"],
            "depth_error_m": None if depth_result is None else depth_result["zed_z"] - lidar_z,
            "xyz_error_m": None if depth_result is None else math.dist(
                (depth_result["zed_x"], depth_result["zed_y"], depth_result["zed_z"]),
                (lidar_x, lidar_y, lidar_z),
            ),
            "pixel_dx_px": None if depth_result is None else depth_result["u"] - projected_u,
            "pixel_dv_px": None if depth_result is None else depth_result["v"] - projected_v,
            "pixel_error_px": None if depth_result is None else math.hypot(
                depth_result["u"] - projected_u, depth_result["v"] - projected_v
            ),
            "frame_file": frame["image_file"],
        }
        records.append(record)

        rgb_overlay = draw_overlay(image, component_bbox, depth_result, (projected_u, projected_v), is_depth=False)
        depth_view = colorize_depth(depth, valid_depth(depth))
        depth_bbox = component_bbox
        depth_overlay = draw_overlay(depth_view, depth_bbox, depth_result, (projected_u, projected_v), is_depth=True)
        panel_width, panel_height = 640, 360
        rgb_panel = cv2.resize(rgb_overlay, (panel_width, panel_height), interpolation=cv2.INTER_AREA)
        depth_panel = cv2.resize(depth_overlay, (panel_width, panel_height), interpolation=cv2.INTER_AREA)
        composite = np.hstack((rgb_panel, depth_panel))
        title = f"ZED {camera_time:.2f}s | LiDAR {lidar_time:.2f}s | range {lidar_z:.2f}m"
        cv2.rectangle(composite, (0, 42), (composite.shape[1], 74), (12, 12, 12), -1)
        cv2.putText(composite, title, (12, 65), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2, cv2.LINE_AA)
        cv2.imwrite(str(assets / f"sample_{len(records)-1:03d}.jpg"), composite, [cv2.IMWRITE_JPEG_QUALITY, 88])

    metrics = summarize(records)
    csv_path = output.with_suffix(".csv")
    json_path = output.with_suffix(".json")
    fields = [
        "sample_index", "camera_elapsed_s", "lidar_time_s", "sync_residual_s", "lidar_z_m",
        "lidar_x_m", "lidar_y_m", "point_count", "projected_u_px", "projected_v_px",
        "person_bbox", "zed_depth_m", "zed_x_m", "zed_y_m", "zed_u_px", "zed_v_px",
        "zed_depth_pixels", "depth_error_m", "xyz_error_m", "pixel_dx_px", "pixel_dv_px",
        "pixel_error_px", "frame_file",
    ]
    with csv_path.open("w", newline="", encoding="utf-8-sig") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        for record in records:
            row = dict(record)
            row["person_bbox"] = "" if row["person_bbox"] is None else ",".join(map(str, row["person_bbox"]))
            writer.writerow(row)
    json_path.write_text(json.dumps({"metrics": metrics, "records": records}, ensure_ascii=False, indent=2), encoding="utf-8")

    metric_html = "".join(
        f"<div class='metric'><b>{html.escape(label)}</b><span>{html.escape(value)}</span></div>"
        for label, value in [
            ("同期済みフレーム", f"{metrics['matched_frames']} 枚"),
            ("深度ROIを抽出", f"{metrics['depth_samples']} 枚"),
            ("Z深度差 中央値 / 90%点", f"{metrics.get('median_abs_depth_error_m', float('nan')):.2f} / {metrics.get('p90_abs_depth_error_m', float('nan')):.2f} m"),
            ("3D位置差 中央値 / 90%点", f"{metrics.get('median_3d_centroid_difference_m', float('nan')):.2f} / {metrics.get('p90_3d_centroid_difference_m', float('nan')):.2f} m"),
            ("投影点との画素差 中央値", f"{metrics.get('median_pixel_center_difference_px', float('nan')):.1f} px"),
            ("10m以上の深度差 中央値", f"{metrics.get('far_10m_plus_median_abs_depth_error_m', float('nan')):.2f} m"),
            ("LiDAR最大距離", f"{metrics.get('max_lidar_z_m', float('nan')):.2f} m"),
        ]
    )
    rows_html = []
    for index, record in enumerate(records):
        zed_depth = "—" if record["zed_depth_m"] is None else f"{record['zed_depth_m']:.2f} m"
        error = "—" if record["depth_error_m"] is None else f"{record['depth_error_m']:+.2f} m"
        pixel = "—" if record["pixel_error_px"] is None else f"{record['pixel_error_px']:.1f} px"
        rows_html.append(
            f"<tr onclick='showFrame({index})'><td>{record['camera_elapsed_s']:.2f}</td>"
            f"<td>{record['lidar_z_m']:.2f} m</td><td>{zed_depth}</td><td>{error}</td>"
            f"<td>{pixel}</td><td>{record['point_count']}</td></tr>"
        )
    initial_image = f"{assets.name}/sample_000.jpg" if records else ""
    data = [
        {
            "image": f"{assets.name}/sample_{i:03d}.jpg",
            "camera_s": r["camera_elapsed_s"], "lidar_s": r["lidar_time_s"],
            "range": r["lidar_z_m"], "zed": r["zed_depth_m"],
            "depth_error": r["depth_error_m"], "pixel_error": r["pixel_error_px"],
        }
        for i, r in enumerate(records)
    ]
    page = f"""<!doctype html><html lang="ja"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>ZED深度とLiDAR位置の比較</title><style>
body{{font:15px/1.55 system-ui,sans-serif;margin:24px auto;padding:0 16px;max-width:1200px;color:#20252b;background:#f5f7f9}}
h1{{font-size:1.55rem;margin:.2rem 0}} .note{{background:#fff8df;border-left:4px solid #e0aa23;padding:12px 15px;margin:14px 0}}
.metrics{{display:grid;grid-template-columns:repeat(auto-fit,minmax(210px,1fr));gap:8px;margin:14px 0}}
.metric{{background:white;border:1px solid #d9e0e7;border-radius:7px;padding:9px 12px}} .metric b{{display:block;font-size:.82rem;color:#596674}} .metric span{{font-size:1.08rem}}
.viewer{{background:white;padding:12px;border-radius:8px;border:1px solid #d9e0e7}} .viewer img{{display:block;width:100%;height:auto}}
.controls{{display:flex;align-items:center;gap:12px;margin-bottom:8px}} input{{flex:1}} button{{padding:6px 12px}}
table{{border-collapse:collapse;width:100%;background:white;margin-top:14px}} th,td{{padding:7px 9px;border:1px solid #d9e0e7;text-align:right}} th{{background:#eaf0f5;position:sticky;top:0}} tbody tr{{cursor:pointer}} tbody tr:hover{{background:#eef8ff}}
small{{color:#596674}} code{{background:#eef0f2;padding:1px 4px}}
</style><body><h1>ZED深度とLiDAR人物位置の比較</h1>
<p>録画内の確定トラック ID {html.escape(args.track_id)} を、ZEDのRGB・登録済み深度と照合したレビューです。</p>
<div class="note"><b>読み方：</b>緑枠は空の廊下との差分から抽出した移動領域、黄色点はその領域内のZED深度中央値、紫十字はLiDAR重心をZED画像へ投影した位置です。ZED SDKの人物認識結果ではなく、深度と画像の前景差分を使った候補位置です。紫十字の投影には初期取り付け値の外部パラメーターを使うため、上下ずれは外部校正誤差も含みます。</div>
<div class="metrics">{metric_html}</div>
<div class="viewer"><div class="controls"><button onclick="move(-1)">前へ</button><input id="slider" type="range" min="0" max="{max(0,len(records)-1)}" value="0" oninput="showFrame(+this.value)"><button onclick="move(1)">次へ</button></div>
<div id="caption"></div><img id="frame" src="{initial_image}" alt="同期したRGBと深度の比較"></div>
<small>左右の画像は左がRGB、右がZED深度（表示範囲 0.2〜16 m）です。深度領域内のピクセルをカメラ内部パラメーターで3D化しています。照合時刻は camera elapsed + {args.lidar_time_offset_s:.6f}s です。</small>
<h2>フレーム別の数値</h2><table><thead><tr><th>ZED時刻</th><th>LiDAR Z</th><th>ZED深度中央値</th><th>深度差</th><th>2D位置差</th><th>LiDAR点数</th></tr></thead><tbody>{''.join(rows_html)}</tbody></table>
<script>const frames={json.dumps(data,ensure_ascii=False)};function showFrame(i){{i=Math.max(0,Math.min(frames.length-1,i));const f=frames[i];document.getElementById('slider').value=i;document.getElementById('frame').src=f.image;document.getElementById('caption').textContent=`${{i+1}}/${{frames.length}}  ZED ${{f.camera_s.toFixed(2)}} s / LiDAR ${{f.lidar_s.toFixed(2)}} s / LiDAR ${{f.range.toFixed(2)}} m / ZED深度 ${{f.zed===null?'抽出なし':f.zed.toFixed(2)+' m'}} / 深度差 ${{f.depth_error===null?'—':(f.depth_error>=0?'+':'')+f.depth_error.toFixed(2)+' m'}} / 2D差 ${{f.pixel_error===null?'—':f.pixel_error.toFixed(1)+' px'}}`;}}function move(n){{showFrame(+document.getElementById('slider').value+n)}}showFrame(0);</script></body></html>"""
    output.write_text(page, encoding="utf-8")
    print(f"HTML: {output}")
    print(f"CSV:  {csv_path}")
    print(f"JSON: {json_path}")
    print(json.dumps(metrics, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
