#!/usr/bin/env python3
"""保存済みのZED画像・LiDAR点群・追跡結果を画像へ描画する。

録画ごとのoffsetと時間ゲートで近い点群を選び、カメラ投影図と廊下平面図を作る。
実機は起動しない。重なり具合は校正・同期の確認材料であり、独立した真値ではない。
"""

import argparse
import csv
import hashlib
import html
import json
import math
from pathlib import Path

import cv2
import numpy as np

from track_people import (
    _iter_bag_clouds,
    _message_timestamp_ns,
    _pointcloud2_to_numpy,
    _custom_msg_to_numpy,
    load_settings,
)


def read_csv(path):
    with path.open(encoding="utf-8-sig", newline="") as stream:
        return list(csv.DictReader(stream))


def main():
    """指定時刻の近傍点群を画像と廊下図へ重ね、新規フォルダーに来歴付きで保存する。"""

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag", type=Path, required=True)
    parser.add_argument("--review-dir", type=Path, required=True)
    parser.add_argument("--tracks", type=Path, required=True)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--timestamp-offset-ms", type=float, required=True)
    parser.add_argument("--sync-note", required=True)
    parser.add_argument(
        "--times-s", nargs="+", type=float, required=True, help="ZED elapsed seconds"
    )
    parser.add_argument("--max-time-gap-ms", type=float, default=60)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()
    if args.output_dir.exists():
        parser.error("Choose a new output directory.")
    if any(
        not math.isfinite(v)
        for v in [args.timestamp_offset_ms, *args.times_s, args.max_time_gap_ms]
    ):
        parser.error("Times must be finite.")
    if args.max_time_gap_ms <= 0 or not args.sync_note.strip():
        parser.error("A positive time gate and nonempty sync note are required.")
    camera = json.loads((args.review_dir / "camera.json").read_text())
    # 古い書き出しでも、補正済み左画像と明記されていればIMAGE座標として扱う。
    # カメラ内部パラメータは、この左画像の画素と光学軸を前提とする。
    optical = str(camera.get("coordinate_system", "")).startswith("IMAGE") or (
        "coordinate_system" not in camera and camera.get("image_view") == "VIEW.LEFT (rectified)"
    )
    if camera.get("coordinate_units") != "meter" or not optical:
        parser.error("Expected rectified IMAGE camera export in meters.")
    settings = load_settings(args.config)
    if settings.corridor_frame is None or settings.lidar_to_zed_frame is None:
        parser.error("Both corridor and extrinsic calibrations are required.")
    frames = read_csv(args.review_dir / "frames.csv")
    selected = [min(frames, key=lambda r: abs(float(r["elapsed_s"]) - t)) for t in args.times_s]
    targets = [
        int(r["image_timestamp_ns"]) + round(args.timestamp_offset_ms * 1e6) for r in selected
    ]
    nearest = [None] * len(selected)
    for msg, bag_ns, kind in _iter_bag_clouds(args.bag, settings.topic, "auto", "mcap"):
        stamp = _message_timestamp_ns(msg, bag_ns)
        indices = [
            i
            for i, target in enumerate(targets)
            if abs(stamp - target) <= args.max_time_gap_ms * 1e6
            and (nearest[i] is None or abs(stamp - target) < abs(nearest[i][0] - target))
        ]
        if indices:
            points = _custom_msg_to_numpy(msg) if kind == "custom" else _pointcloud2_to_numpy(msg)
            for i in indices:
                nearest[i] = (stamp, points)
    tracks = read_csv(args.tracks)
    args.output_dir.mkdir(parents=True)
    records = []
    for index, (frame, target, cloud) in enumerate(zip(selected, targets, nearest)):
        if cloud is None:
            records.append(dict(requested_s=args.times_s[index], status="no_cloud_within_gate"))
            continue
        stamp, raw = cloud
        raw = raw[np.isfinite(raw).all(axis=1)]
        level = raw @ settings.gravity_alignment_rotation.T + settings.gravity_alignment_translation
        camera_points = (
            level @ settings.lidar_to_zed_frame.rotation.T + settings.lidar_to_zed_frame.translation
        )
        image_path = args.review_dir / frame["image_file"]
        image = cv2.imread(str(image_path))
        if image is None:
            raise ValueError(f"Cannot read {image_path}")
        if image.shape[:2] != (int(camera["height"]), int(camera["width"])):
            raise ValueError("Image size does not match intrinsics.")
        valid = (camera_points[:, 2] > 0.2) & (camera_points[:, 2] < 25)
        cp = camera_points[valid]
        uv = cp[:, :2] / cp[:, 2:3] * [camera["fx"], camera["fy"]] + [camera["cx"], camera["cy"]]
        inside = (
            (uv[:, 0] >= 0)
            & (uv[:, 0] < image.shape[1])
            & (uv[:, 1] >= 0)
            & (uv[:, 1] < image.shape[0])
        )
        cp, uv = cp[inside], np.rint(uv[inside]).astype(int)
        # 遠い点から描き、近い点を後で重ねる。遮蔽の正確な判定は行っていない。
        colors = cv2.applyColorMap(
            np.uint8(np.clip(cp[:, 2] / 16, 0, 1) * 255).reshape(-1, 1), cv2.COLORMAP_TURBO
        )
        overlay = image.copy()
        for j in np.argsort(cp[:, 2])[::-1]:
            cv2.circle(overlay, tuple(uv[j]), 1, tuple(int(c) for c in colors[j, 0]), -1)
        image = cv2.addWeighted(image, 0.45, overlay, 0.55, 0)
        corridor = settings.corridor_frame
        xy = level[:, :2] - corridor.origin_xy
        cx = xy @ corridor.longitudinal_xy
        cy = xy @ corridor.lateral_xy
        plan = np.full((500, image.shape[1], 3), 248, dtype=np.uint8)
        # 縦横を同じ縮尺にし、X=-2〜18m・Y=-3.5〜3.5mの範囲を描く。
        scale = min((plan.shape[1] - 80) / 20, (plan.shape[0] - 70) / 7)

        def pixel(x, y):
            return (int(40 + (x + 2) * scale), int(35 + (3.5 - y) * scale))

        for x in range(0, 19, 2):
            cv2.line(plan, pixel(x, -3.5), pixel(x, 3.5), (215, 215, 215), 1)
            cv2.putText(
                plan, f"{x} m", pixel(x, -3.5), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (40, 40, 40), 1
            )
        for x, y, z in zip(cx, cy, level[:, 2]):
            if -2 <= x <= 18 and abs(y) <= 3.5 and 0.1 <= z <= 2.4:
                cv2.circle(plan, pixel(x, y), 1, (140, 140, 140), -1)
        current = [
            r
            for r in tracks
            if int(r["timestamp_ns"]) == stamp and r["status"] in ("confirmed", "predicted")
        ]
        for r in current:
            color = (
                (40, 160, 20)
                if r["status"] == "confirmed" and r["detected"] == "1"
                else (20, 30, 220)
            )
            text = f"ID {r['track_id']} {r['status']} n={r['point_count']}"
            cv2.circle(plan, pixel(float(r["corridor_x"]), float(r["corridor_y"])), 7, color, 2)
            cv2.putText(
                plan,
                text,
                pixel(float(r["corridor_x"]) + 0.2, float(r["corridor_y"]) + 0.15),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.45,
                color,
                1,
            )
            x, y, z = (float(r["zed_" + a]) for a in "xyz")
            if z > 0.2:
                p = (
                    round(camera["fx"] * x / z + camera["cx"]),
                    round(camera["fy"] * y / z + camera["cy"]),
                )
                cv2.circle(image, p, 9, color, 2)
                cv2.putText(image, text, (p[0] + 10, p[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
        title = f"ZED elapsed {frame['elapsed_s']}s | cloud-image offset residual {(stamp-target)/1e6:+.2f} ms"
        cv2.rectangle(image, (0, 0), (image.shape[1], 35), (20, 20, 20), -1)
        cv2.putText(image, title, (8, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1)
        cv2.putText(
            plan,
            "Corridor X / Y [m], equal scale; raw cloud + confirmed/predicted track centers",
            (12, 20),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (20, 20, 20),
            1,
        )
        name = f"sample_{index:02d}.jpg"
        if not cv2.imwrite(str(args.output_dir / name), np.vstack((image, plan))):
            raise OSError("Could not write review image.")
        records.append(
            dict(
                file=name,
                source_image=str(image_path),
                cloud_timestamp_ns=stamp,
                image_timestamp_ns=int(frame["image_timestamp_ns"]),
                sync_residual_ms=(stamp - target) / 1e6,
                tracks=len(current),
            )
        )
    provenance = dict(
        parameters={k: str(v) if isinstance(v, Path) else v for k, v in vars(args).items()},
        records=records,
        source_sha256={
            str(p): hashlib.sha256(p.read_bytes()).hexdigest()
            for p in (
                args.config,
                args.tracks,
                settings.corridor_frame_file,
                settings.lidar_to_zed_frame_file,
                Path(__file__),
            )
        },
    )
    (args.output_dir / "manifest.json").write_text(
        json.dumps(provenance, ensure_ascii=False, indent=2) + "\n"
    )
    images = "".join(
        f'<img style="max-width:100%" src="{r["file"]}">' for r in records if "file" in r
    )
    (args.output_dir / "index.html").write_text(
        '<!doctype html><meta charset="utf-8"><title>保存センサー投影レビュー</title>'
        '<h1>ZED画像・LiDAR点群・廊下上の検出</h1>'
        '<p>色は深度。緑は確定検出、赤は予測。点の重なりは校正・同期の確認用であり、測位誤差の真値ではありません。</p>'
        f'<p>{html.escape(args.sync_note)}</p>{images}',
        encoding="utf-8",
    )
    print(args.output_dir / "index.html")


if __name__ == "__main__":
    main()
