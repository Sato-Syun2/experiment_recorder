#!/usr/bin/env python3
"""Assess one-person walking from LiDAR and ZED outputs independently."""

from __future__ import annotations

import csv
import json
import math
from pathlib import Path
from typing import Any

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
TRACKS = ROOT / "reports/review_20261020/final/tracks.csv"
BODY_DIR = Path("/data/experiment_data/recording_001_zed_body_tracking")
OUT = Path(__file__).resolve().parent
TRACK_ID = "44"


def rows_from(path: Path) -> list[dict[str, str]]:
    with path.open(encoding="utf-8-sig", newline="") as stream:
        return list(csv.DictReader(stream))


def percentiles(values: np.ndarray, keys: tuple[int, ...] = (0, 10, 50, 90, 95, 100)) -> dict[str, float]:
    return {str(key): float(np.percentile(values, key)) for key in keys} if len(values) else {}


def line_svg(points: list[tuple[float, float]], path: Path, title: str, x_label: str, y_label: str) -> None:
    width, height = 900, 520
    margin = (82, 28, 48, 58)
    x0, x1 = min(x for x, _ in points), max(x for x, _ in points)
    y0, y1 = min(y for _, y in points), max(y for _, y in points)
    if x1 <= x0:
        x1 = x0 + 1
    if y1 <= y0:
        y1 = y0 + 1
    left, top, right, bottom = margin
    pw, ph = width - left - right, height - top - bottom
    xy = [(left + (x - x0) / (x1 - x0) * pw, top + (y1 - y) / (y1 - y0) * ph) for x, y in points]
    polyline = " ".join(f"{x:.1f},{y:.1f}" for x, y in xy)
    ticks_x = [x0 + (x1 - x0) * i / 5 for i in range(6)]
    ticks_y = [y0 + (y1 - y0) * i / 5 for i in range(6)]
    parts = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
        '<rect width="100%" height="100%" fill="white"/>',
        f'<text x="{width/2}" y="30" text-anchor="middle" font-family="sans-serif" font-size="20">{title}</text>',
    ]
    for tick in ticks_x:
        px = left + (tick - x0) / (x1 - x0) * pw
        parts.append(f'<line x1="{px:.1f}" y1="{top}" x2="{px:.1f}" y2="{top+ph}" stroke="#e2e8f0"/>')
        parts.append(f'<text x="{px:.1f}" y="{top+ph+22}" text-anchor="middle" font-family="sans-serif" font-size="12">{tick:.1f}</text>')
    for tick in ticks_y:
        py = top + (y1 - tick) / (y1 - y0) * ph
        parts.append(f'<line x1="{left}" y1="{py:.1f}" x2="{left+pw}" y2="{py:.1f}" stroke="#e2e8f0"/>')
        parts.append(f'<text x="{left-10}" y="{py+4:.1f}" text-anchor="end" font-family="sans-serif" font-size="12">{tick:.2f}</text>')
    parts.extend(
        [
            f'<polyline points="{polyline}" fill="none" stroke="#0f766e" stroke-width="2.2"/>',
            f'<text x="{left+pw/2}" y="{height-8}" text-anchor="middle" font-family="sans-serif" font-size="14">{x_label}</text>',
            f'<text x="18" y="{top+ph/2}" transform="rotate(-90 18 {top+ph/2})" text-anchor="middle" font-family="sans-serif" font-size="14">{y_label}</text>',
            '</svg>',
        ]
    )
    path.write_text("\n".join(parts), encoding="utf-8")


def main() -> None:
    track_rows = [
        row for row in rows_from(TRACKS)
        if row.get("track_id") == TRACK_ID and row.get("status") == "confirmed" and row.get("detected") == "1"
    ]
    track_rows.sort(key=lambda row: int(row["timestamp_ns"]))
    if len(track_rows) < 3:
        raise RuntimeError(f"Need at least three confirmed observations for LiDAR track {TRACK_ID}")
    t = np.asarray([int(row["timestamp_ns"]) for row in track_rows], dtype=np.float64) / 1e9
    x = np.asarray([float(row["corridor_x"]) for row in track_rows])
    y = np.asarray([float(row["corridor_y"]) for row in track_rows])
    z = np.asarray([float(row["corridor_z"]) for row in track_rows])
    dt = np.diff(t)
    dx, dy = np.diff(x), np.diff(y)
    speed = np.hypot(dx, dy) / dt
    usable = np.isfinite(speed) & (dt > 0) & (dt <= 0.25)
    usable_speed = speed[usable]
    turn = int(np.argmax(x))
    outbound_deltas = np.diff(x[: turn + 1])
    return_deltas = np.diff(x[turn:])
    point_count = np.asarray([int(row["point_count"]) for row in track_rows], dtype=float)

    bodies = rows_from(BODY_DIR / "bodies.csv")
    ok_bodies = [row for row in bodies if row.get("tracking_state") == "OK"]
    body_camera = json.loads((BODY_DIR / "camera.json").read_text(encoding="utf-8"))
    keypoint_names = body_camera["keypoint_names"]
    body_confidence = np.asarray([float(row["confidence"]) for row in ok_bodies])
    body_elapsed = np.asarray(
        [int(row["image_timestamp_ns"]) for row in ok_bodies], dtype=np.float64
    ) / 1e9
    body_distance = np.asarray([float(row["position_camera_z_m"]) for row in ok_bodies])
    keypoint_confidence = np.asarray(
        [json.loads(row["keypoint_confidence_json"]) for row in ok_bodies], dtype=float
    )
    joint_names = ("PELVIS", "LEFT_KNEE", "RIGHT_KNEE", "LEFT_ANKLE", "RIGHT_ANKLE")
    distance_bins = ((0.0, 3.0), (3.0, 6.0), (6.0, 9.0), (9.0, 100.0))
    joint_summary: dict[str, Any] = {}
    for joint_name in joint_names:
        j = keypoint_names.index(joint_name)
        values = keypoint_confidence[:, j]
        joint_summary[joint_name] = {
            "median": float(np.nanmedian(values)),
            "p10": float(np.nanpercentile(values, 10)),
            "fraction_at_least_50": float(np.mean(values >= 50.0)),
            "fraction_at_least_80": float(np.mean(values >= 80.0)),
            "median_by_distance_m": {
                f"{lo:g}-{hi:g}": (
                    float(np.nanmedian(values[(body_distance >= lo) & (body_distance < hi)]))
                    if np.any((body_distance >= lo) & (body_distance < hi)) else None
                )
                for lo, hi in distance_bins
            },
        }

    metrics = {
        "recording": "20261013_group001/recording_001",
        "scope": "LiDAR-only corridor track and ZED-only body-tracking output; no LiDAR-to-ZED transform or fused trajectory is used.",
        "lidar": {
            "track_id": TRACK_ID,
            "source_rows": str(TRACKS),
            "confirmed_detected_observations": int(len(track_rows)),
            "duration_s": float(t[-1] - t[0]),
            "median_observation_interval_ms": float(np.median(dt) * 1000.0),
            "gaps_over_0_25s": int(np.sum(dt > 0.25)),
            "maximum_observation_gap_s": float(np.max(dt)),
            "corridor_x_m_min_median_max": [float(np.min(x)), float(np.median(x)), float(np.max(x))],
            "corridor_y_m_min_median_max": [float(np.min(y)), float(np.median(y)), float(np.max(y))],
            "corridor_z_m_min_median_max": [float(np.min(z)), float(np.median(z)), float(np.max(z))],
            "horizontal_speed_mps": percentiles(usable_speed),
            "horizontal_speed_samples_over_2mps": int(np.sum(usable_speed > 2.0)),
            "speed_sample_count_used": int(np.sum(usable)),
            "xy_path_length_m": float(np.sum(np.hypot(dx, dy))),
            "xy_start_to_end_displacement_m": float(math.hypot(x[-1] - x[0], y[-1] - y[0])),
            "turnaround": {
                "time_from_first_confirmed_s": float(t[turn] - t[0]),
                "corridor_x_m": float(x[turn]),
                "outbound_positive_x_fraction": float(np.mean(outbound_deltas > 0.0)),
                "return_negative_x_fraction": float(np.mean(return_deltas < 0.0)),
            },
            "cluster_point_count_median_p10": [float(np.median(point_count)), float(np.percentile(point_count, 10))],
        },
        "zed": {
            "source_rows": str(BODY_DIR / "bodies.csv"),
            "sdk_version": body_camera.get("sdk_version"),
            "body_tracking_model": body_camera.get("body_tracking_model"),
            "body_format": body_camera.get("body_format"),
            "body_fitting": body_camera.get("body_fitting"),
            "body_segmentation": body_camera.get("body_segmentation"),
            "depth_mode": body_camera.get("depth_mode"),
            "confidence_threshold": body_camera.get("detection_confidence_threshold"),
            "ok_body_observations": int(len(ok_bodies)),
            "tracking_ids": sorted({row["body_id"] for row in ok_bodies}),
            "tracking_state_counts": {state: sum(row["tracking_state"] == state for row in bodies) for state in sorted({row["tracking_state"] for row in bodies})},
            "elapsed_span_s": float(body_elapsed[-1] - body_elapsed[0]),
            "median_body_detector_confidence": float(np.median(body_confidence)),
            "p05_body_detector_confidence": float(np.percentile(body_confidence, 5)),
            "camera_depth_m_min_median_max": [float(np.min(body_distance)), float(np.median(body_distance)), float(np.max(body_distance))],
            "keypoint_confidence": joint_summary,
            "interpretation": "Body-level confidence and tracking state measure person detection/tracking, not accuracy of every anatomical joint.",
        },
        "limitations": [
            "The LiDAR corridor-frame axes/origin come from LiDAR-only gravity/wall calibration; the LiDAR-to-ZED extrinsic is not used.",
            "LiDAR cluster representative points are not anatomical pelvis or floor positions.",
            "This recording contains one walker, so it cannot validate two-person identity preservation during overtaking.",
            "No independent motion-capture or manually labeled image reference is available; naturalness is a plausibility check, not an accuracy score.",
        ],
    }
    OUT.mkdir(parents=True, exist_ok=True)
    (OUT / "metrics.json").write_text(json.dumps(metrics, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    line_svg(list(zip(x.tolist(), y.tolist())), OUT / "lidar_xy.svg", "LiDAR-only confirmed path", "Corridor X (m)", "Corridor Y (m)")
    line_svg(list(zip((t - t[0]).tolist(), x.tolist())), OUT / "lidar_x_time.svg", "LiDAR-only longitudinal position", "Seconds from first confirmed observation", "Corridor X (m)")

    lidar = metrics["lidar"]
    zed = metrics["zed"]
    knee_l, knee_r = (joint_summary[n]["median"] for n in ("LEFT_KNEE", "RIGHT_KNEE"))
    ankle_l, ankle_r = (joint_summary[n]["median"] for n in ("LEFT_ANKLE", "RIGHT_ANKLE"))
    readme = f"""# 校正変換から独立した1人歩行の確認

## 結果

LiDAR単体の確認では、確定トラックID {TRACK_ID} は廊下方向へ進んで折り返し、戻る軌跡を保っています。位置の往復は自然な歩行と矛盾しません。水平速度の中央値は **{np.percentile(usable_speed, 50):.2f} m/s**、p90は **{np.percentile(usable_speed, 90):.2f} m/s**、使った連続検出間隔は最大250 msに制限しました。折返し前後で廊下方向の増減がそれぞれ{100*lidar['turnaround']['outbound_positive_x_fraction']:.1f}%・{100*lidar['turnaround']['return_negative_x_fraction']:.1f}%同じ向きを保ち、0.25秒を超える欠測は{lidar['gaps_over_0_25s']}回（最大{lidar['maximum_observation_gap_s']:.3f}秒）です。

ただし、この記録は1人が廊下を往復する試験です。LiDARの確定点は{lidar['confirmed_detected_observations']}点、XY移動距離は{lidar['xy_path_length_m']:.2f} m、始点と終点の差は{lidar['xy_start_to_end_displacement_m']:.2f} mでした。横方向の範囲は{lidar['corridor_y_m_min_median_max'][0]:.2f}〜{lidar['corridor_y_m_min_median_max'][2]:.2f} mです。クラスタ代表点の高さは{lidar['corridor_z_m_min_median_max'][0]:.2f}〜{lidar['corridor_z_m_min_median_max'][2]:.2f} mと変動するため、LiDARのZやクラスタ中心を解剖学的な身体中心とはみなせません。短い速度ピークを含み、独立な正解軌跡もないので、ここで言えるのは動線の見た目が妥当という範囲です。

この解析は [LiDAR追跡CSV]({TRACKS}) の `corridor_x/y/z` と検出点だけを使っています。`lidar_to_zed_left_eye.json` によるLiDAR→ZED変換も、ZEDとの融合結果も計算に使っていません。廊下座標はLiDAR単独の重力・壁面校正に依存します。

## ZED人物検出と関節推定

ZED SDK出力は**人物検出・ID追跡そのものは安定**しています。この単独歩行の可視区間で{zed['ok_body_observations']}件が`OK`、IDは{', '.join(zed['tracking_ids'])}のみ、人物検出confidenceの中央値は{zed['median_body_detector_confidence']:.1f}/100でした。BODY_34のSDK書き出しはSDK {zed['sdk_version']}の`{zed['body_tracking_model']}`、body fitting={zed['body_fitting']}、segmentation={zed['body_segmentation']}、深度`{zed['depth_mode']}`で行われています。

問題は人物の有無より**脚・足首の関節推定**にあります。全距離での関節confidence中央値は膝が左右{min(knee_l,knee_r):.1f}〜{max(knee_l,knee_r):.1f}、足首が左右{min(ankle_l,ankle_r):.1f}〜{max(ankle_l,ankle_r):.1f}です。カメラ深度9 m以上では足首confidence中央値が左{joint_summary['LEFT_ANKLE']['median_by_distance_m']['9-100']:.1f}、右{joint_summary['RIGHT_ANKLE']['median_by_distance_m']['9-100']:.1f}まで低下します。前段の融合歩容レポートで左右の足首角がそれぞれ約129.806°にほぼ固定されていた点も、歩行周期を測るには足元の骨格出力が不足していることと整合します。

したがって、今回確認した範囲では**廊下上の大まかな歩行軌跡はLiDAR側で妥当に見え、歩容の自然さを評価できない主因はZEDの人物検出ではなく足元の姿勢推定精度**です。とはいえ2人追越し時のID維持は未評価です。

## SDKで改善できる範囲

現在のデータは既に`HUMAN_BODY_ACCURATE`と`BODY_34`、fitting、segmentationを使っています。これはSDKの精度優先の検出モデルです。現在のSDK 5.4世代から設定を少し変えるだけで人物検出が大きく改善するとは期待しにくく、信頼度しきい値を下げても低確度の足首点を拾いやすくなる代わりに誤点も増えます。

既存SVO2をSDK 5.5以降で再解析すれば、GEN_2モデルを比較できます。StereolabsはGEN_2について、混雑・通常と異なる身体向きへの頑健性向上と精度改善を説明しています。ただし今回の単独歩行での足首精度や歩容評価に対する改善保証ではありません。既存の記録でSDK 5.4/5.5のBODY_34出力を並べ、低確度関節の数・連続性・動画上の明らかなずれを比較する価値はあります。体幹・腕の姿勢や2人の人物IDには改善可能性がありますが、身体マーカーの代わりになるほどの足関節計測を保証するものではありません。

足接地や歩行周期を定量評価するなら、側面からの高解像度撮影で足元が十分な画素数を持つようにするか、マーカー式モーションキャプチャなど独立参照を追加してください。SDKの`minimum_keypoints_threshold`や`skeleton_smoothing`は低品質スケルトンの除外・時間平滑には使えますが、見えていない足位置の真値を復元する機能ではありません。

## 成果物

- 数値: [metrics.json](metrics.json)
- LiDAR平面軌跡: [lidar_xy.svg](lidar_xy.svg)
- LiDAR廊下方向と時間: [lidar_x_time.svg](lidar_x_time.svg)
- ZEDの既存オーバーレイ: [{BODY_DIR / 'contact_sheet.jpg'}]({BODY_DIR / 'contact_sheet.jpg'})
- [前の融合歩行評価](../20261013_group001_recording_001_gait_assessment/index.html)はZED–LiDAR融合値なので、この独立確認の数値には使っていません。
"""
    (OUT / "README.md").write_text(readme, encoding="utf-8")
    print(json.dumps({"output": str(OUT), "lidar": metrics["lidar"], "zed": metrics["zed"]}, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
