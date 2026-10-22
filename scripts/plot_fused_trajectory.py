#!/usr/bin/env python3
"""融合CSVから軌跡・位置・速度・関節角のSVGと確認レポートを作る。

有効区間と外れ値を分けて可視化する。折返し推定などは1人往復の確認用途を想定する。
図の滑らかさや内部統計を、独立真値に対する歩行計測精度とはみなさない。
"""

from __future__ import annotations

import argparse
import csv
import html
import json
import math
import statistics
from pathlib import Path
from typing import Any, Iterable, Optional

PROJECT_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CSV = (
    PROJECT_ROOT
    / "tracking/20261013_group001_recording_001_zed_lidar_fusion_plane_calibrated/fused_trajectory.csv"
)
DEFAULT_CORRIDOR = PROJECT_ROOT / "tracking/corridor_frame_livox_level.json"
DEFAULT_OUTPUT = PROJECT_ROOT / "reports/20261013_group001_recording_001_gait_assessment"

NAVY = "#155e75"
BLUE = "#1479a8"
ORANGE = "#d97706"
GREEN = "#15803d"
RED = "#c2413b"
GRAY = "#64748b"
GRID = "#dbe3ea"
TEXT = "#1f2937"


def number(row: dict[str, str], key: str) -> Optional[float]:
    try:
        value = float(row.get(key, ""))
    except (TypeError, ValueError):
        return None
    return value if math.isfinite(value) else None


def quantile(values: list[float], q: float) -> Optional[float]:
    if not values:
        return None
    ordered = sorted(values)
    index = min(len(ordered) - 1, max(0, round(q * (len(ordered) - 1))))
    return ordered[index]


def fmt(value: Optional[float], digits: int = 2) -> str:
    return "—" if value is None else f"{value:.{digits}f}"


def xml(value: Any) -> str:
    return html.escape(str(value), quote=True)


def svg_start(title: str, width: int, height: int) -> list[str]:
    return [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
        '<style>text{font-family:system-ui,-apple-system,"Segoe UI",sans-serif;fill:#1f2937}.title{font-size:21px;font-weight:650}.axis{font-size:13px}.small{font-size:12px}.grid{stroke:#dbe3ea;stroke-width:1}.tick{stroke:#94a3b8;stroke-width:1}</style>',
        f'<rect width="{width}" height="{height}" fill="white"/><text class="title" x="36" y="34">{xml(title)}</text>',
    ]


def map_x(
    value: float, limits: tuple[float, float], box: tuple[float, float, float, float]
) -> float:
    left, top, width, height = box
    lo, hi = limits
    return left + (value - lo) / (hi - lo) * width


def map_y(
    value: float, limits: tuple[float, float], box: tuple[float, float, float, float]
) -> float:
    left, top, width, height = box
    lo, hi = limits
    return top + height - (value - lo) / (hi - lo) * height


def draw_axes(
    parts: list[str],
    box: tuple[float, float, float, float],
    xlim: tuple[float, float],
    ylim: tuple[float, float],
    xticks: Iterable[float],
    yticks: Iterable[float],
    xlabel: str,
    ylabel: str,
    title: Optional[str] = None,
    y_format: str = "{:.1f}",
) -> None:
    left, top, width, height = box
    if title:
        parts.append(
            f'<text class="axis" font-weight="650" x="{left}" y="{top - 13}">{xml(title)}</text>'
        )
    for value in xticks:
        if xlim[0] <= value <= xlim[1]:
            x = map_x(value, xlim, box)
            parts.append(
                f'<line class="grid" x1="{x:.2f}" y1="{top}" x2="{x:.2f}" y2="{top + height}"/>'
            )
            parts.append(
                f'<text class="small" text-anchor="middle" x="{x:.2f}" y="{top + height + 19}">{value:g}</text>'
            )
    for value in yticks:
        if ylim[0] <= value <= ylim[1]:
            y = map_y(value, ylim, box)
            parts.append(
                f'<line class="grid" x1="{left}" y1="{y:.2f}" x2="{left + width}" y2="{y:.2f}"/>'
            )
            parts.append(
                f'<text class="small" text-anchor="end" x="{left - 9}" y="{y + 4:.2f}">{y_format.format(value)}</text>'
            )
    parts.append(
        f'<rect x="{left}" y="{top}" width="{width}" height="{height}" fill="none" stroke="#64748b" stroke-width="1"/>'
    )
    parts.append(
        f'<text class="axis" text-anchor="middle" x="{left + width / 2}" y="{top + height + 45}">{xml(xlabel)}</text>'
    )
    parts.append(
        f'<text class="axis" text-anchor="middle" transform="translate({left - 58},{top + height / 2}) rotate(-90)">{xml(ylabel)}</text>'
    )


def contiguous_segments(
    rows: list[dict[str, str]], valid_only: bool = True
) -> list[list[dict[str, str]]]:
    """無効行や区間IDの変化で線を分け、欠損をまたぐ軌跡を描かないようにする。"""

    result: list[list[dict[str, str]]] = []
    current: list[dict[str, str]] = []
    current_segment: Optional[str] = None
    for row in rows:
        valid = row.get("motion_valid") == "1" if valid_only else True
        segment = row.get("motion_segment_id", "")
        if not valid or (current and segment != current_segment):
            if current:
                result.append(current)
            current = []
            current_segment = None
        if valid:
            current.append(row)
            current_segment = segment
    if current:
        result.append(current)
    return result


def draw_time_lines(
    parts: list[str],
    rows: list[dict[str, str]],
    key: str,
    box: tuple[float, float, float, float],
    xlim: tuple[float, float],
    ylim: tuple[float, float],
    color: str,
    width: float = 1.7,
    valid_only: bool = True,
) -> None:
    groups = contiguous_segments(rows, valid_only=valid_only)
    for group in groups:
        points: list[str] = []
        for row in group:
            t, value = number(row, "elapsed_s"), number(row, key)
            if t is None or value is None:
                if len(points) > 1:
                    parts.append(
                        f'<polyline points="{" ".join(points)}" fill="none" stroke="{color}" stroke-width="{width}" stroke-linejoin="round" stroke-linecap="round"/>'
                    )
                points = []
                continue
            points.append(f'{map_x(t, xlim, box):.2f},{map_y(value, ylim, box):.2f}')
        if len(points) > 1:
            parts.append(
                f'<polyline points="{" ".join(points)}" fill="none" stroke="{color}" stroke-width="{width}" stroke-linejoin="round" stroke-linecap="round"/>'
            )


def draw_outlier_crosses(
    parts: list[str], rows: list[dict[str, str]], box, xlim, ylim, y_key: str
) -> None:
    for row in rows:
        if row.get("motion_outlier") != "1":
            continue
        t, value = number(row, "elapsed_s"), number(row, y_key)
        if (
            t is None
            or value is None
            or not (xlim[0] <= t <= xlim[1] and ylim[0] <= value <= ylim[1])
        ):
            continue
        x, y = map_x(t, xlim, box), map_y(value, ylim, box)
        parts.append(
            f'<path d="M{x-3:.2f},{y-3:.2f} L{x+3:.2f},{y+3:.2f} M{x+3:.2f},{y-3:.2f} L{x-3:.2f},{y+3:.2f}" stroke="{RED}" stroke-width="1.6"/>'
        )


def centered_median_speeds(rows: list[dict[str, str]], window_s: float = 1.0) -> dict[int, float]:
    """同じ有効区間内の時間窓で、表示用の速度中央値を求める。"""

    result: dict[int, float] = {}
    groups = contiguous_segments(rows)
    for group in groups:
        times = [number(row, "elapsed_s") for row in group]
        speeds = [number(row, "fused_speed_corridor_mps") for row in group]
        for index, (time, speed) in enumerate(zip(times, speeds)):
            if time is None or speed is None:
                continue
            window = [
                candidate
                for candidate, candidate_time in zip(speeds, times)
                if candidate is not None
                and candidate_time is not None
                and abs(candidate_time - time) <= window_s / 2
            ]
            result[id(group[index])] = statistics.median(window)
    return result


def trajectory_svg(rows: list[dict[str, str]], wall_gap: float) -> str:
    width, height = 1200, 455
    parts = svg_start("校正後の廊下平面軌跡", width, height)
    xlim, ylim = (0.0, 13.5), (-1.35, 1.35)
    box = (150.0, 86.0, 900.0, 180.0)  # equal meters-per-pixel on both axes
    draw_axes(
        parts,
        box,
        xlim,
        ylim,
        range(0, 14, 2),
        [-1.0, -0.5, 0, 0.5, 1.0],
        "廊下方向 x (m)",
        "横方向 y (m)",
        y_format="{:.1f}",
    )
    for side in (-1, 1):
        y = side * wall_gap / 2
        py = map_y(y, ylim, box)
        parts.append(
            f'<line x1="{box[0]}" y1="{py:.2f}" x2="{box[0]+box[2]}" y2="{py:.2f}" stroke="#94a3b8" stroke-dasharray="8 6" stroke-width="1.5"/>'
        )
    for group in contiguous_segments(rows):
        coords = []
        for row in group:
            x, y = number(row, "fused_position_corridor_x_m"), number(
                row, "fused_position_corridor_y_m"
            )
            if x is not None and y is not None:
                coords.append(f"{map_x(x, xlim, box):.2f},{map_y(y, ylim, box):.2f}")
        if len(coords) > 1:
            parts.append(
                f'<polyline points="{" ".join(coords)}" fill="none" stroke="{NAVY}" stroke-width="2.4" stroke-linejoin="round" stroke-linecap="round"/>'
            )
    for row in rows:
        if row.get("motion_outlier") != "1":
            continue
        x, y = number(row, "fused_position_corridor_x_m"), number(
            row, "fused_position_corridor_y_m"
        )
        if x is None or y is None:
            continue
        px, py = map_x(x, xlim, box), map_y(y, ylim, box)
        parts.append(
            f'<path d="M{px-4:.1f},{py-4:.1f} L{px+4:.1f},{py+4:.1f} M{px+4:.1f},{py-4:.1f} L{px-4:.1f},{py+4:.1f}" stroke="{RED}" stroke-width="1.5"/>'
        )
    valid = [row for row in rows if row.get("motion_valid") == "1"]
    start = valid[0] if valid else None
    end = valid[-1] if valid else None
    turn = (
        max(valid, key=lambda row: number(row, "fused_position_corridor_x_m") or -math.inf)
        if valid
        else None
    )
    for row, color, label, offset in (
        (start, GREEN, "開始", (8, -8)),
        (turn, ORANGE, "折返し", (8, -9)),
        (end, BLUE, "終了", (8, 16)),
    ):
        if row is None:
            continue
        x, y = number(row, "fused_position_corridor_x_m"), number(
            row, "fused_position_corridor_y_m"
        )
        if x is None or y is None:
            continue
        px, py = map_x(x, xlim, box), map_y(y, ylim, box)
        parts.append(
            f'<circle cx="{px:.1f}" cy="{py:.1f}" r="5.5" fill="{color}" stroke="white" stroke-width="1.5"/>'
        )
        parts.append(
            f'<text class="small" x="{px+offset[0]}" y="{py+offset[1]}" fill="{color}">{label}</text>'
        )
    parts.append(
        f'<line x1="800" y1="320" x2="835" y2="320" stroke="{NAVY}" stroke-width="2.4"/><text class="small" x="843" y="324">採用された軌跡</text>'
    )
    parts.append(
        f'<path d="M1014,316 l8,8 m0,-8 l-8,8" stroke="{RED}" stroke-width="1.5"/><text class="small" x="1030" y="324">連続性しきい値で除外</text>'
    )
    parts.append(
        f'<text class="small" x="150" y="375">点線: 廊下側壁（幅 {wall_gap:.2f} m）</text>'
    )
    parts.append(
        '<text class="small" x="150" y="400" fill="#64748b">corridor_x / corridor_y。軸の縮尺は同じ。移動方向を確認しやすいよう時刻ではなく位置で描画。</text>'
    )
    parts.append("</svg>")
    return "\n".join(parts)


def position_svg(rows: list[dict[str, str]], tlim: tuple[float, float]) -> str:
    width, height = 1200, 800
    parts = svg_start("廊下座標の位置時系列", width, height)
    panels = [
        ("fused_position_corridor_x_m", "廊下方向 x (m)", (0, 13.5), range(0, 14, 2), NAVY),
        (
            "fused_position_corridor_y_m",
            "横方向 y (m)",
            (-0.5, 0.5),
            [-0.4, -0.2, 0, 0.2, 0.4],
            BLUE,
        ),
        ("fused_position_corridor_z_m", "高さ z (m)", (0.5, 1.2), [0.5, 0.7, 0.9, 1.1], GREEN),
    ]
    for index, (key, ylabel, ylim, yticks, color) in enumerate(panels):
        box = (112.0, 86.0 + index * 225.0, 1030.0, 155.0)
        draw_axes(
            parts,
            box,
            tlim,
            ylim,
            range(math.ceil(tlim[0] / 2) * 2, int(tlim[1]) + 1, 2),
            yticks,
            "経過時間 (s)" if index == 2 else "",
            ylabel,
            title=["廊下方向", "横方向", "高さ"][index],
        )
        draw_time_lines(parts, rows, key, box, tlim, ylim, color, width=1.8)
        draw_outlier_crosses(parts, rows, box, tlim, ylim, key)
    parts.append(
        f'<path d="M884,765 l8,8 m0,-8 l-8,8" stroke="{RED}" stroke-width="1.5"/><text class="small" x="900" y="773">速度不連続として除外されたフレーム</text>'
    )
    parts.append("</svg>")
    return "\n".join(parts)


def speed_svg(rows: list[dict[str, str]], tlim: tuple[float, float]) -> str:
    width, height = 1200, 690
    parts = svg_start("統合水平速度と進行方向", width, height)
    turn_start, turn_end = 24.0, 27.0
    medians = centered_median_speeds(rows, window_s=1.0)
    for index, (key, ylabel, ylim, yticks, color) in enumerate(
        [
            (
                "fused_speed_corridor_mps",
                "水平速度 (m/s)",
                (0, 3.4),
                [0, 0.5, 1, 1.5, 2, 2.5, 3],
                NAVY,
            ),
            (
                "fused_velocity_corridor_x_mps",
                "廊下方向 vx (m/s)",
                (-3.5, 3.5),
                [-3, -2, -1, 0, 1, 2, 3],
                BLUE,
            ),
        ]
    ):
        box = (112.0, 92.0 + index * 260.0, 1030.0, 182.0)
        draw_axes(
            parts,
            box,
            tlim,
            ylim,
            range(math.ceil(tlim[0] / 2) * 2, int(tlim[1]) + 1, 2),
            yticks,
            "経過時間 (s)" if index == 1 else "",
            ylabel,
            title=["水平速度（位置の時間微分）", "廊下方向の符号付き速度"][index],
        )
        x0, x1 = map_x(turn_start, tlim, box), map_x(turn_end, tlim, box)
        parts.append(
            f'<rect x="{x0:.2f}" y="{box[1]}" width="{x1-x0:.2f}" height="{box[3]}" fill="#f59e0b" opacity="0.10"/>'
        )
        if index == 0:
            ref = map_y(1.3, ylim, box)
            parts.append(
                f'<line x1="{box[0]}" y1="{ref:.2f}" x2="{box[0]+box[2]}" y2="{ref:.2f}" stroke="{ORANGE}" stroke-dasharray="7 5" stroke-width="1.5"/>'
            )
        draw_time_lines(
            parts, rows, key, box, tlim, ylim, "#94a3b8" if index == 0 else color, width=1.0
        )
        if index == 0:
            for group in contiguous_segments(rows):
                points = []
                for row in group:
                    time, speed = number(row, "elapsed_s"), medians.get(id(row))
                    if time is not None and speed is not None:
                        points.append(f'{map_x(time, tlim, box):.2f},{map_y(speed, ylim, box):.2f}')
                if len(points) > 1:
                    parts.append(
                        f'<polyline points="{" ".join(points)}" fill="none" stroke="{NAVY}" stroke-width="2.6" stroke-linejoin="round" stroke-linecap="round"/>'
                    )
        draw_outlier_crosses(parts, rows, box, tlim, ylim, key)
    parts.append(
        f'<line x1="175" y1="615" x2="210" y2="615" stroke="#94a3b8" stroke-width="1.2"/><text class="small" x="218" y="619">フレームごとの水平速度</text>'
    )
    parts.append(
        f'<line x1="390" y1="615" x2="425" y2="615" stroke="{NAVY}" stroke-width="2.6"/><text class="small" x="433" y="619">同一連続区間内の中央1秒中央値</text>'
    )
    parts.append(
        f'<line x1="742" y1="615" x2="777" y2="615" stroke="{ORANGE}" stroke-dasharray="7 5" stroke-width="1.5"/><text class="small" x="785" y="619">参考線 1.3 m/s</text>'
    )
    parts.append(
        '<rect x="925" y="605" width="30" height="18" fill="#f59e0b" opacity="0.15"/><text class="small" x="963" y="619">折返し付近</text>'
    )
    parts.append(
        '<text class="small" x="112" y="655" fill="#64748b">参考線は健康な成人の快適歩行速度として報告された集団平均の目安。個人の正常判定ではない。</text>'
    )
    parts.append("</svg>")
    return "\n".join(parts)


def joint_svg(rows: list[dict[str, str]], tlim: tuple[float, float]) -> str:
    width, height = 1200, 560
    parts = svg_start("ZED BODY_34から得た姿勢指標", width, height)
    panels = [
        ("left_knee_angle_deg", "左膝", BLUE),
        ("right_knee_angle_deg", "右膝", ORANGE),
    ]
    box = (112.0, 88.0, 1030.0, 205.0)
    draw_axes(
        parts,
        box,
        tlim,
        (140, 181),
        range(math.ceil(tlim[0] / 2) * 2, int(tlim[1]) + 1, 2),
        [140, 150, 160, 170, 180],
        "経過時間 (s)",
        "膝角度 (deg)",
        title="左右の膝角度（骨盤・膝・足首の推定点から算出）",
    )
    for key, label, color in panels:
        draw_time_lines(
            parts, rows, key, box, tlim, (140, 181), color, width=1.45, valid_only=False
        )
    parts.append(
        f'<line x1="790" y1="321" x2="825" y2="321" stroke="{BLUE}" stroke-width="2"/><text class="small" x="833" y="325">左膝</text>'
    )
    parts.append(
        f'<line x1="900" y1="321" x2="935" y2="321" stroke="{ORANGE}" stroke-width="2"/><text class="small" x="943" y="325">右膝</text>'
    )
    box2 = (112.0, 383.0, 1030.0, 110.0)
    draw_axes(
        parts,
        box2,
        tlim,
        (0, 16),
        range(math.ceil(tlim[0] / 2) * 2, int(tlim[1]) + 1, 2),
        [0, 5, 10, 15],
        "経過時間 (s)",
        "傾き (deg)",
        title="体幹の鉛直からの傾き",
    )
    draw_time_lines(
        parts,
        rows,
        "trunk_lean_from_vertical_deg",
        box2,
        tlim,
        (0, 16),
        GREEN,
        width=1.4,
        valid_only=False,
    )
    parts.append(
        '<text class="small" x="112" y="536" fill="#64748b">角度はSDKの推定骨格点から計算した近似値。解剖学的な関節中心や歩行周期の計測値ではない。</text>'
    )
    parts.append("</svg>")
    return "\n".join(parts)


def summarize(rows: list[dict[str, str]], wall_gap: float) -> dict[str, Any]:
    """有効位置・速度・角度の分布をまとめ、1人往復を想定した折返し位置も求める。"""

    times = [number(row, "elapsed_s") for row in rows]
    times = [value for value in times if value is not None]
    valid = [row for row in rows if row.get("motion_valid") == "1"]
    outliers = [row for row in rows if row.get("motion_outlier") == "1"]
    speeds = [number(row, "fused_speed_corridor_mps") for row in valid]
    speeds = [value for value in speeds if value is not None]
    moving = [value for value in speeds if value >= 0.4]
    segments = {row.get("motion_segment_id") for row in valid if row.get("motion_segment_id")}
    positions = {
        axis: [number(row, f"fused_position_corridor_{axis}_m") for row in valid] for axis in "xyz"
    }
    positions = {
        axis: [value for value in values if value is not None] for axis, values in positions.items()
    }
    adjacent_delta: dict[str, list[float]] = {axis: [] for axis in "xyz"}
    for a, b in zip(rows, rows[1:]):
        if (
            a.get("motion_valid") != "1"
            or b.get("motion_valid") != "1"
            or a.get("motion_segment_id") != b.get("motion_segment_id")
        ):
            continue
        for axis in "xyz":
            va, vb = number(a, f"fused_position_corridor_{axis}_m"), number(
                b, f"fused_position_corridor_{axis}_m"
            )
            if va is not None and vb is not None:
                adjacent_delta[axis].append(abs(vb - va))
    # 表示・集計用の速度へ1秒窓の中央移動中央値をかけ、加速度傾向の雑音を抑える。
    smooth = centered_median_speeds(rows, 1.0)
    smooth_accel: list[float] = []
    for a, b in zip(rows, rows[1:]):
        if (
            a.get("motion_valid") != "1"
            or b.get("motion_valid") != "1"
            or a.get("motion_segment_id") != b.get("motion_segment_id")
        ):
            continue
        ta, tb = number(a, "elapsed_s"), number(b, "elapsed_s")
        sa, sb = smooth.get(id(a)), smooth.get(id(b))
        if ta is not None and tb is not None and tb > ta and sa is not None and sb is not None:
            smooth_accel.append(abs(sb - sa) / (tb - ta))
    turn = (
        max(valid, key=lambda row: number(row, "fused_position_corridor_x_m") or -math.inf)
        if valid
        else None
    )
    turn_time = number(turn, "elapsed_s") if turn else None
    angle_stats: dict[str, dict[str, Any]] = {}
    for key in (
        "left_knee_angle_deg",
        "right_knee_angle_deg",
        "left_ankle_angle_deg",
        "right_ankle_angle_deg",
        "trunk_lean_from_vertical_deg",
    ):
        values = [number(row, key) for row in rows]
        values = [value for value in values if value is not None]
        angle_stats[key] = {
            "count": len(values),
            "median_deg": quantile(values, 0.5),
            "p05_deg": quantile(values, 0.05),
            "p95_deg": quantile(values, 0.95),
            "min_deg": min(values) if values else None,
            "max_deg": max(values) if values else None,
            "distinct_0_001_deg": len({round(value, 3) for value in values}),
        }
    lidar_count = sum(1 for row in rows if row.get("lidar_track_id"))
    valid_dts = [b - a for a, b in zip(times, times[1:]) if b > a]
    return {
        "source_csv": str(DEFAULT_CSV),
        "frame_count": len(rows),
        "time_start_s": min(times) if times else None,
        "time_end_s": max(times) if times else None,
        "duration_s": max(times) - min(times) if times else None,
        "median_frame_interval_ms": quantile(valid_dts, 0.5) * 1000 if valid_dts else None,
        "lidar_associated_frames": lidar_count,
        "lidar_association_percent": 100 * lidar_count / len(rows) if rows else None,
        "motion_valid_frames": len(valid),
        "motion_outlier_frames": len(outliers),
        "motion_outlier_percent": 100 * len(outliers) / len(rows) if rows else None,
        "motion_segment_count": len(segments),
        "horizontal_speed_mps": {
            "all_valid_median": quantile(speeds, 0.5),
            "p90": quantile(speeds, 0.9),
            "p95": quantile(speeds, 0.95),
            "max": max(speeds) if speeds else None,
            "frames_at_least_0_4_mps": len(moving),
            "moving_median_at_least_0_4_mps": quantile(moving, 0.5),
            "moving_p10_at_least_0_4_mps": quantile(moving, 0.1),
            "moving_p90_at_least_0_4_mps": quantile(moving, 0.9),
            "frames_above_2_mps": sum(value > 2.0 for value in speeds),
        },
        "corridor_positions_m": {
            axis: {
                "min": min(values) if values else None,
                "median": quantile(values, 0.5),
                "max": max(values) if values else None,
            }
            for axis, values in positions.items()
        },
        "horizontal_path_lateral_span_m": (
            (max(positions["y"]) - min(positions["y"])) if positions["y"] else None
        ),
        "corridor_wall_gap_m": wall_gap,
        "turnaround": {
            "time_s": turn_time,
            "corridor_x_m": number(turn, "fused_position_corridor_x_m") if turn else None,
        },
        "frame_to_frame_position_change_p95_m": {
            axis: quantile(values, 0.95) for axis, values in adjacent_delta.items()
        },
        "one_second_median_speed_abs_acceleration_p95_mps2": quantile(smooth_accel, 0.95),
        "joint_angles": angle_stats,
        "notes": {
            "horizontal_speed": "sqrt(fused_velocity_corridor_x_mps^2 + fused_velocity_corridor_y_mps^2); excludes motion_valid=0 frames.",
            "one_second_smoothing": "Centered 1.0-second median over valid samples within each continuous motion segment.",
            "turnaround": "Time and position of the maximum valid fused corridor_x.",
            "position_smoothing": "Uses the fused CSV positions, which have a within-segment five-sample coordinate median applied by fuse_zed_lidar.py.",
        },
    }


def render_html(metrics: dict[str, Any]) -> str:
    speed = metrics["horizontal_speed_mps"]
    pos = metrics["corridor_positions_m"]
    angles = metrics["joint_angles"]
    lidar_pct = metrics["lidar_association_percent"]
    outlier_pct = metrics["motion_outlier_percent"]
    html_text = f'''<!doctype html>
<html lang="ja"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>校正後の歩行軌跡・速度の評価</title>
<style>
body{{font-family:system-ui,sans-serif;max-width:1120px;margin:32px auto;padding:0 20px;color:#1f2937;line-height:1.65}}
h1,h2{{line-height:1.25}}.summary{{background:#f0f7fa;border-left:4px solid #155e75;padding:14px 18px}}
table{{border-collapse:collapse;width:100%;margin:16px 0}}td,th{{border-bottom:1px solid #dbe3ea;padding:8px 10px;text-align:left;vertical-align:top}}
img{{width:100%;height:auto;border:1px solid #e2e8f0;margin:10px 0 24px}}.note{{color:#475569;font-size:.94em}}
</style>
<h1>校正後の歩行軌跡・速度の評価</h1>
<p>入力: <code>{xml(Path(metrics['source_csv']).name)}</code>。ZEDとLiDARは録画内タイムスタンプで対応付け、校正後の廊下座標で統合しています。</p>
<div class="summary"><b>評価:</b> 水平軌跡と速度の時間変化は、人が廊下を往復歩行した動きとして概ね整合し、粗い動線・速度の評価には使えます。速度は全有効フレーム中央値 {fmt(speed['all_valid_median'])} m/s、0.4 m/s以上では {fmt(speed['moving_median_at_least_0_4_mps'])} m/sです。ただし、局所的な速度ピーク、上下位置の揺れ、ZED骨格角の制約があり、歩容・歩行周期の生体力学的な自然さを高信頼で判定できる品質ではありません。</div>
<h2>主な数値</h2>
<table><tbody>
<tr><th>録画内観測</th><td>{metrics['frame_count']} frames / {fmt(metrics['duration_s'])} s、中央値フレーム間隔 {fmt(metrics['median_frame_interval_ms'],1)} ms</td></tr>
<tr><th>センサー統合</th><td>LiDAR対応 {metrics['lidar_associated_frames']}/{metrics['frame_count']} ({fmt(lidar_pct,1)}%)</td></tr>
<tr><th>連続性判定</th><td>有効 {metrics['motion_valid_frames']} frames、除外 {metrics['motion_outlier_frames']} ({fmt(outlier_pct,1)}%)、{metrics['motion_segment_count']}連続区間</td></tr>
<tr><th>廊下方向 x</th><td>{fmt(pos['x']['min'])}–{fmt(pos['x']['max'])} m</td></tr>
<tr><th>横方向 y</th><td>{fmt(pos['y']['min'])}–{fmt(pos['y']['max'])} m (幅 {fmt(metrics['horizontal_path_lateral_span_m'])} m)</td></tr>
<tr><th>高さ z</th><td>{fmt(pos['z']['min'])}–{fmt(pos['z']['max'])} m</td></tr>
<tr><th>水平速度</th><td>中央値 {fmt(speed['all_valid_median'])} m/s、p90 {fmt(speed['p90'])}、p95 {fmt(speed['p95'])}、最大 {fmt(speed['max'])}。2 m/s超は {speed['frames_above_2_mps']} frames</td></tr>
<tr><th>折返し</th><td>約 {fmt(metrics['turnaround']['time_s'],1)} s、x={fmt(metrics['turnaround']['corridor_x_m'])} m</td></tr>
<tr><th>連続点の高さ変化</th><td>フレーム間絶対変化 p95 {fmt(metrics['frame_to_frame_position_change_p95_m']['z'],3)} m。高周波の上下揺れは実際の身体運動と深度・骨格推定誤差を分離できません。</td></tr>
</tbody></table>
<h2>軌跡と速度</h2>
<img src="trajectory.svg" alt="廊下平面の歩行軌跡"><img src="position_time.svg" alt="廊下座標位置の時系列"><img src="speed.svg" alt="水平速度と進行方向速度">
<h2>姿勢の補助確認</h2>
<img src="joint_angles.svg" alt="膝角と体幹傾き">
<p>膝角度中央値は左 {fmt(angles['left_knee_angle_deg']['median_deg'],1)}°、右 {fmt(angles['right_knee_angle_deg']['median_deg'],1)}°で、記録の大半はほぼ伸展側にあります。体幹傾き中央値は {fmt(angles['trunk_lean_from_vertical_deg']['median_deg'],1)}°です。両足首角はそれぞれ {fmt(angles['left_ankle_angle_deg']['min_deg'],3)}–{fmt(angles['left_ankle_angle_deg']['max_deg'],3)}°、{fmt(angles['right_ankle_angle_deg']['min_deg'],3)}–{fmt(angles['right_ankle_angle_deg']['max_deg'],3)}°とほぼ一定で、歩行中の足首運動を評価する信号としては使えません。</p>
<h2>自然さ・なめらかさの判定</h2>
<ul>
<li><b>動線:</b> 廊下方向へ進んで端で折り返し、戻る連続した往復軌道です。横方向変位は全幅 {fmt(metrics['horizontal_path_lateral_span_m'])} mで、廊下幅 {fmt(metrics['corridor_wall_gap_m'])} mの中を進んでいます。</li>
<li><b>速度:</b> 0.4 m/s未満を除くフレームの中央値は {fmt(speed['moving_median_at_least_0_4_mps'])} m/s。研究で報告された健康成人の快適歩行速度の集団平均約1.3–1.4 m/sより少し低めですが、通常歩行としてあり得る範囲です。年齢、歩行指示、靴、折返し・開始停止の影響が不明なため、個人評価の基準にはしません。</li>
<li><b>なめらかさ:</b> 位置は各連続区間で5フレーム中央値処理済みです。水平軌跡は比較的滑らかですが、除外点 {fmt(outlier_pct,1)}%、2 m/s超の短い速度ピークがあり、速度の瞬時値は中程度の信頼度です。1秒中央値の速度変化 p95 は {fmt(metrics['one_second_median_speed_abs_acceleration_p95_mps2'])} m/s²です。</li>
<li><b>姿勢:</b> ZED SDKの骨格は歩行周期の足接地・蹴り出しを直接計測しておらず、足首角はほぼ固定です。歩容の周期性、左右対称性、自然さの確定には使えません。</li>
</ul>
<p class="note">速度の参考値: 若年成人の快適な地上歩行速度 1.37 ± 0.16 m/s を報告した研究。年齢や条件により変わる集団平均であり、この記録の正常・異常を判定する閾値ではありません。<a href="https://pubmed.ncbi.nlm.nih.gov/28687953/">PubMed: The effect of treadmill and overground walking on preferred walking speed and gait kinematics in healthy, physically active older adults</a></p>
<p class="note">位置はZED SDKの骨盤付近深度とLiDAR人物クラスタ中心を信頼度加重で統合し、廊下方向はLiDARを最低70%反映した値です。LiDAR対応率は {fmt(lidar_pct,1)}%で、残りはZED単独です。床・平行壁による外部校正では廊下方向の並進が観測できないため、その成分は初期取り付け測定値を保持しています。廊下方向の絶対原点にはその不確かさが残ります。融合位置のセンサー間残差は真値に対する位置誤差ではありません。フレーム連続性しきい値で除外された点はグラフに赤い×で示します。</p>
</html>'''
    return html_text


def main() -> int:
    """融合CSVと廊下幅を読み、SVGグラフと確認用HTMLを保存する。"""

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--csv", type=Path, default=DEFAULT_CSV, help="fuse_zed_lidar.py output CSV"
    )
    parser.add_argument(
        "--corridor-frame",
        type=Path,
        default=DEFAULT_CORRIDOR,
        help="corridor frame JSON with measured wall gap",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=DEFAULT_OUTPUT,
        help="new directory for SVG graphs and report",
    )
    args = parser.parse_args()
    if not args.csv.is_file():
        parser.error(f"input CSV not found: {args.csv}")
    if not args.corridor_frame.is_file():
        parser.error(f"corridor frame JSON not found: {args.corridor_frame}")
    output_names = {
        "trajectory.svg",
        "position_time.svg",
        "speed.svg",
        "joint_angles.svg",
        "metrics.json",
        "index.html",
    }
    if args.output_dir.exists():
        unexpected = {item.name for item in args.output_dir.iterdir()} - output_names
        if unexpected:
            parser.error(
                f"output directory contains unrelated files; choose a new directory: {args.output_dir}"
            )
    rows = list(csv.DictReader(args.csv.open(newline="", encoding="utf-8-sig")))
    if not rows:
        parser.error("input CSV contains no observations")
    frame = json.loads(args.corridor_frame.read_text(encoding="utf-8"))
    wall_gap = float(frame["wall_gap"])
    metrics = summarize(rows, wall_gap)
    metrics["source_csv"] = str(args.csv.resolve())
    times = [number(row, "elapsed_s") for row in rows]
    tlim = (
        math.floor(min(x for x in times if x is not None)),
        math.ceil(max(x for x in times if x is not None)),
    )
    args.output_dir.mkdir(parents=True, exist_ok=True)
    (args.output_dir / "trajectory.svg").write_text(
        trajectory_svg(rows, wall_gap), encoding="utf-8"
    )
    (args.output_dir / "position_time.svg").write_text(position_svg(rows, tlim), encoding="utf-8")
    (args.output_dir / "speed.svg").write_text(speed_svg(rows, tlim), encoding="utf-8")
    (args.output_dir / "joint_angles.svg").write_text(joint_svg(rows, tlim), encoding="utf-8")
    (args.output_dir / "metrics.json").write_text(
        json.dumps(metrics, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    (args.output_dir / "index.html").write_text(render_html(metrics), encoding="utf-8")
    print(f"Wrote assessment to {args.output_dir}")
    print(
        f"Frames: {len(rows)}; valid motion: {metrics['motion_valid_frames']}; outliers: {metrics['motion_outlier_frames']}"
    )
    print(
        f"Horizontal speed median: {fmt(metrics['horizontal_speed_mps']['all_valid_median'])} m/s; moving median: {fmt(metrics['horizontal_speed_mps']['moving_median_at_least_0_4_mps'])} m/s"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
