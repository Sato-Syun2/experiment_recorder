#!/usr/bin/env python3
"""LiDAR点群から廊下中心座標系を推定し、位置・速度を変換する。

廊下の左右壁をXY平面上の2本の平行線として推定する。
推定した中心線を原点とし、廊下方向をX、廊下幅方向をY、
高さをZとする座標系へ変換する。
"""

from __future__ import annotations

import dataclasses
import json
import math
from pathlib import Path
from typing import Optional, Sequence

import numpy as np


class CorridorFrameError(ValueError):
    """廊下座標系を推定または読み込みできない場合の例外。"""


@dataclasses.dataclass(frozen=True)
class CorridorFrame:
    """点群処理座標から廊下方向・横方向・高さのメートル座標へ変換する。"""

    origin_xy: np.ndarray
    longitudinal_xy: np.ndarray
    lateral_xy: np.ndarray
    wall_offsets: tuple[float, float]
    wall_gap: float
    wall_inlier_counts: tuple[int, int]
    source_frame: str = "livox_frame"
    name: str = "corridor"

    def __post_init__(self) -> None:
        # 原点と直交単位軸を検証し、座標変換が距離を変えないようにする。
        origin = np.asarray(self.origin_xy, dtype=float).reshape(-1)
        longitudinal = np.asarray(self.longitudinal_xy, dtype=float).reshape(-1)
        lateral = np.asarray(self.lateral_xy, dtype=float).reshape(-1)
        if origin.size != 2 or longitudinal.size != 2 or lateral.size != 2:
            raise CorridorFrameError("Corridor frame vectors must have two elements.")
        if not np.all(np.isfinite(origin)) or not np.all(np.isfinite(longitudinal)) or not np.all(np.isfinite(lateral)):
            raise CorridorFrameError("Corridor frame vectors must be finite.")
        if not math.isclose(float(np.linalg.norm(longitudinal)), 1.0, abs_tol=1e-4) or not math.isclose(float(np.linalg.norm(lateral)), 1.0, abs_tol=1e-4):
            raise CorridorFrameError("Corridor axes must be unit vectors.")
        if abs(float(np.dot(longitudinal, lateral))) > 1e-4:
            raise CorridorFrameError("Corridor axes must be perpendicular.")
        if len(self.wall_offsets) != 2 or self.wall_offsets[0] >= self.wall_offsets[1] or self.wall_gap <= 0:
            raise CorridorFrameError("Corridor wall fields are invalid.")
        object.__setattr__(self, "origin_xy", origin)
        object.__setattr__(self, "longitudinal_xy", longitudinal)
        object.__setattr__(self, "lateral_xy", lateral)

    def transform_position(self, position: Sequence[float]) -> np.ndarray:
        """点群座標の位置を廊下中心座標へ変換する。"""

        value = np.asarray(position, dtype=float).reshape(-1)
        if value.size != 3:
            raise CorridorFrameError("A position must contain three values.")
        delta = value[:2] - self.origin_xy
        return np.array([self.longitudinal_xy @ delta, self.lateral_xy @ delta, value[2]], dtype=float)

    def transform_velocity(self, velocity: Sequence[float]) -> np.ndarray:
        """速度ベクトルを廊下座標の軸へ回転する。原点移動は行わない。"""

        value = np.asarray(velocity, dtype=float).reshape(-1)
        if value.size != 3:
            raise CorridorFrameError("A velocity must contain three values.")
        return np.array([self.longitudinal_xy @ value[:2], self.lateral_xy @ value[:2], value[2]], dtype=float)

    def to_dict(self) -> dict[str, object]:
        """校正結果をJSONへ保存できる基本型へ変換する。"""

        return {
            "version": 1, "name": self.name, "source_frame": self.source_frame,
            "coordinate_frame": "corridor", "origin_xy": self.origin_xy.tolist(),
            "longitudinal_xy": self.longitudinal_xy.tolist(), "lateral_xy": self.lateral_xy.tolist(),
            "wall_offsets": list(self.wall_offsets), "wall_gap": self.wall_gap,
            "wall_inlier_counts": list(self.wall_inlier_counts),
        }

    def write_json(self, path: Path) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(self.to_dict(), ensure_ascii=False, indent=2) + "\n", encoding="utf-8")

    @classmethod
    def from_dict(cls, value: object) -> "CorridorFrame":
        """JSON形式の値を検証し、廊下座標系を復元する。"""
        if not isinstance(value, dict):
            raise CorridorFrameError("Corridor frame JSON must contain an object.")
        try:
            offsets = tuple(float(item) for item in value["wall_offsets"])
            counts = tuple(int(item) for item in value.get("wall_inlier_counts", (0, 0)))
            if len(offsets) != 2 or len(counts) != 2:
                raise ValueError
            return cls(
                origin_xy=np.asarray(value["origin_xy"], dtype=float),
                longitudinal_xy=np.asarray(value["longitudinal_xy"], dtype=float),
                lateral_xy=np.asarray(value["lateral_xy"], dtype=float),
                wall_offsets=(offsets[0], offsets[1]), wall_gap=float(value["wall_gap"]),
                wall_inlier_counts=(counts[0], counts[1]),
                source_frame=str(value.get("source_frame", "livox_frame")),
                name=str(value.get("name", "corridor")),
            )
        except (KeyError, TypeError, ValueError) as exc:
            raise CorridorFrameError("Corridor frame JSON has invalid fields.") from exc

    @classmethod
    def from_json(cls, path: Path) -> "CorridorFrame":
        try:
            return cls.from_dict(json.loads(path.read_text(encoding="utf-8")))
        except (OSError, json.JSONDecodeError) as exc:
            raise CorridorFrameError(f"Could not read corridor frame {path}: {exc}") from exc


@dataclasses.dataclass(frozen=True)
class _WallLine:
    normal: np.ndarray
    offset: float
    inlier_count: int
    span: float


def _fit_line(points: np.ndarray, tolerance: float) -> Optional[_WallLine]:
    """点群を主成分分析で直線近似し、直線上の点数と長さを求める。"""

    if len(points) < 2:
        return None
    center = points[:, :2].mean(axis=0)
    _, _, vh = np.linalg.svd(points[:, :2] - center, full_matrices=False)
    direction = vh[0]
    direction /= np.linalg.norm(direction)
    normal = np.array([-direction[1], direction[0]], dtype=float)
    offset = float(normal @ center)
    distances = np.abs(points[:, :2] @ normal - offset)
    inliers = points[distances <= tolerance]
    if len(inliers) < 2:
        return None
    projected = inliers[:, :2] @ direction
    return _WallLine(normal, offset, len(inliers), float(projected.max() - projected.min()))


def _ransac_line(points: np.ndarray, rng: np.random.Generator, iterations: int, tolerance: float, min_span: float) -> Optional[_WallLine]:
    """RANSACで、長く連続した壁らしい直線候補を1本探す。"""

    best: Optional[_WallLine] = None
    best_score = -1.0
    if len(points) < 2:
        return None
    for _ in range(iterations):
        first, second = rng.integers(0, len(points), size=2)
        if first == second:
            continue
        delta = points[second, :2] - points[first, :2]
        length = float(np.linalg.norm(delta))
        if length < min_span:
            continue
        direction = delta / length
        normal = np.array([-direction[1], direction[0]])
        offset = float(normal @ points[first, :2])
        inliers = points[np.abs(points[:, :2] @ normal - offset) <= tolerance]
        candidate = _fit_line(inliers, tolerance)
        if candidate is None or candidate.span < min_span:
            continue
        score = candidate.inlier_count * min(2.0, candidate.span / min_span)
        if score > best_score:
            best, best_score = candidate, score
    return best


def _refit(points: np.ndarray, line: _WallLine, tolerance: float) -> Optional[_WallLine]:
    """選んだ直線候補を、元の点群全体で再度フィッティングする。"""

    return _fit_line(points[np.abs(points[:, :2] @ line.normal - line.offset) <= tolerance], tolerance)


def estimate_corridor_frame(
    points: np.ndarray, *, z_min: float = 0.5, z_max: float = 2.2,
    x_min: float = -30.0, x_max: float = 30.0, y_min: float = -15.0, y_max: float = 15.0,
    voxel_size: float = 0.10, line_tolerance: float = 0.12, min_wall_span: float = 3.0,
    min_wall_gap: float = 0.8, max_wall_gap: float = 8.0, max_parallel_angle_deg: float = 15.0,
    ransac_iterations: int = 2500, candidate_count: int = 8, seed: int = 20260924,
) -> CorridorFrame:
    """左右の壁を2本の平行線として推定し、廊下中心座標系を返す。"""

    # 壁の高さ帯とセンサー周辺のROIだけを使う。
    values = np.asarray(points, dtype=float)
    if values.ndim != 2 or values.shape[1] != 3:
        raise CorridorFrameError("Point cloud must be an N x 3 array.")
    # NaN/Infを除き、床や天井の点を高さ条件で除外する。
    mask = np.isfinite(values).all(axis=1)
    mask &= ((values[:, 0] >= x_min) & (values[:, 0] <= x_max) & (values[:, 1] >= y_min) & (values[:, 1] <= y_max) & (values[:, 2] >= z_min) & (values[:, 2] <= z_max))
    values = values[mask]
    if len(values) < 20:
        raise CorridorFrameError("Not enough finite points in the wall estimation ROI.")
    # 同じXYボクセルの点を1点にまとめ、RANSACの計算量と偏りを抑える。
    keys = np.floor(values[:, :2] / voxel_size).astype(np.int64)
    _, indices = np.unique(keys, axis=0, return_index=True)
    values = values[np.sort(indices)]
    rng = np.random.default_rng(seed)
    # 最も強い壁を見つけたらその点を除き、別の壁候補を順番に探す。
    remaining = values
    candidates: list[_WallLine] = []
    for _ in range(max(2, candidate_count)):
        line = _ransac_line(remaining, rng, ransac_iterations, line_tolerance, min_wall_span)
        if line is None:
            break
        line = _refit(values, line, line_tolerance)
        if line is None:
            break
        candidates.append(line)
        remaining = remaining[np.abs(remaining[:, :2] @ line.normal - line.offset) > line_tolerance * 1.5]
        if len(remaining) < 20:
            break
    if len(candidates) < 2:
        raise CorridorFrameError("Could not find two extended wall lines. Adjust wall height/ROI.")

    # 候補の中から、平行で壁間隔が妥当な2本の組み合わせを選ぶ。
    best: Optional[tuple[_WallLine, _WallLine]] = None
    best_score = -1
    cos_limit = math.cos(math.radians(max_parallel_angle_deg))
    for index, first in enumerate(candidates):
        for second in candidates[index + 1:]:
            if abs(float(first.normal @ second.normal)) < cos_limit:
                continue
            second_normal = second.normal.copy()
            second_offset = second.offset
            if float(first.normal @ second_normal) < 0:
                second_normal *= -1
                second_offset *= -1
            gap = abs(second_offset - first.offset)
            if not min_wall_gap <= gap <= max_wall_gap:
                continue
            if first.inlier_count + second.inlier_count > best_score:
                best = (first, _WallLine(second_normal, second_offset, second.inlier_count, second.span))
                best_score = first.inlier_count + second.inlier_count
    if best is None:
        raise CorridorFrameError("Could not find two parallel wall lines with a plausible corridor width.")
    first, second = best
    # 2本の壁の法線を平均し、左右方向の軸を安定させる。
    normal = first.normal + second.normal
    normal /= np.linalg.norm(normal)
    second_offset = second.offset if first.normal @ second.normal >= 0 else -second.offset
    second_normal = second.normal if first.normal @ second.normal >= 0 else -second.normal
    offsets = sorted((first.offset * float(normal @ first.normal), second_offset * float(normal @ second_normal)))
    gap = offsets[1] - offsets[0]
    if not min_wall_gap <= gap <= max_wall_gap:
        raise CorridorFrameError("The fitted wall gap is outside the valid range.")
    lateral = normal
    # 法線と直交する軸が廊下方向。センサーの+X側を正にそろえる。
    longitudinal = np.array([lateral[1], -lateral[0]], dtype=float)
    longitudinal /= np.linalg.norm(longitudinal)
    if longitudinal[0] < 0 or (abs(longitudinal[0]) < 1e-8 and longitudinal[1] < 0):
        longitudinal *= -1
    counts = tuple(sorted((first.inlier_count, second.inlier_count)))
    return CorridorFrame(
        origin_xy=lateral * ((offsets[0] + offsets[1]) / 2),
        longitudinal_xy=longitudinal, lateral_xy=lateral,
        wall_offsets=(offsets[0], offsets[1]), wall_gap=gap,
        wall_inlier_counts=(int(counts[0]), int(counts[1])),
    )
