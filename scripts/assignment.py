"""追加の最適化ライブラリを使わず、距離制限付きの1対1対応を求める。"""

from __future__ import annotations

import numpy as np


def gated_assignment(costs: np.ndarray, allowed: np.ndarray) -> list[tuple[int, int]]:
    """許可された対応数を最大にし、その中で総コストを最小にする。

    矩形のハンガリアン法を用い、未対応を表すダミー列を追加する。
    allowedが偽の辺は選ばず、有効な辺のコストには有限の非負値を要求する。
    """

    costs = np.asarray(costs, dtype=float)
    allowed = np.asarray(allowed, dtype=bool)
    if costs.ndim != 2 or costs.shape != allowed.shape:
        raise ValueError("Costs and gate must be equally shaped matrices.")
    n, m = costs.shape
    if not n or not m:
        return []
    if np.any(~np.isfinite(costs[allowed])) or np.any(costs[allowed] < 0):
        raise ValueError("Allowed assignment costs must be finite and nonnegative.")
    scale = max(1.0, float(np.max(costs[allowed])) if allowed.any() else 1.0)
    # 未対応の罰則を十分大きくし、総コストより有効対応数を優先する。
    penalty = (min(n, m) + 1) * scale
    # 行数分のダミー列で未対応を許す。禁止辺はダミーより高コストにする。
    matrix = np.full((n, m + n), penalty)
    matrix[:, :m] = np.where(allowed, costs, penalty * (n + 2))
    columns = matrix.shape[1]
    u, v = np.zeros(n + 1), np.zeros(columns + 1)
    p, way = np.zeros(columns + 1, dtype=int), np.zeros(columns + 1, dtype=int)
    # 各行について増加路を探索し、既存の対応を組み替えながら最適化する。
    for i in range(1, n + 1):
        p[0] = i
        j0 = 0
        minimum = np.full(columns + 1, np.inf)
        used = np.zeros(columns + 1, dtype=bool)
        while True:
            used[j0] = True
            i0 = p[j0]
            delta, j1 = np.inf, 0
            for j in range(1, columns + 1):
                if not used[j]:
                    current = matrix[i0 - 1, j - 1] - u[i0] - v[j]
                    if current < minimum[j]:
                        minimum[j], way[j] = current, j0
                    if minimum[j] < delta:
                        delta, j1 = minimum[j], j
            for j in range(columns + 1):
                if used[j]:
                    u[p[j]] += delta
                    v[j] -= delta
                else:
                    minimum[j] -= delta
            j0 = j1
            if p[j0] == 0:
                break
        # 見つけた増加路を逆にたどり、今回の対応を確定する。
        while j0:
            j1 = way[j0]
            p[j0] = p[j1]
            j0 = j1
    return sorted(
        (int(p[j] - 1), j - 1) for j in range(1, m + 1) if p[j] and allowed[p[j] - 1, j - 1]
    )
