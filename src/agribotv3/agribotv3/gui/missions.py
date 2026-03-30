"""Mission utility helpers for GUI interactions."""
from __future__ import annotations

import math
import random


def random_waypoints_in_area(area: tuple[float, float, float, float], count: int) -> list[tuple[float, float]]:
    x0, y0, x1, y1 = area
    return [(random.uniform(x0, x1), random.uniform(y0, y1)) for _ in range(max(0, count))]


def plant_cell_centres_in_area(
    plants: list[dict],
    area: tuple[float, float, float, float],
    cell_size: float,
) -> list[tuple[float, float]]:
    x0, y0, x1, y1 = area
    half_cell = cell_size / 2
    centres: list[tuple[float, float]] = []
    seen: set[tuple[float, float]] = set()

    for plant in plants:
        px = float(plant['x'])
        py = float(plant['y'])
        cx = math.floor(px / cell_size) * cell_size + half_cell
        cy = math.floor(py / cell_size) * cell_size + half_cell
        if x0 <= cx <= x1 and y0 <= cy <= y1:
            key = (round(cx, 3), round(cy, 3))
            if key in seen:
                continue
            seen.add(key)
            centres.append((cx, cy))

    return centres
