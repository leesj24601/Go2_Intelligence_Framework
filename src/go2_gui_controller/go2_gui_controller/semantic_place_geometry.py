from __future__ import annotations

from typing import Iterable


Point = tuple[float, float]


def point_in_polygon(x: float, y: float, polygon: Iterable[Point]) -> bool:
    points = list(polygon)
    if len(points) < 3:
        return False

    inside = False
    j = len(points) - 1
    for i, (xi, yi) in enumerate(points):
        xj, yj = points[j]
        crosses = (yi > y) != (yj > y)
        if crosses:
            x_at_y = (xj - xi) * (y - yi) / ((yj - yi) or 1e-12) + xi
            if x < x_at_y:
                inside = not inside
        j = i
    return inside


def polygon_area(polygon: Iterable[Point]) -> float:
    points = list(polygon)
    if len(points) < 3:
        return 0.0

    total = 0.0
    for index, (x1, y1) in enumerate(points):
        x2, y2 = points[(index + 1) % len(points)]
        total += x1 * y2 - x2 * y1
    return abs(total) * 0.5


def polygon_centroid(polygon: Iterable[Point]) -> Point:
    points = list(polygon)
    if not points:
        raise ValueError("polygon must contain at least one point")

    signed_area_twice = 0.0
    cx = 0.0
    cy = 0.0
    for index, (x1, y1) in enumerate(points):
        x2, y2 = points[(index + 1) % len(points)]
        cross = x1 * y2 - x2 * y1
        signed_area_twice += cross
        cx += (x1 + x2) * cross
        cy += (y1 + y2) * cross

    if abs(signed_area_twice) < 1e-12:
        return (sum(x for x, _ in points) / len(points), sum(y for _, y in points) / len(points))
    return (cx / (3.0 * signed_area_twice), cy / (3.0 * signed_area_twice))
