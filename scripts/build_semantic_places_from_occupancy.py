#!/usr/bin/env python3
from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

import cv2
import numpy as np
import yaml
from PIL import Image


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src" / "go2_gui_controller"))

from go2_gui_controller.semantic_place_geometry import point_in_polygon, polygon_area, polygon_centroid


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Build semantic place candidates from a Nav2 occupancy map."
    )
    parser.add_argument("--map-yaml", required=True, type=Path, help="Nav2 map YAML path.")
    parser.add_argument("--semantic-input", required=True, type=Path, help="Input semantic YAML path.")
    parser.add_argument("--output", required=True, type=Path, help="Output semantic YAML path.")
    parser.add_argument("--min-region-area-m2", type=float, default=1.0)
    parser.add_argument("--inflate-cells", type=int, default=1)
    parser.add_argument("--simplify-tolerance-m", type=float, default=0.25)
    parser.add_argument("--corridor-aspect-ratio", type=float, default=3.0)
    parser.add_argument("--max-place-assignment-distance-m", type=float, default=1.5)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    map_spec = load_map_yaml(args.map_yaml)
    image_path = resolve_image_path(args.map_yaml, map_spec)
    image = load_grayscale(image_path)
    free_mask = build_free_mask(image, map_spec)
    if args.inflate_cells > 0:
        free_mask = remove_inflated_obstacle_neighbors(free_mask, args.inflate_cells)

    places = build_places(
        free_mask=free_mask,
        resolution=float(map_spec["resolution"]),
        origin=tuple(float(value) for value in map_spec.get("origin", [0.0, 0.0, 0.0])[:2]),
        min_region_area_m2=args.min_region_area_m2,
        simplify_tolerance_m=args.simplify_tolerance_m,
        corridor_aspect_ratio=args.corridor_aspect_ratio,
    )

    document = yaml.safe_load(args.semantic_input.read_text(encoding="utf-8")) or {}
    document["places"] = places
    assign_object_places(document, max_assignment_distance_m=args.max_place_assignment_distance_m)

    manifest = document.setdefault("manifest", {})
    manifest["source_nav2_map"] = str(args.map_yaml)
    manifest["place_builder_version"] = "semantic_place_builder_v2"

    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(yaml.safe_dump(document, allow_unicode=True, sort_keys=False), encoding="utf-8")
    print(f"places: {len(places)}")
    print(f"output: {args.output}")
    return 0


def load_map_yaml(path: Path) -> dict:
    spec = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    if "image" not in spec:
        raise ValueError(f"missing image in {path}")
    if "resolution" not in spec:
        raise ValueError(f"missing resolution in {path}")
    return spec


def resolve_image_path(map_yaml: Path, spec: dict) -> Path:
    image_path = Path(str(spec["image"]))
    if not image_path.is_absolute():
        image_path = map_yaml.parent / image_path
    return image_path


def load_grayscale(path: Path) -> np.ndarray:
    with Image.open(path) as image:
        return np.asarray(image.convert("L"), dtype=np.uint8)


def build_free_mask(image: np.ndarray, spec: dict) -> np.ndarray:
    negate = int(spec.get("negate", 0))
    free_thresh = float(spec.get("free_thresh", 0.196))
    if negate:
        occupancy_probability = image.astype(float) / 255.0
    else:
        occupancy_probability = (255.0 - image.astype(float)) / 255.0
    free_mask = occupancy_probability <= free_thresh
    if str(spec.get("mode", "trinary")).lower() == "trinary":
        free_mask &= image != 205
    return free_mask.astype(np.uint8)


def remove_inflated_obstacle_neighbors(free_mask: np.ndarray, inflate_cells: int) -> np.ndarray:
    obstacle_mask = (free_mask == 0).astype(np.uint8)
    kernel_size = inflate_cells * 2 + 1
    kernel = np.ones((kernel_size, kernel_size), dtype=np.uint8)
    inflated_obstacles = cv2.dilate(obstacle_mask, kernel, iterations=1)
    return ((free_mask == 1) & (inflated_obstacles == 0)).astype(np.uint8)


def build_places(
    *,
    free_mask: np.ndarray,
    resolution: float,
    origin: tuple[float, float],
    min_region_area_m2: float,
    simplify_tolerance_m: float,
    corridor_aspect_ratio: float,
) -> dict:
    candidates = []
    for region_mask in build_region_masks(
        free_mask=free_mask,
        resolution=resolution,
        min_region_area_m2=min_region_area_m2,
    ):
        region_area_m2 = float(np.count_nonzero(region_mask)) * resolution * resolution
        if region_area_m2 < min_region_area_m2:
            continue
        contour = largest_contour(region_mask.astype(np.uint8) * 255)
        if contour is None:
            continue
        polygon = contour_to_polygon(contour, free_mask.shape[0], resolution, origin, simplify_tolerance_m)
        if len(polygon) < 3:
            polygon = stats_to_polygon(mask_stats(region_mask), free_mask.shape[0], resolution, origin)
        area_m2 = polygon_area(polygon)
        if area_m2 < min_region_area_m2:
            continue
        _x, _y, width, height, _area = mask_stats(region_mask)
        width_m = float(width) * resolution
        height_m = float(height) * resolution
        aspect_ratio = max(width_m, height_m) / max(min(width_m, height_m), 1e-6)
        place_label = "corridor" if aspect_ratio >= corridor_aspect_ratio else "room"
        centroid_x, centroid_y = polygon_centroid(polygon)
        candidates.append(
            {
                "label": place_label,
                "polygon": [[round(x, 4), round(y, 4)] for x, y in polygon],
                "centroid_x": round(float(centroid_x), 4),
                "centroid_y": round(float(centroid_y), 4),
                "area_m2": round(float(area_m2), 4),
                "confidence": round(min(region_area_m2 / max(min_region_area_m2, 1e-6), 10.0) / 10.0, 4),
            }
        )

    counters = {"room": 0, "corridor": 0}
    places = {}
    for candidate in sorted(candidates, key=lambda item: (item["label"], item["centroid_x"], item["centroid_y"])):
        counters[candidate["label"]] += 1
        place_id = f"{candidate['label']}_candidate_{counters[candidate['label']]}"
        places[place_id] = {
            "label": candidate["label"],
            "aliases": [],
            "frame_id": "map",
            "polygon": candidate["polygon"],
            "centroid_x": candidate["centroid_x"],
            "centroid_y": candidate["centroid_y"],
            "area_m2": candidate["area_m2"],
            "confidence": candidate["confidence"],
            "source": "auto_occupancy_watershed",
            "enabled": True,
        }
    return places


def build_region_masks(
    *,
    free_mask: np.ndarray,
    resolution: float,
    min_region_area_m2: float,
) -> list[np.ndarray]:
    labels_count, labels, stats, _centroids = cv2.connectedComponentsWithStats(free_mask, connectivity=8)
    region_masks: list[np.ndarray] = []
    for component_label in range(1, labels_count):
        component_area_m2 = float(stats[component_label, cv2.CC_STAT_AREA]) * resolution * resolution
        if component_area_m2 < min_region_area_m2:
            continue
        component_mask = (labels == component_label).astype(np.uint8)
        region_masks.extend(
            split_component_by_distance_seeds(
                component_mask=component_mask,
                resolution=resolution,
                min_region_area_m2=min_region_area_m2,
            )
        )
    return region_masks


def split_component_by_distance_seeds(
    *,
    component_mask: np.ndarray,
    resolution: float,
    min_region_area_m2: float,
) -> list[np.ndarray]:
    seeds = find_distance_seeds(
        component_mask=component_mask,
        resolution=resolution,
        min_region_area_m2=min_region_area_m2,
    )
    if len(seeds) < 2:
        return [component_mask]

    rows, cols = np.nonzero(component_mask)
    seed_rows = np.asarray([seed[0] for seed in seeds], dtype=np.int32)
    seed_cols = np.asarray([seed[1] for seed in seeds], dtype=np.int32)
    manhattan_distances = (
        np.abs(rows[:, None] - seed_rows[None, :])
        + np.abs(cols[:, None] - seed_cols[None, :])
    )
    nearest_seed_indexes = np.argmin(manhattan_distances, axis=1)

    region_masks = []
    for seed_index in range(len(seeds)):
        region_mask = np.zeros_like(component_mask, dtype=np.uint8)
        selected = nearest_seed_indexes == seed_index
        region_mask[rows[selected], cols[selected]] = 1
        region_masks.extend(connected_subregion_masks(region_mask))

    min_region_cells = max(1, math.ceil(min_region_area_m2 / max(resolution * resolution, 1e-12)))
    valid_region_masks = [
        region_mask
        for region_mask in region_masks
        if int(np.count_nonzero(region_mask)) >= min_region_cells
    ]
    if len(valid_region_masks) < 2:
        return [component_mask]

    component_cells = int(np.count_nonzero(component_mask))
    valid_cells = sum(int(np.count_nonzero(region_mask)) for region_mask in valid_region_masks)
    if valid_cells / max(component_cells, 1) < 0.7:
        return [component_mask]

    return sorted(
        valid_region_masks,
        key=lambda mask: (mask_stats(mask)[cv2.CC_STAT_TOP], mask_stats(mask)[cv2.CC_STAT_LEFT]),
    )


def find_distance_seeds(
    *,
    component_mask: np.ndarray,
    resolution: float,
    min_region_area_m2: float,
) -> list[tuple[int, int, float]]:
    distance = cv2.distanceTransform(component_mask.astype(np.uint8), cv2.DIST_L1, 3)
    if float(distance.max()) < 2.0:
        return []

    kernel = np.ones((3, 3), dtype=np.uint8)
    local_maximum = distance >= cv2.dilate(distance, kernel) - 1e-6
    peak_mask = ((component_mask == 1) & local_maximum & (distance >= 2.0)).astype(np.uint8)
    labels_count, labels, _stats, _centroids = cv2.connectedComponentsWithStats(peak_mask, connectivity=8)

    raw_seeds: list[tuple[int, int, float]] = []
    for label in range(1, labels_count):
        coordinates = np.argwhere(labels == label)
        if len(coordinates) == 0:
            continue
        centroid = coordinates.mean(axis=0)
        best_row, best_col = min(
            ((int(row), int(col)) for row, col in coordinates),
            key=lambda point: (
                -float(distance[point[0], point[1]]),
                (point[0] - centroid[0]) ** 2 + (point[1] - centroid[1]) ** 2,
                point[0],
                point[1],
            ),
        )
        raw_seeds.append((best_row, best_col, float(distance[best_row, best_col])))

    raw_seeds.sort(key=lambda seed: (-seed[2], seed[0], seed[1]))
    min_seed_spacing_cells = max(
        2.0,
        0.5 * math.sqrt(max(min_region_area_m2, 0.0)) / max(resolution, 1e-9),
    )
    seeds: list[tuple[int, int, float]] = []
    for row, col, score in raw_seeds:
        has_enough_spacing = all(
            abs(row - kept_row) + abs(col - kept_col) >= min_seed_spacing_cells
            for kept_row, kept_col, _ in seeds
        )
        if has_enough_spacing:
            seeds.append((row, col, score))
    return sorted(seeds, key=lambda seed: (seed[0], seed[1]))


def connected_subregion_masks(mask: np.ndarray) -> list[np.ndarray]:
    labels_count, labels, _stats, _centroids = cv2.connectedComponentsWithStats(mask.astype(np.uint8), connectivity=8)
    return [(labels == label).astype(np.uint8) for label in range(1, labels_count)]


def mask_stats(mask: np.ndarray) -> np.ndarray:
    rows, cols = np.nonzero(mask)
    if len(rows) == 0:
        return np.asarray([0, 0, 0, 0, 0], dtype=np.int32)
    left = int(cols.min())
    top = int(rows.min())
    width = int(cols.max() - left + 1)
    height = int(rows.max() - top + 1)
    area = int(len(rows))
    return np.asarray([left, top, width, height, area], dtype=np.int32)


def largest_contour(mask: np.ndarray):
    contours, _hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None
    return max(contours, key=cv2.contourArea)


def contour_to_polygon(
    contour,
    image_height: int,
    resolution: float,
    origin: tuple[float, float],
    simplify_tolerance_m: float,
) -> tuple[tuple[float, float], ...]:
    epsilon = max(simplify_tolerance_m / max(resolution, 1e-9), 0.5)
    approx = cv2.approxPolyDP(contour, epsilon, True)
    points = []
    for point in approx.reshape(-1, 2):
        col = float(point[0])
        row = float(point[1])
        points.append(pixel_to_map(col, row, image_height, resolution, origin))
    return tuple(points)


def stats_to_polygon(
    stats_row,
    image_height: int,
    resolution: float,
    origin: tuple[float, float],
) -> tuple[tuple[float, float], ...]:
    col = float(stats_row[cv2.CC_STAT_LEFT])
    row = float(stats_row[cv2.CC_STAT_TOP])
    width = float(stats_row[cv2.CC_STAT_WIDTH])
    height = float(stats_row[cv2.CC_STAT_HEIGHT])
    return (
        pixel_boundary_to_map(col, row + height, image_height, resolution, origin),
        pixel_boundary_to_map(col + width, row + height, image_height, resolution, origin),
        pixel_boundary_to_map(col + width, row, image_height, resolution, origin),
        pixel_boundary_to_map(col, row, image_height, resolution, origin),
    )


def pixel_to_map(
    col: float,
    row: float,
    image_height: int,
    resolution: float,
    origin: tuple[float, float],
) -> tuple[float, float]:
    return pixel_boundary_to_map(col + 0.5, row + 0.5, image_height, resolution, origin)


def pixel_boundary_to_map(
    col: float,
    row: float,
    image_height: int,
    resolution: float,
    origin: tuple[float, float],
) -> tuple[float, float]:
    x = origin[0] + col * resolution
    y = origin[1] + (image_height - row) * resolution
    return float(x), float(y)


def assign_object_places(document: dict, max_assignment_distance_m: float = 1.5) -> None:
    places = document.get("places") or {}
    place_polygons = [
        (place_id, tuple((float(x), float(y)) for x, y in place.get("polygon", [])), float(place.get("area_m2", 0.0)))
        for place_id, place in places.items()
        if place.get("enabled", True)
    ]
    place_polygons.sort(key=lambda item: (item[2], item[0]))

    for entry in (document.get("objects") or {}).values():
        entry.pop("place_id", None)
        candidate_points = list(object_place_candidate_points(entry))
        for x, y in candidate_points:
            matched = False
            for place_id, polygon, _area in place_polygons:
                if point_in_polygon(x, y, polygon):
                    entry["place_id"] = place_id
                    matched = True
                    break
            if matched:
                break
        if "place_id" not in entry:
            nearest = nearest_place(candidate_points, place_polygons)
            if nearest is not None and nearest[0] <= max_assignment_distance_m:
                entry["place_id"] = nearest[2]


def object_place_candidate_points(entry: dict):
    for x_key, y_key in (("x", "y"), ("observer_x", "observer_y")):
        try:
            yield float(entry[x_key]), float(entry[y_key])
        except (KeyError, TypeError, ValueError):
            continue


def nearest_place(
    points: list[tuple[float, float]],
    place_polygons: list[tuple[str, tuple[tuple[float, float], ...], float]],
) -> tuple[float, float, str] | None:
    nearest: tuple[float, float, str] | None = None
    for x, y in points:
        for place_id, polygon, area_m2 in place_polygons:
            distance = point_to_polygon_distance(x, y, polygon)
            candidate = (distance, area_m2, place_id)
            if nearest is None or candidate < nearest:
                nearest = candidate
    return nearest


def point_to_polygon_distance(x: float, y: float, polygon: tuple[tuple[float, float], ...]) -> float:
    if not polygon:
        return math.inf
    if point_in_polygon(x, y, polygon):
        return 0.0
    if len(polygon) == 1:
        px, py = polygon[0]
        return math.hypot(x - px, y - py)
    return min(
        point_to_segment_distance(x, y, ax, ay, bx, by)
        for (ax, ay), (bx, by) in zip(polygon, polygon[1:] + polygon[:1])
    )


def point_to_segment_distance(
    px: float,
    py: float,
    ax: float,
    ay: float,
    bx: float,
    by: float,
) -> float:
    dx = bx - ax
    dy = by - ay
    if abs(dx) < 1e-12 and abs(dy) < 1e-12:
        return math.hypot(px - ax, py - ay)
    t = ((px - ax) * dx + (py - ay) * dy) / (dx * dx + dy * dy)
    t = max(0.0, min(1.0, t))
    closest_x = ax + t * dx
    closest_y = ay + t * dy
    return math.hypot(px - closest_x, py - closest_y)


if __name__ == "__main__":
    raise SystemExit(main())
