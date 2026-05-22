#!/usr/bin/env python3
from __future__ import annotations

import argparse
import math
import sys
from collections import deque
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
    parser.add_argument(
        "--unknown-bridge-cells",
        type=int,
        default=3,
        help="Fill small unknown gaps up to this cell radius for place segmentation only.",
    )
    parser.add_argument(
        "--seed-inflate-cells",
        type=int,
        default=None,
        help="Obstacle inflation radius used to form coarse place seeds. Defaults to inflate + unknown bridge + 5.",
    )
    parser.add_argument(
        "--max-region-seeds",
        type=int,
        default=12,
        help="Maximum distance-peak seeds per connected free-space component. Use 0 to disable peak growth.",
    )
    parser.add_argument("--seed-spacing-m", type=float, default=3.5)
    parser.add_argument("--min-seed-obstacle-distance-m", type=float, default=0.25)
    parser.add_argument("--simplify-tolerance-m", type=float, default=0.25)
    parser.add_argument("--corridor-aspect-ratio", type=float, default=3.0)
    parser.add_argument(
        "--hall-core-distance-m",
        type=float,
        default=1.3,
        help="Minimum obstacle distance used to seed the main hall in hall_core mode.",
    )
    parser.add_argument(
        "--hall-growth-m",
        type=float,
        default=1.5,
        help="Geodesic growth distance from the hall seed in hall_core mode.",
    )
    parser.add_argument(
        "--gateway-width-m",
        type=float,
        default=1.2,
        help="Free-space necks narrower than this are treated as gateway cut candidates.",
    )
    parser.add_argument("--max-place-assignment-distance-m", type=float, default=1.5)
    parser.add_argument(
        "--segmentation-mode",
        choices=("generic", "hall_core", "gateway_watershed", "auto"),
        default="generic",
        help="Segmentation algorithm to use with the occupancy map.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    map_spec = load_map_yaml(args.map_yaml)
    image_path = resolve_image_path(args.map_yaml, map_spec)
    image = load_grayscale(image_path)
    free_mask = build_place_free_mask(image, map_spec, args.unknown_bridge_cells)
    if args.inflate_cells > 0:
        free_mask = remove_inflated_obstacle_neighbors(free_mask, args.inflate_cells)
    resolution = float(map_spec["resolution"])
    seed_inflate_cells = args.seed_inflate_cells
    if seed_inflate_cells is None:
        seed_inflate_cells = max(args.inflate_cells + 1, math.ceil(0.65 / max(resolution, 1e-9)))

    origin = tuple(float(value) for value in map_spec.get("origin", [0.0, 0.0, 0.0])[:2])
    document = yaml.safe_load(args.semantic_input.read_text(encoding="utf-8")) or {}
    selected_segmentation_mode = args.segmentation_mode
    auto_scores: dict[str, float] = {}

    if args.segmentation_mode == "hall_core":
        places = build_hall_core_places(
            free_mask=free_mask,
            resolution=resolution,
            origin=origin,
            min_region_area_m2=args.min_region_area_m2,
            simplify_tolerance_m=args.simplify_tolerance_m,
            corridor_aspect_ratio=args.corridor_aspect_ratio,
            hall_core_distance_m=args.hall_core_distance_m,
            hall_growth_m=args.hall_growth_m,
        )
    elif args.segmentation_mode == "gateway_watershed":
        places = build_gateway_watershed_places(
            free_mask=free_mask,
            resolution=resolution,
            origin=origin,
            min_region_area_m2=args.min_region_area_m2,
            simplify_tolerance_m=args.simplify_tolerance_m,
            corridor_aspect_ratio=args.corridor_aspect_ratio,
            gateway_width_m=args.gateway_width_m,
        )
    elif args.segmentation_mode == "auto":
        places, selected_segmentation_mode, auto_scores = build_auto_places(
            free_mask=free_mask,
            resolution=resolution,
            origin=origin,
            min_region_area_m2=args.min_region_area_m2,
            simplify_tolerance_m=args.simplify_tolerance_m,
            corridor_aspect_ratio=args.corridor_aspect_ratio,
            seed_inflate_cells=seed_inflate_cells,
            max_region_seeds=args.max_region_seeds,
            seed_spacing_m=args.seed_spacing_m,
            min_seed_obstacle_distance_m=args.min_seed_obstacle_distance_m,
            hall_core_distance_m=args.hall_core_distance_m,
            hall_growth_m=args.hall_growth_m,
            gateway_width_m=args.gateway_width_m,
            semantic_document=document,
            max_assignment_distance_m=args.max_place_assignment_distance_m,
        )
    else:
        places = build_places(
            free_mask=free_mask,
            resolution=resolution,
            origin=origin,
            min_region_area_m2=args.min_region_area_m2,
            simplify_tolerance_m=args.simplify_tolerance_m,
            corridor_aspect_ratio=args.corridor_aspect_ratio,
            seed_inflate_cells=seed_inflate_cells,
            max_region_seeds=args.max_region_seeds,
            seed_spacing_m=args.seed_spacing_m,
            min_seed_obstacle_distance_m=args.min_seed_obstacle_distance_m,
        )

    document["places"] = places
    assign_object_places(document, max_assignment_distance_m=args.max_place_assignment_distance_m)

    manifest = document.setdefault("manifest", {})
    manifest["source_nav2_map"] = str(args.map_yaml)
    manifest["place_builder_version"] = "semantic_place_builder_v2"
    manifest["place_builder_segmentation_mode"] = args.segmentation_mode
    if args.segmentation_mode == "auto":
        manifest["place_builder_selected_segmentation_mode"] = selected_segmentation_mode
        manifest["place_builder_auto_scores"] = {
            key: round(float(value), 4)
            for key, value in sorted(auto_scores.items())
        }
    if selected_segmentation_mode == "hall_core":
        manifest["place_builder_hall_core_distance_m"] = float(args.hall_core_distance_m)
        manifest["place_builder_hall_growth_m"] = float(args.hall_growth_m)
    elif selected_segmentation_mode == "gateway_watershed":
        manifest["place_builder_gateway_width_m"] = float(args.gateway_width_m)
    else:
        manifest["place_builder_seed_inflate_cells"] = int(seed_inflate_cells)
        manifest["place_builder_max_region_seeds"] = int(args.max_region_seeds)

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


def build_place_free_mask(image: np.ndarray, spec: dict, unknown_bridge_cells: int = 3) -> np.ndarray:
    free_mask = build_free_mask(image, spec)
    if unknown_bridge_cells <= 0:
        closed_free = free_mask
    else:
        kernel_size = unknown_bridge_cells * 2 + 1
        kernel = np.ones((kernel_size, kernel_size), dtype=np.uint8)
        closed_free = cv2.morphologyEx(free_mask.astype(np.uint8), cv2.MORPH_CLOSE, kernel)

    occupied_mask = build_occupied_mask(image, spec)
    structural_occupied_mask = build_structural_occupied_mask(image, spec, occupied_mask, free_mask)
    nonstructural_occupied_mask = (occupied_mask == 1) & (structural_occupied_mask == 0)
    return (((closed_free == 1) | nonstructural_occupied_mask) & (structural_occupied_mask == 0)).astype(np.uint8)


def build_occupied_mask(image: np.ndarray, spec: dict) -> np.ndarray:
    negate = int(spec.get("negate", 0))
    occupied_thresh = float(spec.get("occupied_thresh", 0.65))
    if negate:
        occupancy_probability = image.astype(float) / 255.0
    else:
        occupancy_probability = (255.0 - image.astype(float)) / 255.0
    return (occupancy_probability >= occupied_thresh).astype(np.uint8)


def build_structural_occupied_mask(
    image: np.ndarray,
    spec: dict,
    occupied_mask: np.ndarray,
    free_mask: np.ndarray,
) -> np.ndarray:
    mode = str(spec.get("mode", "trinary")).lower()
    unknown_mask = ((image == 205) & (mode == "trinary")).astype(np.uint8)
    labels_count, labels, stats, _centroids = cv2.connectedComponentsWithStats(occupied_mask, connectivity=8)
    structural_mask = np.zeros_like(occupied_mask, dtype=np.uint8)
    kernel = np.ones((3, 3), dtype=np.uint8)

    for label in range(1, labels_count):
        component = (labels == label).astype(np.uint8)
        left = int(stats[label, cv2.CC_STAT_LEFT])
        top = int(stats[label, cv2.CC_STAT_TOP])
        width = int(stats[label, cv2.CC_STAT_WIDTH])
        height = int(stats[label, cv2.CC_STAT_HEIGHT])
        touches_image_border = (
            left == 0
            or top == 0
            or left + width >= occupied_mask.shape[1]
            or top + height >= occupied_mask.shape[0]
        )
        expanded = cv2.dilate(component, kernel, iterations=1)
        adjacent_unknown_cells = int(np.count_nonzero((expanded == 1) & (unknown_mask == 1)))
        adjacent_free_cells = int(np.count_nonzero((expanded == 1) & (free_mask == 1)))
        if touches_image_border or adjacent_unknown_cells >= adjacent_free_cells * 0.5:
            structural_mask[labels == label] = 1

    return structural_mask


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
    seed_inflate_cells: int = 0,
    max_region_seeds: int = 0,
    seed_spacing_m: float = 3.5,
    min_seed_obstacle_distance_m: float = 0.25,
) -> dict:
    candidates = []
    for region_mask in build_region_masks(
        free_mask=free_mask,
        resolution=resolution,
        min_region_area_m2=min_region_area_m2,
        seed_inflate_cells=seed_inflate_cells,
        max_region_seeds=max_region_seeds,
        seed_spacing_m=seed_spacing_m,
        min_seed_obstacle_distance_m=min_seed_obstacle_distance_m,
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


def build_hall_core_places(
    *,
    free_mask: np.ndarray,
    resolution: float,
    origin: tuple[float, float],
    min_region_area_m2: float,
    simplify_tolerance_m: float,
    corridor_aspect_ratio: float,
    hall_core_distance_m: float = 1.3,
    hall_growth_m: float = 1.5,
) -> dict:
    distance_m = cv2.distanceTransform(free_mask.astype(np.uint8), cv2.DIST_L2, 5) * resolution
    core_mask = ((free_mask == 1) & (distance_m >= hall_core_distance_m)).astype(np.uint8)
    core_labels_count, core_labels, core_stats, _centroids = cv2.connectedComponentsWithStats(
        core_mask,
        connectivity=8,
    )
    if core_labels_count <= 1:
        return build_places(
            free_mask=free_mask,
            resolution=resolution,
            origin=origin,
            min_region_area_m2=min_region_area_m2,
            simplify_tolerance_m=simplify_tolerance_m,
            corridor_aspect_ratio=corridor_aspect_ratio,
            seed_inflate_cells=0,
            max_region_seeds=0,
        )

    main_core_label = max(
        range(1, core_labels_count),
        key=lambda label: int(core_stats[label, cv2.CC_STAT_AREA]),
    )
    hall_seed_mask = (core_labels == main_core_label).astype(np.uint8)
    effective_hall_growth_m = hall_growth_m + hall_core_distance_m * 0.5
    hall_growth_cells = max(0, int(math.ceil(effective_hall_growth_m / max(resolution, 1e-9))))
    hall_mask = grow_mask_within_free_space(
        seed_mask=hall_seed_mask,
        free_mask=free_mask,
        max_steps=hall_growth_cells,
    )

    places = {}
    hall_place = place_entry_from_mask(
        mask=hall_mask,
        label="hall",
        free_mask=free_mask,
        resolution=resolution,
        origin=origin,
        min_region_area_m2=min_region_area_m2,
        simplify_tolerance_m=simplify_tolerance_m,
        confidence=0.85,
        source="auto_occupancy_hall_core",
    )
    if hall_place is None:
        return {}
    places["main_hall"] = hall_place

    remaining_mask = ((free_mask == 1) & (hall_mask == 0)).astype(np.uint8)
    remaining_candidates: list[tuple[str, np.ndarray]] = []
    for component_mask in connected_subregion_masks(remaining_mask):
        component_area_m2 = float(np.count_nonzero(component_mask)) * resolution * resolution
        if component_area_m2 < min_region_area_m2:
            continue
        if component_area_m2 >= 20.0:
            split_masks = split_component_by_peak_seed_growth(
                component_mask=component_mask,
                max_region_seeds=4,
                seed_spacing_m=3.0,
                min_seed_obstacle_distance_m=0.45,
                resolution=resolution,
                min_region_area_m2=max(min_region_area_m2, 4.0),
            )
        else:
            split_masks = [component_mask]
        for split_mask in split_masks:
            if float(np.count_nonzero(split_mask)) * resolution * resolution < min_region_area_m2:
                continue
            place_label = classify_room_or_corridor(
                mask=split_mask,
                resolution=resolution,
                corridor_aspect_ratio=corridor_aspect_ratio,
            )
            remaining_candidates.append((place_label, split_mask))

    counters = {"room": 0, "corridor": 0}
    sorted_candidates = sorted(
        remaining_candidates,
        key=lambda item: (
            mask_stats(item[1])[cv2.CC_STAT_TOP],
            mask_stats(item[1])[cv2.CC_STAT_LEFT],
            item[0],
        ),
    )
    for place_label, region_mask in sorted_candidates:
        place_entry = place_entry_from_mask(
            mask=region_mask,
            label=place_label,
            free_mask=free_mask,
            resolution=resolution,
            origin=origin,
            min_region_area_m2=min_region_area_m2,
            simplify_tolerance_m=simplify_tolerance_m,
            confidence=0.7,
            source="auto_occupancy_hall_core",
        )
        if place_entry is None:
            continue
        counters[place_label] += 1
        places[f"{place_label}_candidate_{counters[place_label]}"] = place_entry
    return places


def build_gateway_watershed_places(
    *,
    free_mask: np.ndarray,
    resolution: float,
    origin: tuple[float, float],
    min_region_area_m2: float,
    simplify_tolerance_m: float,
    corridor_aspect_ratio: float,
    gateway_width_m: float = 1.2,
) -> dict:
    candidates: list[tuple[str, np.ndarray]] = []
    for region_mask in build_gateway_region_masks(
        free_mask=free_mask,
        resolution=resolution,
        min_region_area_m2=min_region_area_m2,
        gateway_width_m=gateway_width_m,
    ):
        place_label = classify_room_or_corridor(
            mask=region_mask,
            resolution=resolution,
            corridor_aspect_ratio=corridor_aspect_ratio,
        )
        candidates.append((place_label, region_mask))

    counters = {"room": 0, "corridor": 0}
    places = {}
    for place_label, region_mask in sorted(
        candidates,
        key=lambda item: (
            item[0],
            mask_stats(item[1])[cv2.CC_STAT_TOP],
            mask_stats(item[1])[cv2.CC_STAT_LEFT],
        ),
    ):
        place_entry = place_entry_from_mask(
            mask=region_mask,
            label=place_label,
            free_mask=free_mask,
            resolution=resolution,
            origin=origin,
            min_region_area_m2=min_region_area_m2,
            simplify_tolerance_m=simplify_tolerance_m,
            confidence=0.78,
            source="auto_occupancy_gateway_watershed",
        )
        if place_entry is None:
            continue
        counters[place_label] += 1
        places[f"{place_label}_candidate_{counters[place_label]}"] = place_entry
    return places


def build_gateway_region_masks(
    *,
    free_mask: np.ndarray,
    resolution: float,
    min_region_area_m2: float,
    gateway_width_m: float,
) -> list[np.ndarray]:
    labels_count, labels, stats, _centroids = cv2.connectedComponentsWithStats(free_mask, connectivity=8)
    region_masks: list[np.ndarray] = []
    for component_label in range(1, labels_count):
        component_area_m2 = float(stats[component_label, cv2.CC_STAT_AREA]) * resolution * resolution
        if component_area_m2 < min_region_area_m2:
            continue
        component_mask = (labels == component_label).astype(np.uint8)
        region_masks.extend(
            split_component_by_gateway_watershed(
                component_mask=component_mask,
                resolution=resolution,
                min_region_area_m2=min_region_area_m2,
                gateway_width_m=gateway_width_m,
            )
        )
    return region_masks


def split_component_by_gateway_watershed(
    *,
    component_mask: np.ndarray,
    resolution: float,
    min_region_area_m2: float,
    gateway_width_m: float,
) -> list[np.ndarray]:
    distance_m = cv2.distanceTransform(component_mask.astype(np.uint8), cv2.DIST_L2, 5) * resolution
    seed_distance_m = max(gateway_width_m * 0.5, resolution)
    seed_mask = ((component_mask == 1) & (distance_m >= seed_distance_m)).astype(np.uint8)
    seed_labels_count, seed_labels, seed_stats, _centroids = cv2.connectedComponentsWithStats(
        seed_mask,
        connectivity=8,
    )
    min_seed_area_m2 = max(min_region_area_m2 * 0.05, gateway_width_m * gateway_width_m * 0.1)
    min_seed_cells = max(1, math.ceil(min_seed_area_m2 / max(resolution * resolution, 1e-12)))

    filtered_seed_labels = np.zeros_like(seed_labels, dtype=np.int32)
    seed_index = 0
    for label in range(1, seed_labels_count):
        if int(seed_stats[label, cv2.CC_STAT_AREA]) < min_seed_cells:
            continue
        seed_index += 1
        filtered_seed_labels[seed_labels == label] = seed_index

    if seed_index < 2:
        return [component_mask]

    grown_labels = grow_component_labels_from_seed_labels(
        component_mask=component_mask,
        seed_labels=filtered_seed_labels,
    )
    min_region_cells = max(1, math.ceil(min_region_area_m2 / max(resolution * resolution, 1e-12)))
    region_masks = []
    for label in range(1, seed_index + 1):
        region_masks.extend(connected_subregion_masks((grown_labels == label).astype(np.uint8)))

    valid_region_masks = [
        region_mask
        for region_mask in region_masks
        if int(np.count_nonzero(region_mask)) >= min_region_cells
    ]
    if len(valid_region_masks) < 2:
        return [component_mask]

    component_cells = int(np.count_nonzero(component_mask))
    valid_cells = sum(int(np.count_nonzero(region_mask)) for region_mask in valid_region_masks)
    if valid_cells / max(component_cells, 1) < 0.85:
        return [component_mask]

    return sorted(
        valid_region_masks,
        key=lambda mask: (mask_stats(mask)[cv2.CC_STAT_TOP], mask_stats(mask)[cv2.CC_STAT_LEFT]),
    )


def grow_component_labels_from_seed_labels(
    *,
    component_mask: np.ndarray,
    seed_labels: np.ndarray,
) -> np.ndarray:
    grown_labels = np.zeros(component_mask.shape, dtype=np.int32)
    queue: deque[tuple[int, int, int]] = deque()
    rows, cols = np.nonzero((component_mask == 1) & (seed_labels > 0))
    for row, col in zip(rows, cols):
        label = int(seed_labels[row, col])
        grown_labels[row, col] = label
        queue.append((int(row), int(col), label))

    neighbor_offsets = ((-1, 0), (1, 0), (0, -1), (0, 1))
    while queue:
        row, col, label = queue.popleft()
        for row_offset, col_offset in neighbor_offsets:
            next_row = row + row_offset
            next_col = col + col_offset
            if (
                next_row < 0
                or next_col < 0
                or next_row >= component_mask.shape[0]
                or next_col >= component_mask.shape[1]
            ):
                continue
            if component_mask[next_row, next_col] != 1 or grown_labels[next_row, next_col] > 0:
                continue
            grown_labels[next_row, next_col] = label
            queue.append((next_row, next_col, label))
    return grown_labels


def build_auto_places(
    *,
    free_mask: np.ndarray,
    resolution: float,
    origin: tuple[float, float],
    min_region_area_m2: float,
    simplify_tolerance_m: float,
    corridor_aspect_ratio: float,
    seed_inflate_cells: int,
    max_region_seeds: int,
    seed_spacing_m: float,
    min_seed_obstacle_distance_m: float,
    hall_core_distance_m: float,
    hall_growth_m: float,
    gateway_width_m: float,
    semantic_document: dict,
    max_assignment_distance_m: float,
) -> tuple[dict, str, dict[str, float]]:
    candidate_places = {
        "gateway_watershed": build_gateway_watershed_places(
            free_mask=free_mask,
            resolution=resolution,
            origin=origin,
            min_region_area_m2=min_region_area_m2,
            simplify_tolerance_m=simplify_tolerance_m,
            corridor_aspect_ratio=corridor_aspect_ratio,
            gateway_width_m=gateway_width_m,
        ),
        "hall_core": build_hall_core_places(
            free_mask=free_mask,
            resolution=resolution,
            origin=origin,
            min_region_area_m2=min_region_area_m2,
            simplify_tolerance_m=simplify_tolerance_m,
            corridor_aspect_ratio=corridor_aspect_ratio,
            hall_core_distance_m=hall_core_distance_m,
            hall_growth_m=hall_growth_m,
        ),
        "generic": build_places(
            free_mask=free_mask,
            resolution=resolution,
            origin=origin,
            min_region_area_m2=min_region_area_m2,
            simplify_tolerance_m=simplify_tolerance_m,
            corridor_aspect_ratio=corridor_aspect_ratio,
            seed_inflate_cells=seed_inflate_cells,
            max_region_seeds=max_region_seeds,
            seed_spacing_m=seed_spacing_m,
            min_seed_obstacle_distance_m=min_seed_obstacle_distance_m,
        ),
    }
    free_area_m2 = float(np.count_nonzero(free_mask)) * resolution * resolution
    scores = {
        mode: score_place_candidates(
            places=places,
            free_area_m2=free_area_m2,
            min_region_area_m2=min_region_area_m2,
            semantic_document=semantic_document,
            max_assignment_distance_m=max_assignment_distance_m,
        )
        for mode, places in candidate_places.items()
    }
    priority = {"gateway_watershed": 2, "hall_core": 1, "generic": 0}
    selected_mode = max(scores.keys(), key=lambda mode: (scores[mode], priority[mode]))
    return candidate_places[selected_mode], selected_mode, scores


def score_place_candidates(
    *,
    places: dict,
    free_area_m2: float,
    min_region_area_m2: float,
    semantic_document: dict,
    max_assignment_distance_m: float,
) -> float:
    enabled_places = [place for place in places.values() if place.get("enabled", True)]
    if not enabled_places or free_area_m2 <= 0.0:
        return -1_000_000.0

    areas = [max(float(place.get("area_m2", 0.0)), 0.0) for place in enabled_places]
    largest_ratio = max(areas) / max(free_area_m2, 1e-9)
    coverage_ratio = min(sum(areas) / max(free_area_m2, 1e-9), 1.0)
    coverage_score = 1.0 - abs(1.0 - coverage_ratio)
    small_region_count = sum(1 for area in areas if area < min_region_area_m2 * 1.5)

    score = coverage_score
    score += min(len(enabled_places), 8) * 0.08
    if len(enabled_places) == 1:
        score -= 0.35
    score -= max(0.0, largest_ratio - 0.82) * 1.8
    score -= max(0, len(enabled_places) - 8) * 0.18
    score -= small_region_count * 0.12

    assignment_ratio = place_assignment_ratio(
        places=places,
        semantic_document=semantic_document,
        max_assignment_distance_m=max_assignment_distance_m,
    )
    if assignment_ratio is not None:
        score += assignment_ratio * 0.4
    return float(score)


def place_assignment_ratio(
    *,
    places: dict,
    semantic_document: dict,
    max_assignment_distance_m: float,
) -> float | None:
    objects = semantic_document.get("objects") or {}
    if not objects:
        return None
    place_polygons = [
        (place_id, tuple((float(x), float(y)) for x, y in place.get("polygon", [])), float(place.get("area_m2", 0.0)))
        for place_id, place in places.items()
        if place.get("enabled", True)
    ]
    if not place_polygons:
        return 0.0

    assigned = 0
    for entry in objects.values():
        candidate_points = list(object_place_candidate_points(entry))
        matched = False
        for x, y in candidate_points:
            for _place_id, polygon, _area in place_polygons:
                if point_in_polygon(x, y, polygon):
                    matched = True
                    break
            if matched:
                break
        if not matched:
            nearest = nearest_place(candidate_points, place_polygons)
            matched = nearest is not None and nearest[0] <= max_assignment_distance_m
        if matched:
            assigned += 1
    return assigned / max(len(objects), 1)


def grow_mask_within_free_space(
    *,
    seed_mask: np.ndarray,
    free_mask: np.ndarray,
    max_steps: int,
) -> np.ndarray:
    grown_mask = np.zeros_like(free_mask, dtype=np.uint8)
    distances = np.full(free_mask.shape, -1, dtype=np.int32)
    queue: deque[tuple[int, int]] = deque()
    seed_rows, seed_cols = np.nonzero((seed_mask == 1) & (free_mask == 1))
    for row, col in zip(seed_rows, seed_cols):
        distances[row, col] = 0
        grown_mask[row, col] = 1
        queue.append((int(row), int(col)))

    neighbor_offsets = ((-1, 0), (1, 0), (0, -1), (0, 1))
    while queue:
        row, col = queue.popleft()
        if distances[row, col] >= max_steps:
            continue
        for row_offset, col_offset in neighbor_offsets:
            next_row = row + row_offset
            next_col = col + col_offset
            if (
                next_row < 0
                or next_col < 0
                or next_row >= free_mask.shape[0]
                or next_col >= free_mask.shape[1]
            ):
                continue
            if free_mask[next_row, next_col] != 1 or distances[next_row, next_col] >= 0:
                continue
            distances[next_row, next_col] = distances[row, col] + 1
            grown_mask[next_row, next_col] = 1
            queue.append((next_row, next_col))
    return grown_mask


def classify_room_or_corridor(
    *,
    mask: np.ndarray,
    resolution: float,
    corridor_aspect_ratio: float,
) -> str:
    _left, _top, width, height, area_cells = mask_stats(mask)
    width_m = float(width) * resolution
    height_m = float(height) * resolution
    area_m2 = float(area_cells) * resolution * resolution
    aspect_ratio = max(width_m, height_m) / max(min(width_m, height_m), 1e-6)
    mean_width_m = area_m2 / max(max(width_m, height_m), 1e-6)
    if aspect_ratio >= corridor_aspect_ratio or (area_m2 >= 8.0 and mean_width_m <= 3.2):
        return "corridor"
    return "room"


def place_entry_from_mask(
    *,
    mask: np.ndarray,
    label: str,
    free_mask: np.ndarray,
    resolution: float,
    origin: tuple[float, float],
    min_region_area_m2: float,
    simplify_tolerance_m: float,
    confidence: float,
    source: str,
) -> dict | None:
    region_area_m2 = float(np.count_nonzero(mask)) * resolution * resolution
    if region_area_m2 < min_region_area_m2:
        return None
    contour = largest_contour(mask.astype(np.uint8) * 255)
    if contour is None:
        return None
    polygon = contour_to_polygon(contour, free_mask.shape[0], resolution, origin, simplify_tolerance_m)
    if len(polygon) < 3:
        polygon = stats_to_polygon(mask_stats(mask), free_mask.shape[0], resolution, origin)
    area_m2 = polygon_area(polygon)
    if area_m2 < min_region_area_m2:
        return None
    centroid_x, centroid_y = polygon_centroid(polygon)
    return {
        "label": label,
        "aliases": [],
        "frame_id": "map",
        "polygon": [[round(x, 4), round(y, 4)] for x, y in polygon],
        "centroid_x": round(float(centroid_x), 4),
        "centroid_y": round(float(centroid_y), 4),
        "area_m2": round(float(area_m2), 4),
        "confidence": round(float(confidence), 4),
        "source": source,
        "enabled": True,
    }


def build_region_masks(
    *,
    free_mask: np.ndarray,
    resolution: float,
    min_region_area_m2: float,
    seed_inflate_cells: int = 0,
    max_region_seeds: int = 0,
    seed_spacing_m: float = 3.5,
    min_seed_obstacle_distance_m: float = 0.25,
) -> list[np.ndarray]:
    labels_count, labels, stats, _centroids = cv2.connectedComponentsWithStats(free_mask, connectivity=8)
    region_masks: list[np.ndarray] = []
    for component_label in range(1, labels_count):
        component_area_m2 = float(stats[component_label, cv2.CC_STAT_AREA]) * resolution * resolution
        if component_area_m2 < min_region_area_m2:
            continue
        component_mask = (labels == component_label).astype(np.uint8)
        if seed_inflate_cells > 0:
            if max_region_seeds > 0:
                region_masks.extend(
                    split_component_by_peak_seed_growth(
                        component_mask=component_mask,
                        max_region_seeds=max_region_seeds,
                        seed_spacing_m=seed_spacing_m,
                        min_seed_obstacle_distance_m=min_seed_obstacle_distance_m,
                        resolution=resolution,
                        min_region_area_m2=min_region_area_m2,
                    )
                )
            else:
                region_masks.extend(
                    split_component_by_eroded_seed_growth(
                        component_mask=component_mask,
                        seed_inflate_cells=seed_inflate_cells,
                        resolution=resolution,
                        min_region_area_m2=min_region_area_m2,
                    )
                )
        elif max_region_seeds > 0:
            region_masks.extend(
                split_component_by_peak_seed_growth(
                    component_mask=component_mask,
                    max_region_seeds=max_region_seeds,
                    seed_spacing_m=seed_spacing_m,
                    min_seed_obstacle_distance_m=min_seed_obstacle_distance_m,
                    resolution=resolution,
                    min_region_area_m2=min_region_area_m2,
                )
            )
        else:
            region_masks.extend(
                split_component_by_distance_seeds(
                    component_mask=component_mask,
                    resolution=resolution,
                    min_region_area_m2=min_region_area_m2,
                )
            )
    return region_masks


def split_component_by_peak_seed_growth(
    *,
    component_mask: np.ndarray,
    max_region_seeds: int,
    seed_spacing_m: float,
    min_seed_obstacle_distance_m: float,
    resolution: float,
    min_region_area_m2: float,
) -> list[np.ndarray]:
    seeds = find_peak_growth_seeds(
        component_mask=component_mask,
        max_region_seeds=max_region_seeds,
        seed_spacing_m=seed_spacing_m,
        min_seed_obstacle_distance_m=min_seed_obstacle_distance_m,
        resolution=resolution,
    )
    if len(seeds) < 2:
        return [component_mask]
    return grow_component_from_seed_points(
        component_mask=component_mask,
        seeds=seeds,
        resolution=resolution,
        min_region_area_m2=min_region_area_m2,
    )


def find_peak_growth_seeds(
    *,
    component_mask: np.ndarray,
    max_region_seeds: int,
    seed_spacing_m: float,
    min_seed_obstacle_distance_m: float,
    resolution: float,
) -> list[tuple[int, int, float]]:
    distance = cv2.distanceTransform(component_mask.astype(np.uint8), cv2.DIST_L2, 5)
    if float(distance.max()) * resolution < min_seed_obstacle_distance_m:
        return []

    kernel = np.ones((7, 7), dtype=np.uint8)
    local_maximum = distance >= cv2.dilate(distance, kernel) - 1e-6
    peak_mask = (
        (component_mask == 1)
        & local_maximum
        & (distance * resolution >= min_seed_obstacle_distance_m)
    ).astype(np.uint8)
    labels_count, labels, _stats, _centroids = cv2.connectedComponentsWithStats(peak_mask, connectivity=8)

    raw_seeds: list[tuple[int, int, float]] = []
    for label in range(1, labels_count):
        coordinates = np.argwhere(labels == label)
        if len(coordinates) == 0:
            continue
        best_row, best_col = max(
            ((int(row), int(col)) for row, col in coordinates),
            key=lambda point: (
                float(distance[point[0], point[1]]),
                -point[0],
                -point[1],
            ),
        )
        raw_seeds.append((best_row, best_col, float(distance[best_row, best_col])))

    raw_seeds.sort(key=lambda seed: (-seed[2], seed[0], seed[1]))
    min_seed_spacing_cells = max(seed_spacing_m / max(resolution, 1e-9), 1.0)
    seeds: list[tuple[int, int, float]] = []
    for row, col, score in raw_seeds:
        has_enough_spacing = all(
            (row - kept_row) ** 2 + (col - kept_col) ** 2 >= min_seed_spacing_cells ** 2
            for kept_row, kept_col, _score in seeds
        )
        if has_enough_spacing:
            seeds.append((row, col, score))
            if len(seeds) >= max_region_seeds:
                break
    return sorted(seeds, key=lambda seed: (seed[0], seed[1]))


def grow_component_from_seed_points(
    *,
    component_mask: np.ndarray,
    seeds: list[tuple[int, int, float]],
    resolution: float,
    min_region_area_m2: float,
) -> list[np.ndarray]:
    grown_labels = np.full(component_mask.shape, -1, dtype=np.int32)
    queue: deque[tuple[int, int, int]] = deque()
    for seed_index, (row, col, _score) in enumerate(seeds):
        if component_mask[row, col] != 1:
            continue
        grown_labels[row, col] = seed_index
        queue.append((int(row), int(col), seed_index))

    neighbor_offsets = ((-1, 0), (1, 0), (0, -1), (0, 1))
    while queue:
        row, col, label = queue.popleft()
        for row_offset, col_offset in neighbor_offsets:
            next_row = row + row_offset
            next_col = col + col_offset
            if (
                next_row < 0
                or next_col < 0
                or next_row >= component_mask.shape[0]
                or next_col >= component_mask.shape[1]
            ):
                continue
            if component_mask[next_row, next_col] != 1 or grown_labels[next_row, next_col] >= 0:
                continue
            grown_labels[next_row, next_col] = label
            queue.append((next_row, next_col, label))

    min_region_cells = max(1, math.ceil(min_region_area_m2 / max(resolution * resolution, 1e-12)))
    region_masks = []
    for label in range(len(seeds)):
        region_masks.extend(connected_subregion_masks((grown_labels == label).astype(np.uint8)))

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


def split_component_by_eroded_seed_growth(
    *,
    component_mask: np.ndarray,
    seed_inflate_cells: int,
    resolution: float,
    min_region_area_m2: float,
) -> list[np.ndarray]:
    seed_mask = remove_inflated_obstacle_neighbors(component_mask, seed_inflate_cells)
    min_region_cells = max(1, math.ceil(min_region_area_m2 / max(resolution * resolution, 1e-12)))
    labels_count, labels, stats, _centroids = cv2.connectedComponentsWithStats(seed_mask, connectivity=8)

    grown_labels = np.full(component_mask.shape, -1, dtype=np.int32)
    queue: deque[tuple[int, int, int]] = deque()
    seed_index = 0
    for label in range(1, labels_count):
        if int(stats[label, cv2.CC_STAT_AREA]) < min_region_cells:
            continue
        rows, cols = np.nonzero(labels == label)
        for row, col in zip(rows, cols):
            if component_mask[row, col] == 1 and grown_labels[row, col] < 0:
                grown_labels[row, col] = seed_index
                queue.append((int(row), int(col), seed_index))
        seed_index += 1

    if seed_index < 2:
        return [component_mask]

    neighbor_offsets = ((-1, 0), (1, 0), (0, -1), (0, 1))
    while queue:
        row, col, label = queue.popleft()
        for row_offset, col_offset in neighbor_offsets:
            next_row = row + row_offset
            next_col = col + col_offset
            if (
                next_row < 0
                or next_col < 0
                or next_row >= component_mask.shape[0]
                or next_col >= component_mask.shape[1]
            ):
                continue
            if component_mask[next_row, next_col] != 1 or grown_labels[next_row, next_col] >= 0:
                continue
            grown_labels[next_row, next_col] = label
            queue.append((next_row, next_col, label))

    region_masks = []
    for label in range(seed_index):
        region_masks.extend(connected_subregion_masks((grown_labels == label).astype(np.uint8)))

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
