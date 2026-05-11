#!/usr/bin/env python3
from __future__ import annotations

import argparse
import hashlib
import math
import re
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Iterable

import numpy as np
import yaml
from PIL import Image


COCO_NAMES = {
    0: "person",
    1: "bicycle",
    2: "car",
    3: "motorcycle",
    4: "airplane",
    5: "bus",
    6: "train",
    7: "truck",
    8: "boat",
    9: "traffic light",
    10: "fire hydrant",
    11: "stop sign",
    12: "parking meter",
    13: "bench",
    14: "bird",
    15: "cat",
    16: "dog",
    17: "horse",
    18: "sheep",
    19: "cow",
    20: "elephant",
    21: "bear",
    22: "zebra",
    23: "giraffe",
    24: "backpack",
    25: "umbrella",
    26: "handbag",
    27: "tie",
    28: "suitcase",
    29: "frisbee",
    30: "skis",
    31: "snowboard",
    32: "sports ball",
    33: "kite",
    34: "baseball bat",
    35: "baseball glove",
    36: "skateboard",
    37: "surfboard",
    38: "tennis racket",
    39: "bottle",
    40: "wine glass",
    41: "cup",
    42: "fork",
    43: "knife",
    44: "spoon",
    45: "bowl",
    46: "banana",
    47: "apple",
    48: "sandwich",
    49: "orange",
    50: "broccoli",
    51: "carrot",
    52: "hot dog",
    53: "pizza",
    54: "donut",
    55: "cake",
    56: "chair",
    57: "couch",
    58: "potted plant",
    59: "bed",
    60: "dining table",
    61: "toilet",
    62: "tv",
    63: "laptop",
    64: "mouse",
    65: "remote",
    66: "keyboard",
    67: "cell phone",
    68: "microwave",
    69: "oven",
    70: "toaster",
    71: "sink",
    72: "refrigerator",
    73: "book",
    74: "clock",
    75: "vase",
    76: "scissors",
    77: "teddy bear",
    78: "hair drier",
    79: "toothbrush",
}

DEFAULT_INCLUDE_LABELS = {
    "bench",
    "chair",
    "couch",
    "dining table",
    "laptop",
    "potted plant",
    "refrigerator",
    "tv",
    "vase",
}

ALIASES = {
    "bench": ["벤치"],
    "chair": ["의자"],
    "couch": ["소파", "sofa"],
    "dining table": ["테이블", "책상", "table", "desk"],
    "laptop": ["노트북"],
    "potted plant": ["화분", "식물", "plant"],
    "refrigerator": ["냉장고"],
    "tv": ["티비", "텔레비전", "모니터", "monitor"],
    "vase": ["꽃병"],
}


@dataclass(frozen=True)
class CameraCalibration:
    width: int
    height: int
    fx: float
    fy: float
    cx: float
    cy: float


@dataclass(frozen=True)
class CameraPose:
    node_id: int
    stamp: float
    translation: np.ndarray
    quaternion_xyzw: np.ndarray


@dataclass(frozen=True)
class Observation:
    node_id: int
    label: str
    confidence: float
    point_map: np.ndarray
    observer_xy: tuple[float, float]


@dataclass(frozen=True)
class Cluster:
    label: str
    observations: tuple[Observation, ...]

    @property
    def confidence(self) -> float:
        return max(obs.confidence for obs in self.observations)

    @property
    def centroid(self) -> np.ndarray:
        weights = np.array([max(obs.confidence, 1e-6) for obs in self.observations], dtype=float)
        points = np.array([obs.point_map for obs in self.observations], dtype=float)
        return np.average(points, axis=0, weights=weights)

    @property
    def observer_xy(self) -> tuple[float, float]:
        weights = np.array([max(obs.confidence, 1e-6) for obs in self.observations], dtype=float)
        observers = np.array([obs.observer_xy for obs in self.observations], dtype=float)
        x, y = np.average(observers, axis=0, weights=weights)
        return float(x), float(y)

    @property
    def radius_m(self) -> float:
        centroid = self.centroid
        distances = [
            float(np.linalg.norm(obs.point_map[:2] - centroid[:2]))
            for obs in self.observations
        ]
        return min(max((max(distances) if distances else 0.0) + 0.25, 0.3), 1.5)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Build a semantic object YAML map from RTAB-Map exports and Ultralytics YOLO labels."
    )
    parser.add_argument("--export-dir", required=True, type=Path, help="Directory created by rtabmap-export.")
    parser.add_argument("--labels-dir", required=True, type=Path, help="Ultralytics labels directory.")
    parser.add_argument("--output", required=True, type=Path, help="Output semantic_objects YAML path.")
    parser.add_argument("--source-db", type=Path, default=Path(""), help="Source RTAB-Map database path.")
    parser.add_argument("--map-id", default="", help="Manifest map_id. Defaults to source DB stem.")
    parser.add_argument("--source-fingerprint", default="", help="Manifest source fingerprint. Defaults to sha256 of source DB.")
    parser.add_argument("--detector-name", default="ultralytics-yolo")
    parser.add_argument("--detector-model", default="models/yolo11n.pt")
    parser.add_argument(
        "--include-label",
        action="append",
        default=[],
        help="COCO label to include. Repeatable. Defaults to office-relevant labels.",
    )
    parser.add_argument("--min-confidence", type=float, default=0.25)
    parser.add_argument("--min-observations", type=int, default=1)
    parser.add_argument("--merge-distance-m", type=float, default=0.75)
    parser.add_argument("--depth-scale", type=float, default=0.001, help="Scale raw depth units to meters.")
    parser.add_argument("--min-depth-m", type=float, default=0.2)
    parser.add_argument("--max-depth-m", type=float, default=10.0)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    include_labels = {label.strip().lower() for label in args.include_label if label.strip()}
    if not include_labels:
        include_labels = DEFAULT_INCLUDE_LABELS

    poses = load_camera_poses(find_single(args.export_dir, "*camera_poses.txt"))
    observations = build_observations(
        export_dir=args.export_dir,
        labels_dir=args.labels_dir,
        poses=poses,
        include_labels=include_labels,
        min_confidence=args.min_confidence,
        depth_scale=args.depth_scale,
        min_depth_m=args.min_depth_m,
        max_depth_m=args.max_depth_m,
    )
    clusters = cluster_observations(observations, args.merge_distance_m, args.min_observations)
    document = build_yaml_document(args, clusters)

    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(yaml.safe_dump(document, allow_unicode=True, sort_keys=False), encoding="utf-8")
    print(f"observations: {len(observations)}")
    print(f"objects: {sum(len(items) for items in clusters.values())}")
    print(f"output: {args.output}")
    return 0


def find_single(root: Path, pattern: str) -> Path:
    matches = sorted(root.glob(pattern))
    if len(matches) != 1:
        raise FileNotFoundError(f"expected one {pattern} under {root}, found {len(matches)}")
    return matches[0]


def load_camera_poses(path: Path) -> dict[int, CameraPose]:
    poses: dict[int, CameraPose] = {}
    for line in path.read_text(encoding="utf-8").splitlines():
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        parts = line.split()
        if len(parts) != 9:
            continue
        stamp, x, y, z, qx, qy, qz, qw, node_id = parts
        node = int(node_id)
        poses[node] = CameraPose(
            node_id=node,
            stamp=float(stamp),
            translation=np.array([float(x), float(y), float(z)], dtype=float),
            quaternion_xyzw=np.array([float(qx), float(qy), float(qz), float(qw)], dtype=float),
        )
    if not poses:
        raise ValueError(f"no camera poses loaded from {path}")
    return poses


def build_observations(
    *,
    export_dir: Path,
    labels_dir: Path,
    poses: dict[int, CameraPose],
    include_labels: set[str],
    min_confidence: float,
    depth_scale: float,
    min_depth_m: float,
    max_depth_m: float,
) -> list[Observation]:
    rgb_dir = find_single_dir(export_dir, "*_rgb")
    depth_dir = find_single_dir(export_dir, "*_depth")
    calib_dir = find_single_dir(export_dir, "*_calib")
    observations: list[Observation] = []

    for label_path in sorted(labels_dir.glob("*.txt"), key=lambda path: int(path.stem)):
        node_id = int(label_path.stem)
        pose = poses.get(node_id)
        if pose is None:
            continue
        rgb_path = rgb_dir / f"{node_id}.jpg"
        depth_path = depth_dir / f"{node_id}.png"
        calib_path = calib_dir / f"{node_id}.yaml"
        if not rgb_path.exists() or not depth_path.exists() or not calib_path.exists():
            continue

        calibration = load_calibration(calib_path)
        with Image.open(depth_path) as depth_image:
            depth = np.asarray(depth_image)

        for raw in label_path.read_text(encoding="utf-8").splitlines():
            parsed = parse_yolo_label(raw)
            if parsed is None:
                continue
            class_id, x_center, y_center, width, height, confidence = parsed
            label = COCO_NAMES.get(class_id, f"class_{class_id}")
            if label.lower() not in include_labels or confidence < min_confidence:
                continue

            depth_m = sample_depth_m(
                depth,
                x_center=x_center,
                y_center=y_center,
                width=width,
                height=height,
                scale=depth_scale,
                min_depth_m=min_depth_m,
                max_depth_m=max_depth_m,
            )
            if depth_m is None:
                continue

            u = x_center * calibration.width
            v = y_center * calibration.height
            point_camera = pixel_to_camera(u, v, depth_m, calibration)
            point_map = transform_point(point_camera, pose)
            observations.append(
                Observation(
                    node_id=node_id,
                    label=label,
                    confidence=confidence,
                    point_map=point_map,
                    observer_xy=(float(pose.translation[0]), float(pose.translation[1])),
                )
            )
    return observations


def find_single_dir(root: Path, pattern: str) -> Path:
    matches = [path for path in sorted(root.glob(pattern)) if path.is_dir()]
    if len(matches) != 1:
        raise FileNotFoundError(f"expected one {pattern} directory under {root}, found {len(matches)}")
    return matches[0]


def load_calibration(path: Path) -> CameraCalibration:
    text = path.read_text(encoding="utf-8")
    width = int(extract_scalar(text, "image_width"))
    height = int(extract_scalar(text, "image_height"))
    matrix = extract_matrix_data(text, "camera_matrix")
    if len(matrix) != 9:
        raise ValueError(f"camera_matrix in {path} has {len(matrix)} values")
    return CameraCalibration(
        width=width,
        height=height,
        fx=float(matrix[0]),
        fy=float(matrix[4]),
        cx=float(matrix[2]),
        cy=float(matrix[5]),
    )


def extract_scalar(text: str, key: str) -> float:
    match = re.search(rf"^{re.escape(key)}:\s*([^\n]+)", text, flags=re.MULTILINE)
    if not match:
        raise ValueError(f"missing {key}")
    return float(match.group(1).strip().strip('"'))


def extract_matrix_data(text: str, key: str) -> list[float]:
    match = re.search(rf"{re.escape(key)}:.*?data:\s*\[(.*?)\]", text, flags=re.DOTALL)
    if not match:
        raise ValueError(f"missing {key}.data")
    return [float(value) for value in re.findall(r"[-+]?\d+(?:\.\d*)?(?:[eE][-+]?\d+)?", match.group(1))]


def parse_yolo_label(line: str) -> tuple[int, float, float, float, float, float] | None:
    parts = line.split()
    if len(parts) < 6:
        return None
    return (
        int(parts[0]),
        float(parts[1]),
        float(parts[2]),
        float(parts[3]),
        float(parts[4]),
        float(parts[5]),
    )


def sample_depth_m(
    depth: np.ndarray,
    *,
    x_center: float,
    y_center: float,
    width: float,
    height: float,
    scale: float,
    min_depth_m: float,
    max_depth_m: float,
) -> float | None:
    image_height, image_width = depth.shape[:2]
    box_w = max(width * image_width, 1.0)
    box_h = max(height * image_height, 1.0)
    cx = x_center * image_width
    cy = y_center * image_height

    # Use the central half of the box to reduce background leakage from large boxes.
    x1 = max(int(math.floor(cx - box_w * 0.25)), 0)
    x2 = min(int(math.ceil(cx + box_w * 0.25)), image_width)
    y1 = max(int(math.floor(cy - box_h * 0.25)), 0)
    y2 = min(int(math.ceil(cy + box_h * 0.25)), image_height)
    crop = depth[y1:y2, x1:x2]
    if crop.size == 0:
        return None

    values = crop.astype(float).reshape(-1) * scale
    valid = values[(values >= min_depth_m) & (values <= max_depth_m) & np.isfinite(values)]
    if valid.size == 0:
        return None
    return float(np.median(valid))


def pixel_to_camera(u: float, v: float, depth_m: float, calibration: CameraCalibration) -> np.ndarray:
    x = (u - calibration.cx) * depth_m / calibration.fx
    y = (v - calibration.cy) * depth_m / calibration.fy
    z = depth_m
    return np.array([x, y, z], dtype=float)


def transform_point(point_camera: np.ndarray, pose: CameraPose) -> np.ndarray:
    return quaternion_to_matrix(pose.quaternion_xyzw) @ point_camera + pose.translation


def quaternion_to_matrix(quaternion_xyzw: np.ndarray) -> np.ndarray:
    x, y, z, w = quaternion_xyzw
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm == 0.0:
        raise ValueError("zero-length quaternion")
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=float,
    )


def cluster_observations(
    observations: Iterable[Observation],
    merge_distance_m: float,
    min_observations: int,
) -> dict[str, list[Cluster]]:
    clusters_by_label: dict[str, list[list[Observation]]] = {}
    for obs in sorted(observations, key=lambda item: (-item.confidence, item.node_id)):
        clusters = clusters_by_label.setdefault(obs.label, [])
        target = None
        for cluster in clusters:
            centroid = np.mean([item.point_map for item in cluster], axis=0)
            if float(np.linalg.norm(obs.point_map[:2] - centroid[:2])) <= merge_distance_m:
                target = cluster
                break
        if target is None:
            clusters.append([obs])
        else:
            target.append(obs)

    result: dict[str, list[Cluster]] = {}
    for label, clusters in clusters_by_label.items():
        kept = [
            Cluster(label=label, observations=tuple(cluster))
            for cluster in clusters
            if len(cluster) >= min_observations
        ]
        kept.sort(key=lambda cluster: (-cluster.confidence, -len(cluster.observations), cluster.centroid[0], cluster.centroid[1]))
        result[label] = kept
    return result


def build_yaml_document(args: argparse.Namespace, clusters_by_label: dict[str, list[Cluster]]) -> dict:
    generated_at = datetime.now(timezone.utc).replace(microsecond=0).isoformat().replace("+00:00", "Z")
    source_db = str(args.source_db) if str(args.source_db) else ""
    map_id = args.map_id or (args.source_db.stem if str(args.source_db) else "semantic_map")
    source_fingerprint = args.source_fingerprint or (fingerprint(args.source_db) if str(args.source_db) else "manual:unknown")

    objects = {}
    for label in sorted(clusters_by_label.keys()):
        for index, cluster in enumerate(clusters_by_label[label], start=1):
            object_id = f"{slug(label)}_{index}"
            centroid = cluster.centroid
            observer_x, observer_y = cluster.observer_xy
            objects[object_id] = {
                "label": label,
                "aliases": list(ALIASES.get(label, [])),
                "frame_id": "map",
                "x": round(float(centroid[0]), 4),
                "y": round(float(centroid[1]), 4),
                "z": round(float(centroid[2]), 4),
                "yaw_deg": None,
                "radius_m": round(cluster.radius_m, 3),
                "confidence": round(cluster.confidence, 4),
                "observation_count": len(cluster.observations),
                "updated_at": generated_at,
                "source": "offline_yolo_builder",
                "observer_x": round(observer_x, 4),
                "observer_y": round(observer_y, 4),
            }

    return {
        "manifest": {
            "map_id": map_id,
            "source_fingerprint": source_fingerprint,
            "frame_id": "map",
            "source_rtabmap_db": source_db,
            "generated_at": generated_at,
            "builder_version": "offline_yolo_builder_v1",
            "detector": {
                "name": args.detector_name,
                "model": args.detector_model,
            },
        },
        "objects": objects,
    }


def slug(label: str) -> str:
    return re.sub(r"[^a-z0-9]+", "_", label.lower()).strip("_") or "object"


def fingerprint(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return f"sha256:{digest.hexdigest()}"


if __name__ == "__main__":
    raise SystemExit(main())
