"""
Deterministic view-data extracted from a real PipelineState.

No synthetic detections. Traversability is a documented mapping of
AdaptiveCell.semantic_class (GROUND / MIXED / OBSTACLE) produced by
RANSAC ground vs obstacle point counts — not a learned model.
"""

from typing import Any, Dict, List, Sequence, Tuple

import numpy as np

from visualization.color_schemes import (
    PROPOSAL_COLORS,
    TRAV_LABELS,
    proposal_color,
)


MAX_LIDAR_POINTS = 25000
MAX_BEV_POINTS = 20000
MAX_TRACK_LABELS = 8
MAX_PROPOSAL_LABELS = 5


def subsample_indices(n: int, max_points: int, seed: int = 0) -> np.ndarray:
    if n <= max_points:
        return np.arange(n, dtype=np.int64)
    rng = np.random.default_rng(seed)
    return np.sort(rng.choice(n, size=max_points, replace=False))


def elevation_values(points: np.ndarray) -> np.ndarray:
    points = np.asarray(points)
    if len(points) == 0:
        return np.zeros(0, dtype=np.float64)
    return points[:, 2].astype(np.float64)


def terrain_point_labels(ground_mask: np.ndarray) -> np.ndarray:
    """0 = RANSAC ground, 1 = non-ground. Aligned with filtered XYZ."""
    mask = np.asarray(ground_mask, dtype=bool)
    labels = np.ones(len(mask), dtype=np.int8)
    labels[mask] = 0
    return labels


def cell_records(grid) -> List[Dict[str, Any]]:
    records = []
    for cell in grid.cells.values():
        x, y = cell.center
        ground_z = cell.ground_elevation
        records.append(
            {
                "x": float(x),
                "y": float(y),
                "resolution": float(cell.resolution),
                "semantic_class": str(cell.semantic_class),
                "ground_elevation": (
                    None if ground_z is None else float(ground_z)
                ),
                "z_mean": float(cell.z_mean),
                "point_count": int(cell.point_count),
            }
        )
    return records


def traversability_code(semantic_class: str) -> int:
    """
    Map geometric cell class to a prototype risk code.

    0 GROUND   — only ground points in the cell
    1 MIXED    — ground and obstacle points share the cell
    2 OBSTACLE — only obstacle points in the cell

    This is not a learned traversability or drivability model.
    """

    name = str(semantic_class)
    if name == "GROUND":
        return 0
    if name == "MIXED":
        return 1
    return 2


def elevation_layer(records: Sequence[Dict[str, Any]]) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    xs, ys, zs = [], [], []
    for rec in records:
        z = rec["ground_elevation"]
        if z is None or not np.isfinite(z):
            continue
        xs.append(rec["x"])
        ys.append(rec["y"])
        zs.append(z)
    return (
        np.asarray(xs, dtype=np.float64),
        np.asarray(ys, dtype=np.float64),
        np.asarray(zs, dtype=np.float64),
    )


def traversability_layer(
    records: Sequence[Dict[str, Any]],
) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    xs, ys, codes, sizes = [], [], [], []
    for rec in records:
        xs.append(rec["x"])
        ys.append(rec["y"])
        codes.append(traversability_code(rec["semantic_class"]))
        sizes.append(max(rec["resolution"] * 80.0, 2.0))
    return (
        np.asarray(xs, dtype=np.float64),
        np.asarray(ys, dtype=np.float64),
        np.asarray(codes, dtype=np.int8),
        np.asarray(sizes, dtype=np.float64),
    )


def select_track_labels(tracks, max_labels: int = MAX_TRACK_LABELS):
    tracks = list(tracks)
    if not tracks:
        return []

    def distance(track):
        pos = np.asarray(track.position, dtype=np.float64)
        return float(np.linalg.norm(pos[:2]))

    confirmed = [t for t in tracks if t.confirmed]
    pool = confirmed if confirmed else tracks
    pool = sorted(pool, key=distance)
    return pool[:max_labels]


def select_proposal_labels(proposals, max_labels: int = MAX_PROPOSAL_LABELS):
    ranked = sorted(
        list(proposals),
        key=lambda p: (-float(p.confidence), float(p.distance)),
    )
    return ranked[:max_labels]


def track_trails(world_objects) -> List[np.ndarray]:
    """XY polylines from WorldObject.history (tracker/world-model only)."""

    trails = []
    for obj in world_objects:
        history = getattr(obj, "history", None)
        if not history or len(history) < 2:
            continue
        pts = np.asarray(history, dtype=np.float64)
        if pts.ndim != 2 or pts.shape[1] < 2:
            continue
        trails.append(pts[:, :2].copy())
    return trails


def proposal_style(classification: str) -> str:
    return proposal_color(classification)


def class_legend_items() -> List[Tuple[str, str]]:
    return list(PROPOSAL_COLORS.items())


def traversability_legend_items() -> List[Tuple[int, str]]:
    return list(TRAV_LABELS.items())
