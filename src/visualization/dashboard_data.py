"""
Deterministic view-data extracted from real PipelineState objects.

No synthetic detections, trajectories, or metrics.
Cell colour codes map AdaptiveCell.semantic_class (GROUND / MIXED /
OBSTACLE) from RANSAC ground vs obstacle counts — not a learned model.

Ego trajectory is taken from PipelineState.ego_xy / ego_heading_rad
written by OrbitSystem from EgoMotionCompensator (KITTI poses.txt +
calib Tr, LiDAR → LiDAR frame 0). It is never interpolated or faked.
"""

from typing import Any, Dict, List, Optional, Sequence, Tuple

import numpy as np

from visualization.color_schemes import (
    PROPOSAL_COLORS,
    TRAV_LABELS,
    proposal_color,
)


MAX_LIDAR_POINTS = 28000
MAX_BEV_POINTS = 22000
MAX_WORLD_POINTS = 48000
MAX_CELL_PATCHES = 5500
MAX_TRACK_LABELS = 8
MAX_TRACK_LIST = 8
MAX_PROPOSAL_LABELS = 4


def subsample_indices(n: int, max_points: int, seed: int = 0) -> np.ndarray:
    if n <= max_points:
        return np.arange(n, dtype=np.int64)
    rng = np.random.default_rng(seed)
    return np.sort(rng.choice(n, size=max_points, replace=False))


def subsample_xyz(points: np.ndarray, max_points: int, seed: int) -> np.ndarray:
    points = np.asarray(points)
    if len(points) == 0:
        return np.zeros((0, 3), dtype=np.float64)
    idx = subsample_indices(len(points), max_points, seed=seed)
    return np.asarray(points[idx][:, :3], dtype=np.float64)


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


def subsample_records(records: Sequence[Dict[str, Any]], max_n: int, seed: int) -> List[Dict[str, Any]]:
    records = list(records)
    if len(records) <= max_n:
        return records
    idx = subsample_indices(len(records), max_n, seed=seed)
    return [records[int(i)] for i in idx]


def traversability_code(semantic_class: str) -> int:
    """
    Map geometric cell class to a display code.

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


def elevation_patches(
    records: Sequence[Dict[str, Any]],
) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """Cell centres, resolutions, and ground Z for square patches."""

    xs, ys, res, zs = [], [], [], []
    for rec in records:
        z = rec["ground_elevation"]
        if z is None or not np.isfinite(z):
            continue
        xs.append(rec["x"])
        ys.append(rec["y"])
        res.append(rec["resolution"])
        zs.append(z)
    return (
        np.asarray(xs, dtype=np.float64),
        np.asarray(ys, dtype=np.float64),
        np.asarray(res, dtype=np.float64),
        np.asarray(zs, dtype=np.float64),
    )


def class_patches(
    records: Sequence[Dict[str, Any]],
) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    xs, ys, res, codes = [], [], [], []
    for rec in records:
        xs.append(rec["x"])
        ys.append(rec["y"])
        res.append(rec["resolution"])
        codes.append(traversability_code(rec["semantic_class"]))
    return (
        np.asarray(xs, dtype=np.float64),
        np.asarray(ys, dtype=np.float64),
        np.asarray(res, dtype=np.float64),
        np.asarray(codes, dtype=np.int8),
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


def _track_distance(track) -> float:
    pos = np.asarray(track.position, dtype=np.float64)
    return float(np.linalg.norm(pos[:2]))


def select_track_labels(
    tracks,
    max_labels: int = MAX_TRACK_LABELS,
    confirmed_only: bool = False,
):
    """
    Prefer confirmed, long-lived, nearby tracks.
    Never returns more than max_labels items.
    """

    tracks = list(tracks)
    if not tracks:
        return []

    confirmed = [t for t in tracks if t.confirmed]
    if confirmed_only:
        pool = confirmed
    else:
        pool = confirmed if confirmed else tracks

    pool = sorted(
        pool,
        key=lambda t: (-int(getattr(t, "hits", 0)), _track_distance(t)),
    )
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


def extract_trajectory(states: Sequence, up_to: Optional[int] = None) -> np.ndarray:
    """
    Nx2 ego XY in LiDAR frame 0, one row per processed state.

    Values come from PipelineState.ego_xy (OrbitSystem / KITTI poses).
    """

    if up_to is None:
        up_to = len(states) - 1
    up_to = max(-1, min(up_to, len(states) - 1))
    rows = []
    for state in states[: up_to + 1]:
        xy = getattr(state, "ego_xy", (0.0, 0.0))
        rows.append([float(xy[0]), float(xy[1])])
    if not rows:
        return np.zeros((0, 2), dtype=np.float64)
    return np.asarray(rows, dtype=np.float64)


def extract_headings(states: Sequence, up_to: Optional[int] = None) -> np.ndarray:
    if up_to is None:
        up_to = len(states) - 1
    up_to = max(-1, min(up_to, len(states) - 1))
    values = [
        float(getattr(state, "ego_heading_rad", 0.0))
        for state in states[: up_to + 1]
    ]
    return np.asarray(values, dtype=np.float64)


def pose_source_of(states: Sequence) -> str:
    if not states:
        return "none"
    return str(getattr(states[-1], "pose_source", "unknown"))


def pose_caption(pose_source: str) -> str:
    """Short header label. ORBIT world is LiDAR frame 0, never nuScenes global."""

    src = str(pose_source)
    if src.startswith("nuscenes") or src.startswith("kitti"):
        return "LIDAR FRAME 0"
    if src.startswith("identity"):
        return "IDENTITY"
    return "LIDAR FRAME 0"


def source_badge(source: str, sequence: str) -> str:
    src = str(source).lower()
    seq = str(sequence)
    if src == "nuscenes":
        return f"NUSCENES  ·  {seq.upper()}  ·  LIDAR_TOP"
    if src == "kitti":
        return f"KITTI  ·  SEQ {seq}  ·  VELODYNE"
    return f"SYNTHETIC  ·  {seq}"


def compact_count(n) -> str:
    n = int(n)
    if n >= 10000:
        return f"{n / 1000.0:.1f}K"
    if n >= 1000:
        return f"{n / 1000.0:.1f}K"
    return str(n)


def travel_metres(states: Sequence, up_to: Optional[int] = None) -> float:
    """Path length of real ego_xy samples. Not a hardcoded scene length."""

    traj = extract_trajectory(states, up_to=up_to)
    if len(traj) < 2:
        if len(traj) == 1:
            return float(np.hypot(traj[0, 0], traj[0, 1]))
        return 0.0
    delta = np.diff(traj, axis=0)
    return float(np.sum(np.sqrt(np.sum(delta * delta, axis=1))))


def track_class_counts(tracks) -> List[Tuple[str, int]]:
    counts = {}
    for track in tracks:
        name = str(getattr(track, "class_name", "OBSTACLE"))
        counts[name] = counts.get(name, 0) + 1
    order = ["VEHICLE-LIKE", "POLE", "WALL", "OBSTACLE"]
    items = [(k, counts[k]) for k in order if k in counts]
    extra = sorted(
        ((k, v) for k, v in counts.items() if k not in order),
        key=lambda kv: -kv[1],
    )
    return items + extra


def xy_bounds(xs, ys, pad_frac: float = 0.12, min_span: float = 10.0):
    xs = np.asarray(xs, dtype=np.float64).ravel()
    ys = np.asarray(ys, dtype=np.float64).ravel()
    if len(xs) == 0 or len(ys) == 0:
        return -min_span, min_span, -min_span, min_span
    x0, x1 = np.percentile(xs, 2), np.percentile(xs, 98)
    y0, y1 = np.percentile(ys, 2), np.percentile(ys, 98)
    if not np.isfinite(x0):
        x0, x1 = float(np.min(xs)), float(np.max(xs))
        y0, y1 = float(np.min(ys)), float(np.max(ys))
    span_x = max(float(x1 - x0), min_span)
    span_y = max(float(y1 - y0), min_span)
    span = max(span_x, span_y)
    pad = span * pad_frac
    cx = 0.5 * (x0 + x1)
    cy = 0.5 * (y0 + y1)
    half = 0.5 * span + pad
    return cx - half, cx + half, cy - half, cy + half


def accumulated_world_cloud(states: Sequence, index: int, max_points: int = MAX_WORLD_POINTS) -> np.ndarray:
    """
    Concatenate per-frame world-frame LiDAR subsamples up to `index`.

    Each state's world_points were already transformed into LiDAR frame 0
    by OrbitSystem. Uniform stride downsample if the stack is too large
    (deterministic; no per-slider reshuffle of earlier frames).
    """

    chunks = []
    for state in states[: index + 1]:
        cloud = getattr(state, "world_points", None)
        if cloud is None:
            continue
        cloud = np.asarray(cloud)
        if cloud.ndim != 2 or cloud.shape[1] < 2 or len(cloud) == 0:
            continue
        chunks.append(cloud[:, :3] if cloud.shape[1] >= 3 else cloud)
    if not chunks:
        return np.zeros((0, 3), dtype=np.float64)
    stacked = np.vstack(chunks)
    if len(stacked) <= max_points:
        return stacked
    stride = int(np.ceil(len(stacked) / float(max_points)))
    return stacked[::stride]


def world_to_ego_xy(xy_world: np.ndarray, T_ego_to_world: np.ndarray) -> np.ndarray:
    """Map world (frame 0) XY onto the current LiDAR XY plane."""

    T = np.asarray(T_ego_to_world, dtype=np.float64)
    if T.shape != (4, 4):
        return np.asarray(xy_world, dtype=np.float64)
    T_inv = np.linalg.inv(T)
    p = np.asarray(xy_world, dtype=np.float64).reshape(-1)
    hom = np.array([p[0], p[1], 0.0, 1.0], dtype=np.float64)
    out = T_inv @ hom
    return out[:2]


def proposal_style(classification: str) -> str:
    return proposal_color(classification)


def class_legend_items() -> List[Tuple[str, str]]:
    return list(PROPOSAL_COLORS.items())


def traversability_legend_items() -> List[Tuple[int, str]]:
    return list(TRAV_LABELS.items())
