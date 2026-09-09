"""
Export real ORBIT PipelineState frames to website JSON.

Does not rerun algorithms in a second implementation: each frame is
produced by OrbitSystem.process_frame. Does not import or call the
Matplotlib dashboard.

Usage (repo root):

    PYTHONPATH=src python -m web_export.export_orbit_data \\
        --source nuscenes --dataset-root /path/to/nuscenes \\
        --scene scene-0061 --start 0 --end 19 \\
        --output exported_data
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence

import numpy as np

SRC_DIR = Path(__file__).resolve().parents[1]
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

from orbit_system import (  # noqa: E402
    OrbitSystem,
    PipelineState,
    load_kitti_xyz,
    load_synthetic_points,
    resolve_nuscenes_root,
)
from tracking.orbit_tracker import WORLD_REFERENCE_FRAME  # noqa: E402

SCHEMA_VERSION = 1
DEFAULT_MAX_POINTS = 5000
WORLD_FRAME_NAME = "lidar_frame_0"


def json_ready(value: Any) -> Any:
    """Convert NumPy / Path values to JSON-native types."""

    if value is None:
        return None
    if isinstance(value, (str, bool, int)):
        return value
    if isinstance(value, float):
        return value if math.isfinite(value) else None
    if isinstance(value, Path):
        return str(value)
    if isinstance(value, np.generic):
        item = value.item()
        if isinstance(item, float) and not math.isfinite(item):
            return None
        return item
    if isinstance(value, np.ndarray):
        return [json_ready(v) for v in value.tolist()]
    if isinstance(value, (list, tuple)):
        return [json_ready(v) for v in value]
    if isinstance(value, dict):
        return {str(k): json_ready(v) for k, v in value.items()}
    raise TypeError(f"cannot JSON-encode {type(value)!r}")


def even_indices(n: int, max_points: int) -> np.ndarray:
    """Deterministic downsample: evenly spaced original indices, no RNG."""

    n = int(n)
    max_points = int(max_points)
    if n <= 0:
        return np.zeros(0, dtype=np.int64)
    if n <= max_points:
        return np.arange(n, dtype=np.int64)
    return np.round(np.linspace(0, n - 1, num=max_points)).astype(np.int64)


def apply_T_xyz(xyz: np.ndarray, T: np.ndarray) -> np.ndarray:
    """Apply a 4x4 transform to Nx3 points (same convention as ego_motion)."""

    xyz = np.asarray(xyz, dtype=np.float64)
    if len(xyz) == 0:
        return np.zeros((0, 3), dtype=np.float64)
    T = np.asarray(T, dtype=np.float64)
    ones = np.ones((len(xyz), 1), dtype=np.float64)
    return (np.hstack([xyz[:, :3], ones]) @ T.T)[:, :3]


def _finite(value: Any) -> Optional[float]:
    if value is None:
        return None
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    if not math.isfinite(number):
        return None
    return number


def serialize_cell(cell) -> Dict[str, Any]:
    cx, cy = cell.center
    record: Dict[str, Any] = {
        "ix": int(cell.ix),
        "iy": int(cell.iy),
        "level": int(cell.level),
        "center": [float(cx), float(cy)],
        "resolution": float(cell.resolution),
        "semantic_class": str(cell.semantic_class),
        "ground_elevation": _finite(cell.ground_elevation),
        "obstacle_elevation": _finite(getattr(cell, "obstacle_elevation", None)),
        "z_mean": _finite(cell.z_mean),
        "point_count": int(cell.point_count),
        "ground_count": int(cell.ground_count),
        "obstacle_count": int(cell.obstacle_count),
    }
    z_min = _finite(cell.z_min)
    z_max = _finite(cell.z_max)
    if z_min is not None:
        record["z_min"] = z_min
    if z_max is not None:
        record["z_max"] = z_max
    return record


def serialize_obstacle_cell(cell) -> Dict[str, Any]:
    cx, cy = cell.center
    return {
        "ix": int(cell.ix),
        "iy": int(cell.iy),
        "level": int(cell.level),
        "center": [float(cx), float(cy)],
        "resolution": float(cell.resolution),
        "ground_elevation": _finite(cell.ground_elevation),
        "obstacle_elevation": _finite(cell.obstacle_elevation),
        "obstacle_height": _finite(cell.obstacle_height),
        "obstacle_count": int(cell.obstacle_count),
        "ground_source": str(cell.ground_source),
    }


def serialize_track(track) -> Dict[str, Any]:
    position = np.asarray(track.position, dtype=np.float64).reshape(-1)
    dimensions = np.asarray(track.dimensions, dtype=np.float64).reshape(-1)
    velocity = np.asarray(track.velocity, dtype=np.float64).reshape(-1)
    record: Dict[str, Any] = {
        "track_id": int(track.track_id),
        "class_name": str(track.class_name),
        "position": [float(position[0]), float(position[1])],
        "frame": WORLD_FRAME_NAME,
        "dimensions_xy": [float(dimensions[0]), float(dimensions[1])]
        if dimensions.size >= 2
        else [],
        "velocity_xy": [float(velocity[0]), float(velocity[1])]
        if velocity.size >= 2
        else [],
        "hits": int(track.hits),
        "missed": int(track.missed),
        "age": int(track.age),
        "confirmed": bool(track.confirmed),
        "first_frame": int(track.first_frame),
        "last_frame": int(track.last_frame),
        "motion_state": str(track.motion_state),
        "confidence": float(track.confidence),
    }
    history = getattr(track, "motion_history", None)
    if history:
        record["motion_history_speed"] = [
            float(s) for s in history if math.isfinite(float(s))
        ]
    return record


def serialize_world_object(obj) -> Dict[str, Any]:
    position = np.asarray(obj.position, dtype=np.float64).reshape(-1)
    history = []
    for item in getattr(obj, "history", []) or []:
        xy = np.asarray(item, dtype=np.float64).reshape(-1)
        if xy.size >= 2:
            history.append([float(xy[0]), float(xy[1])])
    record: Dict[str, Any] = {
        "track_id": int(obj.track_id),
        "class_name": str(obj.class_name),
        "position": [float(position[0]), float(position[1])],
        "frame": WORLD_FRAME_NAME,
        "hits": int(obj.hits),
        "missed": int(obj.missed),
        "age": int(obj.age),
        "confirmed": bool(obj.confirmed),
        "first_frame": int(obj.first_frame),
        "last_frame": int(obj.last_frame),
        "motion_state": str(obj.motion_state),
        "confidence": float(obj.confidence),
        "history_xy": history,
    }
    dims = np.asarray(obj.dimensions, dtype=np.float64).reshape(-1)
    if dims.size >= 2:
        record["dimensions_xy"] = [float(dims[0]), float(dims[1])]
    return record


def serialize_proposal(proposal) -> Dict[str, Any]:
    data = proposal.as_dict()
    data["frame"] = "current_lidar"
    return json_ready(data)


def serialize_frame(
    state: PipelineState,
    *,
    max_points: int = DEFAULT_MAX_POINTS,
) -> Dict[str, Any]:
    xyz = np.asarray(state.points, dtype=np.float64)
    if xyz.ndim != 2 or xyz.shape[1] < 3:
        xyz = np.zeros((0, 3), dtype=np.float64)
    else:
        xyz = xyz[:, :3]

    idx = even_indices(len(xyz), max_points)
    points_out = xyz[idx] if len(idx) else np.zeros((0, 3), dtype=np.float64)

    T = state.T_ego_to_world
    if T is None:
        T = np.eye(4, dtype=np.float64)
    world_out = apply_T_xyz(points_out, T)

    metrics_src = dict(state.metrics or {})
    cells = list(getattr(state.grid, "cells", {}).values())
    tracks = list(state.tracks or [])
    proposals = list(state.proposals or [])
    obstacle_cells = list(state.obstacle_cells or [])
    world_objects = list(state.world_objects or [])

    metrics = {
        "latency_ms": metrics_src.get("latency_ms"),
        "input_points": metrics_src.get("input_points"),
        "mapped_points": metrics_src.get("mapped_points", int(len(xyz))),
        "ground_points": metrics_src.get("ground_points"),
        "adaptive_cells": metrics_src.get("adaptive_cells", len(cells)),
        "obstacle_cells": metrics_src.get("obstacle_cells", len(obstacle_cells)),
        "raw_components": metrics_src.get("raw_components"),
        "filtered_components": metrics_src.get("filtered_components"),
        "proposals": metrics_src.get("proposals", len(proposals)),
        "live_tracks": metrics_src.get("live_tracks", len(tracks)),
        "confirmed_tracks": metrics_src.get("confirmed_tracks"),
        "world_objects": metrics_src.get("world_objects", len(world_objects)),
        "retired_ids": metrics_src.get("retired_ids", list(state.retired_ids or [])),
        "world_frame": metrics_src.get("world_frame", WORLD_FRAME_NAME),
        "pose_source": state.pose_source,
        "ego_x": state.ego_xy[0],
        "ego_y": state.ego_xy[1],
        "ego_heading_rad": state.ego_heading_rad,
        "point_count_full": int(len(xyz)),
        "point_count_exported": int(len(points_out)),
        "adaptive_cells_exported": int(len(cells)),
        "world_points_exported": int(len(world_out)),
    }

    ground = state.ground
    ground_info = {
        "method": str(ground.method),
        "inlier_count": int(ground.inlier_count),
        "plane_model": json_ready(ground.plane_model)
        if ground.plane_model is not None
        else None,
    }

    return json_ready(
        {
            "schema_version": SCHEMA_VERSION,
            "frame_index": int(state.frame_index),
            "points_frame": "current_lidar",
            "world_points_frame": WORLD_FRAME_NAME,
            "pose": {
                "ego_xy": [float(state.ego_xy[0]), float(state.ego_xy[1])],
                "heading_rad": float(state.ego_heading_rad),
                "pose_source": str(state.pose_source),
                "T_ego_to_world": np.asarray(T, dtype=np.float64).tolist(),
                "world_frame": WORLD_FRAME_NAME,
                "world_reference_frame_index": int(WORLD_REFERENCE_FRAME),
            },
            "metrics": metrics,
            "ground": ground_info,
            "points": points_out.tolist(),
            "world_points": world_out.tolist(),
            "tracks": [serialize_track(t) for t in tracks],
            "adaptive_cells": [serialize_cell(c) for c in cells],
            "obstacle_cells": [serialize_obstacle_cell(c) for c in obstacle_cells],
            "proposals": [serialize_proposal(p) for p in proposals],
            "world_objects": [serialize_world_object(o) for o in world_objects],
            "retired_ids": list(state.retired_ids or []),
        }
    )


def scene_slug(source: str, scene_id: str) -> str:
    raw = scene_id.strip() or source
    safe = "".join(ch if ch.isalnum() or ch in "-_" else "-" for ch in raw)
    return safe.strip("-") or source


def write_export(
    output_root: Path,
    states: Sequence[PipelineState],
    *,
    source: str,
    scene_id: str,
    extra_manifest: Optional[Dict[str, Any]] = None,
    max_points: int = DEFAULT_MAX_POINTS,
) -> Path:
    if not states:
        raise ValueError("no PipelineState frames to export")

    scene_dir = output_root / scene_slug(source, scene_id)
    scene_dir.mkdir(parents=True, exist_ok=True)

    frame_files: List[str] = []
    trajectory: List[Dict[str, Any]] = []

    for state in states:
        payload = serialize_frame(state, max_points=max_points)
        name = f"frame_{int(state.frame_index):04d}.json"
        path = scene_dir / name
        path.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
        frame_files.append(name)
        trajectory.append(
            {
                "frame_index": int(state.frame_index),
                "ego_xy": [float(state.ego_xy[0]), float(state.ego_xy[1])],
                "heading_rad": float(state.ego_heading_rad),
                "pose_source": str(state.pose_source),
                "file": name,
            }
        )

    last = states[-1]
    manifest: Dict[str, Any] = {
        "schema_version": SCHEMA_VERSION,
        "scene_id": scene_id,
        "source": source,
        "frame_count": len(states),
        "frame_indices": [int(s.frame_index) for s in states],
        "world_frame": WORLD_FRAME_NAME,
        "world_reference_frame_index": int(WORLD_REFERENCE_FRAME),
        "coordinate_system": {
            "world": WORLD_FRAME_NAME,
            "note": (
                "World is the first processed LiDAR frame (index "
                f"{int(WORLD_REFERENCE_FRAME)}). ORBIT does not convert "
                "to nuScenes global. Sensor-frame points are current-LiDAR "
                "XYZ after range filtering. Heading is atan2 of LiDAR +X "
                "after T_ego_to_world."
            ),
            "lidar_forward": "+X",
        },
        "downsample": {
            "method": "evenly_spaced_indices",
            "max_points": int(max_points),
            "description": (
                "points and world_points share np.round(linspace(0, N-1, max_points)) "
                "into mapped XYZ. Full mapped_points stay in metrics.point_count_full."
            ),
        },
        "pose_source": str(last.pose_source),
        "files": frame_files,
        "website_note": (
            "The website visualizes exported ORBIT pipeline results. "
            "It does not execute perception, mapping, tracking, or "
            "world-model algorithms in the browser."
        ),
    }
    if extra_manifest:
        manifest.update(extra_manifest)

    (scene_dir / "manifest.json").write_text(
        json.dumps(json_ready(manifest), indent=2) + "\n",
        encoding="utf-8",
    )
    (scene_dir / "trajectory.json").write_text(
        json.dumps(json_ready({"world_frame": WORLD_FRAME_NAME, "samples": trajectory}), indent=2)
        + "\n",
        encoding="utf-8",
    )
    return scene_dir


def _frame_list(start: int, end: Optional[int], max_frames: Optional[int], n_available: int) -> List[int]:
    if start < 0:
        raise SystemExit("--start must be >= 0")
    last = n_available - 1
    if end is None:
        end = last
    if max_frames is not None:
        end = min(end, start + int(max_frames) - 1)
    end = min(end, last)
    if end < start:
        raise SystemExit(
            f"empty frame range start={start} end={end} available=0..{last}"
        )
    return list(range(start, end + 1))


def load_clouds(
    args: argparse.Namespace,
) -> tuple[OrbitSystem, List[int], Dict[int, np.ndarray], str, Dict[str, Any]]:
    extra: Dict[str, Any] = {}

    if args.source == "kitti":
        from tracking.ego_motion import EgoMotionCompensator

        if args.end is None and args.max_frames is None:
            raise SystemExit("KITTI export requires --end or --max-frames")
        ego_motion = EgoMotionCompensator(args.dataset_root, sequence=args.sequence)
        scene_id = f"kitti-{args.sequence}"
        last = args.end if args.end is not None else args.start + int(args.max_frames) - 1
        frames = _frame_list(args.start, last, args.max_frames, last + 1)
        clouds = {
            frame: load_kitti_xyz(args.dataset_root, args.sequence, frame) for frame in frames
        }
        system = OrbitSystem(ego_motion=ego_motion)
        extra["sequence"] = args.sequence
        extra["dataset_root"] = str(args.dataset_root)
        return system, frames, clouds, scene_id, extra

    if args.source == "nuscenes":
        from datasets.nuscenes_loader import NuScenesMiniLoader
        from tracking.nuscenes_ego_motion import NuScenesEgoMotion

        nusc_root = resolve_nuscenes_root(args.dataset_root)
        probe = NuScenesMiniLoader(nusc_root)
        scene_name = args.scene
        if not scene_name:
            names = probe.list_scenes()
            if not names:
                raise SystemExit(f"No scenes in {nusc_root / 'v1.0-mini'}")
            scene_name = names[0]
            print(f"No --scene given; using first scene {scene_name}")
        loader = NuScenesMiniLoader(nusc_root, scene_name=scene_name)
        extra["nuscenes_scene"] = loader.scene_info(scene_name)
        extra["dataset_root"] = str(nusc_root)
        n_frames = loader.num_frames()
        frames = _frame_list(args.start, args.end, args.max_frames, n_frames)
        clouds = {frame: loader.load_xyz(frame) for frame in frames}
        ego_motion = NuScenesEgoMotion(loader)
        system = OrbitSystem(ego_motion=ego_motion)
        system.tracker.set_timestamps(loader.timestamps_seconds())
        return system, frames, clouds, scene_name, extra

    points = load_synthetic_points()
    frames = [0]
    clouds = {0: points}
    system = OrbitSystem(ego_motion=None)
    extra["synthetic_ply"] = True
    return system, frames, clouds, "synthetic", extra


def get_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Export real ORBIT PipelineState JSON for the website"
    )
    parser.add_argument(
        "--source",
        choices=("synthetic", "kitti", "nuscenes"),
        default="nuscenes",
    )
    parser.add_argument("--sequence", default="00")
    parser.add_argument(
        "--scene",
        default="scene-0061",
        help="nuScenes scene name. Ignored for KITTI. Default scene-0061.",
    )
    parser.add_argument("--start", type=int, default=0)
    parser.add_argument(
        "--end",
        type=int,
        default=None,
        help="Inclusive last frame index. Default: last available (or start if KITTI needs --max-frames).",
    )
    parser.add_argument(
        "--max-frames",
        type=int,
        default=None,
        help="Optional cap on the number of frames from --start.",
    )
    parser.add_argument(
        "--dataset-root",
        default=os.environ.get("ORBIT_NUSCENES_ROOT", ""),
        help="KITTI SemanticKITTI root or nuScenes extract. nuScenes: also ORBIT_NUSCENES_ROOT.",
    )
    parser.add_argument(
        "--output",
        default="exported_data",
        help="Root directory (a per-scene folder is created inside).",
    )
    parser.add_argument(
        "--max-points",
        type=int,
        default=DEFAULT_MAX_POINTS,
        help="Max XYZ points written per frame (evenly spaced indices).",
    )
    return parser.parse_args(argv)


def main(argv: Optional[Sequence[str]] = None) -> Path:
    args = get_args(argv)
    system, frames, clouds, scene_id, extra = load_clouds(args)
    states: List[PipelineState] = []
    for frame in frames:
        state = system.process_frame(clouds[frame], frame)
        states.append(state)
        print(
            f"exported frame {frame}: "
            f"mapped={state.metrics.get('mapped_points')} "
            f"cells={state.metrics.get('adaptive_cells')} "
            f"tracks={state.metrics.get('live_tracks')} "
            f"ego={state.ego_xy}"
        )
    out = write_export(
        Path(args.output),
        states,
        source=args.source,
        scene_id=scene_id,
        extra_manifest=extra,
        max_points=args.max_points,
    )
    print(f"Wrote {len(states)} frames to {out}")
    return out


if __name__ == "__main__":
    main()
