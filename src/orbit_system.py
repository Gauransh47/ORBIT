"""
ORBIT end-to-end prototype orchestrator.

Data flow (no SemanticKITTI labels, no DBSCAN, no neural nets):

    raw LiDAR XYZ
      → range preprocessing
      → canonical RANSAC ground estimate
      → AdaptiveGrid (discretization only)
      → terrain-relative obstacle cells
      → geometric ObjectProposal detector
      → tracker detections (adapter)
      → ego-motion into LiDAR frame 0
      → OrbitTracker (identity / lifecycle)
      → OrbitWorldModel.sync (live tracks; prune = retire)
      → visualization / metrics from this state

World / reference frame
-----------------------
Prototype world is **LiDAR frame 0** (OrbitTracker.WORLD_REFERENCE_FRAME).
Each new detection is transformed into that frame. Existing tracks are
never transformed again.

When ego_motion is None (synthetic / identity), detections stay in the
current LiDAR frame, which is treated as the world frame.

Usage (repo root):

    PYTHONPATH=src python src/orbit_system.py --source synthetic
    PYTHONPATH=src python src/orbit_system.py --source kitti --start 0 --end 4
"""

from __future__ import annotations

import argparse
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Optional

SRC_DIR = Path(__file__).resolve().parent
REPO_ROOT = SRC_DIR.parent

if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

import numpy as np

from common.paths import SEMANTIC_KITTI_ROOT, SYNTHETIC_SCENE
from mapping.adaptive_grid import AdaptiveGrid
from perception.ground_estimation import GroundEstimate, estimate_ground
from perception.object_proposal import ObjectProposal
from perception.orbit_perception import OrbitPerception
from perception.preprocess import filter_range
from tracking.orbit_tracker import OrbitTracker, WORLD_REFERENCE_FRAME
from tracking.proposal_adapter import proposals_to_detections
from visualization.pipeline_view import view_model_from_state
from visualization.dashboard import (
    launch_dashboard,
    save_dashboard_frames,
)
from visualization.architecture_view import render_architecture_diagram
from visualization.dashboard_data import subsample_xyz
from world_model.orbit_world_model import OrbitWorldModel


# Deterministic per-frame world-map subsample. Full mapped_points stay in metrics.
WORLD_SCAN_CAP = 8000

# KITTI LiDAR +X is forward in the velodyne frame used by this prototype.
LIDAR_FORWARD = np.array([1.0, 0.0, 0.0], dtype=np.float64)


@dataclass
class PipelineState:
    frame_index: int
    points: np.ndarray
    ground: GroundEstimate
    grid: AdaptiveGrid
    obstacle_cells: list
    proposals: List[ObjectProposal]
    detections: list
    tracks: list
    world_objects: list
    retired_ids: list
    metrics: Dict[str, Any] = field(default_factory=dict)
    view_model: Dict[str, Any] = field(default_factory=dict)
    # Ego pose in LiDAR frame 0. Identity when no odometry is attached.
    ego_xy: tuple = (0.0, 0.0)
    ego_heading_rad: float = 0.0
    pose_source: str = "identity_no_odometry"
    T_ego_to_world: Optional[np.ndarray] = None
    # Current scan transformed into LiDAR frame 0 (subsampled).
    world_points: Optional[np.ndarray] = None


class OrbitSystem:
    """One object that owns perception, tracker, and world model."""

    def __init__(
        self,
        ego_motion=None,
        world_reference_frame=WORLD_REFERENCE_FRAME,
        perception=None,
    ):
        self.perception = perception or OrbitPerception()
        self.tracker = OrbitTracker(
            ego_motion=ego_motion,
            world_reference_frame=world_reference_frame,
        )
        self.world_model = OrbitWorldModel()
        self.world_reference_frame = int(world_reference_frame)

    def _pose_source_label(self) -> str:
        ego_motion = self.tracker.ego_motion
        if ego_motion is None:
            return "identity_no_odometry"
        poses = getattr(ego_motion, "poses", None)
        if poses is None or len(poses) == 0:
            return "identity_empty_poses"
        return "kitti_poses_lidar_to_frame0"

    def ego_to_world_matrix(self, frame_index: int) -> np.ndarray:
        """4x4: current LiDAR frame → LiDAR frame 0."""

        ego_motion = self.tracker.ego_motion
        if ego_motion is None:
            return np.eye(4, dtype=np.float64)
        return np.asarray(
            ego_motion.transform(int(frame_index), self.world_reference_frame),
            dtype=np.float64,
        )

    def ego_pose_in_world(self, frame_index: int):
        """
        Sensor origin and heading in LiDAR frame 0.

        Heading is atan2 of KITTI LiDAR +X (forward) after the same
        transform the tracker uses for detections. Not a GPS heading.
        """

        T = self.ego_to_world_matrix(frame_index)
        origin = T[:3, 3]
        forward = T[:3, :3] @ LIDAR_FORWARD
        heading = float(np.arctan2(forward[1], forward[0]))
        return (
            (float(origin[0]), float(origin[1])),
            heading,
            T,
            self._pose_source_label(),
        )

    def process_frame(self, points, frame_index: int) -> PipelineState:
        start = time.perf_counter()

        points = np.asarray(points, dtype=np.float64)
        if points.ndim != 2 or points.shape[1] < 3:
            raise ValueError("points must have shape (N, 3) or (N, 4)")

        xyz, _ = filter_range(points)
        xyz = xyz[:, :3]

        ground = estimate_ground(
            xyz,
            distance_threshold=self.perception.ground_distance_threshold,
            num_iterations=self.perception.ransac_iterations,
        )

        grid = AdaptiveGrid()
        grid.build(xyz, ground.mask)

        detection = self.perception.detect_from_grid(grid)
        proposals = detection["proposals"]
        tracker_detections = proposals_to_detections(proposals)

        self.tracker.associate(
            tracker_detections,
            frame_index,
            actual_frame=frame_index,
        )

        live_tracks = list(self.tracker.tracks)
        retired_ids = self.world_model.sync(live_tracks, frame_index)

        elapsed = time.perf_counter() - start

        ego_xy, heading, T_ego_to_world, pose_source = self.ego_pose_in_world(
            frame_index
        )
        scan_sub = subsample_xyz(xyz, WORLD_SCAN_CAP, seed=10_000 + int(frame_index))
        ego_motion = self.tracker.ego_motion
        if ego_motion is not None and len(scan_sub):
            world_points = ego_motion.transform_points(
                scan_sub,
                source_frame=int(frame_index),
                target_frame=self.world_reference_frame,
            )
        else:
            world_points = scan_sub

        metrics = {
            "latency_ms": elapsed * 1000.0,
            "input_points": int(len(points)),
            "mapped_points": int(len(xyz)),
            "ground_points": int(ground.inlier_count),
            "adaptive_cells": int(len(grid.cells)),
            "obstacle_cells": int(len(detection["obstacle_cells"])),
            "raw_components": int(len(detection["raw_components"])),
            "filtered_components": int(len(detection["filtered_components"])),
            "proposals": int(len(proposals)),
            "live_tracks": int(len(live_tracks)),
            "confirmed_tracks": int(len(self.tracker.active_tracks())),
            "world_objects": int(len(self.world_model.objects)),
            "retired_ids": list(retired_ids),
            "world_frame": "lidar_frame_0",
            "pose_source": pose_source,
            "ego_x": ego_xy[0],
            "ego_y": ego_xy[1],
            "ego_heading_rad": heading,
        }

        state = PipelineState(
            frame_index=int(frame_index),
            points=xyz,
            ground=ground,
            grid=grid,
            obstacle_cells=detection["obstacle_cells"],
            proposals=proposals,
            detections=tracker_detections,
            tracks=live_tracks,
            world_objects=list(self.world_model.objects.values()),
            retired_ids=list(retired_ids),
            metrics=metrics,
            ego_xy=ego_xy,
            ego_heading_rad=heading,
            pose_source=pose_source,
            T_ego_to_world=T_ego_to_world,
            world_points=world_points,
        )
        state.view_model = view_model_from_state(state)
        return state


def print_frame_metrics(state: PipelineState):
    m = state.metrics
    print("=" * 70)
    print(f"ORBIT SYSTEM  frame {state.frame_index}")
    print("=" * 70)
    print(f"  World frame:     {m['world_frame']}")
    print(f"  Ego XY (frame0): ({state.ego_xy[0]:.3f}, {state.ego_xy[1]:.3f})")
    print(f"  Pose source:     {state.pose_source}")
    print(f"  Mapped points:   {m['mapped_points']:,}")
    print(f"  Ground inliers:  {m['ground_points']:,}")
    print(f"  Adaptive cells:  {m['adaptive_cells']:,}")
    print(f"  Obstacle cells:  {m['obstacle_cells']:,}")
    print(f"  Proposals:       {m['proposals']}")
    print(f"  Live tracks:     {m['live_tracks']}")
    print(f"  Confirmed:       {m['confirmed_tracks']}")
    print(f"  World objects:   {m['world_objects']}")
    print(f"  Retired IDs:     {m['retired_ids']}")
    print(f"  Latency:         {m['latency_ms']:.1f} ms")

    if state.proposals:
        print("\n  Proposals:")
        for proposal in state.proposals[:12]:
            print(
                f"    #{proposal.proposal_id} {proposal.classification:<14} "
                f"({proposal.center_x:6.2f}, {proposal.center_y:6.2f}) "
                f"conf={proposal.confidence:.2f}"
            )

    if state.tracks:
        print("\n  Tracks:")
        for track in state.tracks[:12]:
            print(
                f"    #{track.track_id:03d} {track.class_name:<14} "
                f"hits={track.hits} missed={track.missed} "
                f"confirmed={track.confirmed}"
            )


def load_synthetic_points():
    import open3d as o3d

    if not SYNTHETIC_SCENE.exists():
        raise FileNotFoundError(
            f"Missing {SYNTHETIC_SCENE}\n"
            "Generate with: PYTHONPATH=src python src/synthetic_scene.py"
        )

    cloud = o3d.io.read_point_cloud(str(SYNTHETIC_SCENE))
    return np.asarray(cloud.points)


def load_kitti_xyz(dataset_root, sequence, frame_id):
    from datasets.kitti_loader import KittiLoader

    loader = KittiLoader(dataset_root, sequence=sequence)
    return loader.load_xyz(frame_id)


def get_args():
    parser = argparse.ArgumentParser(
        description="ORBIT geometric end-to-end prototype"
    )
    parser.add_argument(
        "--source",
        choices=("synthetic", "kitti"),
        default="synthetic",
    )
    parser.add_argument("--sequence", default="00")
    parser.add_argument("--start", type=int, default=0)
    parser.add_argument("--end", type=int, default=0)
    parser.add_argument(
        "--dataset-root",
        default=str(SEMANTIC_KITTI_ROOT),
    )
    parser.add_argument(
        "--show",
        action="store_true",
        help="Open the Visual Intelligence dashboard (Matplotlib)",
    )
    parser.add_argument(
        "--dashboard",
        action="store_true",
        help="Same as --show: multi-panel dashboard from real pipeline states",
    )
    parser.add_argument(
        "--save-figures",
        type=str,
        default=None,
        help="Directory for dashboard PNG frames",
    )
    parser.add_argument(
        "--lidar-color",
        choices=("elevation", "terrain"),
        default="elevation",
        help="3D LiDAR colour mode: Z elevation or RANSAC ground mask",
    )
    parser.add_argument(
        "--save-architecture",
        type=str,
        default=None,
        help="Write a static architecture diagram PNG (not live data)",
    )
    return parser.parse_args()


def main():
    args = get_args()

    ego_motion = None
    if args.source == "kitti":
        from tracking.ego_motion import EgoMotionCompensator

        ego_motion = EgoMotionCompensator(
            args.dataset_root,
            sequence=args.sequence,
        )

    system = OrbitSystem(ego_motion=ego_motion)

    print("ORBIT prototype")
    print("World / reference frame: LiDAR frame 0")
    print("Detector: geometric OrbitPerception (no SemanticKITTI GT)")
    if ego_motion is None:
        print("Ego pose: identity (no KITTI poses.txt — trajectory stays at origin)")
    else:
        print("Ego pose: KITTI poses.txt + calib Tr  →  LiDAR frame 0")
    print()

    if args.source == "synthetic":
        points = load_synthetic_points()
        frames = [0]
        clouds = {0: points}
    else:
        frames = list(range(args.start, args.end + 1))
        clouds = {
            frame: load_kitti_xyz(
                args.dataset_root,
                args.sequence,
                frame,
            )
            for frame in frames
        }

    last_state = None
    states = []
    for frame in frames:
        state = system.process_frame(clouds[frame], frame)
        print_frame_metrics(state)
        last_state = state
        states.append(state)

    if args.save_architecture:
        render_architecture_diagram(
            save_path=args.save_architecture,
            show=False,
        )
        print(f"Wrote architecture diagram: {args.save_architecture}")

    if args.save_figures and states:
        paths = save_dashboard_frames(
            states,
            args.save_figures,
            sequence=str(args.sequence),
            source=args.source,
            lidar_mode=args.lidar_color,
        )
        print(f"Wrote {len(paths)} dashboard frames to {args.save_figures}")

    if (args.show or args.dashboard) and states:
        launch_dashboard(
            states,
            sequence=str(args.sequence),
            source=args.source,
            lidar_mode=args.lidar_color,
            show=True,
        )


if __name__ == "__main__":
    main()
