import sys
from pathlib import Path

import numpy as np

sys.path.append(str(Path(__file__).resolve().parents[1] / "src"))

from mapping.adaptive_grid import AdaptiveGrid
from perception.ground_estimation import estimate_ground
from perception.object_proposal import ObjectProposal
from perception.orbit_perception import OrbitPerception
from perception.terrain_obstacle_detection import extract_obstacle_cells
from orbit_system import OrbitSystem
from tracking.orbit_tracker import MAX_MISSED_FRAMES
from tracking.proposal_adapter import proposals_to_detections
from visualization.pipeline_view import render_topdown, view_model_from_state


def _cloud(box_x=3.0, with_box=True, seed=0):
    """Deterministic ground plane plus an optional filled box."""

    gx, gy = np.meshgrid(
        np.linspace(-8.0, 8.0, 80),
        np.linspace(-8.0, 8.0, 80),
    )
    ground = np.column_stack(
        [
            gx.ravel(),
            gy.ravel(),
            np.zeros(gx.size),
        ]
    )

    if not with_box:
        return ground

    bx, by, bz = np.meshgrid(
        np.linspace(box_x, box_x + 2.0, 50),
        np.linspace(-0.9, 0.9, 40),
        np.linspace(0.3, 1.6, 12),
    )
    box = np.column_stack(
        [
            bx.ravel(),
            by.ravel(),
            bz.ravel(),
        ]
    )
    return np.vstack([ground, box])
    rng = np.random.default_rng(seed)
    gx = rng.uniform(-8.0, 8.0, 3500)
    gy = rng.uniform(-8.0, 8.0, 3500)
    gz = rng.normal(0.0, 0.01, 3500)
    ground = np.column_stack([gx, gy, gz])

    if not with_box:
        return ground

    n = 900
    bx = rng.uniform(box_x, box_x + 2.0, n)
    by = rng.uniform(-0.9, 0.9, n)
    bz = rng.uniform(0.25, 1.7, n)
    return np.vstack([ground, np.column_stack([bx, by, bz])])


def test_canonical_ground_ransac_plane():
    points = _cloud(with_box=False)
    estimate = estimate_ground(points)

    assert estimate.method == "ransac"
    assert estimate.mask.shape == (len(points),)
    assert estimate.inlier_count > 0.8 * len(points)
    assert estimate.plane_model is not None


def test_neighbor_ground_recovers_obstacle_only_cell():
    ground = []
    for x in np.linspace(-1.0, 1.0, 40):
        for y in np.linspace(-1.0, 1.0, 40):
            if abs(x) < 0.12 and abs(y) < 0.12:
                continue
            ground.append([x, y, 0.0])

    obstacles = []
    for _ in range(30):
        obstacles.append([0.02, 0.02, 1.2])

    points = np.asarray(ground + obstacles, dtype=np.float64)
    mask = np.zeros(len(points), dtype=bool)
    mask[: len(ground)] = True

    grid = AdaptiveGrid()
    grid.build(points, mask)

    cells = extract_obstacle_cells(
        grid,
        minimum_height=0.15,
        minimum_obstacle_points=3,
    )

    neighbor_cells = [c for c in cells if c.ground_source == "neighbor"]
    assert len(cells) >= 1
    assert len(neighbor_cells) >= 1
    assert neighbor_cells[0].obstacle_height > 0.5


def test_detector_emits_object_proposals_without_gt():
    perception = OrbitPerception()
    result = perception.process(_cloud())

    assert "proposals" in result
    assert len(result["proposals"]) >= 1
    proposal = result["proposals"][0]
    assert isinstance(proposal, ObjectProposal)
    assert proposal.classification
    assert proposal.confidence >= 0.0

    detections = proposals_to_detections(result["proposals"])
    assert detections[0].semantic_class == proposal.classification
    assert detections[0].center_x == proposal.center_x


def test_end_to_end_ids_world_model_and_retirement():
    system = OrbitSystem(ego_motion=None)

    state0 = system.process_frame(_cloud(box_x=3.0, seed=1), frame_index=0)
    assert len(state0.proposals) >= 1
    assert isinstance(state0.proposals[0], ObjectProposal)
    assert len(state0.tracks) >= 1
    assert len(state0.world_objects) >= 1

    ids0 = {track.track_id for track in state0.tracks}

    state1 = system.process_frame(_cloud(box_x=3.25, seed=2), frame_index=1)
    ids1 = {track.track_id for track in state1.tracks}
    assert ids0 & ids1, "expected at least one ID to persist across frames"
    assert ids1.issubset({obj.track_id for obj in state1.world_objects})

    view = view_model_from_state(state1)
    assert view["proposals"]
    assert view["tracks"]
    assert view["world_objects"]
    assert view["world_frame"] == "lidar_frame_0"
    render_topdown(state1, show=False)

    last_live = None
    for frame in range(2, 2 + MAX_MISSED_FRAMES + 2):
        last_live = system.process_frame(
            _cloud(with_box=False, seed=frame),
            frame_index=frame,
        )

    remaining = {obj.track_id for obj in last_live.world_objects}
    assert ids0.isdisjoint(remaining)
