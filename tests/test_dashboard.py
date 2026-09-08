import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")

import numpy as np

sys.path.append(str(Path(__file__).resolve().parents[1] / "src"))

from visualization.dashboard_data import (
    elevation_layer,
    select_proposal_labels,
    select_track_labels,
    subsample_indices,
    terrain_point_labels,
    traversability_code,
    traversability_layer,
)
from visualization.color_schemes import proposal_color
from visualization.architecture_view import render_architecture_diagram
from visualization.dashboard import render_dashboard
from orbit_system import OrbitSystem
from tests.test_orbit_system import _cloud


def test_subsample_is_deterministic_and_capped():
    idx_a = subsample_indices(1000, 50, seed=0)
    idx_b = subsample_indices(1000, 50, seed=0)
    assert len(idx_a) == 50
    assert np.array_equal(idx_a, idx_b)
    assert len(subsample_indices(10, 50)) == 10


def test_terrain_labels_follow_ground_mask():
    mask = np.array([True, False, True])
    labels = terrain_point_labels(mask)
    assert list(labels) == [0, 1, 0]


def test_traversability_codes_match_cell_classes():
    assert traversability_code("GROUND") == 0
    assert traversability_code("MIXED") == 1
    assert traversability_code("OBSTACLE") == 2


def test_elevation_and_traversability_layers_use_records():
    records = [
        {
            "x": 1.0,
            "y": 2.0,
            "resolution": 0.05,
            "semantic_class": "GROUND",
            "ground_elevation": 0.1,
            "z_mean": 0.1,
            "point_count": 4,
        },
        {
            "x": 3.0,
            "y": 4.0,
            "resolution": 0.10,
            "semantic_class": "OBSTACLE",
            "ground_elevation": None,
            "z_mean": 1.5,
            "point_count": 8,
        },
    ]
    xs, ys, zs = elevation_layer(records)
    assert len(zs) == 1
    assert zs[0] == 0.1

    tx, ty, codes, _ = traversability_layer(records)
    assert list(codes) == [0, 2]
    assert len(tx) == 2


def test_proposal_colors_are_stable():
    assert proposal_color("VEHICLE-LIKE") != proposal_color("WALL")
    assert proposal_color("unknown-class") == proposal_color("also-unknown")


class _Track:
    def __init__(self, track_id, confirmed, x, y):
        self.track_id = track_id
        self.confirmed = confirmed
        self.position = np.array([x, y], dtype=np.float32)
        self.class_name = "OBSTACLE"
        self.hits = 2
        self.missed = 0


class _Proposal:
    def __init__(self, confidence, distance, classification="OBSTACLE"):
        self.confidence = confidence
        self.distance = distance
        self.classification = classification
        self.center_x = 0.0
        self.center_y = 0.0
        self.proposal_id = 0


def test_label_selection_prefers_confirmed_and_confidence():
    tracks = [
        _Track(1, False, 1.0, 0.0),
        _Track(2, True, 40.0, 0.0),
        _Track(3, True, 2.0, 0.0),
    ]
    labelled = select_track_labels(tracks, max_labels=1)
    assert labelled[0].track_id == 3

    proposals = [
        _Proposal(0.2, 5.0),
        _Proposal(0.9, 8.0),
        _Proposal(0.9, 1.0),
    ]
    top = select_proposal_labels(proposals, max_labels=1)
    assert top[0].distance == 1.0


def test_dashboard_renders_from_real_pipeline_state(tmp_path):
    system = OrbitSystem(ego_motion=None)
    state = system.process_frame(_cloud(box_x=3.0), frame_index=0)
    assert state.metrics["mapped_points"] == len(state.points)
    assert state.metrics["proposals"] == len(state.proposals)

    out = tmp_path / "dash.png"
    fig = render_dashboard(
        [state],
        index=0,
        sequence="test",
        source="synthetic",
        show=False,
        save_path=str(out),
    )
    assert out.exists()
    assert out.stat().st_size > 1000
    import matplotlib.pyplot as plt
    plt.close(fig)


def test_architecture_diagram_is_not_pipeline_output(tmp_path):
    path = tmp_path / "arch.png"
    render_architecture_diagram(save_path=str(path), show=False)
    assert path.exists()
