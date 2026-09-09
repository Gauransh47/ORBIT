"""Exporter writes real PipelineState JSON. Does not invent detections."""

from __future__ import annotations

import json
import sys
from pathlib import Path

import numpy as np

sys.path.append(str(Path(__file__).resolve().parents[1] / "src"))

from orbit_system import OrbitSystem
from tests.nuscenes_fixture import build_mini_fixture
from tests.test_orbit_system import _cloud
from web_export.export_orbit_data import even_indices, main, write_export


def test_even_indices_are_deterministic_and_bounded():
    a = even_indices(10_000, 500)
    b = even_indices(10_000, 500)
    assert np.array_equal(a, b)
    assert len(a) == 500
    assert int(a[0]) == 0
    assert int(a[-1]) == 9999
    small = even_indices(12, 500)
    assert list(small) == list(range(12))


def test_export_matches_pipeline_state(tmp_path):
    system = OrbitSystem(ego_motion=None)
    state0 = system.process_frame(_cloud(box_x=3.0, seed=1), frame_index=0)
    state1 = system.process_frame(_cloud(box_x=3.25, seed=2), frame_index=1)

    scene_dir = write_export(
        tmp_path,
        [state0, state1],
        source="synthetic",
        scene_id="unit-cloud",
        max_points=400,
    )

    manifest = json.loads((scene_dir / "manifest.json").read_text(encoding="utf-8"))
    assert manifest["scene_id"] == "unit-cloud"
    assert manifest["frame_count"] == 2
    assert manifest["world_frame"] == "lidar_frame_0"
    assert manifest["files"] == ["frame_0000.json", "frame_0001.json"]

    payload = json.loads((scene_dir / "frame_0001.json").read_text(encoding="utf-8"))
    assert payload["frame_index"] == 1
    assert payload["points_frame"] == "current_lidar"
    assert payload["world_points_frame"] == "lidar_frame_0"
    assert payload["metrics"]["point_count_full"] == int(state1.metrics["mapped_points"])
    assert payload["metrics"]["point_count_exported"] == len(payload["points"])
    assert payload["metrics"]["mapped_points"] == int(state1.metrics["mapped_points"])
    assert payload["metrics"]["ground_points"] == int(state1.metrics["ground_points"])
    assert payload["metrics"]["adaptive_cells"] == int(state1.metrics["adaptive_cells"])
    assert payload["metrics"]["live_tracks"] == int(state1.metrics["live_tracks"])
    assert len(payload["adaptive_cells"]) == int(state1.metrics["adaptive_cells"])
    assert payload["pose"]["ego_xy"] == [0.0, 0.0]
    assert payload["pose"]["pose_source"] == "identity_no_odometry"
    assert payload["tracks"]
    track_ids = {t["track_id"] for t in payload["tracks"]}
    assert track_ids == {t.track_id for t in state1.tracks}
    for cell in payload["adaptive_cells"]:
        assert "center" in cell and "resolution" in cell
        assert cell["semantic_class"] in {"GROUND", "MIXED", "OBSTACLE"}
    traj = json.loads((scene_dir / "trajectory.json").read_text(encoding="utf-8"))
    assert len(traj["samples"]) == 2
    assert traj["samples"][1]["ego_xy"] == [0.0, 0.0]


def test_cli_exports_nuscenes_fixture(tmp_path):
    root = build_mini_fixture(tmp_path / "nusc")
    out = tmp_path / "exported"
    dest = main(
        [
            "--source",
            "nuscenes",
            "--dataset-root",
            str(root),
            "--scene",
            "scene-fixture",
            "--start",
            "0",
            "--end",
            "2",
            "--output",
            str(out),
            "--max-points",
            "8",
        ]
    )
    manifest = json.loads((dest / "manifest.json").read_text(encoding="utf-8"))
    assert manifest["scene_id"] == "scene-fixture"
    assert manifest["frame_count"] == 3
    assert manifest["source"] == "nuscenes"
    assert "nuscenes_scene" in manifest
    frame0 = json.loads((dest / "frame_0000.json").read_text(encoding="utf-8"))
    assert frame0["pose"]["ego_xy"][0] == 0.0
    frame2 = json.loads((dest / "frame_0002.json").read_text(encoding="utf-8"))
    # Fixture ego translates +5 m in x each keyframe; sensor offset is identity in LiDAR→frame0 chain.
    assert frame2["pose"]["pose_source"]
    assert frame2["metrics"]["point_count_full"] >= 1
    assert len(frame2["points"][0]) == 3
