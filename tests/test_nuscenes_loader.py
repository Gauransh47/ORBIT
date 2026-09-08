import json
import sys
from pathlib import Path

import numpy as np

sys.path.append(str(Path(__file__).resolve().parents[1] / "src"))

from datasets.nuscenes_loader import (
    NuScenesMiniLoader,
    quaternion_wxyz_to_rotation,
)
from datasets.sequence_interface import LidarSequence
from tracking.nuscenes_ego_motion import NuScenesEgoMotion
from orbit_system import OrbitSystem
from tests.nuscenes_fixture import build_mini_fixture
from tests.test_orbit_system import _cloud


def test_quaternion_wxyz_rz90():
    w = np.sqrt(2.0) / 2.0
    R = quaternion_wxyz_to_rotation([w, 0.0, 0.0, w])
    x = R @ np.array([1.0, 0.0, 0.0])
    assert np.allclose(x, [0.0, 1.0, 0.0], atol=1e-6)


def test_scene_discovery_and_keyframe_order(tmp_path):
    root = build_mini_fixture(tmp_path / "nusc")
    loader = NuScenesMiniLoader(root)
    assert loader.list_scenes() == ["scene-fixture"]
    info = loader.scene_info("scene-fixture")
    assert info["nbr_lidar_keyframes"] == 3
    assert info["first_sample_token"] == "samp0"
    assert info["last_sample_token"] == "samp2"

    loader.select_scene("scene-fixture")
    assert loader.frame_ids() == [0, 1, 2]
    assert isinstance(loader, LidarSequence)

    tokens = [loader.lidar_record(i)["token"] for i in loader.frame_ids()]
    assert tokens == ["sd0", "sd1", "sd2"]
    assert all(loader.lidar_record(i)["is_key_frame"] for i in range(3))
    assert "sweep" not in loader.lidar_record(1)["filename"]


def test_lidar_xyz_timestamps_and_no_annotations(tmp_path):
    root = build_mini_fixture(tmp_path / "nusc")
    loader = NuScenesMiniLoader(root, scene_name="scene-fixture")
    xyz0 = loader.load_xyz(0)
    assert xyz0.shape == (2, 3)
    assert np.allclose(xyz0[1], [1.0, 0.0, 0.0])
    xyz1 = loader.load_xyz(1)
    assert np.allclose(xyz1[1], [2.0, 0.0, 0.0])

    assert loader.timestamp_us(0) == 1_000_000
    assert loader.timestamp_us(1) == 1_500_000
    assert loader.timestamp_us(2) == 2_000_000
    ts = loader.timestamps_seconds()
    assert np.allclose(ts, [1.0, 1.5, 2.0])

    assert "sample_annotation" not in loader._tables
    full = loader.load_points(0)
    assert full.shape == (2, 5)


def test_sample_prev_next_and_lidar_chain_fields(tmp_path):
    root = build_mini_fixture(tmp_path / "nusc")
    loader = NuScenesMiniLoader(root, scene_name="scene-fixture")
    assert loader.lidar_record(0)["prev"] == ""
    assert loader.lidar_record(0)["next"] == "sd1"
    assert loader.lidar_record(1)["prev"] == "sd0"
    assert loader.lidar_record(2)["next"] == ""


def test_global_and_lidar_to_lidar_transforms(tmp_path):
    root = build_mini_fixture(tmp_path / "nusc")
    loader = NuScenesMiniLoader(root, scene_name="scene-fixture")

    T0 = loader.T_global_from_lidar(0)
    # ego t=[0,0,0], sensor t=[1,0,2]
    assert np.allclose(T0[:3, 3], [1.0, 0.0, 2.0])
    T1 = loader.T_global_from_lidar(1)
    assert np.allclose(T1[:3, 3], [6.0, 0.0, 2.0])

    T_1_to_0 = loader.lidar_to_lidar(1, 0)
    origin1_in_0 = T_1_to_0[:3, 3]
    assert np.allclose(origin1_in_0, [5.0, 0.0, 0.0])

    T_frame0 = loader.T_frame0_from_current(2)
    assert np.allclose(T_frame0[:3, 3], [10.0, 0.0, 0.0])

    pose = loader.global_pose(1)
    assert pose["translation"] == [5.0, 0.0, 0.0]


def test_nuscenes_ego_motion_duck_type_and_orbit_state(tmp_path):
    root = build_mini_fixture(tmp_path / "nusc")
    loader = NuScenesMiniLoader(root, scene_name="scene-fixture")
    ego = NuScenesEgoMotion(loader)
    assert ego.pose_source == "nuscenes_ego_pose_lidar_to_frame0"
    assert len(ego.poses) == 3

    p = ego.transform_point(np.zeros(3), source_frame=1, target_frame=0)
    assert np.allclose(p, [5.0, 0.0, 0.0])

    pts = ego.transform_points(np.zeros((2, 3)), source_frame=2, target_frame=0)
    assert np.allclose(pts[0], [10.0, 0.0, 0.0])

    system = OrbitSystem(ego_motion=ego)
    system.tracker.set_timestamps(loader.timestamps_seconds())
    state = system.process_frame(_cloud(box_x=3.0), frame_index=1)
    assert state.pose_source == "nuscenes_ego_pose_lidar_to_frame0"
    assert abs(state.ego_xy[0] - 5.0) < 1e-9
    assert state.T_ego_to_world is not None
    assert state.world_points is not None
    live_med = float(np.median(state.points[:, 0]))
    world_med = float(np.median(state.world_points[:, 0]))
    assert world_med > live_med + 2.0


def test_kitti_pose_source_unchanged_without_nuscenes():
    system = OrbitSystem(ego_motion=None)
    state = system.process_frame(_cloud(box_x=3.0), frame_index=0)
    assert state.pose_source == "identity_no_odometry"


def test_fixture_does_not_use_windows_dataroot(tmp_path):
    root = build_mini_fixture(tmp_path / "nusc")
    loader = NuScenesMiniLoader(root, scene_name="scene-fixture")
    assert "ORBIT_DATA" not in str(loader.dataroot)
    assert "D:" not in str(loader.dataroot)
    # annotations file exists on disk but is not loaded
    assert (root / "v1.0-mini" / "sample_annotation.json").exists()
    dumped = json.loads((root / "v1.0-mini" / "sample_annotation.json").read_text())
    assert dumped[0]["translation"][0] == 99.0
    xyz = loader.load_xyz(0)
    assert 99.0 not in xyz[:, 0]
