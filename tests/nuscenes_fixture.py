"""Synthetic nuScenes v1.0-mini metadata for loader tests. Not real data."""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np

IDENTITY_Q = [1.0, 0.0, 0.0, 0.0]


def write_lidar_bin(path: Path, xyz) -> None:
    xyz = np.asarray(xyz, dtype=np.float32)
    n = len(xyz)
    cols = np.zeros((n, 5), dtype=np.float32)
    cols[:, :3] = xyz
    cols[:, 3] = 0.5
    cols[:, 4] = np.arange(n, dtype=np.float32)
    path.parent.mkdir(parents=True, exist_ok=True)
    cols.tofile(path)


def build_mini_fixture(root: Path) -> Path:
    """
    One LIDAR_TOP sensor, three keyframes, one ignored sweep, one camera row.
    Ego translates +5 m in x each keyframe. Sensor offset (1, 0, 2).
    """

    meta = root / "v1.0-mini"
    meta.mkdir(parents=True, exist_ok=True)

    sensor = [
        {"token": "sen_lidar", "channel": "LIDAR_TOP", "modality": "lidar"},
        {"token": "sen_cam", "channel": "CAM_FRONT", "modality": "camera"},
    ]
    calibrated = [
        {
            "token": "cal_lidar",
            "sensor_token": "sen_lidar",
            "translation": [1.0, 0.0, 2.0],
            "rotation": IDENTITY_Q,
            "camera_intrinsic": [],
        },
        {
            "token": "cal_cam",
            "sensor_token": "sen_cam",
            "translation": [0.0, 0.0, 1.5],
            "rotation": IDENTITY_Q,
            "camera_intrinsic": [[1, 0, 0], [0, 1, 0], [0, 0, 1]],
        },
    ]

    ego_poses = []
    samples = []
    sample_data = []
    clouds = [
        [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]],
        [[0.0, 0.0, 0.0], [2.0, 0.0, 0.0]],
        [[0.0, 0.0, 0.0], [3.0, 0.0, 0.0]],
    ]

    sample_tokens = ["samp0", "samp1", "samp2"]
    for i, tok in enumerate(sample_tokens):
        prev_s = "" if i == 0 else sample_tokens[i - 1]
        next_s = "" if i == 2 else sample_tokens[i + 1]
        ts = 1_000_000 + i * 500_000
        samples.append(
            {
                "token": tok,
                "timestamp": ts,
                "prev": prev_s,
                "next": next_s,
                "scene_token": "sceneA",
            }
        )
        ego_token = f"ego{i}"
        ego_poses.append(
            {
                "token": ego_token,
                "timestamp": ts,
                "rotation": IDENTITY_Q,
                "translation": [float(i * 5), 0.0, 0.0],
            }
        )
        filename = f"samples/LIDAR_TOP/frame_{i}.pcd.bin"
        write_lidar_bin(root / filename, clouds[i])
        prev_sd = "" if i == 0 else f"sd{i - 1}"
        next_sd = "" if i == 2 else f"sd{i + 1}"
        sample_data.append(
            {
                "token": f"sd{i}",
                "sample_token": tok,
                "ego_pose_token": ego_token,
                "calibrated_sensor_token": "cal_lidar",
                "filename": filename,
                "fileformat": "pcd",
                "width": 0,
                "height": 0,
                "timestamp": ts,
                "is_key_frame": True,
                "prev": prev_sd,
                "next": next_sd,
            }
        )

    # Sweep attached to samp1 — must not enter the keyframe list.
    write_lidar_bin(
        root / "sweeps/LIDAR_TOP/sweep.pcd.bin",
        [[9.0, 9.0, 9.0]],
    )
    sample_data.append(
        {
            "token": "sd_sweep",
            "sample_token": "samp1",
            "ego_pose_token": "ego1",
            "calibrated_sensor_token": "cal_lidar",
            "filename": "sweeps/LIDAR_TOP/sweep.pcd.bin",
            "fileformat": "pcd",
            "width": 0,
            "height": 0,
            "timestamp": 1_250_000,
            "is_key_frame": False,
            "prev": "sd0",
            "next": "sd1",
        }
    )

    sample_data.append(
        {
            "token": "sd_cam",
            "sample_token": "samp0",
            "ego_pose_token": "ego0",
            "calibrated_sensor_token": "cal_cam",
            "filename": "samples/CAM_FRONT/cam.jpg",
            "fileformat": "jpg",
            "width": 16,
            "height": 16,
            "timestamp": 1_000_000,
            "is_key_frame": True,
            "prev": "",
            "next": "",
        }
    )

    scenes = [
        {
            "token": "sceneA",
            "log_token": "logA",
            "nbr_samples": 3,
            "first_sample_token": "samp0",
            "last_sample_token": "samp2",
            "name": "scene-fixture",
            "description": "synthetic fixture",
        }
    ]

    annotations = [
        {
            "token": "ann0",
            "sample_token": "samp0",
            "instance_token": "inst0",
            "translation": [99.0, 99.0, 99.0],
            "size": [1, 1, 1],
            "rotation": IDENTITY_Q,
            "category_name": "vehicle.car",
        }
    ]

    tables = {
        "sensor": sensor,
        "calibrated_sensor": calibrated,
        "ego_pose": ego_poses,
        "sample": samples,
        "sample_data": sample_data,
        "scene": scenes,
        "sample_annotation": annotations,
    }
    for name, rows in tables.items():
        (meta / f"{name}.json").write_text(json.dumps(rows), encoding="utf-8")

    return root
