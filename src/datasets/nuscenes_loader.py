"""
nuScenes v1.0-mini LiDAR loader for ORBIT.

Reads official JSON tables and LIDAR_TOP `.pcd.bin` blobs. The dataset
root is external (never copied into this repository).

Keyframe-only playback: walk sample.next from scene.first_sample_token
to last_sample_token and take each sample's keyframe LIDAR_TOP.

ORBIT world remains the first processed LiDAR frame, not nuScenes global.

Does not load sample_annotation.json or any other GT boxes.
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any, Dict, List, Optional

import numpy as np

LIDAR_CHANNEL = "LIDAR_TOP"
DEFAULT_VERSION = "v1.0-mini"


def quaternion_wxyz_to_rotation(quat) -> np.ndarray:
    """nuScenes quaternion (w, x, y, z) → 3x3 rotation matrix."""

    w, x, y, z = np.asarray(quat, dtype=np.float64).reshape(4)
    n = float(np.sqrt(w * w + x * x + y * y + z * z))
    if n == 0.0:
        raise ValueError("zero-length quaternion")
    w, x, y, z = w / n, x / n, y / n, z / n
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def transform_from_rotation_translation(rotation, translation) -> np.ndarray:
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = np.asarray(rotation, dtype=np.float64)
    T[:3, 3] = np.asarray(translation, dtype=np.float64).reshape(3)
    return T


def pose_to_transform(record: Dict[str, Any]) -> np.ndarray:
    """ego_pose or calibrated_sensor row → 4x4 (child → parent)."""

    return transform_from_rotation_translation(
        quaternion_wxyz_to_rotation(record["rotation"]),
        record["translation"],
    )


def load_lidar_pcd_bin(path) -> np.ndarray:
    """
    nuScenes LIDAR_TOP `.pcd.bin`: float32, five columns
    x, y, z, intensity, ring_index.
    """

    path = Path(path)
    raw = np.fromfile(path, dtype=np.float32)
    if raw.size % 5 != 0:
        raise ValueError(
            f"Invalid nuScenes lidar file {path}: "
            f"expected multiples of 5 float32 values, got {raw.size}"
        )
    return raw.reshape(-1, 5)


class NuScenesMiniLoader:
    """
    Integer-frame LIDAR_TOP access over a nuScenes extract.

    Parameters
    ----------
    dataroot : path to the extract (contains samples/, sweeps/, v1.0-mini/)
    scene_name : optional scene to select immediately
    version : metadata folder name, default v1.0-mini
    """

    def __init__(
        self,
        dataroot,
        scene_name: Optional[str] = None,
        version: str = DEFAULT_VERSION,
    ):
        self.dataroot = Path(dataroot)
        self.version = str(version)
        self.meta_dir = self.dataroot / self.version

        if not self.meta_dir.is_dir():
            raise FileNotFoundError(
                f"nuScenes metadata not found:\n{self.meta_dir}\n"
                "Pass --dataset-root at the extract (e.g. D:\\ORBIT_DATA\\nuscenes)."
            )

        self._tables = {
            "scene": self._load_table("scene"),
            "sample": self._load_table("sample"),
            "sample_data": self._load_table("sample_data"),
            "ego_pose": self._load_table("ego_pose"),
            "calibrated_sensor": self._load_table("calibrated_sensor"),
            "sensor": self._load_table("sensor"),
        }

        self._channel_by_calibrated = {}
        for token, calib in self._tables["calibrated_sensor"].items():
            sensor = self._tables["sensor"].get(calib["sensor_token"])
            if sensor is None:
                continue
            self._channel_by_calibrated[token] = sensor.get("channel")

        self._scenes_by_name = {
            row["name"]: row for row in self._tables["scene"].values()
        }

        self.scene_name: Optional[str] = None
        self._frames: List[Dict[str, Any]] = []
        if scene_name is not None:
            self.select_scene(scene_name)

    def _load_table(self, name: str) -> Dict[str, Dict[str, Any]]:
        path = self.meta_dir / f"{name}.json"
        if not path.exists():
            raise FileNotFoundError(f"Missing nuScenes table: {path}")
        rows = json.loads(path.read_text(encoding="utf-8"))
        return {row["token"]: row for row in rows}

    def list_scenes(self) -> List[str]:
        return sorted(self._scenes_by_name)

    def scene_info(self, scene_name: str) -> Dict[str, Any]:
        if scene_name not in self._scenes_by_name:
            raise KeyError(
                f"Unknown scene {scene_name!r}. Available: {self.list_scenes()}"
            )
        row = self._scenes_by_name[scene_name]
        frames = self._keyframe_records(scene_name)
        return {
            "name": row["name"],
            "token": row["token"],
            "description": row.get("description", ""),
            "nbr_samples": int(row.get("nbr_samples", len(frames))),
            "first_sample_token": row["first_sample_token"],
            "last_sample_token": row["last_sample_token"],
            "nbr_lidar_keyframes": len(frames),
        }

    def select_scene(self, scene_name: str) -> None:
        self.scene_info(scene_name)
        self.scene_name = scene_name
        self._frames = self._keyframe_records(scene_name)

    def frame_ids(self, scene_name: Optional[str] = None) -> List[int]:
        records = self._records(scene_name)
        return list(range(len(records)))

    def num_frames(self, scene_name: Optional[str] = None) -> int:
        return len(self._records(scene_name))

    def lidar_record(self, frame_index: int, scene_name: Optional[str] = None) -> Dict[str, Any]:
        records = self._records(scene_name)
        idx = int(frame_index)
        if idx < 0 or idx >= len(records):
            raise IndexError(
                f"LiDAR frame {idx} out of range 0..{len(records) - 1}"
            )
        return records[idx]

    def load_xyz(self, frame_index: int, scene_name: Optional[str] = None) -> np.ndarray:
        points = load_lidar_pcd_bin(self.lidar_path(frame_index, scene_name))
        return np.asarray(points[:, :3], dtype=np.float64)

    def load_points(self, frame_index: int, scene_name: Optional[str] = None) -> np.ndarray:
        """Full five-column lidar array (not used by perception)."""

        return load_lidar_pcd_bin(self.lidar_path(frame_index, scene_name))

    def lidar_path(self, frame_index: int, scene_name: Optional[str] = None) -> Path:
        record = self.lidar_record(frame_index, scene_name)
        path = self.dataroot / record["filename"]
        if not path.exists():
            raise FileNotFoundError(f"LIDAR_TOP file not found:\n{path}")
        return path

    def timestamp_us(self, frame_index: int, scene_name: Optional[str] = None) -> int:
        return int(self.lidar_record(frame_index, scene_name)["timestamp"])

    def timestamps_seconds(self, scene_name: Optional[str] = None) -> np.ndarray:
        records = self._records(scene_name)
        return np.asarray(
            [int(row["timestamp"]) / 1e6 for row in records],
            dtype=np.float64,
        )

    def global_pose(self, frame_index: int, scene_name: Optional[str] = None) -> Dict[str, Any]:
        """ego_pose row for this LIDAR_TOP keyframe (vehicle in global)."""

        record = self.lidar_record(frame_index, scene_name)
        pose = self._tables["ego_pose"][record["ego_pose_token"]]
        return pose

    def calibrated_sensor(self, frame_index: int, scene_name: Optional[str] = None) -> Dict[str, Any]:
        record = self.lidar_record(frame_index, scene_name)
        return self._tables["calibrated_sensor"][record["calibrated_sensor_token"]]

    def T_ego_from_lidar(self, frame_index: int, scene_name: Optional[str] = None) -> np.ndarray:
        return pose_to_transform(self.calibrated_sensor(frame_index, scene_name))

    def T_global_from_ego(self, frame_index: int, scene_name: Optional[str] = None) -> np.ndarray:
        return pose_to_transform(self.global_pose(frame_index, scene_name))

    def T_global_from_lidar(self, frame_index: int, scene_name: Optional[str] = None) -> np.ndarray:
        return (
            self.T_global_from_ego(frame_index, scene_name)
            @ self.T_ego_from_lidar(frame_index, scene_name)
        )

    def lidar_to_lidar(
        self,
        source_frame: int,
        target_frame: int,
        scene_name: Optional[str] = None,
    ) -> np.ndarray:
        """Map points from source LIDAR_TOP into target LIDAR_TOP."""

        T_src = self.T_global_from_lidar(source_frame, scene_name)
        T_dst = self.T_global_from_lidar(target_frame, scene_name)
        return np.linalg.inv(T_dst) @ T_src

    def T_frame0_from_current(self, frame_index: int, scene_name: Optional[str] = None) -> np.ndarray:
        return self.lidar_to_lidar(frame_index, 0, scene_name)

    def _records(self, scene_name: Optional[str]) -> List[Dict[str, Any]]:
        if scene_name is None:
            if not self._frames:
                raise RuntimeError("No scene selected. Call select_scene(name) first.")
            return self._frames
        return self._keyframe_records(scene_name)

    def _keyframe_records(self, scene_name: str) -> List[Dict[str, Any]]:
        """
        Ordered LIDAR_TOP keyframes for a scene.

        Sweep playback can later walk sample_data prev/next instead of
        sample.next; this method stays keyframe-only.
        """

        scene = self._scenes_by_name[scene_name]
        ordered = []
        sample_token = scene["first_sample_token"]
        seen = set()
        last = scene["last_sample_token"]

        while sample_token:
            if sample_token in seen:
                raise ValueError(f"Cycle in sample.next for scene {scene_name}")
            seen.add(sample_token)
            if sample_token not in self._tables["sample"]:
                raise KeyError(f"Missing sample token {sample_token}")
            lidar = self._keyframe_lidar_for_sample(sample_token)
            ordered.append(lidar)
            if sample_token == last:
                break
            nxt = self._tables["sample"][sample_token].get("next") or ""
            if not nxt:
                break
            sample_token = nxt

        return ordered

    def _keyframe_lidar_for_sample(self, sample_token: str) -> Dict[str, Any]:
        matches = []
        for row in self._tables["sample_data"].values():
            if row.get("sample_token") != sample_token:
                continue
            if not row.get("is_key_frame"):
                continue
            channel = self._channel_by_calibrated.get(row.get("calibrated_sensor_token"))
            if channel != LIDAR_CHANNEL:
                continue
            matches.append(row)
        if not matches:
            raise KeyError(
                f"No keyframe {LIDAR_CHANNEL} sample_data for sample {sample_token}"
            )
        matches.sort(key=lambda r: int(r["timestamp"]))
        return matches[0]
