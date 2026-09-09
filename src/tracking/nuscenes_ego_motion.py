"""
nuScenes ego-motion for ORBIT.

Duck-types EgoMotionCompensator methods used by OrbitSystem and
OrbitTracker. Does not modify the KITTI compensator.

World / reference frame is LiDAR keyframe 0 of the selected scene,
not the nuScenes global frame.
"""

from __future__ import annotations

import numpy as np

from datasets.nuscenes_loader import NuScenesMiniLoader


class NuScenesEgoMotion:
    pose_source = "nuscenes_ego_pose_lidar_to_frame0"

    def __init__(self, loader: NuScenesMiniLoader):
        self.loader = loader
        n = loader.num_frames()
        self.poses = [
            loader.T_global_from_lidar(i) for i in range(n)
        ]
        self.timestamps = loader.timestamps_seconds()

    def transform(self, source_frame, target_frame):
        """source LIDAR_TOP → target LIDAR_TOP (ORBIT uses target=0)."""

        return self.loader.lidar_to_lidar(int(source_frame), int(target_frame))

    def transform_point(self, point, source_frame, target_frame=None):
        point = np.asarray(point, dtype=np.float64)
        if point.shape != (3,):
            raise ValueError(f"Expected point with shape (3,), got {point.shape}")
        if target_frame is None:
            T = self.loader.T_global_from_lidar(int(source_frame))
        else:
            T = self.transform(source_frame, target_frame)
        hom = np.ones(4, dtype=np.float64)
        hom[:3] = point
        return (T @ hom)[:3]

    def transform_points(self, points, source_frame, target_frame=None):
        points = np.asarray(points, dtype=np.float64)
        if points.ndim != 2 or points.shape[1] != 3:
            raise ValueError(f"Expected Nx3 array, got {points.shape}")
        if target_frame is None:
            T = self.loader.T_global_from_lidar(int(source_frame))
        else:
            T = self.transform(source_frame, target_frame)
        ones = np.ones((len(points), 1), dtype=np.float64)
        hom = np.hstack([points, ones])
        return (hom @ T.T)[:, :3]
