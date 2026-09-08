"""
Canonical ground / terrain estimation.

AdaptiveGrid must not estimate ground. Callers pass the resulting mask
into AdaptiveGrid.build.

This module reuses the geometric pipeline's Open3D RANSAC plane
(8 cm inlier distance, 1000 iterations) so synthetic and KITTI XYZ
clouds share one estimator.

SemanticKITTI labels are not used.
"""

from dataclasses import dataclass
from typing import Optional

import numpy as np
import open3d as o3d


DEFAULT_DISTANCE_THRESHOLD = 0.08
DEFAULT_RANSAC_N = 3
DEFAULT_RANSAC_ITERATIONS = 1000


@dataclass
class GroundEstimate:
    """Frame-level ground representation."""

    mask: np.ndarray
    plane_model: Optional[np.ndarray]
    method: str
    inlier_count: int

    def plane_elevation(self, x, y):
        """
        Height of the fitted plane at (x, y), or None if no plane.
        Plane is ax + by + cz + d = 0.
        """

        if self.plane_model is None:
            return None

        a, b, c, d = self.plane_model
        if abs(c) < 1e-9:
            return None

        return float(-(a * x + b * y + d) / c)


def estimate_ground(
    points,
    distance_threshold=DEFAULT_DISTANCE_THRESHOLD,
    ransac_n=DEFAULT_RANSAC_N,
    num_iterations=DEFAULT_RANSAC_ITERATIONS,
):
    """
    Estimate a dominant ground plane with RANSAC.

    Parameters
    ----------
    points : ndarray, shape (N, 3)

    Returns
    -------
    GroundEstimate
        mask aligned with `points` (True = ground inlier).
    """

    points = np.asarray(points, dtype=np.float64)

    if points.ndim != 2 or points.shape[1] < 3:
        raise ValueError("points must have shape (N, 3)")

    xyz = points[:, :3]
    n = len(xyz)

    if n < ransac_n:
        return GroundEstimate(
            mask=np.zeros(n, dtype=bool),
            plane_model=None,
            method="ransac",
            inlier_count=0,
        )

    cloud = o3d.geometry.PointCloud()
    cloud.points = o3d.utility.Vector3dVector(xyz)

    plane_model, inliers = cloud.segment_plane(
        distance_threshold=distance_threshold,
        ransac_n=ransac_n,
        num_iterations=num_iterations,
    )

    mask = np.zeros(n, dtype=bool)
    mask[inliers] = True

    return GroundEstimate(
        mask=mask,
        plane_model=np.asarray(plane_model, dtype=np.float64),
        method="ransac",
        inlier_count=int(mask.sum()),
    )
