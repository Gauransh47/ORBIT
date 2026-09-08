"""
Range preprocessing for the end-to-end prototype.

Keeps AdaptiveGrid.MAX_RANGE (100 m) as the operating horizon.
"""

import numpy as np

from mapping.adaptive_grid import AdaptiveGrid


def filter_range(points, max_range=None):
    """
    Keep points with horizontal range < max_range.

    Returns
    -------
    filtered : ndarray
    mask : ndarray of bool
    """

    if max_range is None:
        max_range = AdaptiveGrid.MAX_RANGE

    points = np.asarray(points, dtype=np.float64)
    xyz = points[:, :3]

    distance_sq = xyz[:, 0] ** 2 + xyz[:, 1] ** 2
    mask = distance_sq < (max_range * max_range)

    return points[mask], mask
