"""
Minimal sequence-loader protocol for ORBIT.

Perception, mapping, and tracking consume integer frames and XYZ.
They must not see nuScenes tokens or KITTI filenames.
"""

from typing import Protocol, runtime_checkable

import numpy as np


@runtime_checkable
class LidarSequence(Protocol):
    """Keyframe (or equivalent) LiDAR sequence as 0..N-1 frames."""

    def frame_ids(self) -> list:
        """Integer frame indices in playback order."""

    def load_xyz(self, frame_index: int) -> np.ndarray:
        """Current-sensor XYZ, shape (N, 3)."""

    def timestamp_us(self, frame_index: int) -> int:
        """Acquisition timestamp in microseconds."""
