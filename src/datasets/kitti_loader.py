from pathlib import Path

import numpy as np


class KittiLoader:
    """
    Loader for KITTI / SemanticKITTI-style LiDAR sequences.

    Each .bin file:
        x, y, z, intensity

    Each .label file:
        lower 16 bits -> semantic class
        upper 16 bits -> instance ID

    poses.txt contains one 3x4 pose matrix per frame.
    """

    def __init__(self, dataset_root, sequence="00"):
        self.root = Path(dataset_root)
        self.sequence = str(sequence).zfill(2)

        self.sequence_dir = (
            self.root
            / "sequences"
            / self.sequence
        )

        self.velodyne_dir = (
            self.sequence_dir
            / "velodyne"
        )

        self.label_dir = (
            self.sequence_dir
            / "labels"
        )

        self.poses_path = (
            self.sequence_dir
            / "poses.txt"
        )

        if not self.velodyne_dir.exists():
            raise FileNotFoundError(
                f"Velodyne directory not found:\n"
                f"{self.velodyne_dir}"
            )

    # ---------------------------------------------------------
    # Frame discovery
    # ---------------------------------------------------------

    def frame_ids(self):
        return sorted(
            path.stem
            for path in self.velodyne_dir.glob("*.bin")
            if not path.name.startswith("._")
        )

    # ---------------------------------------------------------
    # Point cloud
    # ---------------------------------------------------------

    def load_points(self, frame_id):

        frame_id = str(frame_id).zfill(6)

        path = (
            self.velodyne_dir
            / f"{frame_id}.bin"
        )

        if not path.exists():
            raise FileNotFoundError(
                f"Point cloud not found:\n{path}"
            )

        points = np.fromfile(
            path,
            dtype=np.float32
        )

        if points.size % 4 != 0:
            raise ValueError(
                f"Invalid KITTI point cloud: {path}\n"
                f"Expected multiples of 4 float32 values, "
                f"got {points.size}."
            )

        return points.reshape(-1, 4)

    # ---------------------------------------------------------
    # Labels
    # ---------------------------------------------------------

    def load_labels(self, frame_id):

        frame_id = str(frame_id).zfill(6)

        path = (
            self.label_dir
            / f"{frame_id}.label"
        )

        if not path.exists():
            raise FileNotFoundError(
                f"Label file not found:\n{path}"
            )

        labels = np.fromfile(
            path,
            dtype=np.uint32
        )

        semantic_ids = (
            labels & np.uint32(0xFFFF)
        ).astype(np.uint32)

        instance_ids = (
            labels >> np.uint32(16)
        ).astype(np.uint32)

        return (
            semantic_ids,
            instance_ids
        )

    # ---------------------------------------------------------
    # Complete frame
    # ---------------------------------------------------------

    def load_frame(
        self,
        frame_id,
        load_labels=True
    ):

        points = self.load_points(
            frame_id
        )

        semantic_ids = None
        instance_ids = None

        if load_labels:

            try:

                (
                    semantic_ids,
                    instance_ids
                ) = self.load_labels(
                    frame_id
                )

            except FileNotFoundError:

                semantic_ids = None
                instance_ids = None

        if semantic_ids is not None:

            if len(points) != len(
                semantic_ids
            ):

                raise ValueError(
                    f"Point/label count mismatch "
                    f"for frame {frame_id}: "
                    f"{len(points)} points vs "
                    f"{len(semantic_ids)} labels"
                )

        return (
            points,
            semantic_ids,
            instance_ids
        )

    # ---------------------------------------------------------
    # ORBIT-compatible XYZ
    # ---------------------------------------------------------

    def load_xyz(self, frame_id):

        points = self.load_points(
            frame_id
        )

        return points[:, :3]

    # =========================================================
    # POSE SUPPORT
    # =========================================================

    def load_poses(self):
        """
        Load all KITTI poses.

        Returns
        -------
        poses : ndarray
            Shape (N, 4, 4)

        Each pose transforms coordinates from
        the LiDAR/camera frame into the KITTI
        world coordinate frame.
        """

        if not self.poses_path.exists():

            raise FileNotFoundError(
                f"Pose file not found:\n"
                f"{self.poses_path}"
            )

        raw = np.loadtxt(
            self.poses_path,
            dtype=np.float64
        )

        if raw.ndim == 1:

            raw = raw.reshape(1, -1)

        if raw.shape[1] != 12:

            raise ValueError(
                f"Invalid KITTI poses file:\n"
                f"Expected 12 values per row, "
                f"got {raw.shape[1]}"
            )

        poses = np.zeros(
            (raw.shape[0], 4, 4),
            dtype=np.float64
        )

        poses[:, :3, :4] = (
            raw.reshape(-1, 3, 4)
        )

        poses[:, 3, 3] = 1.0

        return poses

    # ---------------------------------------------------------

    def load_pose(self, frame_id):
        """
        Return the world pose of one frame.
        """

        frame_id = int(frame_id)

        poses = self.load_poses()

        if frame_id < 0 or frame_id >= len(poses):

            raise IndexError(
                f"Pose frame {frame_id} unavailable. "
                f"Available poses: {len(poses)}"
            )

        return poses[frame_id]

    # ---------------------------------------------------------

    def relative_pose(
        self,
        previous_frame,
        current_frame
    ):
        """
        Return the transform that maps a point
        expressed in the CURRENT LiDAR frame
        into the PREVIOUS LiDAR frame.

        This is the transform needed for
        ego-motion compensation.

        T_prev_current =
            inv(T_world_prev) @ T_world_current
        """

        previous_pose = self.load_pose(
            previous_frame
        )

        current_pose = self.load_pose(
            current_frame
        )

        relative = (
            np.linalg.inv(previous_pose)
            @ current_pose
        )

        return relative

    # ---------------------------------------------------------

    def ego_translation(
        self,
        previous_frame,
        current_frame
    ):
        """
        Return the sensor translation between
        two frames in the previous-frame coordinate
        system.
        """

        relative = self.relative_pose(
            previous_frame,
            current_frame
        )

        return relative[:3, 3]


# ----------------------------------------------------------------
# SemanticKITTI NATIVE CLASS NAMES
# ----------------------------------------------------------------

SEMANTIC_CLASSES = {

    0: "unlabeled",
    1: "outlier",

    10: "car",
    11: "bicycle",
    13: "bus",
    15: "motorcycle",
    16: "on-rails",
    18: "truck",
    20: "other-vehicle",

    30: "person",
    31: "bicyclist",
    32: "motorcyclist",

    40: "road",
    44: "parking",
    48: "sidewalk",
    49: "other-ground",

    50: "building",
    51: "fence",
    52: "other-structure",

    60: "lane-marking",
    70: "vegetation",
    71: "trunk",
    72: "terrain",

    80: "pole",
    81: "traffic-sign",
    99: "other-object",

    252: "moving-car",
    253: "moving-bicyclist",
    254: "moving-person",
    255: "moving-motorcyclist",
    256: "moving-on-rails",
    257: "moving-bus",
    258: "moving-truck",
    259: "moving-other-vehicle",
}


def semantic_name(class_id):

    return SEMANTIC_CLASSES.get(
        int(class_id),
        f"class-{class_id}"
    )