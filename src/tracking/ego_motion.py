"""
ORBIT - EGO MOTION COMPENSATION

SemanticKITTI-aware ego-motion compensation.

Coordinate pipeline:

    LiDAR frame
        |
        | T_cam_velo
        v
    Camera coordinate frame
        |
        | T_world_cam (KITTI pose)
        v
    Persistent world/reference frame

The tracker should store object positions in the persistent
world coordinate system.

For a point p expressed in LiDAR coordinates:

    p_world = T_world_cam @ T_cam_velo @ p_lidar


For a point from source LiDAR frame B into target LiDAR
frame A:

    p_A =
        inv(T_world_cam_A)
        @ T_world_cam_B
        @ p_B_camera

with the LiDAR/camera calibration included.
"""

from pathlib import Path

import numpy as np


class EgoMotionCompensator:
    """
    Ego-motion compensation using SemanticKITTI poses
    and LiDAR/camera calibration.
    """

    # =========================================================
    # INITIALIZATION
    # =========================================================

    def __init__(
        self,
        dataset_root,
        sequence="00",
    ):

        self.root = Path(
            dataset_root
        )

        self.sequence = str(
            sequence
        ).zfill(2)

        self.sequence_dir = (
            self.root
            / "sequences"
            / self.sequence
        )

        self.poses_path = (
            self.sequence_dir
            / "poses.txt"
        )

        self.calib_path = (
            self.sequence_dir
            / "calib.txt"
        )

        if not self.poses_path.exists():

            raise FileNotFoundError(
                "KITTI pose file not found:\n"
                f"{self.poses_path}"
            )

        if not self.calib_path.exists():

            raise FileNotFoundError(
                "SemanticKITTI calibration file "
                "not found:\n"
                f"{self.calib_path}"
            )

        # Load camera/world poses.
        self.poses = (
            self._load_poses()
        )

        # Load LiDAR -> camera calibration.
        self.T_cam_velo = (
            self._load_velodyne_calibration()
        )

        # Camera -> LiDAR.
        self.T_velo_cam = (
            np.linalg.inv(
                self.T_cam_velo
            )
        )

    # =========================================================
    # POSE LOADING
    # =========================================================

    def _load_poses(self):

        raw = np.loadtxt(
            self.poses_path,
            dtype=np.float64,
        )

        if raw.ndim == 1:

            raw = raw.reshape(
                1,
                -1,
            )

        if raw.shape[1] != 12:

            raise ValueError(
                "Invalid KITTI pose file.\n"
                f"Expected 12 values per frame, "
                f"got {raw.shape[1]}."
            )

        poses = np.zeros(
            (
                raw.shape[0],
                4,
                4,
            ),
            dtype=np.float64,
        )

        poses[:, :3, :4] = (
            raw.reshape(
                -1,
                3,
                4,
            )
        )

        poses[:, 3, 3] = 1.0

        return poses

    # =========================================================
    # CALIBRATION LOADING
    # =========================================================

    def _load_velodyne_calibration(self):

        calibration = {}

        with open(
            self.calib_path,
            "r",
            encoding="utf-8",
        ) as file:

            for line in file:

                line = line.strip()

                if not line:
                    continue

                if ":" not in line:
                    continue

                key, values = (
                    line.split(
                        ":",
                        1,
                    )
                )

                values = np.fromstring(
                    values,
                    sep=" ",
                    dtype=np.float64,
                )

                calibration[
                    key.strip()
                ] = values

        # SemanticKITTI commonly uses:
        #
        #     Tr
        #
        # for the Velodyne -> camera transform.
        #
        # Some datasets/tools expose:
        #
        #     Tr_velo_to_cam
        #
        # so support both.

        if "Tr" in calibration:

            values = calibration[
                "Tr"
            ]

        elif "Tr_velo_to_cam" in calibration:

            values = calibration[
                "Tr_velo_to_cam"
            ]

        else:

            raise ValueError(
                "Could not find LiDAR -> camera "
                "calibration in calib.txt.\n"
                f"Available keys: "
                f"{list(calibration.keys())}"
            )

        if values.size != 12:

            raise ValueError(
                "Invalid LiDAR -> camera "
                "calibration.\n"
                f"Expected 12 values, "
                f"got {values.size}."
            )

        T = np.eye(
            4,
            dtype=np.float64,
        )

        T[:3, :4] = (
            values.reshape(
                3,
                4,
            )
        )

        return T

    # =========================================================
    # BASIC ACCESS
    # =========================================================

    def pose(
        self,
        frame,
    ):
        """
        Return KITTI pose for a frame.

        The pose maps camera coordinates of that
        frame into the persistent world/reference
        coordinate system.
        """

        frame = int(frame)

        if (
            frame < 0
            or frame >= len(self.poses)
        ):

            raise IndexError(
                f"Frame {frame} has no pose.\n"
                f"Available poses: "
                f"{len(self.poses)}"
            )

        return (
            self.poses[frame]
            .copy()
        )

    # =========================================================
    # LIDAR -> CAMERA
    # =========================================================

    def lidar_to_camera(
        self,
        point,
    ):
        """
        Transform one LiDAR XYZ point
        into camera coordinates.
        """

        point = np.asarray(
            point,
            dtype=np.float64,
        )

        if point.shape != (3,):

            raise ValueError(
                "Expected point with shape (3,), "
                f"got {point.shape}"
            )

        homogeneous = np.ones(
            4,
            dtype=np.float64,
        )

        homogeneous[:3] = point

        transformed = (
            self.T_cam_velo
            @ homogeneous
        )

        return transformed[:3]

    # =========================================================
    # CAMERA -> LIDAR
    # =========================================================

    def camera_to_lidar(
        self,
        point,
    ):
        """
        Transform one camera XYZ point
        into LiDAR coordinates.
        """

        point = np.asarray(
            point,
            dtype=np.float64,
        )

        if point.shape != (3,):

            raise ValueError(
                "Expected point with shape (3,), "
                f"got {point.shape}"
            )

        homogeneous = np.ones(
            4,
            dtype=np.float64,
        )

        homogeneous[:3] = point

        transformed = (
            self.T_velo_cam
            @ homogeneous
        )

        return transformed[:3]

    # =========================================================
    # LIDAR -> WORLD
    # =========================================================

    def lidar_to_world_transform(
        self,
        frame,
    ):
        """
        Return transform:

            LiDAR(frame) -> World
        """

        return (
            self.pose(frame)
            @ self.T_cam_velo
        )

    # =========================================================
    # WORLD -> LIDAR
    # =========================================================

    def world_to_lidar_transform(
        self,
        frame,
    ):
        """
        Return transform:

            World -> LiDAR(frame)
        """

        return np.linalg.inv(
            self.lidar_to_world_transform(
                frame
            )
        )

    # =========================================================
    # RELATIVE LIDAR TRANSFORM
    # =========================================================

    def transform(
        self,
        source_frame,
        target_frame,
    ):
        """
        Transform points from source LiDAR coordinates
        into target LiDAR coordinates.

        Example:

            transform(1, 0)

        means:

            LiDAR frame 1 -> LiDAR frame 0
        """

        source_world = (
            self.lidar_to_world_transform(
                source_frame
            )
        )

        target_world = (
            self.lidar_to_world_transform(
                target_frame
            )
        )

        return (
            np.linalg.inv(
                target_world
            )
            @ source_world
        )

    # =========================================================
    # POINT -> WORLD
    # =========================================================

    def transform_point(
        self,
        point,
        source_frame,
        target_frame=None,
    ):
        """
        Transform one point.

        If target_frame is None:

            LiDAR -> persistent world

        Otherwise:

            source LiDAR -> target LiDAR
        """

        point = np.asarray(
            point,
            dtype=np.float64,
        )

        if point.shape != (3,):

            raise ValueError(
                "Expected point with shape (3,), "
                f"got {point.shape}"
            )

        homogeneous = np.ones(
            4,
            dtype=np.float64,
        )

        homogeneous[:3] = point

        if target_frame is None:

            T = (
                self.lidar_to_world_transform(
                    source_frame
                )
            )

        else:

            T = self.transform(
                source_frame,
                target_frame,
            )

        transformed = (
            T
            @ homogeneous
        )

        return transformed[:3]

    # =========================================================
    # MULTIPLE POINTS
    # =========================================================

    def transform_points(
        self,
        points,
        source_frame,
        target_frame=None,
    ):
        """
        Transform an Nx3 array.

        target_frame=None:
            source LiDAR -> world

        target_frame specified:
            source LiDAR -> target LiDAR
        """

        points = np.asarray(
            points,
            dtype=np.float64,
        )

        if (
            points.ndim != 2
            or points.shape[1] != 3
        ):

            raise ValueError(
                "Expected Nx3 array, "
                f"got {points.shape}"
            )

        if target_frame is None:

            T = (
                self.lidar_to_world_transform(
                    source_frame
                )
            )

        else:

            T = self.transform(
                source_frame,
                target_frame,
            )

        ones = np.ones(
            (
                len(points),
                1,
            ),
            dtype=np.float64,
        )

        homogeneous = np.hstack(
            [
                points,
                ones,
            ]
        )

        transformed = (
            homogeneous
            @ T.T
        )

        return transformed[:, :3]

    # =========================================================
    # OBJECT POSITION
    # =========================================================

    def transform_position(
        self,
        position,
        source_frame,
        target_frame=None,
    ):
        """
        Convenience wrapper for object centers.
        """

        position = np.asarray(
            position,
            dtype=np.float64,
        )

        if position.shape == (2,):

            return (
                self.lidar_to_world_ground(
                    position,
                    source_frame,
                )
            )

        if position.shape == (3,):

            return (
                self.transform_point(
                    position,
                    source_frame,
                    target_frame,
                )
            )

        raise ValueError(
            "Expected position shape (2,) or (3,), "
            f"got {position.shape}"
        )

    # =========================================================
    # GROUND PLANE
    # =========================================================

    def lidar_to_world_ground(
        self,
        position,
        source_frame,
        lidar_z=0.0,
    ):
        """
        Convert a LiDAR XY position into persistent
        world horizontal coordinates.

        LiDAR:
            X = forward
            Y = left/right
            Z = vertical

        Camera/world:
            X = horizontal
            Y = vertical
            Z = depth

        Therefore we represent the world ground plane
        as:

            [world_X, world_Z]
        """

        position = np.asarray(
            position,
            dtype=np.float64,
        )

        if position.shape != (2,):

            raise ValueError(
                "Expected XY position with shape (2,), "
                f"got {position.shape}"
            )

        lidar_point = np.array(
            [
                position[0],
                position[1],
                lidar_z,
            ],
            dtype=np.float64,
        )

        world_point = (
            self.transform_point(
                lidar_point,
                source_frame=source_frame,
                target_frame=None,
            )
        )

        return np.array(
            [
                world_point[0],
                world_point[2],
            ],
            dtype=np.float64,
        )

    # =========================================================
    # TRANSLATION
    # =========================================================

    def translation(
        self,
        source_frame,
        target_frame,
    ):
        """
        Translation component of:

            source LiDAR -> target LiDAR
        """

        T = self.transform(
            source_frame,
            target_frame,
        )

        return (
            T[:3, 3]
            .copy()
        )

    # =========================================================
    # ROTATION
    # =========================================================

    def rotation(
        self,
        source_frame,
        target_frame,
    ):
        """
        Rotation component of:

            source LiDAR -> target LiDAR
        """

        T = self.transform(
            source_frame,
            target_frame,
        )

        return (
            T[:3, :3]
            .copy()
        )

    # =========================================================
    # VEHICLE DISPLACEMENT
    # =========================================================

    def vehicle_displacement(
        self,
        previous_frame,
        current_frame,
    ):
        """
        Return vehicle displacement between frames
        in the previous frame's LiDAR coordinate system.

        This is NOT object motion.

        It describes how the sensor/vehicle moved.
        """

        T = self.transform(
            current_frame,
            previous_frame,
        )

        return (
            T[:3, 3]
            .copy()
        )

    # =========================================================
    # SUMMARY
    # =========================================================

    def summary(
        self,
        previous_frame,
        current_frame,
    ):
        """
        Return useful ego-motion information.
        """

        T = self.transform(
            current_frame,
            previous_frame,
        )

        translation = (
            T[:3, 3]
        )

        return {
            "source_frame": int(
                current_frame
            ),
            "target_frame": int(
                previous_frame
            ),
            "translation": translation,
            "distance": float(
                np.linalg.norm(
                    translation
                )
            ),
            "rotation": (
                T[:3, :3]
                .copy()
            ),
            "transform": T,
        }


# =============================================================
# STANDALONE TEST
# =============================================================

def main():

    print("=" * 70)
    print(
        "ORBIT - EGO MOTION COMPENSATION"
    )
    print("=" * 70)

    ego = EgoMotionCompensator(
        "data/semantic_kitti",
        sequence="00",
    )

    print(
        f"\nLoaded poses: "
        f"{len(ego.poses)}"
    )

    print(
        "\nLiDAR -> Camera calibration:"
    )

    print(
        ego.T_cam_velo
    )

    print(
        "\nCamera -> LiDAR calibration:"
    )

    print(
        ego.T_velo_cam
    )

    # ---------------------------------------------------------
    # Pose 0
    # ---------------------------------------------------------

    print(
        "\nFrame 0 pose:"
    )

    print(
        ego.pose(0)
    )

    # ---------------------------------------------------------
    # Pose 1
    # ---------------------------------------------------------

    print(
        "\nFrame 1 pose:"
    )

    print(
        ego.pose(1)
    )

    # ---------------------------------------------------------
    # LiDAR -> World
    # ---------------------------------------------------------

    print(
        "\nFrame 0 LiDAR -> World:"
    )

    print(
        ego.lidar_to_world_transform(
            0
        )
    )

    print(
        "\nFrame 1 LiDAR -> World:"
    )

    print(
        ego.lidar_to_world_transform(
            1
        )
    )

    # ---------------------------------------------------------
    # Relative transform
    # ---------------------------------------------------------

    print(
        "\nTransform: frame 1 -> frame 0"
    )

    T = ego.transform(
        1,
        0,
    )

    print(T)

    print(
        "\nTransform translation:"
    )

    print(
        T[:3, 3]
    )

    print(
        "\nTransform translation magnitude:"
    )

    print(
        f"{np.linalg.norm(T[:3, 3]):.4f} m"
    )

    # ---------------------------------------------------------
    # Vehicle displacement
    # ---------------------------------------------------------

    displacement = (
        ego.vehicle_displacement(
            0,
            1,
        )
    )

    print(
        "\nVehicle displacement "
        "(frame 0 coordinates):"
    )

    print(
        displacement
    )

    print(
        "\nVehicle displacement magnitude:"
    )

    print(
        f"{np.linalg.norm(displacement):.4f} m"
    )

    # ---------------------------------------------------------
    # Point transformation
    # ---------------------------------------------------------

    point = np.array(
        [
            10.0,
            0.0,
            0.0,
        ]
    )

    transformed = (
        ego.transform_point(
            point,
            source_frame=1,
            target_frame=0,
        )
    )

    print(
        "\nPoint transformation:"
    )

    print(
        f"Frame 1: {point}"
    )

    print(
        f"Frame 0: {transformed}"
    )

    # ---------------------------------------------------------
    # World point test
    # ---------------------------------------------------------

    world0 = (
        ego.transform_point(
            point,
            source_frame=0,
            target_frame=None,
        )
    )

    world1 = (
        ego.transform_point(
            point,
            source_frame=1,
            target_frame=None,
        )
    )

    print(
        "\nWorld transformation:"
    )

    print(
        f"Frame 0 LiDAR -> World: "
        f"{world0}"
    )

    print(
        f"Frame 1 LiDAR -> World: "
        f"{world1}"
    )

    # ---------------------------------------------------------
    # Ground representation
    # ---------------------------------------------------------

    ground0 = (
        ego.lidar_to_world_ground(
            point[:2],
            source_frame=0,
        )
    )

    ground1 = (
        ego.lidar_to_world_ground(
            point[:2],
            source_frame=1,
        )
    )

    print(
        "\nWorld ground coordinates:"
    )

    print(
        f"Frame 0: {ground0}"
    )

    print(
        f"Frame 1: {ground1}"
    )

    # ---------------------------------------------------------
    # Identity test
    # ---------------------------------------------------------

    identity = (
        ego.transform(
            0,
            0,
        )
    )

    identity_error = np.max(
        np.abs(
            identity
            - np.eye(4)
        )
    )

    print(
        "\nIdentity check:"
    )

    print(identity)

    print(
        "\nIdentity error:"
    )

    print(
        f"{identity_error:.8e}"
    )

    # ---------------------------------------------------------
    # Summary
    # ---------------------------------------------------------

    print(
        "\nMotion summary:"
    )

    print(
        ego.summary(
            0,
            1,
        )
    )

    print(
        "\n" + "=" * 70
    )

    print(
        "EGO MOTION MODULE READY"
    )

    print(
        "=" * 70
    )


if __name__ == "__main__":
    main()