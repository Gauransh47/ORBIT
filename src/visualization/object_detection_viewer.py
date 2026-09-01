import sys
from pathlib import Path

import numpy as np
import open3d as o3d


# Add ORBIT's src directory to Python's import path
SRC_DIR = Path(__file__).resolve().parents[1]

if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))


from perception.object_detection import (
    ObjectDetector,
    detect_ground
)


# ============================================================
# ORBIT - OBJECT DETECTION VIEWER
# ============================================================

def create_bounding_box(proposal):
    """
    Create an Open3D oriented bounding box for an object.
    """

    points = o3d.utility.Vector3dVector(
        proposal.points
    )

    bbox = o3d.geometry.AxisAlignedBoundingBox.create_from_points(
        points
    )

    return bbox


def create_label_geometry(proposal):
    """
    Create a small coordinate frame at the object's center.

    This gives us a visible reference point without relying
    on external text-rendering libraries.
    """

    frame = o3d.geometry.TriangleMesh.create_coordinate_frame(
        size=0.5,
        origin=proposal.center
    )

    return frame


def main():

    print("=" * 70)
    print(
        "ORBIT - 3D OBJECT DETECTION VIEWER"
    )
    print("=" * 70)

    # --------------------------------------------------------
    # Load point cloud
    # --------------------------------------------------------

    cloud = o3d.io.read_point_cloud(
        "data/synthetic_scene.ply"
    )

    points = np.asarray(
        cloud.points
    )

    print(
        f"\nInput points: "
        f"{len(points):,}"
    )

    # --------------------------------------------------------
    # Limit visualization to ORBIT's 100m range
    # --------------------------------------------------------

    distance_squared = (
        points[:, 0] ** 2
        + points[:, 1] ** 2
    )

    range_mask = (
        distance_squared < 100.0 ** 2
    )

    points_100m = points[
        range_mask
    ]

    print(
        f"Points within 100m: "
        f"{len(points_100m):,}"
    )

    # --------------------------------------------------------
    # Ground detection
    # --------------------------------------------------------

    print(
        "\nDetecting ground..."
    )

    ground_mask_full = detect_ground(
        points_100m
    )

    ground_points = points_100m[
        ground_mask_full
    ]

    non_ground_points = points_100m[
        ~ground_mask_full
    ]

    print(
        f"Ground points: "
        f"{len(ground_points):,}"
    )

    print(
        f"Non-ground points: "
        f"{len(non_ground_points):,}"
    )

    # --------------------------------------------------------
    # Object detection
    # --------------------------------------------------------

    print(
        "\nDetecting objects..."
    )

    detector = ObjectDetector(
        cluster_tolerance=0.35,
        min_points=30,
        max_range=100.0
    )

    proposals = detector.detect(
        non_ground_points
    )

    print(
        f"Detected objects: "
        f"{len(proposals)}"
    )

    # ========================================================
    # Prepare visualization
    # ========================================================

    geometries = []

    # --------------------------------------------------------
    # Ground cloud
    # --------------------------------------------------------

    ground_cloud = o3d.geometry.PointCloud()

    ground_cloud.points = (
        o3d.utility.Vector3dVector(
            ground_points
        )
    )

    # Ground gets a neutral gray appearance.
    ground_cloud.paint_uniform_color(
        [0.55, 0.55, 0.55]
    )

    geometries.append(
        ground_cloud
    )

    # --------------------------------------------------------
    # Object points
    # --------------------------------------------------------

    non_ground_cloud = o3d.geometry.PointCloud()

    non_ground_cloud.points = (
        o3d.utility.Vector3dVector(
            non_ground_points
        )
    )

    non_ground_cloud.paint_uniform_color(
        [0.15, 0.45, 0.90]
    )

    geometries.append(
        non_ground_cloud
    )

    # --------------------------------------------------------
    # Bounding boxes
    # --------------------------------------------------------

    for proposal in proposals:

        bbox = create_bounding_box(
            proposal
        )

        # ----------------------------------------------------
        # Color based on semantic class
        # ----------------------------------------------------

        if proposal.classification == "POLE":

            bbox.color = (
                1.0,
                0.1,
                0.1
            )

        elif proposal.classification == "WALL":

            bbox.color = (
                1.0,
                0.7,
                0.1
            )

        elif proposal.classification == "VEHICLE_LIKE":

            bbox.color = (
                0.1,
                1.0,
                0.2
            )

        else:

            bbox.color = (
                0.1,
                0.8,
                1.0
            )

        geometries.append(
            bbox
        )

        # ----------------------------------------------------
        # Add coordinate frame at object center
        # ----------------------------------------------------

        frame = create_label_geometry(
            proposal
        )

        geometries.append(
            frame
        )

    # ========================================================
    # Visualization
    # ========================================================

    print(
        "\nOpening 3D viewer..."
    )

    print(
        "\nBounding-box colors:"
    )

    print(
        "  RED    = POLE"
    )

    print(
        "  ORANGE = WALL"
    )

    print(
        "  GREEN  = VEHICLE-LIKE"
    )

    print(
        "  CYAN   = OBSTACLE"
    )

    o3d.visualization.draw_geometries(
        geometries,
        window_name=(
            "ORBIT - Object Detection"
        ),
        width=1400,
        height=900
    )

    print(
        "\nORBIT object visualization complete."
    )


if __name__ == "__main__":
    main()