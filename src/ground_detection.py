import open3d as o3d
import numpy as np


def detect_ground(point_cloud):
    """
    Detect the dominant ground plane using RANSAC.

    Returns:
        ground      -> points belonging to the plane
        non_ground  -> all remaining points
        plane_model -> [a, b, c, d]
    """

    plane_model, inliers = point_cloud.segment_plane(
        distance_threshold=0.08,
        ransac_n=3,
        num_iterations=1000
    )

    ground = point_cloud.select_by_index(inliers)

    non_ground = point_cloud.select_by_index(
        inliers,
        invert=True
    )

    return ground, non_ground, plane_model


def main():

    # Load our synthetic scene
    scene = o3d.io.read_point_cloud(
        "data/synthetic_scene.ply"
    )

    print(
        f"Loaded {len(scene.points):,} points"
    )

    # Detect ground
    ground, non_ground, plane = detect_ground(scene)

    print("\nDetected ground plane:")
    print(
        f"a={plane[0]:.4f}, "
        f"b={plane[1]:.4f}, "
        f"c={plane[2]:.4f}, "
        f"d={plane[3]:.4f}"
    )

    print(
        f"\nGround points: "
        f"{len(ground.points):,}"
    )

    print(
        f"Non-ground points: "
        f"{len(non_ground.points):,}"
    )

    # Color ground green
    ground.paint_uniform_color(
        [0.1, 0.8, 0.1]
    )

    # Color everything else red
    non_ground.paint_uniform_color(
        [0.9, 0.1, 0.1]
    )

    # Visualize
    o3d.visualization.draw_geometries(
        [ground, non_ground],
        window_name="ORBIT - Ground Detection"
    )


if __name__ == "__main__":
    main()