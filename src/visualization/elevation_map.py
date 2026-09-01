import sys
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt
import open3d as o3d

# Allow imports from src
sys.path.append(
    str(Path(__file__).resolve().parents[1])
)

from mapping.adaptive_grid import AdaptiveGrid


def build_ground_map(grid):
    """
    Convert adaptive ground cells into a
    top-down visualization representation.
    """

    cells = []

    for cell in grid.cells.values():

        if cell.ground_elevation is None:
            continue

        x, y = cell.center

        cells.append(
            (
                x,
                y,
                cell.ground_elevation,
                cell.resolution,
                cell.semantic_class
            )
        )

    return cells


def build_obstacle_map(grid):
    """
    Extract obstacle information from adaptive cells.
    """

    cells = []

    for cell in grid.cells.values():

        if cell.obstacle_count == 0:
            continue

        x, y = cell.center

        cells.append(
            (
                x,
                y,
                cell.obstacle_elevation,
                cell.resolution
            )
        )

    return cells


def main():

    print("=" * 65)
    print("ORBIT - ADAPTIVE 2.5D ELEVATION MAP")
    print("=" * 65)

    # ========================================================
    # Load LiDAR
    # ========================================================

    cloud = o3d.io.read_point_cloud(
        "data/synthetic_scene.ply"
    )

    points = np.asarray(
        cloud.points
    )

    print(
        f"\nInput points: {len(points):,}"
    )

    # ========================================================
    # Ground detection
    # ========================================================

    plane_model, inliers = (
        cloud.segment_plane(
            distance_threshold=0.08,
            ransac_n=3,
            num_iterations=1000
        )
    )

    ground_mask = np.zeros(
        len(points),
        dtype=bool
    )

    ground_mask[inliers] = True

    # ========================================================
    # Build adaptive grid
    # ========================================================

    grid = AdaptiveGrid()

    mapped_points = grid.build(
        points,
        ground_mask
    )

    print(
        f"Mapped points: {mapped_points:,}"
    )

    print(
        f"Adaptive cells: {len(grid.cells):,}"
    )

    # ========================================================
    # Extract maps
    # ========================================================

    ground_cells = build_ground_map(
        grid
    )

    obstacle_cells = build_obstacle_map(
        grid
    )

    print(
        f"Ground map cells: "
        f"{len(ground_cells):,}"
    )

    print(
        f"Obstacle map cells: "
        f"{len(obstacle_cells):,}"
    )

    # ========================================================
    # Ground elevation visualization
    # ========================================================

    if ground_cells:

        ground = np.array(
            [
                [c[0], c[1], c[2]]
                for c in ground_cells
            ]
        )

        plt.figure(
            figsize=(12, 10)
        )

        scatter = plt.scatter(
            ground[:, 0],
            ground[:, 1],
            c=ground[:, 2],
            s=4
        )

        plt.scatter(
            0,
            0,
            marker="x",
            s=100,
            label="LiDAR"
        )

        plt.colorbar(
            scatter,
            label="Ground elevation (m)"
        )

        plt.xlabel("X (m)")
        plt.ylabel("Y (m)")

        plt.title(
            "ORBIT - Adaptive Ground Elevation Map"
        )

        plt.axis("equal")
        plt.legend()

        plt.tight_layout()

        plt.show()

    # ========================================================
    # Obstacle visualization
    # ========================================================

    if obstacle_cells:

        obstacles = np.array(
            [
                [c[0], c[1], c[2]]
                for c in obstacle_cells
            ]
        )

        plt.figure(
            figsize=(12, 10)
        )

        scatter = plt.scatter(
            obstacles[:, 0],
            obstacles[:, 1],
            c=obstacles[:, 2],
            s=5
        )

        plt.scatter(
            0,
            0,
            marker="x",
            s=100,
            label="LiDAR"
        )

        plt.colorbar(
            scatter,
            label="Obstacle elevation (m)"
        )

        plt.xlabel("X (m)")
        plt.ylabel("Y (m)")

        plt.title(
            "ORBIT - Adaptive Obstacle Elevation Map"
        )

        plt.axis("equal")
        plt.legend()

        plt.tight_layout()

        plt.show()


if __name__ == "__main__":
    main()