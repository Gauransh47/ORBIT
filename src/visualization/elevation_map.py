import sys
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt
import open3d as o3d


# ============================================================
# Allow imports from src
# ============================================================

SRC_DIR = Path(__file__).resolve().parents[1]

if str(SRC_DIR) not in sys.path:
    sys.path.append(str(SRC_DIR))


from mapping.adaptive_grid import AdaptiveGrid


# ============================================================
# Build ground elevation map
# ============================================================

def build_ground_map(grid):
    """
    Extract ground elevation information from occupied
    adaptive grid cells.

    Returns:
        List of:
        (
            x,
            y,
            ground_elevation,
            resolution,
            semantic_class
        )
    """

    cells = []

    for cell in grid.cells.values():

        if cell.ground_count <= 0:
            continue

        if cell.ground_elevation is None:
            continue

        x, y = cell.center

        cells.append(
            (
                float(x),
                float(y),
                float(cell.ground_elevation),
                float(cell.resolution),
                cell.semantic_class
            )
        )

    return cells


# ============================================================
# Build obstacle elevation map
# ============================================================

def build_obstacle_map(grid):
    """
    Extract obstacle elevation information from occupied
    adaptive grid cells.

    Returns:
        List of:
        (
            x,
            y,
            obstacle_elevation,
            resolution
        )
    """

    cells = []

    for cell in grid.cells.values():

        if cell.obstacle_count <= 0:
            continue

        if cell.obstacle_elevation is None:
            continue

        x, y = cell.center

        cells.append(
            (
                float(x),
                float(y),
                float(cell.obstacle_elevation),
                float(cell.resolution)
            )
        )

    return cells


# ============================================================
# Ground detection
# ============================================================

def detect_ground(cloud, num_points):
    """
    Detect the dominant ground plane using RANSAC.

    Returns:
        Boolean mask where True indicates ground points.
    """

    print("\nDetecting ground...")

    plane_model, inliers = cloud.segment_plane(
        distance_threshold=0.08,
        ransac_n=3,
        num_iterations=1000
    )

    ground_mask = np.zeros(
        num_points,
        dtype=bool
    )

    ground_mask[inliers] = True

    print(
        f"Ground points: {ground_mask.sum():,}"
    )

    print(
        f"Non-ground points: {(~ground_mask).sum():,}"
    )

    return ground_mask


# ============================================================
# Main
# ============================================================

def main():

    print("=" * 65)
    print("ORBIT - ADAPTIVE 2.5D ELEVATION MAP")
    print("=" * 65)

    # ========================================================
    # Load LiDAR
    # ========================================================

    print("\nLoading ORBIT point cloud...")

    cloud = o3d.io.read_point_cloud(
        "data/synthetic_scene.ply"
    )

    points = np.asarray(
        cloud.points
    )

    if len(points) == 0:

        raise RuntimeError(
            "No points were loaded from "
            "data/synthetic_scene.ply"
        )

    print(
        f"Input points: {len(points):,}"
    )

    # ========================================================
    # Ground detection
    # ========================================================

    ground_mask = detect_ground(
        cloud,
        len(points)
    )

    # ========================================================
    # Build adaptive grid
    # ========================================================

    print("\nBuilding adaptive grid...")

    grid = AdaptiveGrid()

    build_result = grid.build(
        points,
        ground_mask
    )

    # --------------------------------------------------------
    # AdaptiveGrid.build() may return mapped points rather
    # than a count. We determine the count safely.
    # --------------------------------------------------------

    if build_result is None:

        mapped_count = len(points)

    elif isinstance(build_result, (int, np.integer)):

        mapped_count = int(build_result)

    else:

        mapped_count = len(build_result)

    print(
        f"Mapped points: {mapped_count:,}"
    )

    print(
        f"Adaptive cells: {len(grid.cells):,}"
    )

    # ========================================================
    # Extract 2.5D maps
    # ========================================================

    print("\nExtracting elevation layers...")

    ground_cells = build_ground_map(
        grid
    )

    obstacle_cells = build_obstacle_map(
        grid
    )

    print(
        f"Ground map cells: {len(ground_cells):,}"
    )

    print(
        f"Obstacle map cells: {len(obstacle_cells):,}"
    )

    # ========================================================
    # Ground elevation visualization
    # ========================================================

    if ground_cells:

        print(
            "\nRendering ground elevation map..."
        )

        ground = np.asarray(
            [
                [
                    cell[0],
                    cell[1],
                    cell[2]
                ]
                for cell in ground_cells
            ],
            dtype=float
        )

        resolutions = np.asarray(
            [
                cell[3]
                for cell in ground_cells
            ],
            dtype=float
        )

        # Larger adaptive cells should appear visually larger
        marker_sizes = np.clip(
            (resolutions / resolutions.min()) * 4,
            2,
            30
        )

        fig, ax = plt.subplots(
            figsize=(13, 10)
        )

        scatter = ax.scatter(
            ground[:, 0],
            ground[:, 1],
            c=ground[:, 2],
            s=marker_sizes
        )

        ax.scatter(
            0,
            0,
            marker="x",
            s=150,
            linewidths=3,
            label="LiDAR"
        )

        colorbar = fig.colorbar(
            scatter,
            ax=ax
        )

        colorbar.set_label(
            "Ground Elevation (m)"
        )

        ax.set_xlabel("X (m)")
        ax.set_ylabel("Y (m)")

        ax.set_title(
            "ORBIT - Adaptive 2.5D Ground Elevation Map"
        )

        ax.set_aspect(
            "equal",
            adjustable="box"
        )

        ax.grid(
            alpha=0.2
        )

        ax.legend()

        plt.tight_layout()

        plt.show()

    else:

        print(
            "\nWARNING: No ground cells found."
        )

    # ========================================================
    # Obstacle elevation visualization
    # ========================================================

    if obstacle_cells:

        print(
            "\nRendering obstacle elevation map..."
        )

        obstacles = np.asarray(
            [
                [
                    cell[0],
                    cell[1],
                    cell[2]
                ]
                for cell in obstacle_cells
            ],
            dtype=float
        )

        resolutions = np.asarray(
            [
                cell[3]
                for cell in obstacle_cells
            ],
            dtype=float
        )

        marker_sizes = np.clip(
            (resolutions / resolutions.min()) * 5,
            2,
            35
        )

        fig, ax = plt.subplots(
            figsize=(13, 10)
        )

        scatter = ax.scatter(
            obstacles[:, 0],
            obstacles[:, 1],
            c=obstacles[:, 2],
            s=marker_sizes
        )

        ax.scatter(
            0,
            0,
            marker="x",
            s=150,
            linewidths=3,
            label="LiDAR"
        )

        colorbar = fig.colorbar(
            scatter,
            ax=ax
        )

        colorbar.set_label(
            "Obstacle Elevation (m)"
        )

        ax.set_xlabel("X (m)")
        ax.set_ylabel("Y (m)")

        ax.set_title(
            "ORBIT - Adaptive 2.5D Obstacle Elevation Map"
        )

        ax.set_aspect(
            "equal",
            adjustable="box"
        )

        ax.grid(
            alpha=0.2
        )

        ax.legend()

        plt.tight_layout()

        plt.show()

    else:

        print(
            "\nWARNING: No obstacle cells found."
        )

    # ========================================================
    # Final summary
    # ========================================================

    print("\n" + "=" * 65)
    print("ORBIT 2.5D ELEVATION MAP COMPLETE")
    print("=" * 65)

    print(
        f"Input points:        {len(points):,}"
    )

    print(
        f"Adaptive cells:      {len(grid.cells):,}"
    )

    print(
        f"Ground elevation:    {len(ground_cells):,} cells"
    )

    print(
        f"Obstacle elevation:  {len(obstacle_cells):,} cells"
    )


# ============================================================
# Entry point
# ============================================================

if __name__ == "__main__":
    main()