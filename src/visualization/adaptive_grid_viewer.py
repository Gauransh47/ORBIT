"""
ORBIT - Adaptive Grid Viewer

Visualizes the actual adaptive cells produced by AdaptiveGrid.

Uses the current AdaptiveGrid cell representation:
    cell.center
    cell.resolution
    cell.level
    cell.semantic_class

Does not depend on:
    grid.rings
    radial_bin
    sector
"""

import sys
from pathlib import Path

import numpy as np
import open3d as o3d
import matplotlib.pyplot as plt
from matplotlib.collections import PolyCollection
from matplotlib.colors import Normalize


# ============================================================
# Allow imports from src
# ============================================================

SRC_DIR = Path(__file__).resolve().parents[1]

if str(SRC_DIR) not in sys.path:
    sys.path.append(str(SRC_DIR))

from mapping.adaptive_grid import AdaptiveGrid


# ============================================================
# Cell polygon
# ============================================================

def cell_polygon(cell):
    """
    Build a rectangular polygon using the actual
    Cartesian center and resolution of the cell.
    """

    x, y = cell.center

    half = float(cell.resolution) / 2.0

    return [
        (x - half, y - half),
        (x + half, y - half),
        (x + half, y + half),
        (x - half, y + half),
    ]


# ============================================================
# Cell elevation
# ============================================================

def cell_elevation(cell):
    """
    Determine the elevation value used to color a cell.

    Priority:
        1. obstacle elevation
        2. ground elevation
        3. z_mean
        4. zero
    """

    obstacle_count = getattr(
        cell,
        "obstacle_count",
        0
    )

    if obstacle_count > 0:
        return float(
            getattr(
                cell,
                "obstacle_elevation",
                0.0
            )
        )

    ground_count = getattr(
        cell,
        "ground_count",
        0
    )

    if ground_count > 0:
        return float(
            getattr(
                cell,
                "ground_elevation",
                0.0
            )
        )

    if hasattr(cell, "z_mean"):
        return float(cell.z_mean)

    return 0.0


# ============================================================
# Main
# ============================================================

def main():

    print("=" * 70)
    print("ORBIT - TRUE ADAPTIVE RESOLUTION MAP")
    print("=" * 70)

    # ========================================================
    # Load point cloud
    # ========================================================

    cloud_path = Path(
        "data/synthetic_scene.ply"
    )

    print("\nLoading ORBIT point cloud...")

    if not cloud_path.exists():
        raise FileNotFoundError(
            f"Point cloud not found: {cloud_path}"
        )

    cloud = o3d.io.read_point_cloud(
        str(cloud_path)
    )

    points = np.asarray(
        cloud.points
    )

    if len(points) == 0:
        raise RuntimeError(
            "Point cloud is empty."
        )

    print(
        f"Input points: {len(points):,}"
    )

    # ========================================================
    # Ground detection
    # ========================================================

    print("\nDetecting ground...")

    plane_model, inliers = cloud.segment_plane(
        distance_threshold=0.08,
        ransac_n=3,
        num_iterations=1000
    )

    ground_mask = np.zeros(
        len(points),
        dtype=bool
    )

    ground_mask[inliers] = True

    print(
        f"Ground points: "
        f"{int(ground_mask.sum()):,}"
    )

    print(
        f"Non-ground points: "
        f"{int((~ground_mask).sum()):,}"
    )

    # ========================================================
    # Build adaptive grid
    # ========================================================

    print("\nBuilding adaptive grid...")

    grid = AdaptiveGrid()

    # IMPORTANT:
    # AdaptiveGrid.build() returns mapped point data,
    # not an integer count.
    mapped_result = grid.build(
        points,
        ground_mask
    )

    mapped_points = len(
        mapped_result
    )

    print(
        f"Mapped points: "
        f"{mapped_points:,}"
    )

    print(
        f"Adaptive cells: "
        f"{len(grid.cells):,}"
    )

    if not grid.cells:
        raise RuntimeError(
            "Adaptive grid contains no cells."
        )

    # ========================================================
    # Prepare visualization data
    # ========================================================

    polygons = []
    elevations = []

    level_counts = {
        0: 0,
        1: 0,
        2: 0,
        3: 0
    }

    ground_cells = 0
    obstacle_cells = 0
    mixed_cells = 0

    # ========================================================
    # Read actual adaptive cells
    # ========================================================

    for cell in grid.cells.values():

        point_count = getattr(
            cell,
            "point_count",
            0
        )

        if point_count == 0:
            continue

        # ----------------------------------------------------
        # Geometry
        # ----------------------------------------------------

        polygons.append(
            cell_polygon(cell)
        )

        # ----------------------------------------------------
        # Elevation
        # ----------------------------------------------------

        elevations.append(
            cell_elevation(cell)
        )

        # ----------------------------------------------------
        # Resolution level
        # ----------------------------------------------------

        level = int(
            getattr(
                cell,
                "level",
                0
            )
        )

        if level in level_counts:
            level_counts[level] += 1

        # ----------------------------------------------------
        # Semantic class
        # ----------------------------------------------------

        semantic_class = str(
            getattr(
                cell,
                "semantic_class",
                "UNKNOWN"
            )
        ).upper()

        if semantic_class == "GROUND":

            ground_cells += 1

        elif semantic_class == "OBSTACLE":

            obstacle_cells += 1

        else:

            mixed_cells += 1

    # ========================================================
    # Validate visualization data
    # ========================================================

    if not polygons:
        raise RuntimeError(
            "No occupied adaptive cells were available "
            "for visualization."
        )

    elevations = np.asarray(
        elevations,
        dtype=float
    )

    # ========================================================
    # Create figure
    # ========================================================

    fig, ax = plt.subplots(
        figsize=(15, 11)
    )

    # ========================================================
    # Elevation normalization
    # ========================================================

    elevation_min = float(
        np.min(elevations)
    )

    elevation_max = float(
        np.max(elevations)
    )

    # Avoid zero-width normalization.
    if np.isclose(
        elevation_min,
        elevation_max
    ):
        elevation_max = (
            elevation_min + 1e-6
        )

    norm = Normalize(
        vmin=elevation_min,
        vmax=elevation_max
    )

    cmap = plt.get_cmap(
        "viridis"
    )

    facecolors = cmap(
        norm(elevations)
    )

    # ========================================================
    # Draw adaptive cells
    # ========================================================

    collection = PolyCollection(
        polygons,
        facecolors=facecolors,
        edgecolors="none",
        linewidths=0
    )

    ax.add_collection(
        collection
    )

    # ========================================================
    # LiDAR sensor
    # ========================================================

    ax.scatter(
        0,
        0,
        marker="x",
        s=180,
        linewidths=3,
        label="LiDAR"
    )

    # ========================================================
    # Resolution boundaries
    # ========================================================

    for radius in [10, 25, 50, 100]:

        circle = plt.Circle(
            (0, 0),
            radius,
            fill=False,
            linestyle="--",
            linewidth=1.2,
            alpha=0.7
        )

        ax.add_patch(
            circle
        )

    # ========================================================
    # Resolution labels
    # ========================================================

    ax.text(
        3,
        7,
        "5 cm",
        fontsize=11
    )

    ax.text(
        15,
        7,
        "10 cm",
        fontsize=11
    )

    ax.text(
        32,
        7,
        "25 cm",
        fontsize=11
    )

    ax.text(
        65,
        7,
        "50 cm",
        fontsize=11
    )

    # ========================================================
    # Colorbar
    # ========================================================

    scalar_map = plt.cm.ScalarMappable(
        norm=norm,
        cmap=cmap
    )

    scalar_map.set_array(
        elevations
    )

    colorbar = fig.colorbar(
        scalar_map,
        ax=ax
    )

    colorbar.set_label(
        "Elevation (m)"
    )

    # ========================================================
    # Statistics
    # ========================================================

    stats_text = (
        f"Input points: {len(points):,}\n"
        f"Mapped points: {mapped_points:,}\n"
        f"Adaptive cells: {len(grid.cells):,}\n\n"

        f"5 cm cells: {level_counts[0]:,}\n"
        f"10 cm cells: {level_counts[1]:,}\n"
        f"25 cm cells: {level_counts[2]:,}\n"
        f"50 cm cells: {level_counts[3]:,}\n\n"

        f"Ground: {ground_cells:,}\n"
        f"Obstacle: {obstacle_cells:,}\n"
        f"Mixed/Other: {mixed_cells:,}\n\n"

        f"Elevation min: {elevation_min:.2f} m\n"
        f"Elevation max: {elevation_max:.2f} m"
    )

    ax.text(
        1.02,
        0.50,
        stats_text,
        transform=ax.transAxes,
        verticalalignment="center",
        fontsize=10,
        bbox=dict(
            boxstyle="round",
            alpha=0.15
        )
    )

    # ========================================================
    # Formatting
    # ========================================================

    ax.set_title(
        "ORBIT - True Adaptive 2.5D Resolution Map",
        fontsize=18,
        pad=15
    )

    ax.set_xlabel(
        "X (m)"
    )

    ax.set_ylabel(
        "Y (m)"
    )

    ax.set_aspect(
        "equal",
        adjustable="box"
    )

    ax.grid(
        alpha=0.15
    )

    ax.legend(
        loc="upper right"
    )

    ax.autoscale()

    plt.tight_layout()

    # ========================================================
    # Render
    # ========================================================

    print(
        "\nRendering adaptive cells..."
    )

    print(
        "Close the visualization window "
        "to return to PowerShell."
    )

    plt.show()


# ============================================================
# Entry point
# ============================================================

if __name__ == "__main__":
    main()