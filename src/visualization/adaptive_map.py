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
# Build polygon for one adaptive cell
# ============================================================

def cell_polygon(cell):
    """
    Return the four corners of an adaptive cell.

    The rectangle dimensions are the REAL resolution
    of the cell: 5cm, 10cm, 25cm or 50cm.
    """

    x, y = cell.center

    half = cell.resolution / 2.0

    return [
        (x - half, y - half),
        (x + half, y - half),
        (x + half, y + half),
        (x - half, y + half),
    ]


# ============================================================
# Main
# ============================================================

def main():

    print("=" * 70)
    print("ORBIT - TRUE ADAPTIVE RESOLUTION MAP")
    print("=" * 70)

    # --------------------------------------------------------
    # Load LiDAR
    # --------------------------------------------------------

    cloud = o3d.io.read_point_cloud(
        "data/synthetic_scene.ply"
    )

    points = np.asarray(
        cloud.points
    )

    print(
        f"\nInput points: {len(points):,}"
    )

    # --------------------------------------------------------
    # Ground detection
    # --------------------------------------------------------

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

    print(
        f"Ground points: "
        f"{ground_mask.sum():,}"
    )

    print(
        f"Non-ground points: "
        f"{(~ground_mask).sum():,}"
    )

    # --------------------------------------------------------
    # Build adaptive grid
    # --------------------------------------------------------

    grid = AdaptiveGrid()

    mapped_points = grid.build(
        points,
        ground_mask
    )

    print(
        f"Mapped points: "
        f"{mapped_points:,}"
    )

    print(
        f"Adaptive cells: "
        f"{len(grid.cells):,}"
    )

    # ========================================================
    # Prepare polygons
    # ========================================================

    polygons = []
    elevations = []
    classes = []
    levels = []

    for cell in grid.cells.values():

        # Ignore cells that somehow contain nothing
        if cell.point_count == 0:
            continue

        polygons.append(
            cell_polygon(cell)
        )

        levels.append(
            cell.level
        )

        classes.append(
            cell.semantic_class
        )

        # ----------------------------------------------------
        # Elevation used for coloring
        # ----------------------------------------------------

        if cell.obstacle_count > 0:

            elevation = (
                cell.obstacle_elevation
            )

        elif cell.ground_count > 0:

            elevation = (
                cell.ground_elevation
            )

        else:

            elevation = 0.0

        elevations.append(
            elevation
        )

    elevations = np.asarray(
        elevations
    )

    levels = np.asarray(
        levels
    )

    # ========================================================
    # Create figure
    # ========================================================

    fig, ax = plt.subplots(
        figsize=(15, 11)
    )

    # --------------------------------------------------------
    # Elevation normalization
    # --------------------------------------------------------

    norm = Normalize(
        vmin=np.min(elevations),
        vmax=np.max(elevations)
    )

    cmap = plt.get_cmap(
        "viridis"
    )

    facecolors = cmap(
        norm(elevations)
    )

    # --------------------------------------------------------
    # Draw actual cells
    # --------------------------------------------------------

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
    # Sensor
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
    # Labels for resolution zones
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
    # Calculate statistics
    # ========================================================

    level_counts = {
        0: 0,
        1: 0,
        2: 0,
        3: 0
    }

    ground_cells = 0
    obstacle_cells = 0
    mixed_cells = 0

    for cell in grid.cells.values():

        level_counts[
            cell.level
        ] += 1

        if cell.semantic_class == "GROUND":

            ground_cells += 1

        elif cell.semantic_class == "OBSTACLE":

            obstacle_cells += 1

        else:

            mixed_cells += 1

    # ========================================================
    # Statistics panel
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
        f"Mixed: {mixed_cells:,}"
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

    ax.autoscale()

    ax.grid(
        alpha=0.15
    )

    ax.legend(
        loc="upper right"
    )

    plt.tight_layout()

    print(
        "\nRendering adaptive cells..."
    )

    plt.show()


if __name__ == "__main__":
    main()