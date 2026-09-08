import open3d as o3d
import numpy as np

from pathlib import Path
import sys

# Allow importing from src/mapping
sys.path.append(
    str(Path(__file__).resolve().parents[1])
)

from mapping.adaptive_grid import AdaptiveGrid


def create_cell_boxes(grid):
    """
    Convert occupied adaptive cells into
    small 3D boxes for visualization.

    AdaptiveGrid keys are Cartesian (level, ix, iy).
    Box XY matches AdaptiveCell.center.
    """

    boxes = []

    for cell in grid.cells.values():

        resolution = cell.resolution
        x, y = cell.center

        z = 0.0

        if cell.point_count > 0:
            mean_z = cell.z_mean
            if np.isfinite(mean_z):
                z = float(mean_z)

        # Create a small box representing the cell
        box = o3d.geometry.TriangleMesh.create_box(
            width=resolution,
            height=resolution,
            depth=resolution
        )

        box.translate(
            [
                x - resolution / 2,
                y - resolution / 2,
                z
            ]
        )

        # Color according to resolution
        if resolution == 0.05:
            color = [0.1, 0.8, 0.1]

        elif resolution == 0.10:
            color = [0.2, 0.5, 1.0]

        elif resolution == 0.25:
            color = [1.0, 0.7, 0.1]

        else:
            color = [0.9, 0.2, 0.2]

        box.paint_uniform_color(color)

        boxes.append(box)

    return boxes


def main():

    print("Loading ORBIT point cloud...")

    point_cloud = o3d.io.read_point_cloud(
        "data/synthetic_scene.ply"
    )

    points = np.asarray(
        point_cloud.points
    )

    print(
        f"Input points: {len(points):,}"
    )

    # Build adaptive grid
    grid = AdaptiveGrid()
    grid.build(points)

    print(
        f"Adaptive cells: "
        f"{len(grid.cells):,}"
    )

    # Create visualization geometry
    boxes = create_cell_boxes(grid)

    print(
        f"Visualizing {len(boxes):,} adaptive cells..."
    )

    # Add LiDAR sensor marker
    sensor = o3d.geometry.TriangleMesh.create_sphere(
        radius=0.15
    )

    sensor.paint_uniform_color(
        [0.0, 0.0, 0.0]
    )

    # Display
    o3d.visualization.draw_geometries(
        [sensor] + boxes,
        window_name="ORBIT - Adaptive 2.5D Grid"
    )


if __name__ == "__main__":
    main()