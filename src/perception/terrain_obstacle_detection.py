import sys
from pathlib import Path

import numpy as np
import open3d as o3d

# Make src/ importable when running this file directly
SRC_DIR = Path(__file__).resolve().parents[1]
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

from mapping.adaptive_grid import AdaptiveGrid


class ObstacleCell:
    """Compact representation of an obstacle-bearing adaptive cell."""

    def __init__(
        self,
        level,
        ix,
        iy,
        resolution,
        ground_elevation,
        obstacle_elevation,
        obstacle_height,
        obstacle_count,
    ):
        self.level = level
        self.ix = ix
        self.iy = iy
        self.resolution = resolution

        self.ground_elevation = ground_elevation
        self.obstacle_elevation = obstacle_elevation
        self.obstacle_height = obstacle_height
        self.obstacle_count = obstacle_count

    @property
    def center(self):
        return (
            self.ix * self.resolution + self.resolution / 2,
            self.iy * self.resolution + self.resolution / 2,
        )


def detect_ground(points):
    """
    Detect the dominant ground plane using RANSAC.

    Returns:
        ground_mask: boolean array
    """

    cloud = o3d.geometry.PointCloud()
    cloud.points = o3d.utility.Vector3dVector(points)

    _, inliers = cloud.segment_plane(
        distance_threshold=0.08,
        ransac_n=3,
        num_iterations=1000,
    )

    ground_mask = np.zeros(len(points), dtype=bool)
    ground_mask[inliers] = True

    return ground_mask


def build_adaptive_grid(points, ground_mask):
    """Build ORBIT's adaptive 2.5D grid."""

    grid = AdaptiveGrid()

    # The current AdaptiveGrid supports semantic insertion.
    grid.build(points, ground_mask)

    return grid


def estimate_local_ground(
    cell,
    grid,
    search_radius_cells=20,
):
    """
    Estimate terrain elevation using nearby ground cells
    at the same adaptive resolution level.
    """

    resolution = cell.resolution

    weighted_sum = 0.0
    weight_sum = 0.0

    for dx in range(
        -search_radius_cells,
        search_radius_cells + 1,
    ):

        for dy in range(
            -search_radius_cells,
            search_radius_cells + 1,
        ):

            if dx == 0 and dy == 0:
                continue

            neighbor_key = (
                cell.level,
                cell.ix + dx,
                cell.iy + dy,
            )

            neighbor = grid.cells.get(
                neighbor_key
            )

            if neighbor is None:
                continue

            if neighbor.ground_count == 0:
                continue

            distance_cells = np.sqrt(
                dx * dx
                + dy * dy
            )

            distance = (
                distance_cells
                * resolution
            )

            weight = 1.0 / (
                distance + 1e-3
            )

            weighted_sum += (
                weight
                * neighbor.ground_elevation
            )

            weight_sum += weight

    if weight_sum == 0.0:
        return None

    return (
        weighted_sum
        / weight_sum
    )


def extract_obstacle_cells(
    grid,
    minimum_height=0.15,
    minimum_obstacle_points=1,
    ground_search_radius=20,
):

    obstacle_cells = []

    for cell in grid.cells.values():

        # --------------------------------------------------
        # Must contain obstacle observations
        # --------------------------------------------------

        if (
            cell.obstacle_count
            < minimum_obstacle_points
        ):
            continue

        obstacle_z = (
            cell.obstacle_elevation
        )

        if obstacle_z is None:
            continue

        # --------------------------------------------------
        # Get ground reference
        # --------------------------------------------------

        if cell.ground_count > 0:

            ground_z = (
                cell.ground_elevation
            )

        else:

            ground_z = (
                estimate_local_ground(
                    cell,
                    grid,
                    search_radius_cells=
                        ground_search_radius,
                )
            )

        # --------------------------------------------------
        # No terrain estimate available
        # --------------------------------------------------

        if ground_z is None:
            continue

        # --------------------------------------------------
        # Terrain-relative height
        # --------------------------------------------------

        height = (
            obstacle_z
            - ground_z
        )

        if height < minimum_height:
            continue

        # --------------------------------------------------
        # Create obstacle representation
        # --------------------------------------------------

        obstacle_cells.append(

            ObstacleCell(
                level=cell.level,
                ix=cell.ix,
                iy=cell.iy,
                resolution=cell.resolution,
                ground_elevation=ground_z,
                obstacle_elevation=obstacle_z,
                obstacle_height=height,
                obstacle_count=
                    cell.obstacle_count,
            )
        )

    return obstacle_cells


def cell_distance(cell):
    """Distance of an adaptive cell from the LiDAR."""

    x, y = cell.center
    return np.sqrt(x * x + y * y)


def summarize_cells(obstacle_cells):

    if not obstacle_cells:
        return {
            "total": 0,
            "near": 0,
            "mid": 0,
            "far": 0,
        }

    near = 0
    mid = 0
    far = 0

    for cell in obstacle_cells:

        distance = cell_distance(cell)

        if distance < 25:
            near += 1
        elif distance < 50:
            mid += 1
        else:
            far += 1

    return {
        "total": len(obstacle_cells),
        "near": near,
        "mid": mid,
        "far": far,
    }


def print_examples(obstacle_cells, count=15):

    print("\nExample obstacle cells:")
    print("-" * 70)

    # Show highest obstacles first.
    ordered = sorted(
        obstacle_cells,
        key=lambda c: c.obstacle_height,
        reverse=True,
    )

    for i, cell in enumerate(ordered[:count]):

        x, y = cell.center
        distance = np.sqrt(x * x + y * y)

        print(
            f"\nCell #{i + 1}"
        )

        print(
            f"  Level:             {cell.level}"
        )

        print(
            f"  Resolution:        "
            f"{cell.resolution * 100:.0f} cm"
        )

        print(
            f"  Index:             "
            f"({cell.ix}, {cell.iy})"
        )

        print(
            f"  Center:            "
            f"({x:.2f}, {y:.2f}) m"
        )

        print(
            f"  Distance:          "
            f"{distance:.2f} m"
        )

        print(
            f"  Ground elevation:  "
            f"{cell.ground_elevation:.3f} m"
        )

        print(
            f"  Obstacle elevation: "
            f"{cell.obstacle_elevation:.3f} m"
        )

        print(
            f"  Obstacle height:    "
            f"{cell.obstacle_height:.3f} m"
        )

        print(
            f"  Obstacle points:    "
            f"{cell.obstacle_count}"
        )


def main():

    print("=" * 70)
    print("ORBIT - TERRAIN RELATIVE OBSTACLE EXTRACTION")
    print("=" * 70)

    # --------------------------------------------------------
    # Load point cloud
    # --------------------------------------------------------

    cloud = o3d.io.read_point_cloud(
        "data/synthetic_scene.ply"
    )

    points = np.asarray(cloud.points)

    print(
        f"\nInput points: {len(points):,}"
    )

    # --------------------------------------------------------
    # Ground detection
    # --------------------------------------------------------

    print("\nDetecting ground...")

    ground_mask = detect_ground(points)

    print(
        f"Ground points: "
        f"{ground_mask.sum():,}"
    )

    print(
        f"Non-ground points: "
        f"{(~ground_mask).sum():,}"
    )

    # --------------------------------------------------------
    # Adaptive grid
    # --------------------------------------------------------

    print("\nBuilding adaptive 2.5D grid...")

    grid = build_adaptive_grid(
        points,
        ground_mask,
    )

    print(
        f"Mapped points: "
        f"{sum(c.point_count for c in grid.cells.values()):,}"
    )

    print(
        f"Adaptive cells: "
        f"{len(grid.cells):,}"
    )

    # --------------------------------------------------------
    # Extract obstacle cells
    # --------------------------------------------------------

    print(
        "\nExtracting terrain-relative obstacle cells..."
    )

    obstacle_cells = extract_obstacle_cells(
        grid,
        minimum_height=0.15,
        minimum_obstacle_points=1,
    )

    # --------------------------------------------------------
    # Statistics
    # --------------------------------------------------------

    summary = summarize_cells(
        obstacle_cells
    )

    print(
        f"\nObstacle cells: "
        f"{summary['total']:,}"
    )

    print(
        f"  < 25m:   "
        f"{summary['near']:,}"
    )

    print(
        f"  25-50m:  "
        f"{summary['mid']:,}"
    )

    print(
        f"  50-100m: "
        f"{summary['far']:,}"
    )

    # --------------------------------------------------------
    # Height statistics
    # --------------------------------------------------------

    if obstacle_cells:

        heights = np.array(
            [
                cell.obstacle_height
                for cell in obstacle_cells
            ]
        )

        print("\nObstacle height statistics:")
        print(
            f"  Minimum: {heights.min():.3f} m"
        )
        print(
            f"  Mean:    {heights.mean():.3f} m"
        )
        print(
            f"  Maximum: {heights.max():.3f} m"
        )

    # --------------------------------------------------------
    # Examples
    # --------------------------------------------------------

    print_examples(
        obstacle_cells
    )

    print("\n" + "=" * 70)
    print("ORBIT OBSTACLE CELL EXTRACTION COMPLETE")
    print("=" * 70)


if __name__ == "__main__":
    main()