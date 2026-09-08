import sys
from pathlib import Path
from collections import deque

# ------------------------------------------------------------
# Make Orbit/src available for imports
# ------------------------------------------------------------

SRC_DIR = Path(__file__).resolve().parents[1]

if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

import numpy as np
import open3d as o3d

from mapping.adaptive_grid import AdaptiveGrid
from perception.terrain_obstacle_detection import (
    extract_obstacle_cells,
)


class ObjectComponent:
    """
    Represents one connected obstacle component.

    A component may contain cells from different adaptive
    resolution levels.
    """

    def __init__(self, component_id):

        self.component_id = component_id
        self.cells = []

    def add(self, cell):
        self.cells.append(cell)

    @property
    def cell_count(self):
        return len(self.cells)

    @property
    def total_points(self):
        return sum(
            cell.obstacle_count
            for cell in self.cells
        )

    @property
    def center(self):

        if not self.cells:
            return 0.0, 0.0

        centers = np.array(
            [
                cell.center
                for cell in self.cells
            ]
        )

        return (
            float(np.mean(centers[:, 0])),
            float(np.mean(centers[:, 1])),
        )

    @property
    def bounds(self):

        if not self.cells:
            return None

        min_x = min(
            cell.center[0] - cell.resolution / 2
            for cell in self.cells
        )

        max_x = max(
            cell.center[0] + cell.resolution / 2
            for cell in self.cells
        )

        min_y = min(
            cell.center[1] - cell.resolution / 2
            for cell in self.cells
        )

        max_y = max(
            cell.center[1] + cell.resolution / 2
            for cell in self.cells
        )

        min_z = min(
            cell.ground_elevation
            for cell in self.cells
        )

        max_z = max(
            cell.obstacle_elevation
            for cell in self.cells
        )

        return (
            min_x,
            max_x,
            min_y,
            max_y,
            min_z,
            max_z,
        )

    @property
    def dimensions(self):

        bounds = self.bounds

        if bounds is None:
            return 0.0, 0.0, 0.0

        min_x, max_x, min_y, max_y, min_z, max_z = bounds

        return (
            max_x - min_x,
            max_y - min_y,
            max_z - min_z,
        )

    @property
    def maximum_height(self):

        if not self.cells:
            return 0.0

        return max(
            cell.obstacle_height
            for cell in self.cells
        )

    @property
    def mean_height(self):

        if not self.cells:
            return 0.0

        return float(
            np.mean(
                [
                    cell.obstacle_height
                    for cell in self.cells
                ]
            )
        )

    @property
    def distance(self):

        x, y = self.center

        return float(
            np.sqrt(x * x + y * y)
        )

    @property
    def density(self):

        dx, dy, _ = self.dimensions

        area = max(
            dx * dy,
            1e-6
        )

        return self.total_points / area


# ============================================================
# Cell geometry
# ============================================================

def cell_bounds(cell):

    x, y = cell.center
    half = cell.resolution / 2

    return (
        x - half,
        x + half,
        y - half,
        y + half,
    )


def cells_touch(cell_a, cell_b):
    """
    Resolution-aware proximity test.

    Two obstacle cells are connected when their horizontal
    footprints overlap, touch, or are separated by a small
    LiDAR-sized gap.
    """

    ax0, ax1, ay0, ay1 = cell_bounds(cell_a)
    bx0, bx1, by0, by1 = cell_bounds(cell_b)

    # Physical gap between the two rectangles.
    gap_x = max(
        bx0 - ax1,
        ax0 - bx1,
        0.0,
    )

    gap_y = max(
        by0 - ay1,
        ay0 - by1,
        0.0,
    )

    # Euclidean horizontal separation.
    horizontal_gap = np.sqrt(
        gap_x * gap_x +
        gap_y * gap_y
    )

    resolution = max(
        cell_a.resolution,
        cell_b.resolution,
    )

    # Allow sparse LiDAR returns belonging to the same object
    # to bridge a small gap.
    max_gap = max(
        0.25,
        0.75 * resolution,
    )

    return horizontal_gap <= max_gap


# ============================================================
# Spatial hashing
# ============================================================

def build_spatial_hash(cells):
    """
    Build a coarse spatial hash.

    This avoids comparing every obstacle cell with every
    other obstacle cell.
    """

    spatial_hash = {}

    for index, cell in enumerate(cells):

        x, y = cell.center

        # Use the largest resolution as the hash bucket.
        bucket_size = AdaptiveGrid.LEVELS[
            max(AdaptiveGrid.LEVELS)
        ]

        bx = int(
            np.floor(x / bucket_size)
        )

        by = int(
            np.floor(y / bucket_size)
        )

        key = (bx, by)

        if key not in spatial_hash:
            spatial_hash[key] = []

        spatial_hash[key].append(index)

    return spatial_hash


def candidate_neighbors(
    cell_index,
    cells,
    spatial_hash,
):
    """
    Return possible neighboring cells using the spatial hash.
    """

    cell = cells[cell_index]

    x, y = cell.center

    bucket_size = AdaptiveGrid.LEVELS[
        max(AdaptiveGrid.LEVELS)
    ]

    bx = int(
        np.floor(x / bucket_size)
    )

    by = int(
        np.floor(y / bucket_size)
    )

    candidates = []

    # Search surrounding buckets.
    for dx in (-1, 0, 1):
        for dy in (-1, 0, 1):

            key = (
                bx + dx,
                by + dy,
            )

            if key not in spatial_hash:
                continue

            candidates.extend(
                spatial_hash[key]
            )

    return candidates


# ============================================================
# Connected components
# ============================================================

def connected_components(
    obstacle_cells,
):
    """
    Resolution-aware connected component labeling.

    Returns:
        list[ObjectComponent]
    """

    if not obstacle_cells:
        return []

    spatial_hash = build_spatial_hash(
        obstacle_cells
    )

    visited = np.zeros(
        len(obstacle_cells),
        dtype=bool,
    )

    components = []

    component_id = 0

    for start in range(
        len(obstacle_cells)
    ):

        if visited[start]:
            continue

        component = ObjectComponent(
            component_id
        )

        queue = deque(
            [start]
        )

        visited[start] = True

        while queue:

            current = queue.popleft()

            current_cell = (
                obstacle_cells[current]
            )

            component.add(
                current_cell
            )

            candidates = candidate_neighbors(
                current,
                obstacle_cells,
                spatial_hash,
            )

            for neighbor in candidates:

                if visited[neighbor]:
                    continue

                neighbor_cell = (
                    obstacle_cells[neighbor]
                )

                if not cells_touch(
                    current_cell,
                    neighbor_cell,
                ):
                    continue

                # Prevent very different-height surfaces
                # from being merged through a thin boundary.
                height_difference = abs(
                    current_cell.obstacle_height
                    - neighbor_cell.obstacle_height
                )

                max_allowed_height_difference = max(
                    0.75,
                    2.0
                    * max(
                        current_cell.resolution,
                        neighbor_cell.resolution,
                    ),
                )

                if (
                    height_difference
                    > max_allowed_height_difference
                ):
                    continue

                visited[neighbor] = True
                queue.append(neighbor)

        components.append(
            component
        )

        component_id += 1

    return components


# ============================================================
# Filtering
# ============================================================

def filter_components(
    components,
    minimum_cells=3,
    minimum_points=5,
):
    """
    Remove tiny connected components.

    These are usually isolated LiDAR noise points.
    """

    filtered = []

    for component in components:

        if (
            component.cell_count
            < minimum_cells
        ):
            continue

        if (
            component.total_points
            < minimum_points
        ):
            continue

        filtered.append(
            component
        )

    return filtered


# ============================================================
# Component classification
# ============================================================

def classify_component(component):
    """
    Lightweight geometric classification.

    This is NOT the final deep-learning classifier.

    It provides a strong geometric baseline.
    """

    width, length, height = (
        component.dimensions
    )

    height = max(
        height,
        component.maximum_height,
    )

    horizontal = max(
        width,
        length,
        1e-6,
    )

    verticality = (
        height / horizontal
    )

    # Pole:
    # tall relative to footprint.
    if (
        height >= 1.5
        and verticality >= 2.5
        and width <= 1.5
        and length <= 1.5
    ):
        return "POLE"

    # Wall:
    # long horizontal structure.
    if (
        height >= 1.0
        and max(width, length) >= 2.0
        and min(width, length) <= 2.5
    ):
        return "WALL"

    # Vehicle-like:
    # compact object with meaningful height.
    if (
        1.0 <= height <= 3.5
        and width <= 5.0
        and length <= 8.0
        and component.total_points >= 20
    ):
        return "VEHICLE-LIKE"

    return "OBSTACLE"


# ============================================================
# Reporting
# ============================================================

def print_component(component):

    classification = classify_component(
        component
    )

    width, length, height = (
        component.dimensions
    )

    x, y = component.center

    print(
        f"\nObject #{component.component_id}"
    )

    print(
        f"  Classification: {classification}"
    )

    print(
        f"  Cells:          "
        f"{component.cell_count:,}"
    )

    print(
        f"  Points:         "
        f"{component.total_points:,}"
    )

    print(
        f"  Dimensions:     "
        f"{width:.2f}m × "
        f"{length:.2f}m × "
        f"{height:.2f}m"
    )

    print(
        f"  Center:         "
        f"({x:.2f}, {y:.2f})m"
    )

    print(
        f"  Distance:       "
        f"{component.distance:.2f}m"
    )

    print(
        f"  Max height:     "
        f"{component.maximum_height:.2f}m"
    )

    print(
        f"  Mean height:    "
        f"{component.mean_height:.2f}m"
    )

    print(
        f"  Density:        "
        f"{component.density:.2f}"
    )


# ============================================================
# Main
# ============================================================

def main():

    print("=" * 70)
    print(
        "ORBIT - ADAPTIVE CONNECTED COMPONENT DETECTION"
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
    # Ground detection
    # --------------------------------------------------------

    print(
        "\nDetecting ground..."
    )

    ground_mask = detect_ground(
        points
    )

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

    print(
        "\nBuilding adaptive grid..."
    )

    grid = AdaptiveGrid()

    grid.build(
        points,
        ground_mask
    )

    print(
        f"Adaptive cells: "
        f"{len(grid.cells):,}"
    )

    # --------------------------------------------------------
    # Obstacle extraction
    # --------------------------------------------------------

    print(
        "\nExtracting obstacle cells..."
    )

    obstacle_cells = extract_obstacle_cells(
        grid,
        minimum_height=0.15,
        minimum_obstacle_points=3,
    )

    print(
        f"Obstacle cells: "
        f"{len(obstacle_cells):,}"
    )

    # --------------------------------------------------------
    # Connected components
    # --------------------------------------------------------

    print(
        "\nRunning adaptive connected components..."
    )

    components = connected_components(
        obstacle_cells
    )

    print(
        f"Raw components: "
        f"{len(components):,}"
    )

    # --------------------------------------------------------
    # Filter noise
    # --------------------------------------------------------

    filtered = filter_components(
        components,
        minimum_cells=3,
        minimum_points=5,
    )

    print(
        f"After noise filtering: "
        f"{len(filtered):,}"
    )

    # --------------------------------------------------------
    # Object proposals
    # --------------------------------------------------------

    print(
        "\nObject proposals:"
    )

    print(
        "-" * 70
    )

    for component in filtered:

        print_component(
            component
        )

    # --------------------------------------------------------
    # Summary
    # --------------------------------------------------------

    summary = {}

    for component in filtered:

        classification = (
            classify_component(
                component
            )
        )

        summary[classification] = (
            summary.get(
                classification,
                0
            )
            + 1
        )

    print(
        "\n" + "=" * 70
    )

    print(
        "CLASS SUMMARY"
    )

    print(
        "=" * 70
    )

    for name, count in sorted(
        summary.items()
    ):

        print(
            f"  {name:<20} {count}"
        )

    print(
        "\n" + "=" * 70
    )

    print(
        "ORBIT ADAPTIVE OBJECT PROPOSAL COMPLETE"
    )

    print(
        "=" * 70
    )


# ============================================================
# Ground detection helper
# ============================================================

def detect_ground(points):

    cloud = o3d.geometry.PointCloud()

    cloud.points = (
        o3d.utility.Vector3dVector(
            points
        )
    )

    _, inliers = cloud.segment_plane(
        distance_threshold=0.08,
        ransac_n=3,
        num_iterations=1000,
    )

    mask = np.zeros(
        len(points),
        dtype=bool
    )

    mask[inliers] = True

    return mask


if __name__ == "__main__":
    main()