import sys
from pathlib import Path

import numpy as np

# ------------------------------------------------------------
# Make Orbit/src available when running directly
# ------------------------------------------------------------

SRC_DIR = Path(__file__).resolve().parents[1]

if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

from perception.connected_components import (
    connected_components,
    filter_components,
    classify_component,
    detect_ground,
)

from perception.terrain_obstacle_detection import (
    extract_obstacle_cells,
)

from mapping.adaptive_grid import AdaptiveGrid

import open3d as o3d


# ============================================================
# Object Group
# ============================================================

class ObjectGroup:
    """
    Final physical-object hypothesis.

    Multiple connected components can belong to the
    same physical object.
    """

    def __init__(self, object_id):

        self.object_id = object_id
        self.components = []

    def add_component(self, component):

        self.components.append(component)

    @property
    def cell_count(self):

        return sum(
            component.cell_count
            for component in self.components
        )

    @property
    def point_count(self):

        return sum(
            component.total_points
            for component in self.components
        )

    @property
    def center(self):

        if not self.components:
            return 0.0, 0.0

        weights = np.array(
            [
                max(component.total_points, 1)
                for component in self.components
            ],
            dtype=float,
        )

        centers = np.array(
            [
                component.center
                for component in self.components
            ],
            dtype=float,
        )

        center = np.average(
            centers,
            axis=0,
            weights=weights,
        )

        return (
            float(center[0]),
            float(center[1]),
        )

    @property
    def bounds(self):

        bounds = [
            component.bounds
            for component in self.components
            if component.bounds is not None
        ]

        if not bounds:
            return None

        return (
            min(b[0] for b in bounds),
            max(b[1] for b in bounds),
            min(b[2] for b in bounds),
            max(b[3] for b in bounds),
            min(b[4] for b in bounds),
            max(b[5] for b in bounds),
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
    def distance(self):

        x, y = self.center

        return float(
            np.sqrt(x * x + y * y)
        )

    @property
    def maximum_height(self):

        if not self.components:
            return 0.0

        return max(
            component.maximum_height
            for component in self.components
        )

    @property
    def mean_height(self):

        if not self.components:
            return 0.0

        values = []

        for component in self.components:

            for cell in component.cells:
                values.append(
                    cell.obstacle_height
                )

        if not values:
            return 0.0

        return float(
            np.mean(values)
        )

    @property
    def density(self):

        width, length, _ = self.dimensions

        area = max(
            width * length,
            1e-6
        )

        return self.point_count / area

    @property
    def classification(self):

        width, length, height = (
            self.dimensions
        )

        horizontal = max(
            width,
            length,
            1e-6
        )

        verticality = (
            height / horizontal
        )

        # ----------------------------------------------------
        # Pole
        # ----------------------------------------------------

        if (
            height >= 1.5
            and verticality >= 2.0
            and width <= 1.5
            and length <= 1.5
        ):
            return "POLE"

        # ----------------------------------------------------
        # Wall
        # ----------------------------------------------------

        if (
            height >= 1.0
            and max(width, length) >= 2.0
            and min(width, length) <= 2.5
        ):
            return "WALL"

        # ----------------------------------------------------
        # Vehicle-like
        # ----------------------------------------------------

        if (
            1.0 <= height <= 3.5
            and width <= 5.0
            and length <= 8.0
            and self.point_count >= 20
        ):
            return "VEHICLE-LIKE"

        return "OBSTACLE"


# ============================================================
# Geometry helpers
# ============================================================

def horizontal_distance(a, b):

    ax, ay = a.center
    bx, by = b.center

    return float(
        np.sqrt(
            (ax - bx) ** 2
            + (ay - by) ** 2
        )
    )


def vertical_difference(a, b):

    return abs(
        a.maximum_height
        - b.maximum_height
    )


def bounding_boxes_close(a, b):

    a_bounds = a.bounds
    b_bounds = b.bounds

    if a_bounds is None or b_bounds is None:
        return False

    ax0, ax1, ay0, ay1, _, _ = a_bounds
    bx0, bx1, by0, by1, _, _ = b_bounds

    # Expand both boxes slightly.
    margin = max(
        0.75,
        0.5 * max(
            a.dimensions[0],
            a.dimensions[1],
            b.dimensions[0],
            b.dimensions[1],
        ),
    )

    x_close = (
        ax0 <= bx1 + margin
        and ax1 >= bx0 - margin
    )

    y_close = (
        ay0 <= by1 + margin
        and ay1 >= by0 - margin
    )

    return x_close and y_close


# ============================================================
# Association decision
# ============================================================

def should_merge(a, b):
    """
    Determine whether two components likely belong to
    the same physical object.
    """

    distance = horizontal_distance(
        a,
        b,
    )

    height_difference = vertical_difference(
        a,
        b,
    )

    # --------------------------------------------------------
    # Dynamic distance threshold.
    #
    # Larger cells at longer distances require a larger
    # association radius.
    # --------------------------------------------------------

    resolution = max(
        max(
            cell.resolution
            for cell in a.cells
        ),
        max(
            cell.resolution
            for cell in b.cells
        ),
    )

    distance_threshold = max(
        0.75,
        4.0 * resolution,
    )

    # Nearby components with compatible height.
    if (
        distance <= distance_threshold
        and height_difference <= 1.0
    ):
        return True

    # --------------------------------------------------------
    # Special case for fragmented poles.
    #
    # Poles are vertically elongated and can be split into
    # multiple small components.
    # --------------------------------------------------------

    if (
        distance <= 1.5
        and a.maximum_height >= 1.2
        and b.maximum_height >= 1.2
        and a.dimensions[0] <= 1.5
        and a.dimensions[1] <= 1.5
        and b.dimensions[0] <= 1.5
        and b.dimensions[1] <= 1.5
    ):
        return True

    # --------------------------------------------------------
    # Bounding-box based association.
    # --------------------------------------------------------

    if (
        distance <= 2.0
        and bounding_boxes_close(a, b)
        and height_difference <= 1.25
    ):
        return True

    return False


# ============================================================
# Union-Find
# ============================================================

class UnionFind:

    def __init__(self, size):

        self.parent = list(
            range(size)
        )

        self.rank = [0] * size

    def find(self, x):

        while self.parent[x] != x:

            self.parent[x] = (
                self.parent[
                    self.parent[x]
                ]
            )

            x = self.parent[x]

        return x

    def union(self, a, b):

        root_a = self.find(a)
        root_b = self.find(b)

        if root_a == root_b:
            return

        if self.rank[root_a] < self.rank[root_b]:

            self.parent[root_a] = root_b

        elif self.rank[root_a] > self.rank[root_b]:

            self.parent[root_b] = root_a

        else:

            self.parent[root_b] = root_a
            self.rank[root_a] += 1


# ============================================================
# Spatial hashing
# ============================================================

def build_component_hash(components):

    spatial_hash = {}

    # 2 metre search buckets.
    bucket_size = 2.0

    for index, component in enumerate(
        components
    ):

        x, y = component.center

        bx = int(
            np.floor(x / bucket_size)
        )

        by = int(
            np.floor(y / bucket_size)
        )

        key = (
            bx,
            by,
        )

        if key not in spatial_hash:
            spatial_hash[key] = []

        spatial_hash[key].append(
            index
        )

    return spatial_hash


def candidate_components(
    index,
    components,
    spatial_hash,
):

    bucket_size = 2.0

    x, y = components[index].center

    bx = int(
        np.floor(x / bucket_size)
    )

    by = int(
        np.floor(y / bucket_size)
    )

    candidates = []

    for dx in (-1, 0, 1):
        for dy in (-1, 0, 1):

            key = (
                bx + dx,
                by + dy,
            )

            candidates.extend(
                spatial_hash.get(
                    key,
                    []
                )
            )

    return candidates


# ============================================================
# Merge components
# ============================================================

def associate_components(
    components
):
    """
    Merge components that likely belong to the same
    physical object.
    """

    if not components:
        return []

    union_find = UnionFind(
        len(components)
    )

    spatial_hash = (
        build_component_hash(
            components
        )
    )

    for i in range(
        len(components)
    ):

        candidates = (
            candidate_components(
                i,
                components,
                spatial_hash,
            )
        )

        for j in candidates:

            if j <= i:
                continue

            if should_merge(
                components[i],
                components[j],
            ):
                union_find.union(
                    i,
                    j
                )

    groups = {}

    for index in range(
        len(components)
    ):

        root = union_find.find(
            index
        )

        if root not in groups:
            groups[root] = []

        groups[root].append(
            index
        )

    objects = []

    for object_id, indices in enumerate(
        groups.values()
    ):

        obj = ObjectGroup(
            object_id
        )

        for index in indices:

            obj.add_component(
                components[index]
            )

        objects.append(
            obj
        )

    return objects


# ============================================================
# Main
# ============================================================

def main():

    print("=" * 70)
    print(
        "ORBIT - OBJECT ASSOCIATION ENGINE"
    )
    print("=" * 70)

    # --------------------------------------------------------
    # Load data
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
    # Obstacle cells
    # --------------------------------------------------------

    obstacle_cells = (
        extract_obstacle_cells(
            grid,
            minimum_height=0.15,
            minimum_obstacle_points=3,
        )
    )

    print(
        f"Obstacle cells: "
        f"{len(obstacle_cells):,}"
    )

    # --------------------------------------------------------
    # Connected components
    # --------------------------------------------------------

    print(
        "\nRunning connected components..."
    )

    components = (
        connected_components(
            obstacle_cells
        )
    )

    print(
        f"Raw components: "
        f"{len(components):,}"
    )

    components = filter_components(
        components,
        minimum_cells=3,
        minimum_points=5,
    )

    print(
        f"Filtered components: "
        f"{len(components):,}"
    )

    # --------------------------------------------------------
    # Association
    # --------------------------------------------------------

    print(
        "\nAssociating fragmented objects..."
    )

    objects = associate_components(
        components
    )

    print(
        f"Final object hypotheses: "
        f"{len(objects):,}"
    )

    # --------------------------------------------------------
    # Print objects
    # --------------------------------------------------------

    print(
        "\nFinal object proposals:"
    )

    print(
        "-" * 70
    )

    for obj in objects:

        print(
            f"\nObject #{obj.object_id}"
        )

        print(
            f"  Classification: "
            f"{obj.classification}"
        )

        print(
            f"  Components:     "
            f"{len(obj.components)}"
        )

        print(
            f"  Cells:          "
            f"{obj.cell_count}"
        )

        print(
            f"  Points:         "
            f"{obj.point_count:,}"
        )

        width, length, height = (
            obj.dimensions
        )

        print(
            f"  Dimensions:     "
            f"{width:.2f}m × "
            f"{length:.2f}m × "
            f"{height:.2f}m"
        )

        x, y = obj.center

        print(
            f"  Center:         "
            f"({x:.2f}, {y:.2f})m"
        )

        print(
            f"  Distance:       "
            f"{obj.distance:.2f}m"
        )

        print(
            f"  Max height:     "
            f"{obj.maximum_height:.2f}m"
        )

        print(
            f"  Mean height:    "
            f"{obj.mean_height:.2f}m"
        )

        print(
            f"  Density:        "
            f"{obj.density:.2f}"
        )

    # --------------------------------------------------------
    # Summary
    # --------------------------------------------------------

    summary = {}

    for obj in objects:

        name = obj.classification

        summary[name] = (
            summary.get(
                name,
                0
            )
            + 1
        )

    print(
        "\n" + "=" * 70
    )

    print(
        "FINAL CLASS SUMMARY"
    )

    print(
        "=" * 70
    )

    for name, count in sorted(
        summary.items()
    ):

        print(
            f"  {name:<20} "
            f"{count}"
        )

    print(
        "\n" + "=" * 70
    )

    print(
        "ORBIT OBJECT ASSOCIATION COMPLETE"
    )

    print(
        "=" * 70
    )


if __name__ == "__main__":
    main()