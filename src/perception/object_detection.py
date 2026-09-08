import sys
from pathlib import Path
from collections import deque
from dataclasses import dataclass, field

import numpy as np
import open3d as o3d


# ============================================================
# Make src/ importable
# ============================================================

SRC_DIR = Path(__file__).resolve().parents[1]

if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))


from mapping.adaptive_grid import AdaptiveGrid
from perception.terrain_obstacle_detection import (
    extract_obstacle_cells,
)


# ============================================================
# ORBIT - TERRAIN RELATIVE OBJECT DETECTION
# ============================================================


# ============================================================
# SemanticKITTI frame loader
# ============================================================

def load_semantic_kitti_frame(file_path):
    """
    Load one SemanticKITTI Velodyne frame.

    Returns only XYZ coordinates.

    The original point ordering is preserved, so an index in
    the returned array corresponds directly to the matching
    index in the SemanticKITTI label file.
    """

    path = Path(file_path)

    if not path.exists():
        raise FileNotFoundError(
            f"LiDAR frame not found:\n{path}"
        )

    raw = np.fromfile(
        path,
        dtype=np.float32,
    )

    if raw.size % 4 != 0:
        raise ValueError(
            f"Invalid SemanticKITTI frame size: "
            f"{raw.size} float values is not divisible by 4."
        )

    scan = raw.reshape(
        -1,
        4,
    )

    return scan[:, :3].astype(
        np.float64,
        copy=False,
    )


# ============================================================
# Object proposal
# ============================================================

@dataclass
class ObjectProposal:
    """
    Final ORBIT object proposal.

    point_indices refer to indices in the original input
    point cloud passed to ObjectDetector.detect().
    """

    object_id: int

    cells: list

    classification: str

    center: tuple

    dimensions: tuple

    distance: float

    point_count: int

    cell_count: int

    max_height: float

    mean_height: float

    footprint_area: float

    aspect_ratio: float

    verticality: float

    density: float

    point_indices: np.ndarray = field(
        default_factory=lambda: np.empty(
            0,
            dtype=np.int64,
        )
    )

    points: np.ndarray = field(
        default_factory=lambda: np.empty(
            (0, 3),
            dtype=np.float64,
        )
    )

    @property
    def cluster_id(self):
        return self.object_id


# ============================================================
# Object component
# ============================================================

class ObjectComponent:
    """
    Connected group of adaptive obstacle cells.
    """

    def __init__(self, component_id=0):
        self.component_id = component_id
        self.cells = []
        self._bounds_cache = None
        self._mean_height_cache = None

    def add(self, cell):
        if cell not in self.cells:
            self.cells.append(cell)
            self._bounds_cache = None
            self._mean_height_cache = None

    @property
    def bounds(self):
        if self._bounds_cache is not None:
            return self._bounds_cache

        if not self.cells:
            return None

        min_x = min(
            cell.center[0]
            - cell.resolution / 2.0
            for cell in self.cells
        )

        max_x = max(
            cell.center[0]
            + cell.resolution / 2.0
            for cell in self.cells
        )

        min_y = min(
            cell.center[1]
            - cell.resolution / 2.0
            for cell in self.cells
        )

        max_y = max(
            cell.center[1]
            + cell.resolution / 2.0
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

        self._bounds_cache = (
            min_x,
            max_x,
            min_y,
            max_y,
            min_z,
            max_z,
        )

        return self._bounds_cache

    @property
    def center(self):
        bounds = self.bounds

        if bounds is None:
            return 0.0, 0.0

        min_x, max_x, min_y, max_y, _, _ = bounds

        return (
            (min_x + max_x) / 2.0,
            (min_y + max_y) / 2.0,
        )

    @property
    def distance(self):
        x, y = self.center

        return float(
            np.hypot(
                x,
                y,
            )
        )

    @property
    def mean_height(self):
        if self._mean_height_cache is not None:
            return self._mean_height_cache

        if not self.cells:
            return 0.0

        self._mean_height_cache = float(
            sum(
                cell.obstacle_height
                for cell in self.cells
            )
            / len(self.cells)
        )

        return self._mean_height_cache


# ============================================================
# Ground detection
# ============================================================

def detect_ground(
    points,
    distance_threshold=0.08,
    seed=42,
):
    """
    Detect dominant ground plane using RANSAC.

    The fixed Open3D seed makes repeated runs substantially
    more reproducible.
    """

    points = np.asarray(
        points,
        dtype=np.float64,
    )

    if len(points) < 3:
        return np.zeros(
            len(points),
            dtype=bool,
        )

    o3d.utility.random.seed(seed)

    cloud = o3d.geometry.PointCloud()

    cloud.points = (
        o3d.utility.Vector3dVector(
            points
        )
    )

    _, inliers = cloud.segment_plane(
        distance_threshold=distance_threshold,
        ransac_n=3,
        num_iterations=1000,
    )

    ground_mask = np.zeros(
        len(points),
        dtype=bool,
    )

    ground_mask[inliers] = True

    return ground_mask


# ============================================================
# Cell geometry
# ============================================================

def cell_bounds(cell):
    """
    Horizontal XY bounds of one adaptive cell.
    """

    x, y = cell.center

    half = cell.resolution / 2.0

    return (
        x - half,
        x + half,
        y - half,
        y + half,
    )


def cells_touch(cell_a, cell_b):
    """
    Resolution-aware XY adjacency.
    """

    ax0, ax1, ay0, ay1 = cell_bounds(
        cell_a
    )

    bx0, bx1, by0, by1 = cell_bounds(
        cell_b
    )

    tolerance = (
        max(
            cell_a.resolution,
            cell_b.resolution,
        )
        * 0.05
    )

    x_connected = (
        ax0 <= bx1 + tolerance
        and ax1 >= bx0 - tolerance
    )

    y_connected = (
        ay0 <= by1 + tolerance
        and ay1 >= by0 - tolerance
    )

    return (
        x_connected
        and y_connected
    )


# ============================================================
# Adaptive-level helper
# ============================================================

def max_grid_resolution():
    """
    Safely obtain the largest AdaptiveGrid resolution whether
    LEVELS is a dict, list, tuple, or ndarray.
    """

    levels = AdaptiveGrid.LEVELS

    if isinstance(
        levels,
        dict,
    ):
        return float(
            max(levels.values())
        )

    return float(
        np.max(
            np.asarray(
                levels,
                dtype=float,
            )
        )
    )


# ============================================================
# Spatial hashing
# ============================================================

def build_spatial_hash(cells):
    """
    Build a coarse spatial hash for efficient neighborhood
    queries.
    """

    bucket_size = max_grid_resolution()

    spatial_hash = {}

    for index, cell in enumerate(cells):

        x, y = cell.center

        bx = int(
            np.floor(
                x / bucket_size
            )
        )

        by = int(
            np.floor(
                y / bucket_size
            )
        )

        key = (
            bx,
            by,
        )

        spatial_hash.setdefault(
            key,
            []
        ).append(index)

    return (
        spatial_hash,
        bucket_size,
    )


def candidate_neighbors(
    cell_index,
    cells,
    spatial_hash,
    bucket_size,
):
    """
    Return neighboring cell candidates.
    """

    cell = cells[cell_index]

    x, y = cell.center

    bx = int(
        np.floor(
            x / bucket_size
        )
    )

    by = int(
        np.floor(
            y / bucket_size
        )
    )

    candidates = []

    for dx in (
        -1,
        0,
        1,
    ):

        for dy in (
            -1,
            0,
            1,
        ):

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
# Connected components
# ============================================================

def connected_components(
    obstacle_cells,
):
    """
    Resolution-aware connected-component extraction.
    """

    if not obstacle_cells:
        return []

    spatial_hash, bucket_size = (
        build_spatial_hash(
            obstacle_cells
        )
    )

    visited = np.zeros(
        len(obstacle_cells),
        dtype=bool,
    )

    components = []

    for start_index in range(
        len(obstacle_cells)
    ):

        if visited[start_index]:
            continue

        component = ObjectComponent(
            len(components) + 1
        )

        queue = deque(
            [start_index]
        )

        visited[start_index] = True

        while queue:

            current_index = (
                queue.popleft()
            )

            current_cell = (
                obstacle_cells[
                    current_index
                ]
            )

            component.add(
                current_cell
            )

            candidates = candidate_neighbors(
                current_index,
                obstacle_cells,
                spatial_hash,
                bucket_size,
            )

            for neighbor_index in candidates:

                if visited[neighbor_index]:
                    continue

                neighbor_cell = (
                    obstacle_cells[
                        neighbor_index
                    ]
                )

                # ------------------------------------------------
                # Horizontal adjacency
                # ------------------------------------------------

                if not cells_touch(
                    current_cell,
                    neighbor_cell,
                ):
                    continue

                # ------------------------------------------------
                # Vertical compatibility
                # ------------------------------------------------

                height_difference = abs(
                    current_cell.obstacle_height
                    - neighbor_cell.obstacle_height
                )

                allowed_difference = max(
                    0.75,
                    2.0
                    * max(
                        current_cell.resolution,
                        neighbor_cell.resolution,
                    ),
                )

                if (
                    height_difference
                    > allowed_difference
                ):
                    continue

                visited[neighbor_index] = True

                queue.append(
                    neighbor_index
                )

        components.append(
            component
        )

    return components


# ============================================================
# Component filtering
# ============================================================

def filter_components(
    components,
    minimum_cells=3,
    minimum_points=5,
):
    """
    Remove very small components.
    """

    filtered = []

    for component in components:

        cell_count = len(
            component.cells
        )

        point_count = sum(
            cell.obstacle_count
            for cell in component.cells
        )

        if cell_count < minimum_cells:
            continue

        if point_count < minimum_points:
            continue

        filtered.append(
            component
        )

    return filtered


# ============================================================
# Component merging
# ============================================================

def component_horizontal_gap(
    component_a,
    component_b,
):
    """
    Minimum XY bounding-box gap between two components.
    """

    bounds_a = component_a.bounds
    bounds_b = component_b.bounds

    if (
        bounds_a is None
        or bounds_b is None
    ):
        return float("inf")

    ax0, ax1, ay0, ay1, _, _ = (
        bounds_a
    )

    bx0, bx1, by0, by1, _, _ = (
        bounds_b
    )

    if ax1 < bx0:
        gap_x = bx0 - ax1

    elif bx1 < ax0:
        gap_x = ax0 - bx1

    else:
        gap_x = 0.0

    if ay1 < by0:
        gap_y = by0 - ay1

    elif by1 < ay0:
        gap_y = ay0 - by1

    else:
        gap_y = 0.0

    return float(
        np.hypot(
            gap_x,
            gap_y,
        )
    )


def component_height_difference(
    component_a,
    component_b,
):
    return abs(
        component_a.mean_height
        - component_b.mean_height
    )


def merged_dimensions(
    component_a,
    component_b,
):
    """
    Dimensions of the combined bounding box.
    """

    bounds_a = component_a.bounds
    bounds_b = component_b.bounds

    min_x = min(
        bounds_a[0],
        bounds_b[0],
    )

    max_x = max(
        bounds_a[1],
        bounds_b[1],
    )

    min_y = min(
        bounds_a[2],
        bounds_b[2],
    )

    max_y = max(
        bounds_a[3],
        bounds_b[3],
    )

    min_z = min(
        bounds_a[4],
        bounds_b[4],
    )

    max_z = max(
        bounds_a[5],
        bounds_b[5],
    )

    return (
        max_x - min_x,
        max_y - min_y,
        max_z - min_z,
    )


def should_merge_components(
    component_a,
    component_b,
):
    """
    Experimental object reconstruction merge.

    More tolerant than terrain-level merging because a physical
    object may appear as disconnected obstacle components in a
    sparse LiDAR scan.
    """

    gap = component_horizontal_gap(
        component_a,
        component_b,
    )

    distance = min(
        component_a.distance,
        component_b.distance,
    )

    # --------------------------------------------------------
    # More tolerant distance-aware gap
    # --------------------------------------------------------

    if distance < 15.0:
        max_gap = 0.50

    elif distance < 30.0:
        max_gap = 0.60

    elif distance < 60.0:
        max_gap = 0.90

    else:
        max_gap = 1.25

    if gap > max_gap:
        return False

    # --------------------------------------------------------
    # Relaxed height compatibility
    # --------------------------------------------------------

    height_difference = (
        component_height_difference(
            component_a,
            component_b,
        )
    )

    if height_difference > 0.75:
        return False

    # --------------------------------------------------------
    # Combined object dimensions
    # --------------------------------------------------------

    dx, dy, dz = merged_dimensions(
        component_a,
        component_b,
    )

    horizontal_max = max(
        dx,
        dy,
    )

    horizontal_min = min(
        dx,
        dy,
    )

    # Avoid absurdly large merged structures.
    if horizontal_max > 8.0:
        return False

    # Prevent very long thin structures from chain-merging.
    if (
        horizontal_min < 0.10
        and horizontal_max > 6.0
    ):
        return False

    if dz > 4.5:
        return False

    return True


def merge_two_components(
    component_a,
    component_b,
    component_id,
):
    """
    Merge two ObjectComponents.
    """

    merged = ObjectComponent(
        component_id
    )

    for cell in component_a.cells:
        merged.add(cell)

    for cell in component_b.cells:
        merged.add(cell)

    return merged


def merge_components(
    components,
):
    """
    Closest-first conservative component merging.

    Recomputes valid candidates after every merge to prevent
    uncontrolled chain merging.
    """

    components = list(
        components
    )

    if len(components) <= 1:
        return components

    while True:

        best_pair = None

        best_gap = float(
            "inf"
        )

        # ----------------------------------------------------
        # Find closest valid pair
        # ----------------------------------------------------

        for i in range(
            len(components)
        ):

            for j in range(
                i + 1,
                len(components)
            ):

                component_a = (
                    components[i]
                )

                component_b = (
                    components[j]
                )

                if not should_merge_components(
                    component_a,
                    component_b,
                ):
                    continue

                gap = component_horizontal_gap(
                    component_a,
                    component_b,
                )

                if gap < best_gap:

                    best_gap = gap

                    best_pair = (
                        i,
                        j,
                    )

        # ----------------------------------------------------
        # Stop if no merge exists
        # ----------------------------------------------------

        if best_pair is None:
            break

        i, j = best_pair

        merged = merge_two_components(
            components[i],
            components[j],
            components[i].component_id,
        )

        # Remove larger index first.
        components.pop(j)
        components.pop(i)

        components.append(
            merged
        )

    # --------------------------------------------------------
    # Sort spatially / by distance
    # --------------------------------------------------------

    components.sort(
        key=lambda component:
            component.distance
    )

    # --------------------------------------------------------
    # Renumber
    # --------------------------------------------------------

    for index, component in enumerate(
        components,
        start=1,
    ):

        component.component_id = index

    return components


# ============================================================
# Geometry / feature extraction
# ============================================================

def component_features(
    cells,
):
    """
    Calculate geometric features for a component.
    """

    min_x = min(
        cell.center[0]
        - cell.resolution / 2.0
        for cell in cells
    )

    max_x = max(
        cell.center[0]
        + cell.resolution / 2.0
        for cell in cells
    )

    min_y = min(
        cell.center[1]
        - cell.resolution / 2.0
        for cell in cells
    )

    max_y = max(
        cell.center[1]
        + cell.resolution / 2.0
        for cell in cells
    )

    min_z = min(
        cell.ground_elevation
        for cell in cells
    )

    max_z = max(
        cell.obstacle_elevation
        for cell in cells
    )

    width = max_x - min_x
    length = max_y - min_y
    height = max_z - min_z

    center_x = (
        min_x + max_x
    ) / 2.0

    center_y = (
        min_y + max_y
    ) / 2.0

    center_z = (
        min_z + max_z
    ) / 2.0

    distance = float(
        np.hypot(
            center_x,
            center_y,
        )
    )

    obstacle_heights = np.asarray(
        [
            cell.obstacle_height
            for cell in cells
        ],
        dtype=float,
    )

    max_height = float(
        np.max(
            obstacle_heights
        )
    )

    mean_height = float(
        np.mean(
            obstacle_heights
        )
    )

    point_count = int(
        sum(
            cell.obstacle_count
            for cell in cells
        )
    )

    cell_count = len(cells)

    horizontal_max = max(
        width,
        length,
    )

    horizontal_min = max(
        min(
            width,
            length,
        ),
        0.05,
    )

    footprint_area = max(
        width * length,
        0.01,
    )

    aspect_ratio = (
        horizontal_max
        / horizontal_min
    )

    verticality = (
        max_height
        / horizontal_min
    )

    density = (
        point_count
        / footprint_area
    )

    return {
        "center": (
            float(center_x),
            float(center_y),
            float(center_z),
        ),
        "dimensions": (
            float(width),
            float(length),
            float(height),
        ),
        "distance": distance,
        "point_count": point_count,
        "cell_count": cell_count,
        "max_height": max_height,
        "mean_height": mean_height,
        "footprint_area": float(
            footprint_area
        ),
        "aspect_ratio": float(
            aspect_ratio
        ),
        "verticality": float(
            verticality
        ),
        "density": float(
            density
        ),
    }


# ============================================================
# Classification
# ============================================================

def classify_object(
    features,
):
    """
    Conservative geometric classification baseline.
    """

    width, length, height = (
        features["dimensions"]
    )

    max_height = features[
        "max_height"
    ]

    point_count = features[
        "point_count"
    ]

    footprint_area = features[
        "footprint_area"
    ]

    aspect_ratio = features[
        "aspect_ratio"
    ]

    horizontal_max = max(
        width,
        length,
    )

    horizontal_min = min(
        width,
        length,
    )

    # --------------------------------------------------------
    # POLE
    # --------------------------------------------------------

    if (
        max_height >= 1.2
        and horizontal_max <= 1.2
        and horizontal_min <= 1.0
        and features["verticality"] >= 2.0
        and point_count >= 10
    ):
        return "POLE"

    # --------------------------------------------------------
    # WALL
    # --------------------------------------------------------

    if (
        max_height >= 1.0
        and horizontal_max >= 4.0
        and horizontal_min <= 2.5
        and aspect_ratio >= 2.0
    ):
        return "WALL"

    # --------------------------------------------------------
    # VEHICLE-LIKE
    # --------------------------------------------------------

    if (
        1.5 <= horizontal_max <= 8.0
        and 0.8 <= horizontal_min <= 3.5
        and 0.7 <= max_height <= 3.2
        and footprint_area >= 1.2
        and point_count >= 50
    ):
        return "VEHICLE-LIKE"

    # --------------------------------------------------------
    # PERSON-LIKE
    # --------------------------------------------------------

    if (
        1.2 <= max_height <= 2.3
        and horizontal_max <= 1.2
        and horizontal_min <= 1.0
        and point_count >= 15
    ):
        return "PERSON-LIKE"

    # --------------------------------------------------------
    # Generic obstacle
    # --------------------------------------------------------

    return "OBSTACLE"


# ============================================================
# Create proposals
# ============================================================

def create_object_proposals(
    components,
):
    """
    Convert components into ObjectProposal instances.
    """

    proposals = []

    for component in components:

        features = component_features(
            component.cells
        )

        proposal = ObjectProposal(
            object_id=0,
            cells=component.cells,
            classification=classify_object(
                features
            ),
            center=features["center"],
            dimensions=features["dimensions"],
            distance=features["distance"],
            point_count=features["point_count"],
            cell_count=features["cell_count"],
            max_height=features["max_height"],
            mean_height=features["mean_height"],
            footprint_area=features["footprint_area"],
            aspect_ratio=features["aspect_ratio"],
            verticality=features["verticality"],
            density=features["density"],
        )

        proposals.append(
            proposal
        )

    proposals.sort(
        key=lambda proposal:
            proposal.distance
    )

    for index, proposal in enumerate(
        proposals,
        start=1,
    ):

        proposal.object_id = index

    return proposals


# ============================================================
# Exact point-index recovery
# ============================================================

def recover_component_point_indices(
    points,
    component,
):
    """
    Recover exact original point indices belonging to a
    component using its adaptive-cell footprints.

    This works on the exact unchanged point array and therefore
    provides stable indices for SemanticKITTI label lookup.
    """

    points = np.asarray(
        points,
        dtype=np.float64,
    )

    if len(points) == 0:
        return np.empty(
            0,
            dtype=np.int64,
        )

    x = points[:, 0]
    y = points[:, 1]

    component_mask = np.zeros(
        len(points),
        dtype=bool,
    )

    # --------------------------------------------------------
    # Evaluate each component cell.
    #
    # Number of obstacle cells is small, so this is both simple
    # and reliable. The expensive raw-point-to-cell operation
    # can be optimized later.
    # --------------------------------------------------------

    for cell in component.cells:

        cx, cy = cell.center

        half = cell.resolution / 2.0

        cell_mask = (
            (x >= cx - half)
            & (x <= cx + half)
            & (y >= cy - half)
            & (y <= cy + half)
        )

        component_mask |= cell_mask

    return np.flatnonzero(
        component_mask
    ).astype(
        np.int64,
        copy=False,
    )


# ============================================================
# Attach exact points and indices to proposals
# ============================================================

def attach_point_indices(
    proposals,
    components,
    points,
):
    """
    Attach exact original point indices and points to each
    proposal.
    """

    for proposal, component in zip(
        proposals,
        components,
    ):

        indices = (
            recover_component_point_indices(
                points,
                component,
            )
        )

        proposal.point_indices = (
            indices
        )

        proposal.points = (
            points[indices]
            if len(indices)
            else np.empty(
                (0, 3),
                dtype=np.float64,
            )
        )

    return proposals


# ============================================================
# Object detector
# ============================================================

class ObjectDetector:
    """
    ORBIT terrain-relative adaptive object detector.

    Pipeline:

        LiDAR
          ↓
        100m range filter
          ↓
        Ground detection
          ↓
        Adaptive 2.5D grid
          ↓
        Terrain-relative obstacle cells
          ↓
        Connected components
          ↓
        Noise filtering
          ↓
        Conservative component merging
          ↓
        Geometric object classification

    Every ObjectProposal also preserves exact point_indices
    into the input point array.
    """

    def __init__(
        self,
        cluster_tolerance=0.35,
        min_points=5,
        max_range=100.0,
    ):

        # Retained for API compatibility.
        self.cluster_tolerance = (
            cluster_tolerance
        )

        self.min_points = (
            min_points
        )

        self.max_range = (
            max_range
        )

    def detect(
        self,
        points,
    ):
        """
        Detect objects in a point cloud.

        Input points must be Nx3.

        Returned proposal.point_indices refer to the
        original input `points` array.
        """

        original_points = np.asarray(
            points,
            dtype=np.float64,
        )

        if (
            original_points.ndim != 2
            or original_points.shape[1] != 3
        ):
            raise ValueError(
                "points must have shape (N, 3)"
            )

        if len(original_points) == 0:
            return []

        # ----------------------------------------------------
        # Range filter
        #
        # Preserve original indices.
        # ----------------------------------------------------

        distances = np.linalg.norm(
            original_points[:, :2],
            axis=1,
        )

        range_mask = (
            distances <= self.max_range
        )

        filtered_indices = np.flatnonzero(
            range_mask
        ).astype(
            np.int64,
            copy=False,
        )

        points_in_range = (
            original_points[
                filtered_indices
            ]
        )

        if len(points_in_range) == 0:
            return []

        # ----------------------------------------------------
        # Ground
        # ----------------------------------------------------

        ground_mask = detect_ground(
            points_in_range,
            distance_threshold=0.08,
            seed=42,
        )

        # ----------------------------------------------------
        # Adaptive grid
        # ----------------------------------------------------

        grid = AdaptiveGrid()

        grid.build(
            points_in_range,
            ground_mask,
        )

        # ----------------------------------------------------
        # Terrain-relative obstacle cells
        # ----------------------------------------------------

        obstacle_cells = (
            extract_obstacle_cells(
                grid,
                minimum_height=0.15,
                minimum_obstacle_points=3,
            )
        )

        if not obstacle_cells:
            return []

        # ----------------------------------------------------
        # Connected components
        # ----------------------------------------------------

        components = (
            connected_components(
                obstacle_cells
            )
        )

        # ----------------------------------------------------
        # Noise filtering
        # ----------------------------------------------------

        filtered = (
            filter_components(
                components,
                minimum_cells=3,
                minimum_points=self.min_points,
            )
        )

        if not filtered:
            return []

        # ----------------------------------------------------
        # Component merging
        # ----------------------------------------------------

        merged = (
            merge_components(
                filtered
            )
        )

        # ----------------------------------------------------
        # Create proposals
        # ----------------------------------------------------

        proposals = (
            create_object_proposals(
                merged
            )
        )

        # ----------------------------------------------------
        # Recover indices relative to filtered points
        # ----------------------------------------------------

        attach_point_indices(
            proposals,
            merged,
            points_in_range,
        )

        # ----------------------------------------------------
        # Convert filtered-point indices back to ORIGINAL
        # input indices.
        # ----------------------------------------------------

        for proposal in proposals:

            local_indices = (
                proposal.point_indices
            )

            if len(local_indices):

                original_point_indices = (
                    filtered_indices[
                        local_indices
                    ]
                )

                proposal.point_indices = (
                    original_point_indices
                )

                proposal.points = (
                    original_points[
                        original_point_indices
                    ]
                )

            else:

                proposal.point_indices = (
                    np.empty(
                        0,
                        dtype=np.int64,
                    )
                )

                proposal.points = (
                    np.empty(
                        (0, 3),
                        dtype=np.float64,
                    )
                )

        return proposals


# ============================================================
# Reporting
# ============================================================

def print_object(
    proposal,
):
    """
    Print one object proposal.
    """

    width, length, height = (
        proposal.dimensions
    )

    x, y, z = proposal.center

    print(
        f"\nObject #{proposal.object_id}"
    )

    print(
        f"  Classification: "
        f"{proposal.classification}"
    )

    print(
        f"  Cells:          "
        f"{proposal.cell_count:,}"
    )

    print(
        f"  Points:         "
        f"{proposal.point_count:,}"
    )

    print(
        f"  Dimensions:     "
        f"{width:.2f}m × "
        f"{length:.2f}m × "
        f"{height:.2f}m"
    )

    print(
        f"  Center:         "
        f"({x:.2f}, {y:.2f}, {z:.2f})m"
    )

    print(
        f"  Distance:       "
        f"{proposal.distance:.2f}m"
    )

    print(
        f"  Max height:     "
        f"{proposal.max_height:.2f}m"
    )

    print(
        f"  Mean height:    "
        f"{proposal.mean_height:.2f}m"
    )

    print(
        f"  Footprint area: "
        f"{proposal.footprint_area:.2f}m²"
    )

    print(
        f"  Aspect ratio:   "
        f"{proposal.aspect_ratio:.2f}"
    )

    print(
        f"  Verticality:    "
        f"{proposal.verticality:.2f}"
    )

    print(
        f"  Density:        "
        f"{proposal.density:.2f} pts/m²"
    )

    print(
        f"  GT index count: "
        f"{len(proposal.point_indices):,}"
    )


# ============================================================
# Main
# ============================================================

def main():

    print("=" * 70)
    print(
        "ORBIT - TERRAIN RELATIVE OBJECT DETECTION"
    )
    print("=" * 70)

    # --------------------------------------------------------
    # Load SemanticKITTI frame
    # --------------------------------------------------------

    print(
        "\nLoading SemanticKITTI frame..."
    )

    points = load_semantic_kitti_frame(
        "data/semantic_kitti/"
        "sequences/00/velodyne/000000.bin"
    )

    print(
        f"Input points: "
        f"{len(points):,}"
    )

    # --------------------------------------------------------
    # Ground
    # --------------------------------------------------------

    print(
        "\nDetecting ground..."
    )

    ground_mask = detect_ground(
        points,
        distance_threshold=0.08,
        seed=42,
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
    # Detector
    # --------------------------------------------------------

    detector = ObjectDetector(
        min_points=5,
        max_range=100.0,
    )

    print(
        "\nRunning ORBIT terrain-relative "
        "object detection..."
    )

    proposals = detector.detect(
        points
    )

    # --------------------------------------------------------
    # Results
    # --------------------------------------------------------

    print(
        f"\nDetected objects: "
        f"{len(proposals)}"
    )

    print(
        "\nObject proposals:"
    )

    print(
        "-" * 70
    )

    for proposal in proposals:

        print_object(
            proposal
        )

    # --------------------------------------------------------
    # Summary
    # --------------------------------------------------------

    summary = {}

    for proposal in proposals:

        label = proposal.classification

        summary[label] = (
            summary.get(
                label,
                0,
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

    for label, count in sorted(
        summary.items()
    ):

        print(
            f"  {label:<20}"
            f"{count}"
        )

    print(
        "\n" + "=" * 70
    )

    print(
        "ORBIT TERRAIN RELATIVE "
        "OBJECT DETECTION COMPLETE"
    )

    print(
        "=" * 70
    )


if __name__ == "__main__":
    main()