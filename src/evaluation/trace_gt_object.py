import numpy as np
from pathlib import Path
from collections import defaultdict

from src.perception.object_detection import (
    detect_ground,
    load_semantic_kitti_frame,
    connected_components,
    filter_components,
    merge_components,
)

from src.mapping.adaptive_grid import AdaptiveGrid

from src.perception.terrain_obstacle_detection import (
    extract_obstacle_cells,
    estimate_local_ground,
)


# ============================================================
# ORBIT - GT OBJECT POINT TRACE
#
# Traces one SemanticKITTI GT object through the actual ORBIT
# terrain-relative obstacle pipeline.
#
# IMPORTANT:
# This version uses the SAME local-ground estimator as
# extract_obstacle_cells(), so the diagnostic stages agree
# with the actual detector.
# ============================================================


SEMANTIC_LABELS = {

    0: "unlabeled",
    1: "outlier",

    10: "car",
    11: "bicycle",
    13: "bus",
    15: "motorcycle",
    16: "on-rails",
    18: "truck",
    20: "other-vehicle",

    30: "person",
    31: "bicyclist",
    32: "motorcyclist",

    40: "road",
    44: "parking",
    48: "sidewalk",
    49: "other-ground",

    50: "building",
    51: "fence",
    52: "other-structure",

    60: "lane-marking",

    70: "vegetation",
    71: "trunk",
    72: "terrain",

    80: "pole",
    81: "traffic-sign",

    99: "other-object",
}


# ============================================================
# Target GT object
# ============================================================

TARGET_INSTANCE_ID = 212
TARGET_SEMANTIC_ID = 10       # car


# ============================================================
# ORBIT extraction parameters
# Keep these synchronized with ObjectDetector.
# ============================================================

MINIMUM_OBSTACLE_HEIGHT = 0.15
MINIMUM_OBSTACLE_POINTS = 3

MINIMUM_COMPONENT_CELLS = 3
MINIMUM_COMPONENT_POINTS = 5


# ============================================================
# Helper: load SemanticKITTI labels
# ============================================================

def load_labels(path):

    raw = np.fromfile(
        path,
        dtype=np.uint32,
    )

    semantic = (
        raw & 0xFFFF
    ).astype(np.uint16)

    instance = (
        raw >> 16
    ).astype(np.uint16)

    return semantic, instance


# ============================================================
# Helper: find target object points
# ============================================================

def get_target_indices(
    semantic,
    instance,
):

    mask = (
        (semantic == TARGET_SEMANTIC_ID)
        &
        (instance == TARGET_INSTANCE_ID)
    )

    return np.flatnonzero(
        mask
    ).astype(np.int64)


# ============================================================
# Helper: determine adaptive cell
# ============================================================

def point_cell_key(
    x,
    y,
):

    distance_sq = (
        x * x
        + y * y
    )

    if distance_sq < 100.0:

        level = 0

    elif distance_sq < 625.0:

        level = 1

    elif distance_sq < 2500.0:

        level = 2

    elif distance_sq < 10000.0:

        level = 3

    else:

        return None

    resolution = AdaptiveGrid.LEVELS[
        level
    ]

    ix = int(
        np.floor(
            x / resolution
        )
    )

    iy = int(
        np.floor(
            y / resolution
        )
    )

    return (
        level,
        ix,
        iy,
    )


# ============================================================
# Helper: map cells → final component
# ============================================================

def build_component_lookup(
    components,
):

    lookup = {}

    for component_index, component in enumerate(
        components,
        start=1,
    ):

        for cell in component.cells:

            key = (
                cell.level,
                cell.ix,
                cell.iy,
            )

            lookup[key] = component_index

    return lookup


# ============================================================
# Main
# ============================================================

def main():

    print("=" * 70)

    print(
        "ORBIT - GT OBJECT POINT TRACE"
    )

    print("=" * 70)

    # --------------------------------------------------------
    # Paths
    # --------------------------------------------------------

    base = Path(
        "data/semantic_kitti/sequences/00"
    )

    velodyne_path = (
        base
        / "velodyne"
        / "000000.bin"
    )

    label_path = (
        base
        / "labels"
        / "000000.label"
    )

    # --------------------------------------------------------
    # Load point cloud
    # --------------------------------------------------------

    print(
        "\nLoading SemanticKITTI frame..."
    )

    points = load_semantic_kitti_frame(
        velodyne_path
    )

    semantic, instance = load_labels(
        label_path
    )

    # --------------------------------------------------------
    # Validate
    # --------------------------------------------------------

    if len(points) != len(semantic):

        raise ValueError(
            "Point cloud and label count do not match."
        )

    # --------------------------------------------------------
    # Target object
    # --------------------------------------------------------

    target_indices = get_target_indices(
        semantic,
        instance,
    )

    if len(target_indices) == 0:

        raise RuntimeError(
            "Target SemanticKITTI object was not found."
        )

    target_points = points[
        target_indices
    ]

    target_center = np.mean(
        target_points,
        axis=0,
    )

    target_distance = float(
        np.hypot(
            target_center[0],
            target_center[1],
        )
    )

    print(
        "\nTarget GT object:"
    )

    print(
        f"  Semantic class: "
        f"{SEMANTIC_LABELS[TARGET_SEMANTIC_ID]}"
    )

    print(
        f"  Instance ID:    "
        f"{TARGET_INSTANCE_ID}"
    )

    print(
        f"  Total points:   "
        f"{len(target_indices):,}"
    )

    print(
        f"  Center:         "
        f"({target_center[0]:.2f}, "
        f"{target_center[1]:.2f}, "
        f"{target_center[2]:.2f})"
    )

    print(
        f"  Distance:       "
        f"{target_distance:.2f} m"
    )

    # ========================================================
    # Stage 1 - range filter
    # ========================================================

    ranges = np.linalg.norm(
        points[:, :2],
        axis=1,
    )

    range_mask = (
        ranges <= 100.0
    )

    filtered_original_indices = (
        np.flatnonzero(range_mask)
        .astype(np.int64)
    )

    target_in_range_mask = (
        range_mask[target_indices]
    )

    target_range_indices = (
        target_indices[target_in_range_mask]
    )

    print(
        "\n" + "=" * 70
    )

    print(
        "STAGE 1 - RANGE FILTER"
    )

    print(
        "=" * 70
    )

    print(
        f"GT points retained: "
        f"{len(target_range_indices):,} / "
        f"{len(target_indices):,}"
    )

    print(
        f"Retention: "
        f"{len(target_range_indices) / len(target_indices) * 100:.1f}%"
    )

    # ========================================================
    # Stage 2 - ground detection
    # ========================================================

    points_in_range = points[
        filtered_original_indices
    ]

    print(
        "\nRunning ORBIT ground detection..."
    )

    ground_mask = detect_ground(
        points_in_range,
        distance_threshold=0.08,
        seed=42,
    )

    target_local_indices = np.searchsorted(
        filtered_original_indices,
        target_range_indices,
    )

    target_ground_mask = ground_mask[
        target_local_indices
    ]

    ground_count = int(
        np.count_nonzero(
            target_ground_mask
        )
    )

    non_ground_count = (
        len(target_range_indices)
        - ground_count
    )

    print(
        "\n" + "=" * 70
    )

    print(
        "STAGE 2 - GROUND / NON-GROUND"
    )

    print(
        "=" * 70
    )

    print(
        f"GT points classified ground:     "
        f"{ground_count:,}"
    )

    print(
        f"GT points classified non-ground: "
        f"{non_ground_count:,}"
    )

    if len(target_range_indices) > 0:

        print(
            f"Ground percentage:     "
            f"{ground_count / len(target_range_indices) * 100:.1f}%"
        )

        print(
            f"Non-ground percentage: "
            f"{non_ground_count / len(target_range_indices) * 100:.1f}%"
        )

    # ========================================================
    # Stage 3 - adaptive grid
    # ========================================================

    print(
        "\nBuilding adaptive grid..."
    )

    grid = AdaptiveGrid()

    grid.build(
        points_in_range,
        ground_mask,
    )

    print(
        "\n" + "=" * 70
    )

    print(
        "STAGE 3 - ADAPTIVE GRID"
    )

    print(
        "=" * 70
    )

    print(
        f"Adaptive cells: "
        f"{len(grid.cells):,}"
    )

    # --------------------------------------------------------
    # Find cells touched by target object
    # --------------------------------------------------------

    target_cell_keys = defaultdict(list)

    for original_index in target_range_indices:

        x, y = points[
            original_index,
            :2,
        ]

        key = point_cell_key(
            x,
            y,
        )

        if key is not None:

            target_cell_keys[
                key
            ].append(
                original_index
            )

    print(
        f"GT cells touched: "
        f"{len(target_cell_keys):,}"
    )

    # ========================================================
    # Stage 4 - local ground diagnostics
    # ========================================================

    same_cell_ground_points = 0
    local_ground_recovered_points = 0
    no_ground_reference_points = 0

    same_cell_ground_cells = 0
    recovered_ground_cells = 0
    unresolved_cells = 0

    threshold_pass_points = 0
    threshold_fail_points = 0

    cell_diagnostics = []

    for key, gt_indices in (
        target_cell_keys.items()
    ):

        cell = grid.cells.get(
            key
        )

        count = len(
            gt_indices
        )

        if cell is None:
            continue

        # ----------------------------------------------------
        # Same-cell ground
        # ----------------------------------------------------

        if cell.ground_count > 0:

            ground_z = (
                cell.ground_elevation
            )

            reference_type = (
                "SAME-CELL"
            )

            same_cell_ground_cells += 1

            same_cell_ground_points += count

        else:

            # ------------------------------------------------
            # Use the SAME estimator as the actual extraction.
            # ------------------------------------------------

            ground_z = estimate_local_ground(
                cell,
                grid,
            )

            if ground_z is not None:

                reference_type = (
                    "LOCAL-RECOVERED"
                )

                recovered_ground_cells += 1

                local_ground_recovered_points += count

            else:

                reference_type = (
                    "UNRESOLVED"
                )

                unresolved_cells += 1

                no_ground_reference_points += count

        # ----------------------------------------------------
        # Actual height test
        # ----------------------------------------------------

        obstacle_z = (
            cell.obstacle_elevation
        )

        obstacle_eligible = False

        height = None

        if (
            ground_z is not None
            and obstacle_z is not None
            and cell.obstacle_count
            >= MINIMUM_OBSTACLE_POINTS
        ):

            height = (
                obstacle_z
                - ground_z
            )

            if height >= MINIMUM_OBSTACLE_HEIGHT:

                obstacle_eligible = True

                threshold_pass_points += count

            else:

                threshold_fail_points += count

        elif ground_z is not None:

            threshold_fail_points += count

        cell_diagnostics.append({

            "key": key,

            "gt_points": count,

            "ground_count":
                cell.ground_count,

            "obstacle_count":
                cell.obstacle_count,

            "ground_reference":
                reference_type,

            "ground_z":
                ground_z,

            "obstacle_z":
                obstacle_z,

            "height":
                height,

            "obstacle_eligible":
                obstacle_eligible,

        })

    print(
        "\n" + "=" * 70
    )

    print(
        "STAGE 4 - TERRAIN REFERENCE"
    )

    print(
        "=" * 70
    )

    print(
        f"\nSame-cell ground cells: "
        f"{same_cell_ground_cells}"
    )

    print(
        f"GT points with same-cell reference: "
        f"{same_cell_ground_points:,}"
    )

    print(
        f"\nRecovered local-ground cells: "
        f"{recovered_ground_cells}"
    )

    print(
        f"GT points with local-ground reference: "
        f"{local_ground_recovered_points:,}"
    )

    print(
        f"\nUnresolved cells: "
        f"{unresolved_cells}"
    )

    print(
        f"GT points without any ground reference: "
        f"{no_ground_reference_points:,}"
    )

    total_reference_points = (
        same_cell_ground_points
        + local_ground_recovered_points
    )

    print(
        f"\nTotal GT points with terrain reference: "
        f"{total_reference_points:,}"
    )

    if len(target_range_indices) > 0:

        print(
            f"Terrain-reference coverage: "
            f"{total_reference_points / len(target_range_indices) * 100:.1f}%"
        )

    # ========================================================
    # Stage 5 - obstacle extraction
    # ========================================================

    print(
        "\nRunning actual ORBIT obstacle extraction..."
    )

    obstacle_cells = (
        extract_obstacle_cells(
            grid,
            minimum_height=
                MINIMUM_OBSTACLE_HEIGHT,
            minimum_obstacle_points=
                MINIMUM_OBSTACLE_POINTS,
        )
    )

    obstacle_cell_keys = {

        (
            cell.level,
            cell.ix,
            cell.iy,
        )

        for cell in obstacle_cells
    }

    gt_points_in_obstacle_cells = 0

    for key, gt_indices in (
        target_cell_keys.items()
    ):

        if key in obstacle_cell_keys:

            gt_points_in_obstacle_cells += len(
                gt_indices
            )

    print(
        "\n" + "=" * 70
    )

    print(
        "STAGE 5 - OBSTACLE CELL EXTRACTION"
    )

    print(
        "=" * 70
    )

    print(
        f"Obstacle cells: "
        f"{len(obstacle_cells):,}"
    )

    print(
        f"GT points represented by obstacle cells: "
        f"{gt_points_in_obstacle_cells:,} / "
        f"{len(target_range_indices):,}"
    )

    if len(target_range_indices) > 0:

        print(
            f"GT point retention after obstacle "
            f"extraction: "
            f"{gt_points_in_obstacle_cells / len(target_range_indices) * 100:.1f}%"
        )

    # ========================================================
    # Stage 6 - connected components
    # ========================================================

    print(
        "\nRunning connected components..."
    )

    raw_components = connected_components(
        obstacle_cells
    )

    filtered_components = filter_components(
        raw_components,
        minimum_cells=
            MINIMUM_COMPONENT_CELLS,
        minimum_points=
            MINIMUM_COMPONENT_POINTS,
    )

    merged_components = merge_components(
        filtered_components
    )

    print(
        "\n" + "=" * 70
    )

    print(
        "STAGE 6 - COMPONENTS"
    )

    print(
        "=" * 70
    )

    print(
        f"Raw components: "
        f"{len(raw_components)}"
    )

    print(
        f"After filtering: "
        f"{len(filtered_components)}"
    )

    print(
        f"After merging: "
        f"{len(merged_components)}"
    )

    # ========================================================
    # Map target cells into final components
    # ========================================================

    component_lookup = build_component_lookup(
        merged_components
    )

    target_component_counts = defaultdict(
        int
    )

    target_points_in_final_components = 0

    for key, gt_indices in (
        target_cell_keys.items()
    ):

        component_id = (
            component_lookup.get(
                key
            )
        )

        if component_id is None:
            continue

        count = len(
            gt_indices
        )

        target_component_counts[
            component_id
        ] += count

        target_points_in_final_components += count

    # ========================================================
    # Final trace
    # ========================================================

    print(
        "\n" + "=" * 70
    )

    print(
        "FINAL GT POINT TRACE"
    )

    print(
        "=" * 70
    )

    total_gt = len(
        target_indices
    )

    total_range = len(
        target_range_indices
    )

    print(
        f"\nOriginal GT points: "
        f"{total_gt:,}"
    )

    print(
        f"In-range: "
        f"{total_range:,}"
        f" "
        f"({total_range / total_gt * 100:.1f}%)"
    )

    print(
        f"In obstacle cells: "
        f"{gt_points_in_obstacle_cells:,}"
        f" "
        f"({gt_points_in_obstacle_cells / total_gt * 100:.1f}%)"
    )

    print(
        f"In final components: "
        f"{target_points_in_final_components:,}"
        f" "
        f"({target_points_in_final_components / total_gt * 100:.1f}%)"
    )

    # --------------------------------------------------------
    # Final component membership
    # --------------------------------------------------------

    print(
        "\nFinal components containing "
        "GT car points:"
    )

    if not target_component_counts:

        print(
            "  NONE"
        )

    else:

        for component_id, count in sorted(
            target_component_counts.items(),
            key=lambda item:
                -item[1],
        ):

            percentage = (
                count
                / total_gt
                * 100.0
            )

            print(
                f"  Component #{component_id}: "
                f"{count:,} GT points "
                f"({percentage:.1f}%)"
            )

    # ========================================================
    # Loss analysis
    # ========================================================

    print(
        "\n" + "=" * 70
    )

    print(
        "LOSS ANALYSIS"
    )

    print(
        "=" * 70
    )

    print(
        "\nGROUND REFERENCE:"
    )

    print(
        f"  Same-cell ground:       "
        f"{same_cell_ground_points:,}"
    )

    print(
        f"  Local ground recovered: "
        f"{local_ground_recovered_points:,}"
    )

    print(
        f"  No ground reference:    "
        f"{no_ground_reference_points:,}"
    )

    print(
        "\nHEIGHT / OBSTACLE TEST:"
    )

    print(
        f"  Passed height test:     "
        f"{threshold_pass_points:,}"
    )

    print(
        f"  Failed height test:     "
        f"{threshold_fail_points:,}"
    )

    # --------------------------------------------------------
    # Conservation check
    # --------------------------------------------------------

    classified_sum = (
        same_cell_ground_points
        + local_ground_recovered_points
        + no_ground_reference_points
    )

    print(
        "\nDiagnostic accounting:"
    )

    print(
        f"  Accounted GT points:    "
        f"{classified_sum:,}"
    )

    print(
        f"  Expected GT points:     "
        f"{total_range:,}"
    )

    if classified_sum == total_range:

        print(
            "  Status:                 OK"
        )

    else:

        print(
            "  Status:                 "
            "WARNING - accounting mismatch"
        )

    # ========================================================
    # Detailed cell diagnostics
    # ========================================================

    print(
        "\n" + "=" * 70
    )

    print(
        "GT CELL DETAILS"
    )

    print(
        "=" * 70
    )

    ordered = sorted(
        cell_diagnostics,
        key=lambda item:
            item["gt_points"],
        reverse=True,
    )

    for diagnostic in ordered[:20]:

        key = diagnostic[
            "key"
        ]

        print(
            f"\nCell {key}"
        )

        print(
            f"  GT points:          "
            f"{diagnostic['gt_points']}"
        )

        print(
            f"  Ground count:       "
            f"{diagnostic['ground_count']}"
        )

        print(
            f"  Obstacle count:     "
            f"{diagnostic['obstacle_count']}"
        )

        print(
            f"  Ground reference:   "
            f"{diagnostic['ground_reference']}"
        )

        if diagnostic["ground_z"] is None:

            print(
                f"  Ground elevation:   NONE"
            )

        else:

            print(
                f"  Ground elevation:   "
                f"{diagnostic['ground_z']:.3f} m"
            )

        if diagnostic["obstacle_z"] is None:

            print(
                f"  Obstacle elevation:  NONE"
            )

        else:

            print(
                f"  Obstacle elevation:  "
                f"{diagnostic['obstacle_z']:.3f} m"
            )

        if diagnostic["height"] is None:

            print(
                f"  Relative height:     NONE"
            )

        else:

            print(
                f"  Relative height:     "
                f"{diagnostic['height']:.3f} m"
            )

        print(
            f"  Obstacle eligible:  "
            f"{diagnostic['obstacle_eligible']}"
        )

    print(
        "\n" + "=" * 70
    )

    print(
        "GT OBJECT TRACE COMPLETE"
    )

    print(
        "=" * 70
    )


if __name__ == "__main__":
    main()