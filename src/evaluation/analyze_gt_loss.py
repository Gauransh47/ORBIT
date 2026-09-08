import numpy as np
from pathlib import Path
from collections import Counter, defaultdict

from src.perception.object_detection import (
    detect_ground,
    load_semantic_kitti_frame,
)

from src.mapping.adaptive_grid import AdaptiveGrid

from src.perception.terrain_obstacle_detection import (
    extract_obstacle_cells,
    estimate_local_ground,
)


TARGET_INSTANCE_ID = 212
TARGET_SEMANTIC_ID = 10

MINIMUM_OBSTACLE_HEIGHT = 0.15
MINIMUM_OBSTACLE_POINTS = 3


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


def point_cell_key(x, y):

    distance_sq = x * x + y * y

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

    resolution = AdaptiveGrid.LEVELS[level]

    ix = int(np.floor(x / resolution))
    iy = int(np.floor(y / resolution))

    return level, ix, iy


def main():

    print("=" * 70)
    print("ORBIT - GT OBSTACLE LOSS ANALYSIS")
    print("=" * 70)

    base = Path(
        "data/semantic_kitti/sequences/00"
    )

    points = load_semantic_kitti_frame(
        base / "velodyne" / "000000.bin"
    )

    semantic, instance = load_labels(
        base / "labels" / "000000.label"
    )

    target_mask = (
        (semantic == TARGET_SEMANTIC_ID)
        &
        (instance == TARGET_INSTANCE_ID)
    )

    target_indices = np.flatnonzero(
        target_mask
    ).astype(np.int64)

    print(
        f"\nTarget GT car points: "
        f"{len(target_indices):,}"
    )

    # --------------------------------------------------------
    # Ground detection
    # --------------------------------------------------------

    ground_mask = detect_ground(
        points,
        distance_threshold=0.08,
        seed=42,
    )

    # --------------------------------------------------------
    # Range filter
    # --------------------------------------------------------

    distances = np.linalg.norm(
        points[:, :2],
        axis=1,
    )

    range_mask = (
        distances <= 100.0
    )

    filtered_indices = np.flatnonzero(
        range_mask
    ).astype(np.int64)

    points_in_range = points[
        filtered_indices
    ]

    ground_in_range = ground_mask[
        filtered_indices
    ]

    # --------------------------------------------------------
    # Grid
    # --------------------------------------------------------

    grid = AdaptiveGrid()

    grid.build(
        points_in_range,
        ground_in_range,
    )

    # --------------------------------------------------------
    # Target cell analysis
    # --------------------------------------------------------

    target_cells = defaultdict(list)

    for original_index in target_indices:

        if not range_mask[original_index]:
            continue

        x, y = points[
            original_index,
            :2,
        ]

        key = point_cell_key(
            x,
            y,
        )

        if key is not None:

            target_cells[key].append(
                original_index
            )

    # --------------------------------------------------------
    # Analyze every target cell
    # --------------------------------------------------------

    reasons = Counter()

    failed_height_values = []

    failed_point_counts = []

    lost_points = []

    passed_points = []

    details = []

    for key, gt_indices in target_cells.items():

        cell = grid.cells.get(key)

        if cell is None:

            reasons["MISSING_CELL"] += len(gt_indices)

            lost_points.extend(gt_indices)

            continue

        ground_z = None
        ground_source = None

        if cell.ground_count > 0:

            ground_z = cell.ground_elevation

            ground_source = "SAME-CELL"

        else:

            ground_z = estimate_local_ground(
                cell,
                grid,
            )

            if ground_z is not None:
                ground_source = "LOCAL"

        if ground_z is None:

            reasons["NO_GROUND_REFERENCE"] += len(
                gt_indices
            )

            lost_points.extend(
                gt_indices
            )

            continue

        obstacle_z = cell.obstacle_elevation

        if obstacle_z is None:

            reasons["NO_OBSTACLE_ELEVATION"] += len(
                gt_indices
            )

            lost_points.extend(
                gt_indices
            )

            continue

        if cell.obstacle_count < MINIMUM_OBSTACLE_POINTS:

            reasons["TOO_FEW_OBSTACLE_POINTS"] += len(
                gt_indices
            )

            failed_point_counts.append(
                (
                    len(gt_indices),
                    cell.obstacle_count,
                )
            )

            lost_points.extend(
                gt_indices
            )

            continue

        height = (
            obstacle_z
            - ground_z
        )

        if height < MINIMUM_OBSTACLE_HEIGHT:

            reasons["HEIGHT_BELOW_THRESHOLD"] += len(
                gt_indices
            )

            failed_height_values.extend(
                [height] * len(gt_indices)
            )

            lost_points.extend(
                gt_indices
            )

            continue

        reasons["PASSED"] += len(gt_indices)

        passed_points.extend(
            gt_indices
        )

        details.append(
            {
                "key": key,
                "gt_points": len(gt_indices),
                "obstacle_count": cell.obstacle_count,
                "ground_source": ground_source,
                "height": height,
            }
        )

    # --------------------------------------------------------
    # Summary
    # --------------------------------------------------------

    print(
        "\n" + "=" * 70
    )

    print(
        "LOSS REASON SUMMARY"
    )

    print(
        "=" * 70
    )

    total = len(target_indices)

    for reason, count in reasons.most_common():

        print(
            f"\n{reason}:"
        )

        print(
            f"  GT points: "
            f"{count:,}"
        )

        print(
            f"  Percentage: "
            f"{count / total * 100:.2f}%"
        )

    # --------------------------------------------------------
    # Failed height distribution
    # --------------------------------------------------------

    if failed_height_values:

        heights = np.asarray(
            failed_height_values,
            dtype=float,
        )

        print(
            "\n" + "=" * 70
        )

        print(
            "FAILED HEIGHT DISTRIBUTION"
        )

        print(
            "=" * 70
        )

        print(
            f"\nCount: "
            f"{len(heights):,}"
        )

        print(
            f"Minimum: "
            f"{heights.min():.4f} m"
        )

        print(
            f"Mean: "
            f"{heights.mean():.4f} m"
        )

        print(
            f"Median: "
            f"{np.median(heights):.4f} m"
        )

        print(
            f"Maximum: "
            f"{heights.max():.4f} m"
        )

        bins = [
            0.00,
            0.05,
            0.10,
            0.15,
        ]

        print(
            "\nBelow-threshold height buckets:"
        )

        for low, high in zip(
            bins[:-1],
            bins[1:],
        ):

            count = np.count_nonzero(
                (
                    heights >= low
                )
                &
                (
                    heights < high
                )
            )

            print(
                f"  {low:.2f}–{high:.2f}m: "
                f"{count}"
            )

    # --------------------------------------------------------
    # Obstacle-count failures
    # --------------------------------------------------------

    if failed_point_counts:

        counts = np.asarray(
            [
                item[1]
                for item in failed_point_counts
            ],
            dtype=int,
        )

        print(
            "\n" + "=" * 70
        )

        print(
            "LOW OBSTACLE-POINT CELLS"
        )

        print(
            "=" * 70
        )

        print(
            f"\nCells failing point-count threshold: "
            f"{len(counts)}"
        )

        print(
            f"Minimum obstacle count: "
            f"{counts.min()}"
        )

        print(
            f"Maximum obstacle count: "
            f"{counts.max()}"
        )

        print(
            f"Mean obstacle count: "
            f"{counts.mean():.2f}"
        )

    # --------------------------------------------------------
    # Conservation
    # --------------------------------------------------------

    accounted = sum(
        reasons.values()
    )

    print(
        "\n" + "=" * 70
    )

    print(
        "ACCOUNTING"
    )

    print(
        "=" * 70
    )

    print(
        f"\nAccounted points: "
        f"{accounted:,}"
    )

    print(
        f"Expected points:  "
        f"{total:,}"
    )

    print(
        f"Difference:        "
        f"{total - accounted}"
    )

    print(
        "\n" + "=" * 70
    )

    print(
        "GT OBSTACLE LOSS ANALYSIS COMPLETE"
    )

    print(
        "=" * 70
    )


if __name__ == "__main__":
    main()