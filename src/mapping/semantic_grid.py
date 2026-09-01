from pathlib import Path

import numpy as np

from src.datasets.kitti_loader import (
    KittiLoader,
    semantic_name,
)
from src.mapping.adaptive_grid import AdaptiveGrid


# ============================================================
# SEMANTIC GROUND-TRUTH GRID
# ============================================================

class SemanticGrid:
    """
    Attach SemanticKITTI semantic labels to ORBIT's
    adaptive 2.5D grid.

    IMPORTANT:
        This class currently uses SemanticKITTI labels as
        ground truth.

        It is an evaluation / visualization layer.
        It is NOT ORBIT's semantic inference engine yet.
    """

    def __init__(self, adaptive_grid):
        self.grid = adaptive_grid

        # key:
        #     (level, ix, iy)
        #
        # value:
        #     dictionary of semantic_id -> point count
        self.semantic_counts = {}

        self.total_labeled_points = 0

    # ========================================================
    # POINT -> ADAPTIVE CELL
    # ========================================================

    @staticmethod
    def point_cell(x, y):
        """
        Return the ORBIT adaptive cell for a point.
        """

        distance_sq = x * x + y * y

        if distance_sq >= 10000.0:
            return None

        if distance_sq < 100.0:
            level = 0

        elif distance_sq < 625.0:
            level = 1

        elif distance_sq < 2500.0:
            level = 2

        else:
            level = 3

        resolution = AdaptiveGrid.LEVELS[level]

        ix = int(np.floor(x / resolution))
        iy = int(np.floor(y / resolution))

        return (
            level,
            ix,
            iy,
        )

    # ========================================================
    # BUILD
    # ========================================================

    def build(self, points, semantic_ids):
        """
        Aggregate semantic labels into adaptive cells.

        Parameters
        ----------
        points : ndarray
            Shape (N, 3)

        semantic_ids : ndarray
            Shape (N,)
            Native SemanticKITTI IDs.
        """

        points = np.asarray(
            points,
            dtype=np.float64,
        )

        semantic_ids = np.asarray(
            semantic_ids,
            dtype=np.uint32,
        )

        if points.ndim != 2 or points.shape[1] != 3:
            raise ValueError(
                "points must have shape (N, 3)"
            )

        if len(points) != len(semantic_ids):
            raise ValueError(
                "points and semantic_ids "
                "must have the same length"
            )

        self.semantic_counts.clear()
        self.total_labeled_points = 0

        x = points[:, 0]
        y = points[:, 1]

        # ----------------------------------------------------
        # Range filter
        # ----------------------------------------------------

        distance_sq = x * x + y * y

        valid = distance_sq < 10000.0

        x = x[valid]
        y = y[valid]
        labels = semantic_ids[valid]

        if len(x) == 0:
            return

        self.total_labeled_points = len(labels)

        # ----------------------------------------------------
        # Determine adaptive level
        # ----------------------------------------------------

        levels = np.empty(
            len(x),
            dtype=np.int8,
        )

        levels[
            distance_sq[valid] < 100.0
        ] = 0

        levels[
            (distance_sq[valid] >= 100.0)
            & (distance_sq[valid] < 625.0)
        ] = 1

        levels[
            (distance_sq[valid] >= 625.0)
            & (distance_sq[valid] < 2500.0)
        ] = 2

        levels[
            distance_sq[valid] >= 2500.0
        ] = 3

        # ----------------------------------------------------
        # Cell coordinates
        # ----------------------------------------------------

        resolutions = np.array(
            [
                AdaptiveGrid.LEVELS[0],
                AdaptiveGrid.LEVELS[1],
                AdaptiveGrid.LEVELS[2],
                AdaptiveGrid.LEVELS[3],
            ],
            dtype=np.float64,
        )

        ix = np.floor(
            x / resolutions[levels]
        ).astype(np.int64)

        iy = np.floor(
            y / resolutions[levels]
        ).astype(np.int64)

        # ----------------------------------------------------
        # Sort by:
        #
        # level -> ix -> iy -> semantic ID
        # ----------------------------------------------------

        order = np.lexsort(
            (
                labels,
                iy,
                ix,
                levels,
            )
        )

        levels = levels[order]
        ix = ix[order]
        iy = iy[order]
        labels = labels[order]

        # ----------------------------------------------------
        # Detect boundaries
        # ----------------------------------------------------

        n = len(labels)

        boundary = np.empty(
            n,
            dtype=bool,
        )

        boundary[0] = True

        if n > 1:

            boundary[1:] = (
                (levels[1:] != levels[:-1])
                |
                (ix[1:] != ix[:-1])
                |
                (iy[1:] != iy[:-1])
                |
                (labels[1:] != labels[:-1])
            )

        starts = np.flatnonzero(
            boundary
        )

        ends = np.empty(
            len(starts),
            dtype=np.int64,
        )

        if len(starts) > 1:
            ends[:-1] = starts[1:]

        ends[-1] = n

        counts = ends - starts

        # ----------------------------------------------------
        # Store semantic counts
        # ----------------------------------------------------

        for start, count in zip(
            starts,
            counts,
        ):

            key = (
                int(levels[start]),
                int(ix[start]),
                int(iy[start]),
            )

            semantic_id = int(
                labels[start]
            )

            cell_counts = (
                self.semantic_counts.setdefault(
                    key,
                    {}
                )
            )

            cell_counts[
                semantic_id
            ] = int(count)

    # ========================================================
    # CELL QUERY
    # ========================================================

    def get_counts(self, key):
        """
        Return semantic class counts for a cell.
        """

        return self.semantic_counts.get(
            key,
            {},
        )

    # ========================================================
    # DOMINANT CLASS
    # ========================================================

    def dominant_class(self, key):
        """
        Return:

            semantic_id
            confidence
            point_count

        """

        counts = self.get_counts(key)

        if not counts:
            return (
                None,
                0.0,
                0,
            )

        semantic_id = max(
            counts,
            key=counts.get,
        )

        total = sum(
            counts.values()
        )

        confidence = (
            counts[semantic_id]
            / total
        )

        return (
            semantic_id,
            confidence,
            total,
        )

    # ========================================================
    # ENTROPY
    # ========================================================

    def entropy(self, key):
        """
        Semantic uncertainty of a cell.

        0.0 means every point belongs to
        the same semantic class.

        Higher values indicate a mixed cell.
        """

        counts = self.get_counts(key)

        if not counts:
            return 0.0

        values = np.asarray(
            list(counts.values()),
            dtype=np.float64,
        )

        probabilities = (
            values / values.sum()
        )

        return float(
            -np.sum(
                probabilities
                * np.log2(
                    probabilities
                )
            )
        )

    # ========================================================
    # STATISTICS
    # ========================================================

    def statistics(self):

        semantic_cells = len(
            self.semantic_counts
        )

        class_totals = {}

        for counts in (
            self.semantic_counts.values()
        ):

            for semantic_id, count in (
                counts.items()
            ):

                class_totals[semantic_id] = (
                    class_totals.get(
                        semantic_id,
                        0
                    )
                    + count
                )

        return {
            "semantic_cells":
                semantic_cells,

            "labeled_points":
                self.total_labeled_points,

            "unique_classes":
                len(class_totals),

            "class_totals":
                class_totals,
        }


# ============================================================
# DEMO / INSPECTION
# ============================================================

def main():

    print("=" * 70)
    print(
        "ORBIT - SEMANTIC ADAPTIVE GRID"
    )
    print("=" * 70)

    dataset = KittiLoader(
        "data/semantic_kitti",
        sequence="00",
    )

    frame = "000000"

    print(
        f"\nLoading frame: {frame}"
    )

    points, labels, instances = (
        dataset.load_frame(
            frame,
            load_labels=True,
        )
    )

    if labels is None:
        raise RuntimeError(
            "No SemanticKITTI labels found."
        )

    xyz = points[:, :3]

    print(
        f"Points: {len(xyz):,}"
    )

    print(
        f"Labels: {len(labels):,}"
    )

    # --------------------------------------------------------
    # Ground detection
    #
    # Temporary geometry-based detector.
    # Semantic labels are NOT used here.
    # --------------------------------------------------------

    print(
        "\nRunning geometric ground detection..."
    )

    # Simple baseline for now.
    #
    # We will replace this with ORBIT's
    # actual terrain detector / adaptive
    # ground model later.

    ground_mask = (
        xyz[:, 2] < 0.0
    )

    # --------------------------------------------------------
    # Adaptive grid
    # --------------------------------------------------------

    print(
        "Building adaptive grid..."
    )

    grid = AdaptiveGrid()

    mapped = grid.build(
        xyz,
        ground_mask,
    )

    stats = grid.statistics()

    print(
        f"Adaptive cells: "
        f"{stats['total_cells']:,}"
    )

    print(
        f"Mapped points: "
        f"{len(mapped):,}"
    )

    # --------------------------------------------------------
    # Semantic grid
    # --------------------------------------------------------

    print(
        "\nAttaching semantic labels..."
    )

    semantic_grid = SemanticGrid(
        grid
    )

    semantic_grid.build(
        xyz,
        labels,
    )

    semantic_stats = (
        semantic_grid.statistics()
    )

    print(
        f"Semantic cells: "
        f"{semantic_stats['semantic_cells']:,}"
    )

    print(
        f"Semantic points: "
        f"{semantic_stats['labeled_points']:,}"
    )

    print(
        f"Unique classes: "
        f"{semantic_stats['unique_classes']}"
    )

    # --------------------------------------------------------
    # Class distribution
    # --------------------------------------------------------

    print(
        "\nSEMANTIC DISTRIBUTION"
    )

    print("-" * 70)

    class_totals = (
        semantic_stats[
            "class_totals"
        ]
    )

    for semantic_id, count in sorted(
        class_totals.items(),
        key=lambda item: item[1],
        reverse=True,
    ):

        percentage = (
            count
            / semantic_stats["labeled_points"]
            * 100.0
        )

        print(
            f"  {semantic_id:3d} "
            f"{semantic_name(semantic_id):<24} "
            f"{count:>8,} "
            f"({percentage:5.1f}%)"
        )

    # --------------------------------------------------------
    # Example cells
    # --------------------------------------------------------

    print(
        "\nSAMPLE SEMANTIC CELLS"
    )

    print("-" * 70)

    shown = 0

    for key in semantic_grid.semantic_counts:

        semantic_id, confidence, count = (
            semantic_grid.dominant_class(
                key
            )
        )

        entropy = semantic_grid.entropy(
            key
        )

        print(
            f"  Cell {key} | "
            f"{semantic_name(semantic_id):<18} | "
            f"conf={confidence:.2f} | "
            f"points={count:4d} | "
            f"entropy={entropy:.2f}"
        )

        shown += 1

        if shown >= 15:
            break

    print(
        "\n" + "=" * 70
    )

    print(
        "SEMANTIC GRID COMPLETE"
    )

    print("=" * 70)


if __name__ == "__main__":
    main()