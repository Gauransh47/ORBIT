from collections import deque, defaultdict
from dataclasses import dataclass
from math import sqrt
from pathlib import Path

import numpy as np

from src.mapping.semantic_grid import SemanticGrid
from src.mapping.adaptive_grid import AdaptiveGrid
from src.datasets.kitti_loader import (
    KittiLoader,
    semantic_name,
)


# ============================================================
# CONFIGURATION
# ============================================================

# Global defaults. Individual semantic classes override these below.
MIN_POINTS = 8
MIN_CELLS = 2

# Class-aware object quality thresholds.
# These are intentionally conservative: we want physical objects,
# not isolated LiDAR/semantic fragments.
CLASS_QUALITY = {
    # vehicles
    10: {"min_points": 20, "min_cells": 2, "min_extent": 0.20},  # car
    11: {"min_points": 12, "min_cells": 2, "min_extent": 0.15},  # bicycle
    13: {"min_points": 20, "min_cells": 2, "min_extent": 0.20},  # bus
    15: {"min_points": 12, "min_cells": 2, "min_extent": 0.15},  # motorcycle
    18: {"min_points": 20, "min_cells": 2, "min_extent": 0.20},  # truck
    20: {"min_points": 15, "min_cells": 2, "min_extent": 0.15},  # other vehicle

    # people
    30: {"min_points": 10, "min_cells": 2, "min_extent": 0.15},
    31: {"min_points": 10, "min_cells": 2, "min_extent": 0.15},
    32: {"min_points": 10, "min_cells": 2, "min_extent": 0.15},

    # structures
    50: {"min_points": 30, "min_cells": 3, "min_extent": 0.30},  # building
    51: {"min_points": 15, "min_cells": 2, "min_extent": 0.20},  # fence
    52: {"min_points": 20, "min_cells": 2, "min_extent": 0.20},  # structure

    # environment / vertical objects
    71: {"min_points": 10, "min_cells": 2, "min_extent": 0.10},  # trunk
    80: {"min_points": 8, "min_cells": 2, "min_extent": 0.08},   # pole
    81: {"min_points": 5, "min_cells": 2, "min_extent": 0.05},   # sign

    # miscellaneous
    99: {"min_points": 12, "min_cells": 2, "min_extent": 0.10},
}

# Same-class hypotheses closer than this may be fragments of one object.
MERGE_DISTANCE = {
    "default": 0.60,
    50: 1.50,   # buildings
    51: 0.80,   # fences
    52: 1.00,   # structures
    80: 0.45,   # poles
    81: 0.50,   # traffic signs
    71: 0.60,   # trunks
}

NEIGHBOR_SCALE = 1.75

# Spatial-hash bucket size.
#
# 0.25 m gives us a good compromise:
#   - small enough to avoid huge candidate lists
#   - large enough to avoid excessive dictionary buckets
#
# This replaces the previous O(N^2) all-cell comparison.
SPATIAL_BUCKET_SIZE = 0.25


# ============================================================
# SEMANTIC GROUPS
# ============================================================

# Classes that can form physical object hypotheses.
OBJECT_CLASSES = {
    10,   # car
    11,   # bicycle
    13,   # bus
    15,   # motorcycle
    18,   # truck
    20,   # other-vehicle

    30,   # person
    31,   # bicyclist
    32,   # motorcyclist

    50,   # building
    51,   # fence
    52,   # other-structure

    # Lane markings are retained by SemanticGrid but are not
    # promoted to physical object hypotheses.

    71,   # trunk
    80,   # pole
    81,   # traffic-sign
    99,   # other-object
}


BACKGROUND_CLASSES = {
    0,    # unlabeled
    1,    # outlier
    40,   # road
    44,   # parking
    48,   # sidewalk
    49,   # other-ground
    70,   # vegetation
    72,   # terrain
}


# ============================================================
# OBJECT HYPOTHESIS
# ============================================================

@dataclass
class ObjectHypothesis:

    object_id: int
    semantic_id: int
    semantic_class: str

    confidence: float
    entropy: float

    cells: int
    points: int

    center_x: float
    center_y: float

    width: float
    length: float

    min_x: float
    max_x: float
    min_y: float
    max_y: float

    def distance(self):
        return sqrt(
            self.center_x * self.center_x
            + self.center_y * self.center_y
        )

    def as_dict(self):

        return {
            "object_id": self.object_id,
            "semantic_id": self.semantic_id,
            "class": self.semantic_class,
            "confidence": self.confidence,
            "entropy": self.entropy,
            "cells": self.cells,
            "points": self.points,
            "center": (
                self.center_x,
                self.center_y,
            ),
            "dimensions": (
                self.width,
                self.length,
            ),
            "distance": self.distance(),
        }


# ============================================================
# ASSOCIATOR
# ============================================================

class SemanticObjectAssociator:

    """
    Convert semantic 2.5D cells into physical object hypotheses.

    The original implementation searched every available cell
    against every other cell during neighbor discovery.

    That becomes extremely expensive as the semantic grid grows.

    This implementation uses:

        semantic filtering
              ↓
        precomputed cell geometry
              ↓
        spatial hash
              ↓
        local neighbor search
              ↓
        semantic compatibility
              ↓
        connected components
              ↓
        object hypotheses

    Complexity is approximately:

        O(N * K)

    where K is the number of nearby spatial candidates,

    instead of:

        O(N²)
    """

    def __init__(
        self,
        min_points=MIN_POINTS,
        min_cells=MIN_CELLS,
    ):

        self.min_points = int(min_points)
        self.min_cells = int(min_cells)

        self.bucket_size = float(
            SPATIAL_BUCKET_SIZE
        )

        # Cache level resolutions once.
        self.levels = AdaptiveGrid.LEVELS

    # --------------------------------------------------------
    # Cell geometry
    # --------------------------------------------------------

    def cell_geometry(self, key):

        level, ix, iy = key

        resolution = float(
            self.levels[int(level)]
        )

        center_x = (
            (int(ix) + 0.5)
            * resolution
        )

        center_y = (
            (int(iy) + 0.5)
            * resolution
        )

        return (
            resolution,
            center_x,
            center_y,
        )

    # --------------------------------------------------------
    # Spatial bucket
    # --------------------------------------------------------

    def spatial_bucket(
        self,
        x,
        y,
    ):

        return (
            int(np.floor(
                x / self.bucket_size
            )),
            int(np.floor(
                y / self.bucket_size
            )),
        )

    # --------------------------------------------------------
    # Build spatial index
    # --------------------------------------------------------

    def build_spatial_index(
        self,
        metadata,
    ):

        buckets = defaultdict(list)

        for key, info in metadata.items():

            bucket = self.spatial_bucket(
                info["center_x"],
                info["center_y"],
            )

            buckets[bucket].append(key)

        return buckets

    # --------------------------------------------------------
    # Neighbor search
    # --------------------------------------------------------

    def neighboring_cells(
        self,
        key,
        available,
        metadata,
        spatial_index,
    ):

        """
        Find nearby cells using the spatial hash.

        IMPORTANT:

        We never scan all cells.

        Only buckets intersecting the maximum possible
        search radius are examined.
        """

        info = metadata[key]

        resolution = info["resolution"]
        cx = info["center_x"]
        cy = info["center_y"]

        # Maximum possible neighboring resolution.
        max_resolution = max(
            float(value)
            for value in self.levels.values()
        )

        search_radius = (
            max(
                resolution,
                max_resolution,
            )
            * NEIGHBOR_SCALE
            + 0.02
        )

        bucket_radius = int(
            np.ceil(
                search_radius
                / self.bucket_size
            )
        )

        center_bucket = self.spatial_bucket(
            cx,
            cy,
        )

        neighbors = []

        bx, by = center_bucket

        for dx_bucket in range(
            -bucket_radius,
            bucket_radius + 1,
        ):

            for dy_bucket in range(
                -bucket_radius,
                bucket_radius + 1,
            ):

                bucket = (
                    bx + dx_bucket,
                    by + dy_bucket,
                )

                candidates = spatial_index.get(
                    bucket
                )

                if not candidates:
                    continue

                for other in candidates:

                    if other == key:
                        continue

                    if other not in available:
                        continue

                    other_info = metadata[
                        other
                    ]

                    ox = other_info[
                        "center_x"
                    ]

                    oy = other_info[
                        "center_y"
                    ]

                    other_resolution = (
                        other_info[
                            "resolution"
                        ]
                    )

                    threshold = (
                        max(
                            resolution,
                            other_resolution,
                        )
                        * NEIGHBOR_SCALE
                        + 0.02
                    )

                    if (
                        abs(cx - ox)
                        <= threshold
                        and
                        abs(cy - oy)
                        <= threshold
                    ):

                        neighbors.append(
                            other
                        )

        return neighbors

    # --------------------------------------------------------
    # Semantic compatibility
    # --------------------------------------------------------

    @staticmethod
    def semantic_compatible(
        class_a,
        class_b,
    ):

        # Exact semantic match.
        if class_a == class_b:
            return True

        vehicle_classes = {
            10,
            11,
            13,
            15,
            18,
            20,
        }

        person_classes = {
            30,
            31,
            32,
        }

        structure_classes = {
            50,
            51,
            52,
        }

        if (
            class_a in vehicle_classes
            and class_b in vehicle_classes
        ):
            return True

        if (
            class_a in person_classes
            and class_b in person_classes
        ):
            return True

        if (
            class_a in structure_classes
            and class_b in structure_classes
        ):
            return True

        return False

    # --------------------------------------------------------
    # Component statistics
    # --------------------------------------------------------

    def build_hypothesis(
        self,
        component,
        metadata,
        object_id,
    ):

        total_points = sum(
            metadata[key]["points"]
            for key in component
        )

        if (
            len(component)
            < self.min_cells
            and
            total_points
            < self.min_points
        ):
            return None

        class_weights = {}

        confidence_sum = 0.0
        entropy_sum = 0.0

        xs = []
        ys = []

        for key in component:

            info = metadata[key]

            semantic_id = info[
                "semantic_id"
            ]

            points = info["points"]

            class_weights[
                semantic_id
            ] = (
                class_weights.get(
                    semantic_id,
                    0,
                )
                + points
            )

            confidence_sum += (
                info["confidence"]
                * points
            )

            entropy_sum += (
                info["entropy"]
                * points
            )

            xs.append(
                info["center_x"]
            )

            ys.append(
                info["center_y"]
            )

        dominant_id = max(
            class_weights,
            key=class_weights.get,
        )

        dominant_points = (
            class_weights[
                dominant_id
            ]
        )

        semantic_confidence = (
            dominant_points
            / total_points
        )

        weighted_confidence = (
            confidence_sum
            / total_points
        )

        weighted_entropy = (
            entropy_sum
            / total_points
        )

        final_confidence = (
            0.65
            * semantic_confidence
            +
            0.35
            * weighted_confidence
        )

        min_x = min(xs)
        max_x = max(xs)
        min_y = min(ys)
        max_y = max(ys)

        width = (
            max_x
            - min_x
        )

        length = (
            max_y
            - min_y
        )

        center_x = (
            min_x
            + max_x
        ) / 2.0

        center_y = (
            min_y
            + max_y
        ) / 2.0

        return ObjectHypothesis(
            object_id=object_id,
            semantic_id=dominant_id,
            semantic_class=semantic_name(
                dominant_id
            ),
            confidence=float(
                final_confidence
            ),
            entropy=float(
                weighted_entropy
            ),
            cells=len(component),
            points=total_points,
            center_x=float(center_x),
            center_y=float(center_y),
            width=float(width),
            length=float(length),
            min_x=float(min_x),
            max_x=float(max_x),
            min_y=float(min_y),
            max_y=float(max_y),
        )

    # --------------------------------------------------------
    # Object quality filtering
    # --------------------------------------------------------

    def quality_thresholds(self, semantic_id):
        """Return class-specific quality thresholds."""
        return CLASS_QUALITY.get(
            int(semantic_id),
            {
                "min_points": self.min_points,
                "min_cells": self.min_cells,
                "min_extent": 0.05,
            },
        )

    def passes_quality(self, hypothesis):
        """
        Reject tiny semantic fragments while preserving genuinely
        small objects such as poles and traffic signs.
        """
        rules = self.quality_thresholds(
            hypothesis.semantic_id
        )

        extent = max(
            hypothesis.width,
            hypothesis.length,
        )

        if hypothesis.points < rules["min_points"]:
            return False

        if hypothesis.cells < rules["min_cells"]:
            return False

        if extent < rules["min_extent"]:
            return False

        return True

    # --------------------------------------------------------
    # Hypothesis merging
    # --------------------------------------------------------

    @staticmethod
    def _merge_pair(a, b):
        """Merge two compatible object hypotheses."""
        total_points = a.points + b.points

        if total_points <= 0:
            return a

        # Point-weighted center/confidence/entropy.
        a_weight = a.points / total_points
        b_weight = b.points / total_points

        center_x = (
            a.center_x * a_weight
            + b.center_x * b_weight
        )

        center_y = (
            a.center_y * a_weight
            + b.center_y * b_weight
        )

        confidence = (
            a.confidence * a_weight
            + b.confidence * b_weight
        )

        entropy = (
            a.entropy * a_weight
            + b.entropy * b_weight
        )

        min_x = min(a.min_x, b.min_x)
        max_x = max(a.max_x, b.max_x)
        min_y = min(a.min_y, b.min_y)
        max_y = max(a.max_y, b.max_y)

        # Keep the dominant semantic ID.
        semantic_id = (
            a.semantic_id
            if a.points >= b.points
            else b.semantic_id
        )

        return ObjectHypothesis(
            object_id=-1,
            semantic_id=semantic_id,
            semantic_class=semantic_name(semantic_id),
            confidence=float(confidence),
            entropy=float(entropy),
            cells=a.cells + b.cells,
            points=total_points,
            center_x=float(center_x),
            center_y=float(center_y),
            width=float(max_x - min_x),
            length=float(max_y - min_y),
            min_x=float(min_x),
            max_x=float(max_x),
            min_y=float(min_y),
            max_y=float(max_y),
        )

    def merge_fragments(self, objects):
        """
        Merge nearby same-class hypotheses.

        This is deliberately performed after connected components:
        it fixes semantic fragmentation without turning the initial
        grid connectivity search into an overly aggressive merge.
        """
        if len(objects) < 2:
            return objects

        changed = True

        while changed:
            changed = False
            used = [False] * len(objects)
            merged = []

            for i, obj in enumerate(objects):
                if used[i]:
                    continue

                current = obj
                used[i] = True

                for j in range(i + 1, len(objects)):
                    if used[j]:
                        continue

                    other = objects[j]

                    if not self.semantic_compatible(
                        current.semantic_id,
                        other.semantic_id,
                    ):
                        continue

                    # Do not merge across different semantic families.
                    if (
                        current.semantic_id
                        != other.semantic_id
                    ):
                        continue

                    threshold = MERGE_DISTANCE.get(
                        current.semantic_id,
                        MERGE_DISTANCE["default"],
                    )

                    # Bounding-box gap is a better merge criterion
                    # than center distance for elongated structures.
                    gap_x = max(
                        0.0,
                        max(
                            current.min_x - other.max_x,
                            other.min_x - current.max_x,
                        ),
                    )

                    gap_y = max(
                        0.0,
                        max(
                            current.min_y - other.max_y,
                            other.min_y - current.max_y,
                        ),
                    )

                    gap = sqrt(
                        gap_x * gap_x
                        + gap_y * gap_y
                    )

                    if gap <= threshold:
                        current = self._merge_pair(
                            current,
                            other,
                        )
                        used[j] = True
                        changed = True

                merged.append(current)

            objects = merged

        return objects

    # --------------------------------------------------------
    # Association
    # --------------------------------------------------------

    def associate(
        self,
        semantic_grid,
    ):

        cells = (
            semantic_grid.semantic_counts
        )

        if not cells:
            return []

        # ----------------------------------------------------
        # Metadata
        # ----------------------------------------------------

        metadata = {}

        for key in cells:

            semantic_id, confidence, points = (
                semantic_grid.dominant_class(
                    key
                )
            )

            entropy = (
                semantic_grid.entropy(
                    key
                )
            )

            if semantic_id is None:
                continue

            semantic_id = int(
                semantic_id
            )

            # ------------------------------------------------
            # IMPORTANT:
            #
            # Terrain/background cells are not objects.
            #
            # This dramatically reduces the association
            # search space on real driving scenes.
            # ------------------------------------------------

            if semantic_id in BACKGROUND_CLASSES:
                continue

            if semantic_id not in OBJECT_CLASSES:
                continue

            resolution, cx, cy = (
                self.cell_geometry(key)
            )

            metadata[key] = {
                "semantic_id": semantic_id,
                "confidence": float(
                    confidence
                ),
                "entropy": float(
                    entropy
                ),
                "points": int(points),
                "resolution": resolution,
                "center_x": cx,
                "center_y": cy,
            }

        if not metadata:
            return []

        # ----------------------------------------------------
        # Spatial index
        # ----------------------------------------------------

        spatial_index = (
            self.build_spatial_index(
                metadata
            )
        )

        available = set(
            metadata.keys()
        )

        objects = []

        object_id = 0

        # ----------------------------------------------------
        # Connected components
        # ----------------------------------------------------

        while available:

            seed = next(
                iter(available)
            )

            queue = deque([
                seed
            ])

            available.remove(
                seed
            )

            component = []

            while queue:

                current = (
                    queue.popleft()
                )

                component.append(
                    current
                )

                current_class = (
                    metadata[current][
                        "semantic_id"
                    ]
                )

                neighbors = (
                    self.neighboring_cells(
                        current,
                        available,
                        metadata,
                        spatial_index,
                    )
                )

                for neighbor in neighbors:

                    neighbor_class = (
                        metadata[neighbor][
                            "semantic_id"
                        ]
                    )

                    if not self.semantic_compatible(
                        current_class,
                        neighbor_class,
                    ):
                        continue

                    available.remove(
                        neighbor
                    )

                    queue.append(
                        neighbor
                    )

            # ------------------------------------------------
            # Build object
            # ------------------------------------------------

            hypothesis = (
                self.build_hypothesis(
                    component,
                    metadata,
                    object_id,
                )
            )

            if hypothesis is not None:

                if self.passes_quality(
                    hypothesis
                ):
                    objects.append(
                        hypothesis
                    )

                    object_id += 1

        # ----------------------------------------------------
        # Merge fragments left by semantic/grid boundaries
        # ----------------------------------------------------

        objects = self.merge_fragments(
            objects
        )

        # Re-apply quality filtering after merging.
        objects = [
            obj
            for obj in objects
            if self.passes_quality(obj)
        ]

        # ----------------------------------------------------
        # Stable ordering by distance
        # ----------------------------------------------------

        objects.sort(
            key=lambda obj:
                obj.distance()
        )

        # Reassign stable IDs.
        for index, obj in enumerate(
            objects
        ):

            obj.object_id = index

        return objects


# ============================================================
# DISPLAY
# ============================================================

def print_objects(objects):

    print(
        "\nSEMANTIC OBJECT HYPOTHESES"
    )

    print("-" * 70)

    if not objects:

        print(
            "  No objects detected."
        )

        return

    # Keep terminal output useful on real scenes.
    # The complete list remains available through the returned
    # ObjectHypothesis objects.
    max_display = 40

    if len(objects) > max_display:
        print(
            f"  Showing nearest {max_display} "
            f"of {len(objects)} objects."
        )

    for obj in objects[:max_display]:

        print(
            f"\nObject #{obj.object_id}"
        )


        print(
            f"  Class:       "
            f"{obj.semantic_class}"
        )

        print(
            f"  Confidence:  "
            f"{obj.confidence * 100:.1f}%"
        )

        print(
            f"  Entropy:     "
            f"{obj.entropy:.3f}"
        )

        print(
            f"  Cells:       "
            f"{obj.cells}"
        )

        print(
            f"  Points:      "
            f"{obj.points:,}"
        )

        print(
            f"  Center:      "
            f"({obj.center_x:.2f}, "
            f"{obj.center_y:.2f}) m"
        )

        print(
            f"  Distance:    "
            f"{obj.distance():.2f} m"
        )

        print(
            f"  Dimensions:  "
            f"{obj.width:.2f} × "
            f"{obj.length:.2f} m"
        )


def main():

    import argparse

    parser = argparse.ArgumentParser(
        description="ORBIT semantic object association"
    )

    parser.add_argument(
        "--frame",
        type=int,
        default=0,
        help="Frame number to process",
    )

    parser.add_argument(
        "--sequence",
        type=str,
        default="00",
        help="KITTI sequence number",
    )

    args = parser.parse_args()

    frame = f"{args.frame:06d}"

    print("=" * 70)

    print(
        "ORBIT - SEMANTIC OBJECT ASSOCIATION"
    )

    print("=" * 70)

    dataset = KittiLoader(
        "data/semantic_kitti",
        sequence=args.sequence,
    )

    # --------------------------------------------------------
    # Validate frame
    # --------------------------------------------------------

    available_frames = dataset.frame_ids()

    if frame not in available_frames:

        raise RuntimeError(
            f"Frame {frame} not found.\n"
            f"Available frames: "
            f"{available_frames}"
        )

    points, labels, instances = (
        dataset.load_frame(
            frame,
            load_labels=True,
        )
    )

    if labels is None:

        raise RuntimeError(
            "Semantic labels required."
        )

    xyz = points[:, :3]

    print(
        f"\nSequence:     {args.sequence}"
    )

    print(
        f"Frame:        {frame}"
    )

    print(
        f"Input points: {len(xyz):,}"
    )

    # --------------------------------------------------------
    # Adaptive grid
    # --------------------------------------------------------

    print(
        "\nRunning geometric ground detection..."
    )

    grid = AdaptiveGrid()

    try:

        ground_mask = (
            grid.detect_ground(
                xyz
            )
        )

    except (
        AttributeError,
        TypeError,
    ):

        ground_mask = (
            xyz[:, 2] < 0.0
        )

    grid.build(
        xyz,
        ground_mask,
    )

    # --------------------------------------------------------
    # Semantic grid
    # --------------------------------------------------------

    print(
        "Building semantic grid..."
    )

    semantic_grid = SemanticGrid(
        grid
    )

    semantic_grid.build(
        xyz,
        labels,
    )

    # --------------------------------------------------------
    # Association
    # --------------------------------------------------------

    print(
        "Associating semantic objects..."
    )

    associator = (
        SemanticObjectAssociator()
    )

    objects = (
        associator.associate(
            semantic_grid
        )
    )

    print(
        f"\nSemantic cells: "
        f"{len(semantic_grid.semantic_counts):,}"
    )

    print(
        f"Object hypotheses: "
        f"{len(objects):,}"
    )

    print_objects(
        objects
    )

    print(
        "\n" + "=" * 70
    )

    print(
        "SEMANTIC OBJECT ASSOCIATION COMPLETE"
    )

    print("=" * 70)


if __name__ == "__main__":
    main()