import sys
from pathlib import Path
import time

import numpy as np
import open3d as o3d

# ------------------------------------------------------------
# Make src available when running this file directly
# ------------------------------------------------------------

SRC_DIR = Path(__file__).resolve().parents[1]

if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

from perception.object_proposal import ObjectProposal
from perception.ground_estimation import estimate_ground
from mapping.adaptive_grid import AdaptiveGrid

from perception.terrain_obstacle_detection import (
    extract_obstacle_cells,
)

from perception.connected_components import (
    connected_components,
    filter_components,
)

from perception.object_association import (
    associate_components,
)


# ============================================================
# ORBIT PERCEPTION ENGINE
# ============================================================

class OrbitPerception:
    """
    Complete ORBIT perception pipeline.

    Input:
        Nx3 LiDAR point cloud

    Output:
        Structured perception result containing:

        - ground mask
        - adaptive 2.5D grid
        - obstacle cells
        - connected components
        - final object hypotheses
        - performance metrics
    """

    def __init__(
        self,
        ground_distance_threshold=0.08,
        ransac_iterations=1000,
        minimum_obstacle_height=0.15,
        minimum_obstacle_points=3,
        minimum_component_cells=3,
        minimum_component_points=5,
    ):

        self.ground_distance_threshold = (
            ground_distance_threshold
        )

        self.ransac_iterations = (
            ransac_iterations
        )

        self.minimum_obstacle_height = (
            minimum_obstacle_height
        )

        self.minimum_obstacle_points = (
            minimum_obstacle_points
        )

        self.minimum_component_cells = (
            minimum_component_cells
        )

        self.minimum_component_points = (
            minimum_component_points
        )

    # ========================================================
    # Ground detection
    # ========================================================

    def detect_ground(self, points):
        """Canonical RANSAC ground mask (same estimator as the E2E pipeline)."""

        estimate = estimate_ground(
            points,
            distance_threshold=self.ground_distance_threshold,
            num_iterations=self.ransac_iterations,
        )

        return estimate.mask

    def groups_to_proposals(self, objects):
        """Convert ObjectGroup list into tracker-ready ObjectProposal list."""

        proposals = []

        for obj in objects:
            width, length, height = obj.dimensions
            x, y = obj.center
            confidence = self.compute_confidence(obj)

            proposals.append(
                ObjectProposal(
                    proposal_id=int(obj.object_id),
                    center=(float(x), float(y)),
                    width=float(width),
                    length=float(length),
                    height=float(height),
                    classification=str(obj.classification),
                    confidence=float(confidence),
                    distance=float(obj.distance),
                    point_count=int(obj.point_count),
                    cell_count=int(obj.cell_count),
                    max_height=float(obj.maximum_height),
                    mean_height=float(obj.mean_height),
                    density=float(obj.density),
                    component_count=int(len(obj.components)),
                )
            )

        return proposals

    def detect_from_grid(self, grid):
        """
        Geometric detector given a built AdaptiveGrid.

        obstacle cells → connected components → filter → merge → proposals
        """

        obstacle_cells = extract_obstacle_cells(
            grid,
            minimum_height=self.minimum_obstacle_height,
            minimum_obstacle_points=self.minimum_obstacle_points,
        )

        raw_components = connected_components(obstacle_cells)

        filtered_components = filter_components(
            raw_components,
            minimum_cells=self.minimum_component_cells,
            minimum_points=self.minimum_component_points,
        )

        object_groups = associate_components(filtered_components)
        proposals = self.groups_to_proposals(object_groups)

        return {
            "obstacle_cells": obstacle_cells,
            "raw_components": raw_components,
            "filtered_components": filtered_components,
            "object_groups": object_groups,
            "proposals": proposals,
        }

    # ========================================================
    # Range filtering
    # ========================================================

    def filter_range(
        self,
        points,
        max_range=100.0,
    ):
        """
        Keep points inside ORBIT's 100 m perception range.
        """

        distances = np.sqrt(
            points[:, 0] ** 2
            + points[:, 1] ** 2
        )

        mask = distances < max_range

        return mask

    # ========================================================
    # Main processing
    # ========================================================

    def process(self, points, ground_mask=None):

        start_time = time.perf_counter()

        points = np.asarray(
            points,
            dtype=np.float64,
        )

        if points.ndim != 2 or points.shape[1] != 3:

            raise ValueError(
                "Expected point cloud with shape (N, 3)"
            )

        input_points = len(points)

        # ----------------------------------------------------
        # Range filtering
        # ----------------------------------------------------

        range_mask = self.filter_range(
            points
        )

        mapped_points = points[
            range_mask
        ]

        mapped_count = len(
            mapped_points
        )

        outside_range = (
            input_points
            - mapped_count
        )

        # ----------------------------------------------------
        # Ground detection
        # ----------------------------------------------------

        if ground_mask is None:
            ground_mask = self.detect_ground(mapped_points)
        else:
            ground_mask = np.asarray(ground_mask, dtype=bool)
            if len(ground_mask) != mapped_count:
                raise ValueError(
                    "ground_mask length must match range-filtered points"
                )

        ground_count = int(
            ground_mask.sum()
        )

        non_ground_count = (
            mapped_count
            - ground_count
        )

        # ----------------------------------------------------
        # Adaptive grid
        # ----------------------------------------------------

        grid = AdaptiveGrid()

        grid.build(
            mapped_points,
            ground_mask,
        )

        adaptive_cell_count = (
            len(grid.cells)
        )

        # ----------------------------------------------------
        # Obstacle cell extraction
        # ----------------------------------------------------

        detection = self.detect_from_grid(grid)

        obstacle_cells = detection["obstacle_cells"]
        raw_components = detection["raw_components"]
        filtered_components = detection["filtered_components"]
        objects = detection["object_groups"]
        proposals = detection["proposals"]

        # ----------------------------------------------------
        # Object records
        # ----------------------------------------------------

        object_records = []

        for obj in objects:

            width, length, height = (
                obj.dimensions
            )

            x, y = obj.center

            # ----------------------------------------------
            # Simple geometric confidence
            # ----------------------------------------------

            confidence = (
                self.compute_confidence(
                    obj
                )
            )

            object_records.append(
                {
                    "id": int(
                        obj.object_id
                    ),

                    "class": (
                        obj.classification
                    ),

                    "center": (
                        float(x),
                        float(y),
                    ),

                    "dimensions": (
                        float(width),
                        float(length),
                        float(height),
                    ),

                    "distance": float(
                        obj.distance
                    ),

                    "max_height": float(
                        obj.maximum_height
                    ),

                    "mean_height": float(
                        obj.mean_height
                    ),

                    "point_count": int(
                        obj.point_count
                    ),

                    "cell_count": int(
                        obj.cell_count
                    ),

                    "component_count": int(
                        len(
                            obj.components
                        )
                    ),

                    "density": float(
                        obj.density
                    ),

                    "confidence": float(
                        confidence
                    ),
                }
            )

        # ----------------------------------------------------
        # Resolution statistics
        # ----------------------------------------------------

        resolution_stats = (
            self.get_resolution_statistics(
                grid
            )
        )

        # ----------------------------------------------------
        # Timing
        # ----------------------------------------------------

        elapsed = (
            time.perf_counter()
            - start_time
        )

        latency_ms = (
            elapsed * 1000.0
        )

        fps = (
            1.0 / elapsed
            if elapsed > 0
            else 0.0
        )

        # ----------------------------------------------------
        # Final perception result
        # ----------------------------------------------------

        result = {

            "points": {

                "input": int(
                    input_points
                ),

                "mapped": int(
                    mapped_count
                ),

                "outside_range": int(
                    outside_range
                ),

                "ground": int(
                    ground_count
                ),

                "non_ground": int(
                    non_ground_count
                ),
            },

            "grid": {

                "adaptive_cells": int(
                    adaptive_cell_count
                ),

                "obstacle_cells": int(
                    len(
                        obstacle_cells
                    )
                ),

                "resolution": (
                    resolution_stats
                ),
            },

            "objects": object_records,

            "components": {

                "raw": int(
                    len(
                        raw_components
                    )
                ),

                "filtered": int(
                    len(
                        filtered_components
                    )
                ),

                "final_objects": int(
                    len(objects)
                ),
            },

            "performance": {

                "latency_ms": float(
                    latency_ms
                ),

                "fps": float(
                    fps
                ),

                "points_per_second": float(
                    mapped_count / elapsed
                    if elapsed > 0
                    else 0.0
                ),
            },

            # Keep these available for visualization.
            "ground_mask": ground_mask,

            "mapped_points": mapped_points,

            "grid_object": grid,

            "obstacle_cells": obstacle_cells,

            "object_groups": objects,

            "proposals": proposals,
        }

        return result

    # ========================================================
    # Confidence
    # ========================================================

    def compute_confidence(
        self,
        obj,
    ):
        """
        Geometric confidence score.

        This is deliberately NOT presented as ML confidence.

        It measures how strongly the observed geometry
        supports the detected object hypothesis.
        """

        score = 0.0

        # Point support
        if obj.point_count >= 20:
            score += 0.25

        elif obj.point_count >= 10:
            score += 0.15

        else:
            score += 0.05

        # Cell support
        if obj.cell_count >= 10:
            score += 0.25

        elif obj.cell_count >= 5:
            score += 0.15

        else:
            score += 0.05

        # Height consistency
        max_height = (
            obj.maximum_height
        )

        mean_height = (
            obj.mean_height
        )

        if max_height > 0:

            consistency = (
                mean_height
                / max_height
            )

            score += (
                0.25
                * min(
                    consistency,
                    1.0
                )
            )

        # Density support
        if obj.density > 50:
            score += 0.25

        elif obj.density > 10:
            score += 0.15

        else:
            score += 0.05

        return float(
            min(
                score,
                1.0
            )
        )

    # ========================================================
    # Resolution statistics
    # ========================================================

    def get_resolution_statistics(
        self,
        grid,
    ):

        stats = {}

        for level, resolution in (
            grid.LEVELS.items()
        ):

            cells = 0
            points = 0

            for cell in grid.cells.values():

                if cell.level != level:
                    continue

                cells += 1
                points += cell.point_count

            stats[
                f"{resolution:.2f}m"
            ] = {

                "level": int(
                    level
                ),

                "cells": int(
                    cells
                ),

                "points": int(
                    points
                ),
            }

        return stats


# ============================================================
# Pretty console output
# ============================================================

def print_report(result):

    print("=" * 70)
    print(
        "ORBIT - UNIFIED PERCEPTION ENGINE"
    )
    print("=" * 70)

    points = result[
        "points"
    ]

    print(
        "\nPOINT CLOUD"
    )

    print(
        f"  Input:          "
        f"{points['input']:,}"
    )

    print(
        f"  Mapped:         "
        f"{points['mapped']:,}"
    )

    print(
        f"  Outside 100m:   "
        f"{points['outside_range']:,}"
    )

    print(
        f"  Ground:         "
        f"{points['ground']:,}"
    )

    print(
        f"  Non-ground:     "
        f"{points['non_ground']:,}"
    )

    grid = result[
        "grid"
    ]

    print(
        "\nADAPTIVE MAP"
    )

    print(
        f"  Adaptive cells: "
        f"{grid['adaptive_cells']:,}"
    )

    print(
        f"  Obstacle cells: "
        f"{grid['obstacle_cells']:,}"
    )

    print(
        "\nRESOLUTION DISTRIBUTION"
    )

    for name, data in (
        grid["resolution"].items()
    ):

        print(
            f"  {name:>6} → "
            f"{data['cells']:,} cells | "
            f"{data['points']:,} points"
        )

    components = result[
        "components"
    ]

    print(
        "\nOBJECT EXTRACTION"
    )

    print(
        f"  Raw components: "
        f"{components['raw']:,}"
    )

    print(
        f"  Filtered:       "
        f"{components['filtered']:,}"
    )

    print(
        f"  Final objects:  "
        f"{components['final_objects']:,}"
    )

    print(
        "\nOBJECTS"
    )

    print(
        "-" * 70
    )

    for obj in result[
        "objects"
    ]:

        w, l, h = obj[
            "dimensions"
        ]

        x, y = obj[
            "center"
        ]

        print(
            f"\nObject #{obj['id']}"
        )

        print(
            f"  Class:       "
            f"{obj['class']}"
        )

        print(
            f"  Confidence:  "
            f"{obj['confidence'] * 100:.1f}%"
        )

        print(
            f"  Center:      "
            f"({x:.2f}, {y:.2f})m"
        )

        print(
            f"  Distance:    "
            f"{obj['distance']:.2f}m"
        )

        print(
            f"  Dimensions:  "
            f"{w:.2f} × "
            f"{l:.2f} × "
            f"{h:.2f}m"
        )

        print(
            f"  Points:      "
            f"{obj['point_count']:,}"
        )

        print(
            f"  Cells:       "
            f"{obj['cell_count']:,}"
        )

    performance = result[
        "performance"
    ]

    print(
        "\nPERFORMANCE"
    )

    print(
        f"  Latency:     "
        f"{performance['latency_ms']:.2f} ms"
    )

    print(
        f"  FPS:         "
        f"{performance['fps']:.2f}"
    )

    print(
        f"  Throughput:  "
        f"{performance['points_per_second']:,.0f} "
        f"points/sec"
    )

    print(
        "\n" + "=" * 70
    )

    print(
        "ORBIT PERCEPTION COMPLETE"
    )

    print(
        "=" * 70
    )


# ============================================================
# Standalone test
# ============================================================

def main():

    cloud = o3d.io.read_point_cloud(
        "data/synthetic_scene.ply"
    )

    points = np.asarray(
        cloud.points
    )

    engine = OrbitPerception()

    result = engine.process(
        points
    )

    print_report(
        result
    )


if __name__ == "__main__":
    main()