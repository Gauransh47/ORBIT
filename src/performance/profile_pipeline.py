import sys
from pathlib import Path
import time

import numpy as np
import open3d as o3d

# ------------------------------------------------------------
# Make src importable
# ------------------------------------------------------------

SRC_DIR = Path(__file__).resolve().parents[1]

if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

from perception.orbit_perception import OrbitPerception


def timed_call(name, function, *args, **kwargs):

    start = time.perf_counter()

    result = function(
        *args,
        **kwargs
    )

    elapsed = (
        time.perf_counter()
        - start
    )

    return result, elapsed


def main():

    print("=" * 70)
    print(
        "ORBIT - PIPELINE PERFORMANCE PROFILE"
    )
    print("=" * 70)

    # --------------------------------------------------------
    # Load point cloud
    # --------------------------------------------------------

    print(
        "\nLoading point cloud..."
    )

    cloud = o3d.io.read_point_cloud(
        "data/synthetic_scene.ply"
    )

    points = np.asarray(
        cloud.points
    )

    print(
        f"Input points: "
        f"{len(points):,}"
    )

    engine = OrbitPerception()

    # --------------------------------------------------------
    # Warm-up
    # --------------------------------------------------------

    print(
        "\nWarm-up run..."
    )

    engine.process(
        points
    )

    # --------------------------------------------------------
    # Profile
    # --------------------------------------------------------

    print(
        "\nProfiling pipeline..."
    )

    start_total = (
        time.perf_counter()
    )

    # --------------------------------------------------------
    # Range filtering
    # --------------------------------------------------------

    start = time.perf_counter()

    range_mask = (
        engine.filter_range(
            points
        )
    )

    mapped_points = points[
        range_mask
    ]

    range_time = (
        time.perf_counter()
        - start
    )

    # --------------------------------------------------------
    # Ground detection
    # --------------------------------------------------------

    start = time.perf_counter()

    ground_mask = (
        engine.detect_ground(
            mapped_points
        )
    )

    ground_time = (
        time.perf_counter()
        - start
    )

    # --------------------------------------------------------
    # Adaptive mapping
    # --------------------------------------------------------

    from mapping.adaptive_grid import (
        AdaptiveGrid
    )

    start = time.perf_counter()

    grid = AdaptiveGrid()

    grid.build(
        mapped_points,
        ground_mask
    )

    mapping_time = (
        time.perf_counter()
        - start
    )

    # --------------------------------------------------------
    # Obstacle extraction
    # --------------------------------------------------------

    from perception.terrain_obstacle_detection import (
        extract_obstacle_cells
    )

    start = time.perf_counter()

    obstacle_cells = (
        extract_obstacle_cells(
            grid,
            minimum_height=0.15,
            minimum_obstacle_points=3,
        )
    )

    obstacle_time = (
        time.perf_counter()
        - start
    )

    # --------------------------------------------------------
    # Connected components
    # --------------------------------------------------------

    from perception.connected_components import (
        connected_components,
        filter_components,
    )

    start = time.perf_counter()

    components = (
        connected_components(
            obstacle_cells
        )
    )

    components = filter_components(
        components,
        minimum_cells=3,
        minimum_points=5,
    )

    component_time = (
        time.perf_counter()
        - start
    )

    # --------------------------------------------------------
    # Association
    # --------------------------------------------------------

    from perception.object_association import (
        associate_components
    )

    start = time.perf_counter()

    objects = (
        associate_components(
            components
        )
    )

    association_time = (
        time.perf_counter()
        - start
    )

    total_time = (
        time.perf_counter()
        - start_total
    )

    # --------------------------------------------------------
    # Results
    # --------------------------------------------------------

    stages = {

        "Range filtering": range_time,

        "Ground detection": ground_time,

        "Adaptive mapping": mapping_time,

        "Obstacle extraction": obstacle_time,

        "Connected components": component_time,

        "Object association": association_time,
    }

    print(
        "\n" + "-" * 70
    )

    print(
        "STAGE TIMINGS"
    )

    print(
        "-" * 70
    )

    for name, elapsed in stages.items():

        percentage = (
            elapsed
            / total_time
            * 100
        )

        print(
            f"{name:<25}"
            f"{elapsed * 1000:>10.2f} ms"
            f"   ({percentage:>5.1f}%)"
        )

    print(
        "-" * 70
    )

    print(
        f"{'TOTAL':<25}"
        f"{total_time * 1000:>10.2f} ms"
    )

    # --------------------------------------------------------
    # Throughput
    # --------------------------------------------------------

    fps = (
        1.0 / total_time
        if total_time > 0
        else 0
    )

    throughput = (
        len(mapped_points)
        / total_time
        if total_time > 0
        else 0
    )

    print(
        "\nPERFORMANCE"
    )

    print(
        f"  FPS:         {fps:.2f}"
    )

    print(
        f"  Latency:     "
        f"{total_time * 1000:.2f} ms"
    )

    print(
        f"  Throughput:  "
        f"{throughput:,.0f} points/sec"
    )

    # --------------------------------------------------------
    # Pipeline statistics
    # --------------------------------------------------------

    print(
        "\nPIPELINE"
    )

    print(
        f"  Input points:       "
        f"{len(points):,}"
    )

    print(
        f"  Mapped points:      "
        f"{len(mapped_points):,}"
    )

    print(
        f"  Adaptive cells:     "
        f"{len(grid.cells):,}"
    )

    print(
        f"  Obstacle cells:     "
        f"{len(obstacle_cells):,}"
    )

    print(
        f"  Final objects:      "
        f"{len(objects):,}"
    )

    # --------------------------------------------------------
    # Bottleneck
    # --------------------------------------------------------

    bottleneck = max(
        stages,
        key=stages.get
    )

    print(
        "\nBOTTLENECK"
    )

    print(
        f"  {bottleneck}"
    )

    print(
        f"  {stages[bottleneck] * 1000:.2f} ms"
    )

    print(
        "\n" + "=" * 70
    )

    print(
        "ORBIT PERFORMANCE PROFILE COMPLETE"
    )

    print(
        "=" * 70
    )


if __name__ == "__main__":
    main()