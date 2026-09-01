import sys
import time
from pathlib import Path

import numpy as np
import open3d as o3d


# ============================================================
# Allow imports from src
# ============================================================

SRC_DIR = Path(__file__).resolve().parent

if str(SRC_DIR) not in sys.path:
    sys.path.append(str(SRC_DIR))


from mapping.adaptive_grid import AdaptiveGrid


# ============================================================
# Configuration
# ============================================================

MAX_RANGE = 100.0
BASE_RESOLUTION = 0.05


# ============================================================
# Uniform 5 cm grid
# ============================================================

def build_uniform_grid(points, ground_mask):
    """
    Build the baseline representation.

    Every point inside 100m is assigned to a uniform
    5 cm x 5 cm cell.

    This is intentionally a sparse dictionary rather
    than allocating the entire theoretical 100m grid.
    """

    cells = {}

    mapped_points = 0

    for point, is_ground in zip(
        points,
        ground_mask
    ):

        x, y, z = point

        distance = np.sqrt(
            x * x + y * y
        )

        if distance >= MAX_RANGE:
            continue

        ix = int(
            np.floor(
                x / BASE_RESOLUTION
            )
        )

        iy = int(
            np.floor(
                y / BASE_RESOLUTION
            )
        )

        key = (
            ix,
            iy
        )

        if key not in cells:

            cells[key] = {
                "point_count": 0,
                "ground_count": 0,
                "obstacle_count": 0,
                "z_sum": 0.0
            }

        cell = cells[key]

        cell["point_count"] += 1

        if is_ground:

            cell["ground_count"] += 1

        else:

            cell["obstacle_count"] += 1

        cell["z_sum"] += z

        mapped_points += 1

    return cells, mapped_points


# ============================================================
# Theoretical dense uniform grid
# ============================================================

def theoretical_uniform_cell_count():

    area = np.pi * (
        MAX_RANGE ** 2
    )

    cells = area / (
        BASE_RESOLUTION ** 2
    )

    return int(
        np.ceil(cells)
    )


# ============================================================
# Estimate logical storage
# ============================================================

def estimate_storage(
    number_of_cells,
    bytes_per_cell
):
    """
    Estimate logical storage.

    We deliberately use a fixed payload size for both
    representations so the comparison remains fair.

    This is NOT a measurement of Python dictionary RAM.
    It represents the amount of map information stored
    per occupied cell.
    """

    return (
        number_of_cells
        * bytes_per_cell
    )


# ============================================================
# Format bytes
# ============================================================

def format_bytes(value):

    if value < 1024:
        return f"{value:.0f} B"

    if value < 1024 ** 2:
        return f"{value / 1024:.2f} KB"

    if value < 1024 ** 3:
        return f"{value / (1024 ** 2):.2f} MB"

    return f"{value / (1024 ** 3):.2f} GB"


# ============================================================
# Main benchmark
# ============================================================

def main():

    print("=" * 72)
    print("ORBIT - ADAPTIVE VS UNIFORM PERFORMANCE BENCHMARK")
    print("=" * 72)

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
        "\nRunning ground detection..."
    )

    plane_model, inliers = (
        cloud.segment_plane(
            distance_threshold=0.08,
            ransac_n=3,
            num_iterations=1000
        )
    )

    ground_mask = np.zeros(
        len(points),
        dtype=bool
    )

    ground_mask[inliers] = True

    print(
        f"Ground points: "
        f"{ground_mask.sum():,}"
    )

    print(
        f"Non-ground points: "
        f"{(~ground_mask).sum():,}"
    )

    # ========================================================
    # BASELINE: Uniform 5 cm
    # ========================================================

    print("\n" + "-" * 72)
    print("BASELINE - UNIFORM 5 CM GRID")
    print("-" * 72)

    start = time.perf_counter()

    uniform_cells, uniform_mapped = (
        build_uniform_grid(
            points,
            ground_mask
        )
    )

    uniform_time = (
        time.perf_counter()
        - start
    )

    uniform_cell_count = len(
        uniform_cells
    )

    print(
        f"Mapped points: "
        f"{uniform_mapped:,}"
    )

    print(
        f"Occupied cells: "
        f"{uniform_cell_count:,}"
    )

    print(
        f"Build time: "
        f"{uniform_time * 1000:.2f} ms"
    )

    # ========================================================
    # ORBIT
    # ========================================================

    print("\n" + "-" * 72)
    print("ORBIT - HIERARCHICAL ADAPTIVE GRID")
    print("-" * 72)

    grid = AdaptiveGrid()

    start = time.perf_counter()

    orbit_mapped = grid.build(
        points,
        ground_mask
    )

    orbit_time = (
        time.perf_counter()
        - start
    )

    orbit_cell_count = len(
        grid.cells
    )

    print(
        f"Mapped points: "
        f"{orbit_mapped:,}"
    )

    print(
        f"Occupied cells: "
        f"{orbit_cell_count:,}"
    )

    print(
        f"Build time: "
        f"{orbit_time * 1000:.2f} ms"
    )

    # ========================================================
    # Cell reduction
    # ========================================================

    cell_reduction = (
        1.0
        - (
            orbit_cell_count
            / uniform_cell_count
        )
    ) * 100.0

    # ========================================================
    # Storage model
    # ========================================================

    # Logical payload:
    #
    # x/y index       = 4 bytes each
    # resolution      = 4 bytes
    # point count     = 4 bytes
    # ground count    = 4 bytes
    # obstacle count  = 4 bytes
    # elevation stats = 16 bytes
    #
    # Total assumed payload = 40 bytes
    #
    # Same payload is used for both systems.

    BYTES_PER_CELL = 40

    uniform_storage = estimate_storage(
        uniform_cell_count,
        BYTES_PER_CELL
    )

    orbit_storage = estimate_storage(
        orbit_cell_count,
        BYTES_PER_CELL
    )

    storage_reduction = (
        1.0
        - (
            orbit_storage
            / uniform_storage
        )
    ) * 100.0

    # ========================================================
    # Theoretical dense grid
    # ========================================================

    dense_cells = (
        theoretical_uniform_cell_count()
    )

    dense_storage = estimate_storage(
        dense_cells,
        BYTES_PER_CELL
    )

    # ========================================================
    # Throughput
    # ========================================================

    uniform_fps = (
        uniform_mapped
        / uniform_time
    )

    orbit_fps = (
        orbit_mapped
        / orbit_time
    )

    # ========================================================
    # Results
    # ========================================================

    print("\n" + "=" * 72)
    print("RESULTS")
    print("=" * 72)

    print(
        f"\nTheoretical dense 5 cm cells "
        f"within 100m radius:"
    )

    print(
        f"  {dense_cells:,}"
    )

    print(
        f"  Estimated dense storage: "
        f"{format_bytes(dense_storage)}"
    )

    print(
        "\nOccupied-cell comparison:"
    )

    print(
        f"  Uniform 5 cm: "
        f"{uniform_cell_count:,}"
    )

    print(
        f"  ORBIT adaptive: "
        f"{orbit_cell_count:,}"
    )

    print(
        f"  Cell reduction: "
        f"{cell_reduction:.2f}%"
    )

    print(
        "\nLogical map storage:"
    )

    print(
        f"  Uniform: "
        f"{format_bytes(uniform_storage)}"
    )

    print(
        f"  ORBIT:   "
        f"{format_bytes(orbit_storage)}"
    )

    print(
        f"  Storage reduction: "
        f"{storage_reduction:.2f}%"
    )

    print(
        "\nBuild performance:"
    )

    print(
        f"  Uniform: "
        f"{uniform_time * 1000:.2f} ms"
    )

    print(
        f"  ORBIT:   "
        f"{orbit_time * 1000:.2f} ms"
    )

    print(
        "\nPoint processing throughput:"
    )

    print(
        f"  Uniform: "
        f"{uniform_fps:,.0f} points/sec"
    )

    print(
        f"  ORBIT:   "
        f"{orbit_fps:,.0f} points/sec"
    )

    print("\n" + "=" * 72)
    print("ORBIT BENCHMARK COMPLETE")
    print("=" * 72)


if __name__ == "__main__":
    main()