import time
import open3d as o3d
import numpy as np

from mapping.adaptive_grid import AdaptiveGrid


# ======================================================================
# ORBIT - TERRAIN AWARE 2.5D PIPELINE
# ======================================================================

def main():

    print("=" * 65)
    print("ORBIT - TERRAIN AWARE 2.5D PIPELINE")
    print("=" * 65)

    # ------------------------------------------------------------------
    # Load point cloud
    # ------------------------------------------------------------------

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

    # ------------------------------------------------------------------
    # Ground detection
    # ------------------------------------------------------------------

    print(
        "\nDetecting ground..."
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

    ground_mask[
        inliers
    ] = True

    ground_count = int(
        ground_mask.sum()
    )

    non_ground_count = int(
        (~ground_mask).sum()
    )

    print(
        f"Ground points: "
        f"{ground_count:,}"
    )

    print(
        f"Non-ground points: "
        f"{non_ground_count:,}"
    )

    # ------------------------------------------------------------------
    # Build adaptive ORBIT map
    # ------------------------------------------------------------------

    grid = AdaptiveGrid()

    grid.build(
        points,
        ground_mask
    )

    # IMPORTANT:
    # build() now returns the mapped point array,
    # but pipeline uses the integer count.

    mapped_points = int(
        grid.mapped_points
    )

    outside_range = (
        len(points)
        - mapped_points
    )

    print(
        f"\nMapped points: "
        f"{mapped_points:,}"
    )

    print(
        f"Outside 100m range: "
        f"{outside_range:,}"
    )

    print(
        f"Adaptive cells: "
        f"{len(grid.cells):,}"
    )

    # ------------------------------------------------------------------
    # Resolution hierarchy
    # ------------------------------------------------------------------

    stats = (
        grid.statistics()
    )

    print(
        "\nResolution hierarchy:"
    )

    for level, resolution in (
        grid.LEVELS.items()
    ):

        cells = (
            stats[
                "level_counts"
            ][level]
        )

        point_count = (
            stats[
                "point_counts"
            ][level]
        )

        print(
            f"  Level {level}: "
            f"{resolution * 100:.0f} cm"
            f" | cells={cells:,}"
            f" | points={point_count:,}"
        )

    # ------------------------------------------------------------------
    # Cell classification
    # ------------------------------------------------------------------

    ground_cells = 0
    obstacle_cells = 0
    mixed_cells = 0

    for cell in grid.cells.values():

        classification = (
            cell.semantic_class
        )

        if classification == "GROUND":

            ground_cells += 1

        elif classification == "OBSTACLE":

            obstacle_cells += 1

        elif classification == "MIXED":

            mixed_cells += 1

    print(
        "\nCell classification:"
    )

    print(
        f"  Ground cells:   "
        f"{ground_cells:,}"
    )

    print(
        f"  Obstacle cells: "
        f"{obstacle_cells:,}"
    )

    print(
        f"  Mixed cells:    "
        f"{mixed_cells:,}"
    )

    # ------------------------------------------------------------------
    # Example terrain cells
    # ------------------------------------------------------------------

    print(
        "\nExample terrain cells:"
    )

    shown = 0

    for cell in (
        grid.cells.values()
    ):

        # Only show cells containing ground
        if cell.ground_count == 0:
            continue

        print(
            f"\nCell "
            f"level={cell.level} "
            f"index=({cell.ix},{cell.iy})"
        )

        print(
            f"  resolution: "
            f"{cell.resolution * 100:.0f} cm"
        )

        print(
            f"  total points: "
            f"{cell.point_count}"
        )

        print(
            f"  ground points: "
            f"{cell.ground_count}"
        )

        print(
            f"  ground elevation: "
            f"{cell.ground_elevation:.3f} m"
        )

        if cell.obstacle_count > 0:

            print(
                f"  obstacle points: "
                f"{cell.obstacle_count}"
            )

            print(
                f"  obstacle elevation: "
                f"{cell.obstacle_elevation:.3f} m"
            )

        print(
            f"  class: "
            f"{cell.semantic_class}"
        )

        shown += 1

        if shown >= 10:
            break

    # ------------------------------------------------------------------
    # Summary
    # ------------------------------------------------------------------

    print(
        "\n" + "=" * 65
    )

    print(
        "ORBIT PIPELINE COMPLETE"
    )

    print(
        "=" * 65
    )


# ======================================================================
# ENTRY POINT
# ======================================================================

if __name__ == "__main__":
    main()