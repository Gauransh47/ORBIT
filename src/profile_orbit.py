import cProfile
import pstats
import io

import numpy as np
import open3d as o3d

from mapping.adaptive_grid import AdaptiveGrid


def run_pipeline():

    cloud = o3d.io.read_point_cloud(
        "data/synthetic_scene.ply"
    )

    points = np.asarray(
        cloud.points
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

    grid = AdaptiveGrid()

    grid.build(
        points,
        ground_mask
    )


def main():

    print("=" * 70)
    print("ORBIT - PERFORMANCE PROFILE")
    print("=" * 70)

    profiler = cProfile.Profile()

    profiler.enable()

    run_pipeline()

    profiler.disable()

    output = io.StringIO()

    stats = pstats.Stats(
        profiler,
        stream=output
    )

    stats.sort_stats(
        "cumtime"
    )

    stats.print_stats(
        30
    )

    print(output.getvalue())


if __name__ == "__main__":
    main()