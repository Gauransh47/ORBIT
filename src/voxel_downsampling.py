import open3d as o3d


def main():

    scene = o3d.io.read_point_cloud(
        "data/synthetic_scene.ply"
    )

    original_count = len(scene.points)

    print("=" * 60)
    print("ORBIT - VOXEL RESOLUTION EXPERIMENT")
    print("=" * 60)

    print(f"\nOriginal points: {original_count:,}\n")

    voxel_sizes = [
        0.05,   # 5 cm
        0.10,   # 10 cm
        0.25,   # 25 cm
        0.50    # 50 cm
    ]

    results = []

    for voxel_size in voxel_sizes:

        downsampled = scene.voxel_down_sample(
            voxel_size=voxel_size
        )

        count = len(downsampled.points)

        reduction = (
            1 - count / original_count
        ) * 100

        results.append(
            (voxel_size, count, reduction)
        )

        print(
            f"{voxel_size * 100:>5.0f} cm"
            f"   →   "
            f"{count:>6,} points"
            f"   →   "
            f"{reduction:>6.2f}% reduction"
        )

    print("\n" + "=" * 60)


if __name__ == "__main__":
    main()