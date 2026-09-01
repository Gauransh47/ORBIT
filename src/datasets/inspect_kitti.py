from pathlib import Path
import argparse
import numpy as np

from kitti_loader import (
    KittiLoader,
    semantic_name
)


# ------------------------------------------------------------
# DATASET
# ------------------------------------------------------------

DATASET_ROOT = Path(
    "data/semantic_kitti"
)


# ------------------------------------------------------------
# COMMAND-LINE ARGUMENTS
# ------------------------------------------------------------

def get_args():

    parser = argparse.ArgumentParser(
        description="Inspect a SemanticKITTI LiDAR frame"
    )

    parser.add_argument(
        "--frame",
        type=int,
        default=0,
        help="Frame number to inspect"
    )

    parser.add_argument(
        "--sequence",
        type=str,
        default="00",
        help="KITTI sequence number"
    )

    return parser.parse_args()


# ------------------------------------------------------------
# POINT CLOUD RANGE
# ------------------------------------------------------------

def print_range(points):

    xyz = points[:, :3]

    x = xyz[:, 0]
    y = xyz[:, 1]
    z = xyz[:, 2]

    distance = np.sqrt(
        x * x + y * y
    )

    print("\nPOINT CLOUD")
    print("-" * 65)

    print(
        f"  Points:       {len(points):,}"
    )

    print(
        f"  X range:      "
        f"{x.min():.2f} → {x.max():.2f} m"
    )

    print(
        f"  Y range:      "
        f"{y.min():.2f} → {y.max():.2f} m"
    )

    print(
        f"  Z range:      "
        f"{z.min():.2f} → {z.max():.2f} m"
    )

    print(
        f"  Max range:    "
        f"{distance.max():.2f} m"
    )

    print(
        f"  Mean range:   "
        f"{distance.mean():.2f} m"
    )


# ------------------------------------------------------------
# SEMANTIC LABELS
# ------------------------------------------------------------

def print_labels(labels):

    print("\nSEMANTIC LABELS")
    print("-" * 65)

    unique, counts = np.unique(
        labels,
        return_counts=True
    )

    order = np.argsort(
        counts
    )[::-1]

    for index in order:

        class_id = int(
            unique[index]
        )

        count = int(
            counts[index]
        )

        percentage = (
            count
            / len(labels)
            * 100.0
        )

        print(
            f"  {class_id:3d} "
            f"{semantic_name(class_id):<24} "
            f"{count:>8,} "
            f"({percentage:5.1f}%)"
        )


# ------------------------------------------------------------
# MAIN
# ------------------------------------------------------------

def main():

    args = get_args()

    sequence = str(
        args.sequence
    ).zfill(2)

    frame = f"{args.frame:06d}"

    print("=" * 65)
    print("ORBIT - REAL LiDAR DATASET INSPECTOR")
    print("=" * 65)

    print(
        f"\nDataset: {DATASET_ROOT}"
    )

    print(
        f"Sequence: {sequence}"
    )

    print(
        f"Frame:    {frame}"
    )

    # --------------------------------------------------------
    # Loader
    # --------------------------------------------------------

    loader = KittiLoader(
        DATASET_ROOT,
        sequence
    )

    frames = loader.frame_ids()

    print(
        f"\nAvailable frames: "
        f"{len(frames):,}"
    )

    if not frames:

        raise RuntimeError(
            "No .bin frames found."
        )

    if frame not in frames:

        raise RuntimeError(
            f"Frame {frame} not found.\n"
            f"Available frames: "
            f"{frames[0]} → {frames[-1]}"
        )

    # --------------------------------------------------------
    # Load frame
    # --------------------------------------------------------

    points, labels, instances = (
        loader.load_frame(
            frame,
            load_labels=True
        )
    )

    print_range(points)

    # --------------------------------------------------------
    # Intensity
    # --------------------------------------------------------

    intensity = points[:, 3]

    print("\nINTENSITY")
    print("-" * 65)

    print(
        f"  Minimum:      "
        f"{intensity.min():.4f}"
    )

    print(
        f"  Mean:         "
        f"{intensity.mean():.4f}"
    )

    print(
        f"  Maximum:      "
        f"{intensity.max():.4f}"
    )

    # --------------------------------------------------------
    # Labels
    # --------------------------------------------------------

    if labels is None:

        print(
            "\nNo label file found "
            "for this frame."
        )

    else:

        print_labels(
            labels
        )

        unique_instances = (
            np.unique(instances)
        )

        print(
            "\nINSTANCES"
        )

        print("-" * 65)

        print(
            f"  Unique instance IDs: "
            f"{len(unique_instances):,}"
        )

    print("\n" + "=" * 65)
    print("KITTI INSPECTION COMPLETE")
    print("=" * 65)


if __name__ == "__main__":
    main()