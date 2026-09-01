from pathlib import Path
import argparse
import sys

import numpy as np

# ------------------------------------------------------------
# Make src available
# ------------------------------------------------------------

SRC_DIR = Path(__file__).resolve().parents[1]

if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

from datasets.kitti_loader import (
    KittiLoader,
    semantic_name,
)

from perception.orbit_perception import (
    OrbitPerception,
    print_report,
)


# ------------------------------------------------------------
# Configuration
# ------------------------------------------------------------

DATASET_ROOT = Path(
    "data/semantic_kitti"
)


# ------------------------------------------------------------
# Arguments
# ------------------------------------------------------------

def get_args():

    parser = argparse.ArgumentParser(
        description=(
            "Run ORBIT perception on "
            "real SemanticKITTI LiDAR"
        )
    )

    parser.add_argument(
        "--frame",
        type=int,
        default=0,
        help="Frame number"
    )

    parser.add_argument(
        "--sequence",
        type=str,
        default="00",
        help="Sequence number"
    )

    parser.add_argument(
        "--max-range",
        type=float,
        default=100.0,
        help="ORBIT perception range"
    )

    return parser.parse_args()


# ------------------------------------------------------------
# Ground truth summary
# ------------------------------------------------------------

def print_ground_truth(
    labels,
    instances,
):

    print("\nGROUND TRUTH")
    print("-" * 70)

    if labels is None:

        print(
            "  No semantic labels available."
        )

        return

    unique, counts = np.unique(
        labels,
        return_counts=True,
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

    # Instance 0 is normally "no instance".
    real_instances = (
        np.unique(instances[instances > 0])
        if instances is not None
        else []
    )

    print(
        f"\n  Unique object instances: "
        f"{len(real_instances):,}"
    )


# ------------------------------------------------------------
# Main
# ------------------------------------------------------------

def main():

    args = get_args()

    sequence = str(
        args.sequence
    ).zfill(2)

    frame = f"{args.frame:06d}"

    print("=" * 70)
    print(
        "ORBIT - REAL LiDAR PERCEPTION TEST"
    )
    print("=" * 70)

    print(
        f"\nDataset:  {DATASET_ROOT}"
    )

    print(
        f"Sequence: {sequence}"
    )

    print(
        f"Frame:    {frame}"
    )

    # --------------------------------------------------------
    # Load real SemanticKITTI frame
    # --------------------------------------------------------

    loader = KittiLoader(
        DATASET_ROOT,
        sequence,
    )

    available = loader.frame_ids()

    print(
        f"\nAvailable frames: "
        f"{len(available):,}"
    )

    if frame not in available:

        raise ValueError(
            f"Frame {frame} not available.\n"
            f"Available frames: {available}"
        )

    points, labels, instances = (
        loader.load_frame(
            frame,
            load_labels=True,
        )
    )

    xyz = points[:, :3]

    # --------------------------------------------------------
    # Basic point-cloud information
    # --------------------------------------------------------

    distance = np.sqrt(
        xyz[:, 0] ** 2
        + xyz[:, 1] ** 2
    )

    print("\nREAL LiDAR")
    print("-" * 70)

    print(
        f"  Input points: "
        f"{len(xyz):,}"
    )

    print(
        f"  Maximum range: "
        f"{distance.max():.2f} m"
    )

    print(
        f"  Mean range:    "
        f"{distance.mean():.2f} m"
    )

    print_ground_truth(
        labels,
        instances,
    )

    # --------------------------------------------------------
    # Run ORBIT
    # --------------------------------------------------------

    print("\n" + "=" * 70)
    print(
        "RUNNING ORBIT ON REAL LiDAR"
    )
    print("=" * 70)

    engine = OrbitPerception()

    result = engine.process(
        xyz
    )

    # --------------------------------------------------------
    # ORBIT report
    # --------------------------------------------------------

    print_report(
        result
    )

    # --------------------------------------------------------
    # Comparison summary
    # --------------------------------------------------------

    print("\n" + "=" * 70)
    print(
        "REAL DATA TEST SUMMARY"
    )
    print("=" * 70)

    print(
        f"\n  Ground-truth instances: "
        f"{len(np.unique(instances[instances > 0]))}"
    )

    print(
        f"  ORBIT objects detected: "
        f"{len(result['objects'])}"
    )

    print(
        f"  ORBIT latency: "
        f"{result['performance']['latency_ms']:.2f} ms"
    )

    print(
        f"  ORBIT FPS: "
        f"{result['performance']['fps']:.2f}"
    )

    print(
        "\nNOTE:"
    )

    print(
        "  Ground-truth semantic labels and "
        "ORBIT geometric classes are currently"
    )

    print(
        "  kept separate. We will build the "
        "actual evaluation/matching layer next."
    )

    print("\n" + "=" * 70)
    print(
        "ORBIT REAL LiDAR TEST COMPLETE"
    )
    print("=" * 70)


if __name__ == "__main__":
    main()