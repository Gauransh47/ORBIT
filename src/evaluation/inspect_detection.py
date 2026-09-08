import numpy as np
from pathlib import Path
from collections import Counter, defaultdict

from src.perception.object_detection import (
    ObjectDetector,
    detect_ground,
    load_semantic_kitti_frame,
)


# ============================================================
# ORBIT - OBJECT DETECTION INSPECTION / EVALUATION
#
# Two-level evaluation:
#
#   LEVEL 1:
#       Terrain / obstacle consistency
#
#   LEVEL 2:
#       Semantic classification quality
#
# This evaluator uses proposal.point_indices directly.
# Those indices refer to the ORIGINAL input point cloud.
# ============================================================


# ============================================================
# SemanticKITTI label mapping
# ============================================================

SEMANTIC_LABELS = {

    0: "unlabeled",
    1: "outlier",

    10: "car",
    11: "bicycle",
    13: "bus",
    15: "motorcycle",
    16: "on-rails",
    18: "truck",
    20: "other-vehicle",

    30: "person",
    31: "bicyclist",
    32: "motorcyclist",

    40: "road",
    44: "parking",
    48: "sidewalk",
    49: "other-ground",

    50: "building",
    51: "fence",
    52: "other-structure",

    60: "lane-marking",

    70: "vegetation",
    71: "trunk",
    72: "terrain",

    80: "pole",
    81: "traffic-sign",

    99: "other-object",
}


# ============================================================
# Semantic groups
# ============================================================

VEHICLE_CLASSES = {
    "car",
    "bicycle",
    "bus",
    "motorcycle",
    "on-rails",
    "truck",
    "other-vehicle",
}

PERSON_CLASSES = {
    "person",
    "bicyclist",
    "motorcyclist",
}

POLE_CLASSES = {
    "pole",
    "traffic-sign",
    "trunk",
}

STRUCTURE_CLASSES = {
    "building",
    "fence",
    "other-structure",
}

ENVIRONMENTAL_OBSTACLE_CLASSES = {
    "vegetation",
    "terrain",
    "trunk",
}

OTHER_OBSTACLE_CLASSES = {
    "other-object",
}


# ============================================================
# Classes considered navigational obstacles
# ============================================================

OBSTACLE_CLASSES = (
    VEHICLE_CLASSES
    | PERSON_CLASSES
    | POLE_CLASSES
    | STRUCTURE_CLASSES
    | ENVIRONMENTAL_OBSTACLE_CLASSES
    | OTHER_OBSTACLE_CLASSES
)


# ============================================================
# Classes considered non-obstacle / surface
# ============================================================

NON_OBSTACLE_CLASSES = {
    "unlabeled",
    "outlier",
    "road",
    "parking",
    "sidewalk",
    "other-ground",
    "lane-marking",
}


# ============================================================
# ORBIT semantic classification expectations
#
# "preferred" means the geometric classifier's intended class.
# ============================================================

PREFERRED_CLASSES = {

    "POLE": {
        "pole",
        "traffic-sign",
        "trunk",
    },

    "PERSON-LIKE": {
        "person",
        "bicyclist",
        "motorcyclist",
    },

    "VEHICLE-LIKE": {
        "car",
        "truck",
        "bus",
        "motorcycle",
        "bicycle",
        "other-vehicle",
        "on-rails",
    },

    "WALL": {
        "building",
        "fence",
        "other-structure",
    },
}


# ============================================================
# Generic obstacle compatibility for OBSTACLE prediction
#
# OBSTACLE means ORBIT detected geometry that should be treated
# as a navigational hazard, but did not assign a specific type.
# ============================================================

GENERIC_OBSTACLE_CLASSES = {
    "other-object",
    "vegetation",
    "terrain",
    "trunk",
    "building",
    "fence",
    "other-structure",
    "pole",
    "traffic-sign",

    # A generic obstacle can also legitimately be a vehicle or
    # person from the terrain-navigation perspective.
    "car",
    "bicycle",
    "bus",
    "motorcycle",
    "on-rails",
    "truck",
    "other-vehicle",

    "person",
    "bicyclist",
    "motorcyclist",
}


# ============================================================
# Load SemanticKITTI labels
# ============================================================

def load_labels(label_path):
    """
    Load SemanticKITTI labels.

    SemanticKITTI stores:
        lower 16 bits  -> semantic label
        upper 16 bits  -> instance id
    """

    path = Path(label_path)

    if not path.exists():
        raise FileNotFoundError(
            f"SemanticKITTI label file not found:\n{path}"
        )

    labels = np.fromfile(
        path,
        dtype=np.uint32,
    )

    semantic_labels = (
        labels & 0xFFFF
    )

    return semantic_labels


# ============================================================
# Proposal label extraction
# ============================================================

def inspect_object(
    proposal,
    semantic_labels,
):
    """
    Get SemanticKITTI labels corresponding to a detected
    ORBIT proposal.

    Uses exact original point indices produced by
    ObjectDetector.

    No coordinate matching is performed.
    """

    indices = np.asarray(
        proposal.point_indices,
        dtype=np.int64,
    )

    if len(indices) == 0:
        return Counter()

    # --------------------------------------------------------
    # Safety validation
    # --------------------------------------------------------

    valid_mask = (
        (indices >= 0)
        & (indices < len(semantic_labels))
    )

    indices = indices[valid_mask]

    if len(indices) == 0:
        return Counter()

    detected_labels = semantic_labels[
        indices
    ]

    return Counter(
        int(label)
        for label in detected_labels
    )


# ============================================================
# Convert labels into readable names
# ============================================================

def readable_label_counts(counts):
    """
    Return semantic labels sorted by frequency.
    """

    readable = []

    for label_id, count in counts.most_common():

        label_name = SEMANTIC_LABELS.get(
            int(label_id),
            f"unknown-{label_id}",
        )

        readable.append(
            (
                label_name,
                count,
            )
        )

    return readable


# ============================================================
# Dominant semantic class
# ============================================================

def dominant_class(counts):
    """
    Return:

        dominant class name
        purity
    """

    if not counts:
        return (
            "unknown",
            0.0,
        )

    total = sum(
        counts.values()
    )

    label_id, count = (
        counts.most_common(1)[0]
    )

    label_name = SEMANTIC_LABELS.get(
        int(label_id),
        f"unknown-{label_id}",
    )

    purity = (
        count / total
    )

    return (
        label_name,
        purity,
    )


# ============================================================
# Level 1:
# Terrain / obstacle consistency
# ============================================================

def evaluate_obstacle_consistency(
    ground_truth,
):
    """
    Evaluate whether the dominant ground-truth class belongs
    to a navigational obstacle category.

    Returns one of:

        OBSTACLE
        NON-OBSTACLE
        UNKNOWN
    """

    if ground_truth == "unknown":
        return "UNKNOWN"

    if ground_truth in OBSTACLE_CLASSES:
        return "OBSTACLE"

    if ground_truth in NON_OBSTACLE_CLASSES:
        return "NON-OBSTACLE"

    return "UNKNOWN"


# ============================================================
# Level 2:
# Semantic classification
# ============================================================

def evaluate_semantic_classification(
    prediction,
    ground_truth,
):
    """
    Evaluate how specifically ORBIT classified the detected
    object.

    Returns:

        PREFERRED MATCH
            ORBIT semantic class agrees with GT.

        GENERIC OBSTACLE
            ORBIT correctly identified something as an obstacle,
            but did not identify its semantic type.

        MISMATCH
            ORBIT semantic class conflicts with dominant GT.

        UNKNOWN
            No valid GT labels.
    """

    if ground_truth == "unknown":
        return "UNKNOWN"

    preferred = PREFERRED_CLASSES.get(
        prediction,
        set(),
    )

    # --------------------------------------------------------
    # Specific semantic match
    # --------------------------------------------------------

    if ground_truth in preferred:
        return "PREFERRED MATCH"

    # --------------------------------------------------------
    # Generic obstacle result
    # --------------------------------------------------------

    if (
        prediction == "OBSTACLE"
        and ground_truth in GENERIC_OBSTACLE_CLASSES
    ):
        return "GENERIC OBSTACLE"

    # --------------------------------------------------------
    # Otherwise semantic mismatch
    # --------------------------------------------------------

    return "MISMATCH"


# ============================================================
# Distance band
# ============================================================

def distance_band(distance):
    """
    Group detections by distance.
    """

    if distance < 10.0:
        return "0-10m"

    if distance < 25.0:
        return "10-25m"

    if distance < 50.0:
        return "25-50m"

    return "50-100m"


# ============================================================
# Print semantic composition
# ============================================================

def print_label_composition(
    counts,
    maximum_entries=5,
):
    """
    Print the most common semantic labels.
    """

    readable = readable_label_counts(
        counts
    )

    if not readable:
        print(
            "  No ground-truth labels available."
        )
        return

    total = max(
        sum(counts.values()),
        1,
    )

    for label_name, count in readable[
        :maximum_entries
    ]:

        percentage = (
            count / total
        ) * 100.0

        print(
            f"  {label_name:<20}"
            f"{count:>6}"
            f" ({percentage:5.1f}%)"
        )


# ============================================================
# Main evaluation
# ============================================================

def main():

    print("=" * 70)
    print(
        "ORBIT - OBJECT DETECTION INSPECTION / EVALUATION"
    )
    print("=" * 70)

    # --------------------------------------------------------
    # Paths
    # --------------------------------------------------------

    base_path = Path(
        "data/semantic_kitti/sequences/00"
    )

    velodyne_path = (
        base_path
        / "velodyne"
        / "000000.bin"
    )

    label_path = (
        base_path
        / "labels"
        / "000000.label"
    )

    # --------------------------------------------------------
    # Load point cloud
    # --------------------------------------------------------

    print(
        "\nLoading SemanticKITTI frame..."
    )

    points = load_semantic_kitti_frame(
        velodyne_path
    )

    print(
        f"Points: {len(points):,}"
    )

    # --------------------------------------------------------
    # Load labels
    # --------------------------------------------------------

    print(
        "\nLoading SemanticKITTI labels..."
    )

    labels = load_labels(
        label_path
    )

    print(
        f"Labels: {len(labels):,}"
    )

    if len(points) != len(labels):

        raise ValueError(
            "Point cloud and label count do not match."
        )

    # --------------------------------------------------------
    # Ground detection
    # --------------------------------------------------------

    print(
        "\nDetecting ground..."
    )

    ground_mask = detect_ground(
        points,
        distance_threshold=0.08,
        seed=42,
    )

    print(
        f"Ground points: "
        f"{np.sum(ground_mask):,}"
    )

    print(
        f"Non-ground points: "
        f"{np.sum(~ground_mask):,}"
    )

    # --------------------------------------------------------
    # ORBIT detector
    # --------------------------------------------------------

    detector = ObjectDetector(
        min_points=5,
        max_range=100.0,
    )

    print(
        "\nRunning ORBIT detection..."
    )

    proposals = detector.detect(
        points
    )

    print(
        f"\nDetected objects: "
        f"{len(proposals)}"
    )

    # ========================================================
    # Detailed inspection
    # ========================================================

    print(
        "\n" + "=" * 70
    )

    print(
        "DETECTION INSPECTION"
    )

    print(
        "=" * 70
    )

    # --------------------------------------------------------
    # Global counters
    # --------------------------------------------------------

    obstacle_consistent = 0
    non_obstacle = 0
    obstacle_unknown = 0

    preferred_matches = 0
    generic_obstacles = 0
    semantic_mismatches = 0
    semantic_unknown = 0

    # --------------------------------------------------------
    # Distance statistics
    # --------------------------------------------------------

    distance_stats = defaultdict(
        lambda: {
            "total": 0,
            "obstacle": 0,
            "preferred": 0,
            "generic": 0,
            "mismatch": 0,
        }
    )

    # --------------------------------------------------------
    # Classification confusion-style summary
    # --------------------------------------------------------

    prediction_vs_gt = Counter()

    # ========================================================
    # Process proposals
    # ========================================================

    for proposal in proposals:

        print(
            "\n" + "-" * 70
        )

        print(
            f"OBJECT #{proposal.cluster_id}"
        )

        print(
            "-" * 70
        )

        print(
            f"ORBIT prediction: "
            f"{proposal.classification}"
        )

        print(
            f"Distance: "
            f"{proposal.distance:.2f} m"
        )

        print(
            f"ORBIT obstacle points: "
            f"{proposal.point_count:,}"
        )

        print(
            f"Evaluation point indices: "
            f"{len(proposal.point_indices):,}"
        )

        # ----------------------------------------------------
        # Ground truth
        # ----------------------------------------------------

        counts = inspect_object(
            proposal,
            labels,
        )

        ground_truth, purity = (
            dominant_class(
                counts
            )
        )

        print(
            f"\nDominant ground truth: "
            f"{ground_truth}"
        )

        print(
            f"Ground truth purity: "
            f"{purity * 100:.1f}%"
        )

        print(
            "\nSemantic label composition:"
        )

        print_label_composition(
            counts
        )

        # ----------------------------------------------------
        # Level 1
        # ----------------------------------------------------

        obstacle_result = (
            evaluate_obstacle_consistency(
                ground_truth
            )
        )

        print(
            f"\nLEVEL 1 - Terrain obstacle:"
            f" {obstacle_result}"
        )

        if obstacle_result == "OBSTACLE":

            obstacle_consistent += 1

        elif obstacle_result == "NON-OBSTACLE":

            non_obstacle += 1

        else:

            obstacle_unknown += 1

        # ----------------------------------------------------
        # Level 2
        # ----------------------------------------------------

        semantic_result = (
            evaluate_semantic_classification(
                proposal.classification,
                ground_truth,
            )
        )

        print(
            f"LEVEL 2 - Semantic class:"
            f" {semantic_result}"
        )

        if semantic_result == "PREFERRED MATCH":

            preferred_matches += 1

        elif semantic_result == "GENERIC OBSTACLE":

            generic_obstacles += 1

        elif semantic_result == "MISMATCH":

            semantic_mismatches += 1

        else:

            semantic_unknown += 1

        # ----------------------------------------------------
        # Distance statistics
        # ----------------------------------------------------

        band = distance_band(
            proposal.distance
        )

        stats = distance_stats[
            band
        ]

        stats["total"] += 1

        if obstacle_result == "OBSTACLE":
            stats["obstacle"] += 1

        if semantic_result == "PREFERRED MATCH":
            stats["preferred"] += 1

        elif semantic_result == "GENERIC OBSTACLE":
            stats["generic"] += 1

        elif semantic_result == "MISMATCH":
            stats["mismatch"] += 1

        # ----------------------------------------------------
        # Confusion-style pair
        # ----------------------------------------------------

        prediction_vs_gt[
            (
                proposal.classification,
                ground_truth,
            )
        ] += 1

    # ========================================================
    # Summary
    # ========================================================

    total = len(
        proposals
    )

    print(
        "\n" + "=" * 70
    )

    print(
        "LEVEL 1 - TERRAIN / OBSTACLE EVALUATION"
    )

    print(
        "=" * 70
    )

    print(
        f"\nTotal proposals: "
        f"{total}"
    )

    print(
        f"Obstacle-consistent: "
        f"{obstacle_consistent}"
    )

    print(
        f"Non-obstacle: "
        f"{non_obstacle}"
    )

    print(
        f"Unknown: "
        f"{obstacle_unknown}"
    )

    if total > 0:

        obstacle_rate = (
            obstacle_consistent
            / total
        ) * 100.0

        print(
            f"\nObstacle consistency: "
            f"{obstacle_rate:.1f}%"
        )

    # ========================================================
    # Level 2 summary
    # ========================================================

    print(
        "\n" + "=" * 70
    )

    print(
        "LEVEL 2 - SEMANTIC CLASSIFICATION EVALUATION"
    )

    print(
        "=" * 70
    )

    print(
        f"\nPreferred semantic matches: "
        f"{preferred_matches}"
    )

    print(
        f"Generic obstacle detections: "
        f"{generic_obstacles}"
    )

    print(
        f"Semantic mismatches: "
        f"{semantic_mismatches}"
    )

    print(
        f"Unknown: "
        f"{semantic_unknown}"
    )

    known_semantic = (
        preferred_matches
        + generic_obstacles
        + semantic_mismatches
    )

    if known_semantic > 0:

        semantic_specificity = (
            preferred_matches
            / known_semantic
        ) * 100.0

        print(
            f"\nSpecific semantic match rate: "
            f"{semantic_specificity:.1f}%"
        )

        useful_detection_rate = (
            (
                preferred_matches
                + generic_obstacles
            )
            / known_semantic
        ) * 100.0

        print(
            f"Obstacle-compatible classification: "
            f"{useful_detection_rate:.1f}%"
        )

    # ========================================================
    # Distance analysis
    # ========================================================

    print(
        "\n" + "=" * 70
    )

    print(
        "DISTANCE-WISE EVALUATION"
    )

    print(
        "=" * 70
    )

    ordered_bands = [
        "0-10m",
        "10-25m",
        "25-50m",
        "50-100m",
    ]

    for band in ordered_bands:

        stats = distance_stats[
            band
        ]

        if stats["total"] == 0:
            continue

        total_band = stats[
            "total"
        ]

        obstacle_pct = (
            stats["obstacle"]
            / total_band
        ) * 100.0

        specific_total = (
            stats["preferred"]
            + stats["generic"]
            + stats["mismatch"]
        )

        if specific_total > 0:

            preferred_pct = (
                stats["preferred"]
                / specific_total
            ) * 100.0

        else:

            preferred_pct = 0.0

        print(
            f"\n{band}:"
        )

        print(
            f"  Detections:              "
            f"{total_band}"
        )

        print(
            f"  Obstacle consistent:     "
            f"{stats['obstacle']}"
            f" ({obstacle_pct:.1f}%)"
        )

        print(
            f"  Preferred semantic:      "
            f"{stats['preferred']}"
        )

        print(
            f"  Generic obstacle:        "
            f"{stats['generic']}"
        )

        print(
            f"  Semantic mismatch:       "
            f"{stats['mismatch']}"
        )

        print(
            f"  Semantic specificity:     "
            f"{preferred_pct:.1f}%"
        )

    # ========================================================
    # Prediction vs dominant GT
    # ========================================================

    print(
        "\n" + "=" * 70
    )

    print(
        "PREDICTION / GROUND-TRUTH BREAKDOWN"
    )

    print(
        "=" * 70
    )

    prediction_groups = defaultdict(
        list
    )

    for (
        prediction,
        ground_truth,
    ), count in sorted(
        prediction_vs_gt.items()
    ):

        prediction_groups[
            prediction
        ].append(
            (
                ground_truth,
                count,
            )
        )

    for prediction in sorted(
        prediction_groups
    ):

        print(
            f"\n{prediction}:"
        )

        for ground_truth, count in sorted(
            prediction_groups[prediction],
            key=lambda item:
                -item[1],
        ):

            print(
                f"  -> "
                f"{ground_truth:<20}"
                f"{count}"
            )

    # ========================================================
    # Important interpretation
    # ========================================================

    print(
        "\n" + "=" * 70
    )

    print(
        "INTERPRETATION"
    )

    print(
        "=" * 70
    )

    print(
        "\nLevel 1 answers:"
    )

    print(
        "  Did ORBIT detect geometry that corresponds"
        " to an obstacle?"
    )

    print(
        "\nLevel 2 answers:"
    )

    print(
        "  Did ORBIT assign a useful semantic type to it?"
    )

    print(
        "\nNote:"
    )

    print(
        "  Level 1 is proposal-level obstacle consistency,"
        " not full obstacle recall."
    )

    print(
        "  True recall requires comparing ORBIT proposals"
        " against SemanticKITTI instance-level objects,"
    )

    print(
        "  which will be the next object-level evaluation step."
    )

    # ========================================================
    # Completion
    # ========================================================

    print(
        "\n" + "=" * 70
    )

    print(
        "ORBIT INSPECTION COMPLETE"
    )

    print(
        "=" * 70
    )


if __name__ == "__main__":
    main()