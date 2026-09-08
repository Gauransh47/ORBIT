import numpy as np
from pathlib import Path
from collections import defaultdict

from src.perception.object_detection import (
    ObjectDetector,
    load_semantic_kitti_frame,
)


# ============================================================
# ORBIT - OBJECT LEVEL DETECTION EVALUATION
# ============================================================


# ============================================================
# SemanticKITTI labels
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
# Object / "thing" classes
#
# These are classes where instance-level evaluation makes sense.
# ============================================================

THING_CLASSES = {

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
}


# ============================================================
# Load SemanticKITTI labels
# ============================================================

def load_labels(label_path):

    raw_labels = np.fromfile(
        label_path,
        dtype=np.uint32,
    )

    # Lower 16 bits = semantic class
    semantic_labels = (
        raw_labels & 0xFFFF
    ).astype(np.uint16)

    # Upper 16 bits = instance ID
    instance_labels = (
        raw_labels >> 16
    ).astype(np.uint16)

    return (
        semantic_labels,
        instance_labels,
    )


# ============================================================
# Ground truth object
# ============================================================

class GroundTruthObject:

    def __init__(
        self,
        object_id,
        semantic_id,
        instance_id,
        point_indices,
        points,
    ):

        self.object_id = object_id

        self.semantic_id = semantic_id

        self.semantic_name = (
            SEMANTIC_LABELS.get(
                int(semantic_id),
                f"unknown-{semantic_id}",
            )
        )

        self.instance_id = instance_id

        self.point_indices = point_indices

        self.points = points

        self.point_count = len(
            point_indices
        )

        self.center = self.compute_center()

        self.distance = float(
            np.hypot(
                self.center[0],
                self.center[1],
            )
        )

    # --------------------------------------------------------

    def compute_center(self):

        if len(self.points) == 0:

            return (
                0.0,
                0.0,
                0.0,
            )

        center = np.mean(
            self.points,
            axis=0,
        )

        return (
            float(center[0]),
            float(center[1]),
            float(center[2]),
        )


# ============================================================
# Build ground truth object instances
# ============================================================

def build_ground_truth_objects(
    points,
    semantic_labels,
    instance_labels,
    minimum_points=5,
):

    """
    Build SemanticKITTI object instances.

    Each unique:

        semantic class + instance ID

    becomes one ground-truth object.
    """

    objects = []

    # --------------------------------------------------------
    # Group points by:
    #
    # (semantic class, instance ID)
    # --------------------------------------------------------

    groups = defaultdict(list)

    for index, (
        semantic_id,
        instance_id,
    ) in enumerate(
        zip(
            semantic_labels,
            instance_labels,
        )
    ):

        semantic_id = int(
            semantic_id
        )

        instance_id = int(
            instance_id
        )

        # Only evaluate object / thing classes.
        if semantic_id not in THING_CLASSES:
            continue

        # Instance ID = 0 means no usable object instance.
        if instance_id == 0:
            continue

        key = (
            semantic_id,
            instance_id,
        )

        groups[key].append(
            index
        )

    # --------------------------------------------------------
    # Create objects
    # --------------------------------------------------------

    object_id = 1

    for (
        semantic_id,
        instance_id,
    ), indices in groups.items():

        indices = np.asarray(
            indices,
            dtype=np.int64,
        )

        if len(indices) < minimum_points:
            continue

        gt_object = GroundTruthObject(

            object_id=object_id,

            semantic_id=semantic_id,

            instance_id=instance_id,

            point_indices=indices,

            points=points[indices],
        )

        objects.append(
            gt_object
        )

        object_id += 1

    # --------------------------------------------------------
    # Sort by distance
    # --------------------------------------------------------

    objects.sort(
        key=lambda obj:
        obj.distance
    )

    # Renumber
    for index, obj in enumerate(
        objects,
        start=1,
    ):

        obj.object_id = index

    return objects


# ============================================================
# Exact point-index overlap
# ============================================================

def point_overlap(
    proposal_indices,
    gt_indices,
):

    """
    Calculate exact overlap between ORBIT proposal
    and SemanticKITTI object.
    """

    if (
        len(proposal_indices) == 0
        or len(gt_indices) == 0
    ):

        return 0

    return len(
        np.intersect1d(
            proposal_indices,
            gt_indices,
            assume_unique=True,
        )
    )


# ============================================================
# IoU using point indices
# ============================================================

def point_iou(
    proposal_indices,
    gt_indices,
):

    intersection = point_overlap(
        proposal_indices,
        gt_indices,
    )

    if intersection == 0:
        return 0.0

    union = (
        len(proposal_indices)
        + len(gt_indices)
        - intersection
    )

    if union <= 0:
        return 0.0

    return (
        intersection
        / union
    )


# ============================================================
# Proposal coverage
#
# How much of the GT object was captured?
# ============================================================

def ground_truth_coverage(
    proposal_indices,
    gt_indices,
):

    intersection = point_overlap(
        proposal_indices,
        gt_indices,
    )

    if len(gt_indices) == 0:
        return 0.0

    return (
        intersection
        / len(gt_indices)
    )


# ============================================================
# Proposal purity
#
# How much of the proposal belongs to the GT object?
# ============================================================

def proposal_purity(
    proposal_indices,
    gt_indices,
):

    intersection = point_overlap(
        proposal_indices,
        gt_indices,
    )

    if len(proposal_indices) == 0:
        return 0.0

    return (
        intersection
        / len(proposal_indices)
    )


# ============================================================
# Match ORBIT proposals to GT objects
# ============================================================

def match_proposals_to_objects(
    proposals,
    ground_truth_objects,
):

    """
    For every proposal, find the GT object with maximum
    point-index overlap.

    Returns a list of dictionaries containing the best match.
    """

    matches = []

    for proposal in proposals:

        best_object = None

        best_overlap = 0

        best_iou = 0.0

        for gt_object in ground_truth_objects:

            overlap = point_overlap(
                proposal.point_indices,
                gt_object.point_indices,
            )

            if overlap == 0:
                continue

            iou = point_iou(
                proposal.point_indices,
                gt_object.point_indices,
            )

            if overlap > best_overlap:

                best_overlap = overlap

                best_iou = iou

                best_object = gt_object

        if best_object is None:

            matches.append({

                "proposal": proposal,

                "ground_truth": None,

                "overlap": 0,

                "iou": 0.0,

                "coverage": 0.0,

                "purity": 0.0,

            })

        else:

            coverage = (
                ground_truth_coverage(
                    proposal.point_indices,
                    best_object.point_indices,
                )
            )

            purity = (
                proposal_purity(
                    proposal.point_indices,
                    best_object.point_indices,
                )
            )

            matches.append({

                "proposal": proposal,

                "ground_truth": best_object,

                "overlap": best_overlap,

                "iou": best_iou,

                "coverage": coverage,

                "purity": purity,

            })

    return matches


# ============================================================
# Determine whether a GT object was detected
# ============================================================

def evaluate_object_detection(
    ground_truth_objects,
    matches,
    minimum_coverage=0.20,
):

    """
    A ground-truth object is considered detected when at least
    one ORBIT proposal captures `minimum_coverage` of its
    labelled points.

    20% is intentionally permissive for V1 because LiDAR
    objects may be fragmented across multiple components.
    """

    results = []

    for gt_object in ground_truth_objects:

        object_matches = []

        for match in matches:

            if (
                match["ground_truth"]
                is gt_object
            ):

                object_matches.append(
                    match
                )

        # ----------------------------------------------------
        # Best proposal coverage
        # ----------------------------------------------------

        best_coverage = 0.0

        best_match = None

        for match in object_matches:

            if (
                match["coverage"]
                > best_coverage
            ):

                best_coverage = (
                    match["coverage"]
                )

                best_match = match

        detected = (
            best_coverage
            >= minimum_coverage
        )

        results.append({

            "ground_truth": gt_object,

            "detected": detected,

            "best_coverage": best_coverage,

            "best_match": best_match,

            "proposal_count": len(
                object_matches
            ),

        })

    return results


# ============================================================
# Proposal precision
# ============================================================

def evaluate_proposal_precision(
    matches,
    minimum_purity=0.20,
):

    """
    A proposal is considered a valid object proposal if at least
    `minimum_purity` of its points belong to a GT thing object.
    """

    results = []

    for match in matches:

        valid = (

            match["ground_truth"]
            is not None

            and

            match["purity"]
            >= minimum_purity
        )

        results.append({

            "match": match,

            "valid": valid,

        })

    return results


# ============================================================
# Distance bucket
# ============================================================

def distance_bucket(
    distance,
):

    if distance < 10:

        return "0-10m"

    if distance < 25:

        return "10-25m"

    if distance < 50:

        return "25-50m"

    return "50-100m"


# ============================================================
# Print ground truth objects
# ============================================================

def print_ground_truth_objects(
    objects,
):

    print(
        "\n" + "=" * 70
    )

    print(
        "GROUND TRUTH OBJECT INSTANCES"
    )

    print(
        "=" * 70
    )

    if not objects:

        print(
            "\nNo valid thing instances found."
        )

        return

    for obj in objects:

        print(
            f"\nGT OBJECT #{obj.object_id}"
        )

        print(
            f"  Class:       "
            f"{obj.semantic_name}"
        )

        print(
            f"  Instance ID: "
            f"{obj.instance_id}"
        )

        print(
            f"  Points:      "
            f"{obj.point_count}"
        )

        print(
            f"  Distance:    "
            f"{obj.distance:.2f} m"
        )


# ============================================================
# Print proposal matches
# ============================================================

def print_proposal_matches(
    matches,
):

    print(
        "\n" + "=" * 70
    )

    print(
        "PROPOSAL → GROUND TRUTH MATCHING"
    )

    print(
        "=" * 70
    )

    for match in matches:

        proposal = match[
            "proposal"
        ]

        gt_object = match[
            "ground_truth"
        ]

        print(
            "\n" + "-" * 70
        )

        print(
            f"ORBIT PROPOSAL "
            f"#{proposal.object_id}"
        )

        print(
            "-" * 70
        )

        print(
            f"Prediction: "
            f"{proposal.classification}"
        )

        print(
            f"Distance: "
            f"{proposal.distance:.2f} m"
        )

        if gt_object is None:

            print(
                "\nBest GT match: NONE"
            )

            continue

        print(
            f"\nBest GT match: "
            f"{gt_object.semantic_name}"
        )

        print(
            f"GT Object ID: "
            f"{gt_object.object_id}"
        )

        print(
            f"GT Instance ID: "
            f"{gt_object.instance_id}"
        )

        print(
            f"Point overlap: "
            f"{match['overlap']}"
        )

        print(
            f"Point IoU: "
            f"{match['iou'] * 100:.1f}%"
        )

        print(
            f"GT coverage: "
            f"{match['coverage'] * 100:.1f}%"
        )

        print(
            f"Proposal purity: "
            f"{match['purity'] * 100:.1f}%"
        )


# ============================================================
# Evaluation summary
# ============================================================

def print_evaluation_summary(
    object_results,
    proposal_results,
):

    print(
        "\n" + "=" * 70
    )

    print(
        "OBJECT LEVEL EVALUATION SUMMARY"
    )

    print(
        "=" * 70
    )

    total_objects = len(
        object_results
    )

    detected_objects = sum(

        result["detected"]

        for result in object_results
    )

    total_proposals = len(
        proposal_results
    )

    valid_proposals = sum(

        result["valid"]

        for result in proposal_results
    )

    # --------------------------------------------------------
    # Metrics
    # --------------------------------------------------------

    recall = (

        detected_objects
        / total_objects

        if total_objects > 0
        else 0.0
    )

    precision = (

        valid_proposals
        / total_proposals

        if total_proposals > 0
        else 0.0
    )

    if (
        precision + recall
        > 0
    ):

        f1 = (

            2
            * precision
            * recall
            / (
                precision
                + recall
            )
        )

    else:

        f1 = 0.0

    print(
        f"\nGround truth objects: "
        f"{total_objects}"
    )

    print(
        f"Detected objects: "
        f"{detected_objects}"
    )

    print(
        f"Object recall: "
        f"{recall * 100:.1f}%"
    )

    print(
        f"\nORBIT proposals: "
        f"{total_proposals}"
    )

    print(
        f"Valid thing proposals: "
        f"{valid_proposals}"
    )

    print(
        f"Proposal precision: "
        f"{precision * 100:.1f}%"
    )

    print(
        f"\nF1 score: "
        f"{f1 * 100:.1f}%"
    )

    # ========================================================
    # Fragmentation
    # ========================================================

    print(
        "\n" + "=" * 70
    )

    print(
        "OBJECT FRAGMENTATION"
    )

    print(
        "=" * 70
    )

    fragmented = 0

    for result in object_results:

        proposal_count = (
            result["proposal_count"]
        )

        if proposal_count > 1:

            fragmented += 1

            gt_object = result[
                "ground_truth"
            ]

            print(
                f"\nGT Object "
                f"#{gt_object.object_id} "
                f"({gt_object.semantic_name})"
            )

            print(
                f"  Matched proposals: "
                f"{proposal_count}"
            )

            print(
                f"  Best coverage: "
                f"{result['best_coverage'] * 100:.1f}%"
            )

    if fragmented == 0:

        print(
            "\nNo fragmented GT objects."
        )

    else:

        print(
            f"\nFragmented objects: "
            f"{fragmented}"
        )

    # ========================================================
    # Per-class recall
    # ========================================================

    print(
        "\n" + "=" * 70
    )

    print(
        "PER-CLASS OBJECT RECALL"
    )

    print(
        "=" * 70
    )

    class_stats = defaultdict(
        lambda: {
            "total": 0,
            "detected": 0,
        }
    )

    for result in object_results:

        gt_object = result[
            "ground_truth"
        ]

        class_name = (
            gt_object.semantic_name
        )

        class_stats[
            class_name
        ]["total"] += 1

        if result["detected"]:

            class_stats[
                class_name
            ]["detected"] += 1

    for class_name in sorted(
        class_stats
    ):

        stats = class_stats[
            class_name
        ]

        class_recall = (

            stats["detected"]
            / stats["total"]

            if stats["total"] > 0
            else 0.0
        )

        print(
            f"\n{class_name.upper()}"
        )

        print(
            f"  Objects: "
            f"{stats['total']}"
        )

        print(
            f"  Detected: "
            f"{stats['detected']}"
        )

        print(
            f"  Recall: "
            f"{class_recall * 100:.1f}%"
        )

    # ========================================================
    # Distance-wise recall
    # ========================================================

    print(
        "\n" + "=" * 70
    )

    print(
        "DISTANCE-WISE OBJECT RECALL"
    )

    print(
        "=" * 70
    )

    distance_stats = defaultdict(
        lambda: {
            "total": 0,
            "detected": 0,
        }
    )

    for result in object_results:

        gt_object = result[
            "ground_truth"
        ]

        bucket = distance_bucket(
            gt_object.distance
        )

        distance_stats[
            bucket
        ]["total"] += 1

        if result["detected"]:

            distance_stats[
                bucket
            ]["detected"] += 1

    bucket_order = [

        "0-10m",

        "10-25m",

        "25-50m",

        "50-100m",

    ]

    for bucket in bucket_order:

        stats = distance_stats[
            bucket
        ]

        if stats["total"] == 0:
            continue

        recall = (
            stats["detected"]
            / stats["total"]
        )

        print(
            f"\n{bucket}"
        )

        print(
            f"  Objects: "
            f"{stats['total']}"
        )

        print(
            f"  Detected: "
            f"{stats['detected']}"
        )

        print(
            f"  Recall: "
            f"{recall * 100:.1f}%"
        )


# ============================================================
# Main
# ============================================================

def main():

    print("=" * 70)

    print(
        "ORBIT - OBJECT LEVEL DETECTION EVALUATION"
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
    # Load frame
    # --------------------------------------------------------

    print(
        "\nLoading SemanticKITTI frame..."
    )

    points = load_semantic_kitti_frame(
        velodyne_path
    )

    print(
        f"Points: "
        f"{len(points):,}"
    )

    # --------------------------------------------------------
    # Load labels
    # --------------------------------------------------------

    print(
        "\nLoading SemanticKITTI labels..."
    )

    (
        semantic_labels,
        instance_labels,
    ) = load_labels(
        label_path
    )

    print(
        f"Semantic labels: "
        f"{len(semantic_labels):,}"
    )

    print(
        f"Instance labels: "
        f"{len(instance_labels):,}"
    )

    if (
        len(points)
        != len(semantic_labels)
    ):

        raise ValueError(
            "Point count and label count do not match."
        )

    # --------------------------------------------------------
    # Build GT objects
    # --------------------------------------------------------

    print(
        "\nBuilding ground-truth "
        "object instances..."
    )

    ground_truth_objects = (
        build_ground_truth_objects(
            points,
            semantic_labels,
            instance_labels,
            minimum_points=5,
        )
    )

    print(
        f"Ground-truth objects: "
        f"{len(ground_truth_objects)}"
    )

    print_ground_truth_objects(
        ground_truth_objects
    )

    # --------------------------------------------------------
    # Run ORBIT
    # --------------------------------------------------------

    print(
        "\n" + "=" * 70
    )

    print(
        "RUNNING ORBIT DETECTOR"
    )

    print(
        "=" * 70
    )

    detector = ObjectDetector()

    proposals = detector.detect(
        points
    )

    print(
        f"\nORBIT proposals: "
        f"{len(proposals)}"
    )

    # --------------------------------------------------------
    # Match
    # --------------------------------------------------------

    print(
        "\nMatching proposals to "
        "ground-truth objects..."
    )

    matches = (
        match_proposals_to_objects(
            proposals,
            ground_truth_objects,
        )
    )

    print_proposal_matches(
        matches
    )

    # --------------------------------------------------------
    # Object recall
    # --------------------------------------------------------

    object_results = (
        evaluate_object_detection(
            ground_truth_objects,
            matches,
            minimum_coverage=0.20,
        )
    )

    # --------------------------------------------------------
    # Proposal precision
    # --------------------------------------------------------

    proposal_results = (
        evaluate_proposal_precision(
            matches,
            minimum_purity=0.20,
        )
    )

    # --------------------------------------------------------
    # Summary
    # --------------------------------------------------------

    print_evaluation_summary(
        object_results,
        proposal_results,
    )

    print(
        "\n" + "=" * 70
    )

    print(
        "ORBIT OBJECT LEVEL EVALUATION COMPLETE"
    )

    print(
        "=" * 70
    )


if __name__ == "__main__":
    main()