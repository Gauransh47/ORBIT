import numpy as np
from pathlib import Path
from collections import defaultdict

from src.perception.object_detection import (
    ObjectDetector,
    load_semantic_kitti_frame,
)


# ============================================================
# ORBIT - BATCH OBJECT LEVEL EVALUATION
#
# Evaluates SemanticKITTI sequence 00 across multiple frames.
#
# Current local dataset:
#   frames 000000 -> 000009
#
# Metrics:
#   - GT object count
#   - ORBIT proposal count
#   - object recall
#   - proposal precision
#   - F1
#   - GT coverage
#   - point IoU
#   - fragmentation
#   - per-class recall
#   - distance-wise recall
#
# Matching uses exact original point indices.
# ============================================================


# ============================================================
# Configuration
# ============================================================

SEQUENCE = "00"

START_FRAME = 0
END_FRAME = 9

MINIMUM_GT_POINTS = 5

# A GT object is considered detected when an ORBIT proposal
# captures at least this fraction of its points.
MINIMUM_COVERAGE = 0.20

# A proposal is considered a valid object proposal when at
# least this fraction of its evaluation points belong to a
# GT "thing" object.
MINIMUM_PURITY = 0.20


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
# Thing classes
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
# Ground-truth object
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

        self.semantic_id = int(
            semantic_id
        )

        self.semantic_name = (
            SEMANTIC_LABELS.get(
                self.semantic_id,
                f"unknown-{self.semantic_id}",
            )
        )

        self.instance_id = int(
            instance_id
        )

        self.point_indices = np.asarray(
            point_indices,
            dtype=np.int64,
        )

        self.points = points[
            self.point_indices
        ]

        self.point_count = len(
            self.point_indices
        )

        if self.point_count > 0:

            center = np.mean(
                self.points,
                axis=0,
            )

        else:

            center = np.zeros(
                3,
                dtype=float,
            )

        self.center = (

            float(center[0]),
            float(center[1]),
            float(center[2]),
        )

        self.distance = float(
            np.hypot(
                self.center[0],
                self.center[1],
            )
        )


# ============================================================
# Load labels
# ============================================================

def load_labels(
    label_path,
):

    raw = np.fromfile(
        label_path,
        dtype=np.uint32,
    )

    semantic = (
        raw & 0xFFFF
    ).astype(
        np.uint16
    )

    instance = (
        raw >> 16
    ).astype(
        np.uint16
    )

    return (
        semantic,
        instance,
    )


# ============================================================
# Build GT objects
# ============================================================

def build_ground_truth_objects(
    points,
    semantic,
    instance,
):

    grouped = defaultdict(list)

    for index in range(
        len(points)
    ):

        semantic_id = int(
            semantic[index]
        )

        instance_id = int(
            instance[index]
        )

        if semantic_id not in THING_CLASSES:
            continue

        if instance_id == 0:
            continue

        grouped[
            (
                semantic_id,
                instance_id,
            )
        ].append(
            index
        )

    objects = []

    object_id = 1

    for (
        semantic_id,
        instance_id,
    ), indices in grouped.items():

        if len(indices) < MINIMUM_GT_POINTS:
            continue

        obj = GroundTruthObject(

            object_id=object_id,

            semantic_id=semantic_id,

            instance_id=instance_id,

            point_indices=np.asarray(
                indices,
                dtype=np.int64,
            ),

            points=points,
        )

        objects.append(
            obj
        )

        object_id += 1

    objects.sort(
        key=lambda obj:
        obj.distance
    )

    for index, obj in enumerate(
        objects,
        start=1,
    ):

        obj.object_id = index

    return objects


# ============================================================
# Exact point overlap
# ============================================================

def point_overlap(
    proposal_indices,
    gt_indices,
):

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
# Point IoU
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

    return (
        intersection / union
    )


# ============================================================
# GT coverage
# ============================================================

def gt_coverage(
    proposal_indices,
    gt_indices,
):

    if len(gt_indices) == 0:
        return 0.0

    return (
        point_overlap(
            proposal_indices,
            gt_indices,
        )
        / len(gt_indices)
    )


# ============================================================
# Proposal purity
# ============================================================

def proposal_purity(
    proposal_indices,
    gt_indices,
):

    if len(proposal_indices) == 0:
        return 0.0

    return (
        point_overlap(
            proposal_indices,
            gt_indices,
        )
        / len(proposal_indices)
    )


# ============================================================
# Pairwise proposal / GT scores
# ============================================================

def build_candidate_pairs(
    proposals,
    ground_truth_objects,
):

    candidates = []

    for proposal_index, proposal in enumerate(
        proposals
    ):

        proposal_indices = np.asarray(
            proposal.point_indices,
            dtype=np.int64,
        )

        if len(proposal_indices) == 0:
            continue

        for gt_index, gt_object in enumerate(
            ground_truth_objects
        ):

            gt_indices = (
                gt_object.point_indices
            )

            overlap = point_overlap(
                proposal_indices,
                gt_indices,
            )

            if overlap == 0:
                continue

            coverage = gt_coverage(
                proposal_indices,
                gt_indices,
            )

            purity = proposal_purity(
                proposal_indices,
                gt_indices,
            )

            iou = point_iou(
                proposal_indices,
                gt_indices,
            )

            candidates.append({

                "proposal_index":
                    proposal_index,

                "gt_index":
                    gt_index,

                "overlap":
                    overlap,

                "coverage":
                    coverage,

                "purity":
                    purity,

                "iou":
                    iou,

            })

    # Best overlaps first.
    candidates.sort(
        key=lambda item: (
            item["coverage"],
            item["iou"],
            item["purity"],
            item["overlap"],
        ),
        reverse=True,
    )

    return candidates


# ============================================================
# One-to-one matching
# ============================================================

def match_one_to_one(
    proposals,
    ground_truth_objects,
):

    candidates = build_candidate_pairs(
        proposals,
        ground_truth_objects,
    )

    matched_proposals = set()
    matched_gt = set()

    matches = []

    # --------------------------------------------------------
    # Greedy global matching.
    #
    # Candidate list is sorted by GT coverage, then IoU.
    # Each proposal and GT object can be used once.
    # --------------------------------------------------------

    for candidate in candidates:

        proposal_index = candidate[
            "proposal_index"
        ]

        gt_index = candidate[
            "gt_index"
        ]

        if proposal_index in matched_proposals:
            continue

        if gt_index in matched_gt:
            continue

        if (
            candidate["coverage"]
            < MINIMUM_COVERAGE
        ):
            continue

        matched_proposals.add(
            proposal_index
        )

        matched_gt.add(
            gt_index
        )

        matches.append(
            candidate
        )

    return (
        matches,
        matched_proposals,
        matched_gt,
        candidates,
    )


# ============================================================
# Distance bucket
# ============================================================

def distance_bucket(
    distance,
):

    if distance < 10.0:
        return "0-10m"

    if distance < 25.0:
        return "10-25m"

    if distance < 50.0:
        return "25-50m"

    return "50-100m"


# ============================================================
# Evaluate one frame
# ============================================================

def evaluate_frame(
    frame_number,
    detector,
):

    frame_name = (
        f"{frame_number:06d}"
    )

    base = (
        Path("data")
        / "semantic_kitti"
        / "sequences"
        / SEQUENCE
    )

    velodyne_path = (
        base
        / "velodyne"
        / f"{frame_name}.bin"
    )

    label_path = (
        base
        / "labels"
        / f"{frame_name}.label"
    )

    # --------------------------------------------------------
    # File validation
    # --------------------------------------------------------

    if not velodyne_path.exists():

        raise FileNotFoundError(
            f"Missing LiDAR frame:\n"
            f"{velodyne_path}"
        )

    if not label_path.exists():

        raise FileNotFoundError(
            f"Missing label file:\n"
            f"{label_path}"
        )

    # --------------------------------------------------------
    # Load
    # --------------------------------------------------------

    points = load_semantic_kitti_frame(
        velodyne_path
    )

    semantic, instance = load_labels(
        label_path
    )

    if len(points) != len(semantic):

        raise ValueError(
            f"Frame {frame_name}: "
            f"point/label mismatch: "
            f"{len(points)} vs "
            f"{len(semantic)}"
        )

    # --------------------------------------------------------
    # GT
    # --------------------------------------------------------

    gt_objects = (
        build_ground_truth_objects(
            points,
            semantic,
            instance,
        )
    )

    # --------------------------------------------------------
    # ORBIT
    # --------------------------------------------------------

    proposals = detector.detect(
        points
    )

    # --------------------------------------------------------
    # One-to-one matching
    # --------------------------------------------------------

    (
        matches,
        matched_proposals,
        matched_gt,
        candidates,
    ) = match_one_to_one(
        proposals,
        gt_objects,
    )

    # --------------------------------------------------------
    # Valid proposal precision
    #
    # A proposal must have at least MINIMUM_PURITY overlap
    # with some GT thing object.
    # --------------------------------------------------------

    valid_proposals = set()

    for candidate in candidates:

        proposal_index = candidate[
            "proposal_index"
        ]

        if (
            candidate["purity"]
            >= MINIMUM_PURITY
        ):

            valid_proposals.add(
                proposal_index
            )

    # --------------------------------------------------------
    # Fragmentation
    #
    # Count how many proposals meaningfully overlap each GT
    # object, irrespective of whether they passed the recall
    # threshold.
    # --------------------------------------------------------

    gt_candidate_counts = defaultdict(
        int
    )

    for candidate in candidates:

        if (
            candidate["coverage"]
            > 0.0
        ):

            gt_candidate_counts[
                candidate["gt_index"]
            ] += 1

    fragmented_gt = 0

    fragmentation_excess = 0

    for count in gt_candidate_counts.values():

        if count > 1:

            fragmented_gt += 1

            fragmentation_excess += (
                count - 1
            )

    # --------------------------------------------------------
    # Best coverage for every GT object
    # --------------------------------------------------------

    best_gt_coverage = {}

    for gt_index in range(
        len(gt_objects)
    ):

        best = 0.0

        for candidate in candidates:

            if (
                candidate["gt_index"]
                == gt_index
            ):

                best = max(
                    best,
                    candidate["coverage"],
                )

        best_gt_coverage[
            gt_index
        ] = best

    # --------------------------------------------------------
    # Matched-pair statistics
    # --------------------------------------------------------

    matched_coverages = [
        item["coverage"]
        for item in matches
    ]

    matched_ious = [
        item["iou"]
        for item in matches
    ]

    # --------------------------------------------------------
    # Class recall
    # --------------------------------------------------------

    class_stats = defaultdict(
        lambda: {
            "total": 0,
            "detected": 0,
        }
    )

    for gt_index, gt_object in enumerate(
        gt_objects
    ):

        class_name = (
            gt_object.semantic_name
        )

        class_stats[
            class_name
        ]["total"] += 1

        if gt_index in matched_gt:

            class_stats[
                class_name
            ]["detected"] += 1

    # --------------------------------------------------------
    # Distance recall
    # --------------------------------------------------------

    distance_stats = defaultdict(
        lambda: {
            "total": 0,
            "detected": 0,
        }
    )

    for gt_index, gt_object in enumerate(
        gt_objects
    ):

        bucket = distance_bucket(
            gt_object.distance
        )

        distance_stats[
            bucket
        ]["total"] += 1

        if gt_index in matched_gt:

            distance_stats[
                bucket
            ]["detected"] += 1

    # --------------------------------------------------------
    # Frame metrics
    # --------------------------------------------------------

    total_gt = len(
        gt_objects
    )

    total_proposals = len(
        proposals
    )

    detected_gt = len(
        matched_gt
    )

    valid_proposal_count = len(
        valid_proposals
    )

    recall = (

        detected_gt / total_gt

        if total_gt > 0
        else 0.0
    )

    precision = (

        valid_proposal_count
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

    mean_best_coverage = (

        float(
            np.mean(
                list(
                    best_gt_coverage.values()
                )
            )
        )

        if best_gt_coverage
        else 0.0
    )

    mean_matched_iou = (

        float(
            np.mean(
                matched_ious
            )
        )

        if matched_ious
        else 0.0
    )

    return {

        "frame":
            frame_name,

        "points":
            len(points),

        "gt_objects":
            total_gt,

        "proposals":
            total_proposals,

        "detected_gt":
            detected_gt,

        "precision":
            precision,

        "recall":
            recall,

        "f1":
            f1,

        "mean_best_coverage":
            mean_best_coverage,

        "mean_matched_iou":
            mean_matched_iou,

        "fragmented_gt":
            fragmented_gt,

        "fragmentation_excess":
            fragmentation_excess,

        "class_stats":
            class_stats,

        "distance_stats":
            distance_stats,

        "matched_coverages":
            matched_coverages,

        "matched_ious":
            matched_ious,

        "gt_objects_detail":
            gt_objects,

        "proposals_detail":
            proposals,

        "matches":
            matches,

        "valid_proposals":
            valid_proposals,

    }


# ============================================================
# Print frame summary
# ============================================================

def print_frame_summary(
    result,
):

    print(
        f"FRAME {result['frame']} | "
        f"GT: {result['gt_objects']:2d} | "
        f"Props: {result['proposals']:3d} | "
        f"Detected: {result['detected_gt']:2d} | "
        f"Recall: {result['recall'] * 100:5.1f}% | "
        f"Precision: {result['precision'] * 100:5.1f}% | "
        f"F1: {result['f1'] * 100:5.1f}% | "
        f"IoU: {result['mean_matched_iou'] * 100:5.1f}%"
    )


# ============================================================
# Aggregate results
# ============================================================

def aggregate_results(
    results,
):

    total_points = sum(
        result["points"]
        for result in results
    )

    total_gt = sum(
        result["gt_objects"]
        for result in results
    )

    total_proposals = sum(
        result["proposals"]
        for result in results
    )

    total_detected = sum(
        result["detected_gt"]
        for result in results
    )

    total_fragmented = sum(
        result["fragmented_gt"]
        for result in results
    )

    total_fragmentation_excess = sum(
        result["fragmentation_excess"]
        for result in results
    )

    aggregate_recall = (

        total_detected
        / total_gt

        if total_gt > 0
        else 0.0
    )

    # Note:
    # Precision is based on aggregate valid proposals,
    # reconstructed from each frame's valid proposal set.
    total_valid_proposals = sum(
        len(result["valid_proposals"])
        for result in results
    )

    aggregate_precision = (

        total_valid_proposals
        / total_proposals

        if total_proposals > 0
        else 0.0
    )

    if (
        aggregate_precision
        + aggregate_recall
        > 0
    ):

        aggregate_f1 = (

            2
            * aggregate_precision
            * aggregate_recall
            / (
                aggregate_precision
                + aggregate_recall
            )
        )

    else:

        aggregate_f1 = 0.0

    mean_frame_recall = float(
        np.mean(
            [
                result["recall"]
                for result in results
            ]
        )
    )

    mean_frame_precision = float(
        np.mean(
            [
                result["precision"]
                for result in results
            ]
        )
    )

    mean_frame_f1 = float(
        np.mean(
            [
                result["f1"]
                for result in results
            ]
        )
    )

    mean_coverage = float(
        np.mean(
            [
                result[
                    "mean_best_coverage"
                ]
                for result in results
            ]
        )
    )

    mean_iou = float(
        np.mean(
            [
                result[
                    "mean_matched_iou"
                ]
                for result in results
            ]
        )
    )

    # --------------------------------------------------------
    # Class aggregation
    # --------------------------------------------------------

    class_stats = defaultdict(
        lambda: {
            "total": 0,
            "detected": 0,
        }
    )

    for result in results:

        for class_name, stats in (
            result["class_stats"].items()
        ):

            class_stats[
                class_name
            ]["total"] += stats[
                "total"
            ]

            class_stats[
                class_name
            ]["detected"] += stats[
                "detected"
            ]

    # --------------------------------------------------------
    # Distance aggregation
    # --------------------------------------------------------

    distance_stats = defaultdict(
        lambda: {
            "total": 0,
            "detected": 0,
        }
    )

    for result in results:

        for bucket, stats in (
            result["distance_stats"].items()
        ):

            distance_stats[
                bucket
            ]["total"] += stats[
                "total"
            ]

            distance_stats[
                bucket
            ]["detected"] += stats[
                "detected"
            ]

    return {

        "frames": len(results),

        "total_points":
            total_points,

        "total_gt":
            total_gt,

        "total_proposals":
            total_proposals,

        "total_detected":
            total_detected,

        "aggregate_precision":
            aggregate_precision,

        "aggregate_recall":
            aggregate_recall,

        "aggregate_f1":
            aggregate_f1,

        "mean_frame_precision":
            mean_frame_precision,

        "mean_frame_recall":
            mean_frame_recall,

        "mean_frame_f1":
            mean_frame_f1,

        "mean_coverage":
            mean_coverage,

        "mean_iou":
            mean_iou,

        "fragmented_gt":
            total_fragmented,

        "fragmentation_excess":
            total_fragmentation_excess,

        "class_stats":
            class_stats,

        "distance_stats":
            distance_stats,

    }


# ============================================================
# Print aggregate summary
# ============================================================

def print_aggregate_summary(
    aggregate,
):

    print(
        "\n" + "=" * 70
    )

    print(
        "10-FRAME ORBIT EVALUATION SUMMARY"
    )

    print(
        "=" * 70
    )

    print(
        f"\nFrames evaluated:           "
        f"{aggregate['frames']}"
    )

    print(
        f"Total LiDAR points:         "
        f"{aggregate['total_points']:,}"
    )

    print(
        f"Total GT objects:           "
        f"{aggregate['total_gt']}"
    )

    print(
        f"Total ORBIT proposals:      "
        f"{aggregate['total_proposals']}"
    )

    print(
        f"GT objects detected:        "
        f"{aggregate['total_detected']}"
    )

    print(
        "\nAGGREGATE METRICS"
    )

    print(
        "-" * 70
    )

    print(
        f"Aggregate recall:           "
        f"{aggregate['aggregate_recall'] * 100:.1f}%"
    )

    print(
        f"Aggregate precision:        "
        f"{aggregate['aggregate_precision'] * 100:.1f}%"
    )

    print(
        f"Aggregate F1:               "
        f"{aggregate['aggregate_f1'] * 100:.1f}%"
    )

    print(
        "\nFRAME-AVERAGED METRICS"
    )

    print(
        "-" * 70
    )

    print(
        f"Mean frame recall:           "
        f"{aggregate['mean_frame_recall'] * 100:.1f}%"
    )

    print(
        f"Mean frame precision:        "
        f"{aggregate['mean_frame_precision'] * 100:.1f}%"
    )

    print(
        f"Mean frame F1:              "
        f"{aggregate['mean_frame_f1'] * 100:.1f}%"
    )

    print(
        f"Mean best GT coverage:      "
        f"{aggregate['mean_coverage'] * 100:.1f}%"
    )

    print(
        f"Mean matched point IoU:     "
        f"{aggregate['mean_iou'] * 100:.1f}%"
    )

    print(
        "\nFRAGMENTATION"
    )

    print(
        "-" * 70
    )

    print(
        f"GT objects with fragmentation: "
        f"{aggregate['fragmented_gt']}"
    )

    print(
        f"Fragmentation excess:           "
        f"{aggregate['fragmentation_excess']}"
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

    for class_name in sorted(
        aggregate["class_stats"]
    ):

        stats = aggregate[
            "class_stats"
        ][class_name]

        recall = (

            stats["detected"]
            / stats["total"]

            if stats["total"] > 0
            else 0.0
        )

        print(
            f"\n{class_name.upper()}"
        )

        print(
            f"  Objects:  "
            f"{stats['total']}"
        )

        print(
            f"  Detected: "
            f"{stats['detected']}"
        )

        print(
            f"  Recall:   "
            f"{recall * 100:.1f}%"
        )

    # ========================================================
    # Distance recall
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

    order = [
        "0-10m",
        "10-25m",
        "25-50m",
        "50-100m",
    ]

    for bucket in order:

        stats = aggregate[
            "distance_stats"
        ].get(
            bucket,
            {
                "total": 0,
                "detected": 0,
            },
        )

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
            f"  GT objects: "
            f"{stats['total']}"
        )

        print(
            f"  Detected:   "
            f"{stats['detected']}"
        )

        print(
            f"  Recall:     "
            f"{recall * 100:.1f}%"
        )


# ============================================================
# Main
# ============================================================

def main():

    print("=" * 70)

    print(
        "ORBIT - 10-FRAME OBJECT LEVEL EVALUATION"
    )

    print("=" * 70)

    print(
        f"\nSequence: "
        f"{SEQUENCE}"
    )

    print(
        f"Frames:   "
        f"{START_FRAME:06d} -> "
        f"{END_FRAME:06d}"
    )

    print(
        f"Coverage threshold: "
        f"{MINIMUM_COVERAGE * 100:.0f}%"
    )

    print(
        f"Purity threshold:   "
        f"{MINIMUM_PURITY * 100:.0f}%"
    )

    detector = ObjectDetector(
        min_points=5,
        max_range=100.0,
    )

    results = []

    # --------------------------------------------------------
    # Run all frames
    # --------------------------------------------------------

    for frame_number in range(
        START_FRAME,
        END_FRAME + 1,
    ):

        print(
            "\n" + "=" * 70
        )

        print(
            f"PROCESSING FRAME "
            f"{frame_number:06d}"
        )

        print(
            "=" * 70
        )

        try:

            result = evaluate_frame(
                frame_number,
                detector,
            )

            results.append(
                result
            )

            print_frame_summary(
                result
            )

        except Exception as exc:

            print(
                f"\nERROR in frame "
                f"{frame_number:06d}:"
            )

            print(
                f"  {type(exc).__name__}: "
                f"{exc}"
            )

            print(
                "\nSkipping frame and "
                "continuing batch evaluation."
            )

    # --------------------------------------------------------
    # No successful frames
    # --------------------------------------------------------

    if not results:

        raise RuntimeError(
            "No frames were successfully evaluated."
        )

    # --------------------------------------------------------
    # Aggregate
    # --------------------------------------------------------

    aggregate = aggregate_results(
        results
    )

    print_aggregate_summary(
        aggregate
    )

    # --------------------------------------------------------
    # Final
    # --------------------------------------------------------

    print(
        "\n" + "=" * 70
    )

    print(
        "ORBIT 10-FRAME EVALUATION COMPLETE"
    )

    print(
        "=" * 70
    )


if __name__ == "__main__":
    main()