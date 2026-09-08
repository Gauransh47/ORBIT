import numpy as np
from pathlib import Path
from collections import defaultdict, Counter

from src.perception.object_detection import (
    load_semantic_kitti_frame,
    detect_ground,
    connected_components,
    filter_components,
    merge_components,
    create_object_proposals,
    attach_point_indices,
)

from src.mapping.adaptive_grid import AdaptiveGrid

from src.perception.terrain_obstacle_detection import (
    extract_obstacle_cells,
    estimate_local_ground,
)


# ============================================================
# ORBIT - MISSED OBJECT ANALYSIS
#
# Purpose:
#   Fast, stage-aware diagnostics for GT "thing" objects.
#
# IMPORTANT:
#   The expensive ORBIT pipeline is built ONCE per frame.
#   GT analysis reuses the same frame pipeline.
#
# Added diagnostics:
#   - failure taxonomy
#   - distance-wise recall/failure analysis
#   - pipeline stage retention
#   - merged / low-purity proposal detection
#   - sparse-point diagnosis
#   - compact per-object diagnostics
#
# Matching is optimized using one point -> GT-owner array per frame
# instead of repeated np.intersect1d() calls for every pair.
# ============================================================


# ============================================================
# Configuration
# ============================================================

SEQUENCE = "00"

START_FRAME = 0
END_FRAME = 9

MAX_RANGE = 100.0

MINIMUM_GT_POINTS = 5

MINIMUM_COVERAGE = 0.20
MINIMUM_PURITY = 0.20

MINIMUM_OBSTACLE_HEIGHT = 0.15
MINIMUM_OBSTACLE_POINTS = 3

MINIMUM_COMPONENT_CELLS = 3
MINIMUM_COMPONENT_POINTS = 5

# Used only to flag possible ground over-rejection.
# This is a diagnostic flag, not proof of bad ground segmentation.
GROUND_POINT_FRACTION_FLAG = 0.20

DISTANCE_BUCKETS = (
    "0-10m",
    "10-25m",
    "25-50m",
    "50-100m",
)


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
        self.semantic_id = int(semantic_id)
        self.semantic_name = SEMANTIC_LABELS.get(
            self.semantic_id,
            f"unknown-{self.semantic_id}",
        )

        self.instance_id = int(instance_id)

        self.point_indices = np.asarray(
            point_indices,
            dtype=np.int64,
        )

        self.points = points[self.point_indices]
        self.point_count = len(self.point_indices)

        center = np.mean(self.points, axis=0)

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

        self.min_xyz = np.min(self.points, axis=0)
        self.max_xyz = np.max(self.points, axis=0)

        self.dimensions = (
            float(self.max_xyz[0] - self.min_xyz[0]),
            float(self.max_xyz[1] - self.min_xyz[1]),
            float(self.max_xyz[2] - self.min_xyz[2]),
        )


# ============================================================
# Labels
# ============================================================

def load_labels(path):

    raw = np.fromfile(
        path,
        dtype=np.uint32,
    )

    semantic = (
        raw & 0xFFFF
    ).astype(np.uint16)

    instance = (
        raw >> 16
    ).astype(np.uint16)

    return semantic, instance


# ============================================================
# Build GT objects
# ============================================================

def build_ground_truth_objects(
    points,
    semantic,
    instance,
):

    grouped = defaultdict(list)

    for index in range(len(points)):

        semantic_id = int(semantic[index])
        instance_id = int(instance[index])

        if semantic_id not in THING_CLASSES:
            continue

        if instance_id == 0:
            continue

        grouped[
            (semantic_id, instance_id)
        ].append(index)

    objects = []

    for (
        (semantic_id, instance_id),
        indices,
    ) in grouped.items():

        if len(indices) < MINIMUM_GT_POINTS:
            continue

        objects.append(
            GroundTruthObject(
                object_id=0,
                semantic_id=semantic_id,
                instance_id=instance_id,
                point_indices=np.asarray(
                    indices,
                    dtype=np.int64,
                ),
                points=points,
            )
        )

    objects.sort(
        key=lambda obj: obj.distance
    )

    for object_id, obj in enumerate(
        objects,
        start=1,
    ):
        obj.object_id = object_id

    return objects


# ============================================================
# Distance bucket
# ============================================================

def distance_bucket(distance):

    if distance < 10.0:
        return "0-10m"

    if distance < 25.0:
        return "10-25m"

    if distance < 50.0:
        return "25-50m"

    return "50-100m"


# ============================================================
# Point -> adaptive cell
# ============================================================

def point_cell_key(x, y):

    distance_sq = x * x + y * y

    if distance_sq < 100.0:
        level = 0

    elif distance_sq < 625.0:
        level = 1

    elif distance_sq < 2500.0:
        level = 2

    elif distance_sq < 10000.0:
        level = 3

    else:
        return None

    resolution = AdaptiveGrid.LEVELS[level]

    ix = int(np.floor(x / resolution))
    iy = int(np.floor(y / resolution))

    return (
        level,
        ix,
        iy,
    )


# ============================================================
# GT -> cell groups
# ============================================================

def get_target_cells(
    gt_object,
    points,
    range_mask,
):

    cells = defaultdict(list)

    object_indices = gt_object.point_indices

    in_range = object_indices[
        range_mask[object_indices]
    ]

    for index in in_range:

        x = points[index, 0]
        y = points[index, 1]

        key = point_cell_key(x, y)

        if key is not None:
            cells[key].append(int(index))

    return cells


# ============================================================
# Build ORBIT pipeline once per frame
# ============================================================

def build_orbit_frame(points):

    distances = np.linalg.norm(
        points[:, :2],
        axis=1,
    )

    range_mask = (
        distances <= MAX_RANGE
    )

    filtered_indices = np.flatnonzero(
        range_mask
    ).astype(
        np.int64,
        copy=False,
    )

    points_in_range = points[
        filtered_indices
    ]

    ground_in_range = detect_ground(
        points_in_range,
        distance_threshold=0.08,
        seed=42,
    )

    ground_mask = np.zeros(
        len(points),
        dtype=bool,
    )

    ground_mask[
        filtered_indices
    ] = ground_in_range

    grid = AdaptiveGrid()

    grid.build(
        points_in_range,
        ground_in_range,
    )

    obstacle_cells = extract_obstacle_cells(
        grid,
        minimum_height=MINIMUM_OBSTACLE_HEIGHT,
        minimum_obstacle_points=MINIMUM_OBSTACLE_POINTS,
    )

    raw_components = connected_components(
        obstacle_cells
    )

    filtered_components = filter_components(
        raw_components,
        minimum_cells=MINIMUM_COMPONENT_CELLS,
        minimum_points=MINIMUM_COMPONENT_POINTS,
    )

    merged_components = merge_components(
        filtered_components
    )

    proposals = create_object_proposals(
        merged_components
    )

    attach_point_indices(
        proposals,
        merged_components,
        points_in_range,
    )

    for proposal in proposals:

        local_indices = proposal.point_indices

        if len(local_indices):

            original_indices = (
                filtered_indices[
                    local_indices
                ]
            )

            proposal.point_indices = (
                original_indices
            )

            proposal.points = (
                points[original_indices]
            )

        else:

            proposal.point_indices = np.empty(
                0,
                dtype=np.int64,
            )

            proposal.points = np.empty(
                (0, 3),
                dtype=np.float64,
            )

    component_cell_lookup = {}

    for component_index, component in enumerate(
        merged_components
    ):

        for cell in component.cells:

            key = (
                cell.level,
                cell.ix,
                cell.iy,
            )

            component_cell_lookup[key] = (
                component_index
            )

    return {
        "range_mask": range_mask,
        "filtered_indices": filtered_indices,
        "ground_mask": ground_mask,
        "points_in_range": points_in_range,
        "grid": grid,
        "obstacle_cells": obstacle_cells,
        "raw_components": raw_components,
        "filtered_components": filtered_components,
        "merged_components": merged_components,
        "proposals": proposals,
        "component_cell_lookup": component_cell_lookup,
    }


# ============================================================
# Fast proposal/GT overlap table
# ============================================================

def build_gt_owner_map(
    num_points,
    gt_objects,
):

    owner = np.full(
        num_points,
        -1,
        dtype=np.int32,
    )

    for gt_index, gt_object in enumerate(
        gt_objects
    ):

        owner[
            gt_object.point_indices
        ] = gt_index

    return owner


def build_candidate_table(
    proposals,
    gt_objects,
    gt_owner,
):

    candidates_by_gt = defaultdict(list)

    for proposal_index, proposal in enumerate(
        proposals
    ):

        indices = proposal.point_indices

        if len(indices) == 0:
            continue

        owners = gt_owner[indices]

        valid_owners = owners[
            owners >= 0
        ]

        if len(valid_owners) == 0:
            continue

        overlap_counts = np.bincount(
            valid_owners,
            minlength=len(gt_objects),
        )

        proposal_size = len(indices)

        nonzero_gt = np.flatnonzero(
            overlap_counts
        )

        for gt_index in nonzero_gt:

            overlap = int(
                overlap_counts[gt_index]
            )

            gt_size = gt_objects[
                gt_index
            ].point_count

            coverage = (
                overlap / gt_size
            )

            purity = (
                overlap / proposal_size
            )

            union = (
                proposal_size
                + gt_size
                - overlap
            )

            iou = (
                overlap / union
                if union > 0
                else 0.0
            )

            candidates_by_gt[
                gt_index
            ].append({
                "proposal_index": proposal_index,
                "overlap": overlap,
                "coverage": coverage,
                "purity": purity,
                "iou": iou,
            })

    for gt_index in candidates_by_gt:

        candidates_by_gt[gt_index].sort(
            key=lambda item: (
                item["coverage"],
                item["iou"],
                item["purity"],
                item["overlap"],
            ),
            reverse=True,
        )

    return candidates_by_gt


# ============================================================
# One-to-one matching
# ============================================================

def match_one_to_one(
    candidates_by_gt,
    gt_objects,
):

    all_candidates = []

    for gt_index, candidates in (
        candidates_by_gt.items()
    ):

        for candidate in candidates:

            all_candidates.append({
                **candidate,
                "gt_index": gt_index,
            })

    all_candidates.sort(
        key=lambda item: (
            item["coverage"],
            item["iou"],
            item["purity"],
            item["overlap"],
        ),
        reverse=True,
    )

    matched_proposals = set()
    matched_gt = set()
    matches = []

    for candidate in all_candidates:

        proposal_index = (
            candidate["proposal_index"]
        )

        gt_index = (
            candidate["gt_index"]
        )

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

        matches.append(candidate)

    return matches, matched_gt


# ============================================================
# Diagnose one GT object
# ============================================================

def diagnose_gt_object(
    gt_object,
    points,
    pipeline,
):

    range_mask = pipeline["range_mask"]
    ground_mask = pipeline["ground_mask"]
    grid = pipeline["grid"]

    obstacle_cells = pipeline[
        "obstacle_cells"
    ]

    component_cell_lookup = pipeline[
        "component_cell_lookup"
    ]

    raw_components = pipeline[
        "raw_components"
    ]

    filtered_components = pipeline[
        "filtered_components"
    ]

    merged_components = pipeline[
        "merged_components"
    ]

    object_indices = gt_object.point_indices

    in_range_indices = object_indices[
        range_mask[object_indices]
    ]

    range_count = len(
        in_range_indices
    )

    if range_count == 0:

        return {
            "range_points": 0,
            "ground_points": 0,
            "non_ground_points": 0,
            "cell_count": 0,
            "terrain_reference": 0,
            "unresolved_reference": 0,
            "obstacle_cell_points": 0,
            "obstacle_eligible": 0,
            "too_few_points": 0,
            "height_failed": 0,
            "no_obstacle_elevation": 0,
            "final_component_points": 0,
            "best_component_coverage": 0.0,
            "raw_components": len(raw_components),
            "filtered_components": len(filtered_components),
            "merged_components": len(merged_components),
            "ground_fraction": 0.0,
        }

    ground_points = int(
        np.count_nonzero(
            ground_mask[
                in_range_indices
            ]
        )
    )

    non_ground_points = (
        range_count - ground_points
    )

    target_cells = get_target_cells(
        gt_object,
        points,
        range_mask,
    )

    same_cell_reference = 0
    local_reference = 0
    unresolved_reference = 0

    obstacle_eligible = 0
    too_few_points = 0
    height_failed = 0
    no_obstacle_elevation = 0

    for key, indices in target_cells.items():

        count = len(indices)

        cell = grid.cells.get(key)

        if cell is None:

            unresolved_reference += count
            continue

        if cell.ground_count > 0:

            ground_z = (
                cell.ground_elevation
            )

            same_cell_reference += count

        else:

            ground_z = estimate_local_ground(
                cell,
                grid,
            )

            if ground_z is None:

                unresolved_reference += count

            else:

                local_reference += count

        if ground_z is None:
            continue

        obstacle_z = (
            cell.obstacle_elevation
        )

        if obstacle_z is None:

            no_obstacle_elevation += count
            continue

        if (
            cell.obstacle_count
            < MINIMUM_OBSTACLE_POINTS
        ):

            too_few_points += count
            continue

        height = (
            obstacle_z - ground_z
        )

        if height < MINIMUM_OBSTACLE_HEIGHT:

            height_failed += count
            continue

        obstacle_eligible += count

    obstacle_cell_keys = {
        (
            cell.level,
            cell.ix,
            cell.iy,
        )
        for cell in obstacle_cells
    }

    obstacle_cell_points = 0

    for key, indices in target_cells.items():

        if key in obstacle_cell_keys:
            obstacle_cell_points += len(indices)

    component_points = defaultdict(int)

    for key, indices in target_cells.items():

        component_index = (
            component_cell_lookup.get(key)
        )

        if component_index is None:
            continue

        component_points[
            component_index
        ] += len(indices)

    if component_points:

        best_component_index = max(
            component_points,
            key=component_points.get,
        )

        best_component_points = (
            component_points[
                best_component_index
            ]
        )

    else:

        best_component_index = None
        best_component_points = 0

    coverage = (
        best_component_points
        / gt_object.point_count
        if gt_object.point_count > 0
        else 0.0
    )

    terrain_reference = (
        same_cell_reference
        + local_reference
    )

    return {
        "range_points": range_count,
        "ground_points": ground_points,
        "non_ground_points": non_ground_points,
        "cell_count": len(target_cells),
        "terrain_reference": terrain_reference,
        "unresolved_reference": unresolved_reference,
        "obstacle_cell_points": obstacle_cell_points,
        "obstacle_eligible": obstacle_eligible,
        "too_few_points": too_few_points,
        "height_failed": height_failed,
        "no_obstacle_elevation": no_obstacle_elevation,
        "final_component_points": best_component_points,
        "best_component_coverage": coverage,
        "best_component_index": best_component_index,
        "raw_components": len(raw_components),
        "filtered_components": len(filtered_components),
        "merged_components": len(merged_components),
        "ground_fraction": (
            ground_points / range_count
            if range_count > 0
            else 0.0
        ),
    }


# ============================================================
# Failure classification
# ============================================================

def classify_failure(
    diagnostic,
    best_candidate,
    accepted,
):

    if accepted:
        return "DETECTED"

    # A proposal captures >=20% of the GT object but is
    # insufficiently pure. This is a proposal-boundary /
    # merging problem, not a complete detection absence.
    if best_candidate is not None:

        if (
            best_candidate["coverage"]
            >= MINIMUM_COVERAGE
        ):

            if (
                best_candidate["purity"]
                < MINIMUM_PURITY
            ):
                return "MERGED_OR_LOW_PURITY"

            return "MATCHING_FAILURE"

    # No final component captured enough of the object.
    if (
        diagnostic[
            "best_component_coverage"
        ]
        > 0.0
        and diagnostic[
            "best_component_coverage"
        ]
        < MINIMUM_COVERAGE
    ):
        return "LOW_COMPONENT_COVERAGE"

    # Sparse point loss.
    if (
        diagnostic["obstacle_eligible"] == 0
        and diagnostic["too_few_points"] > 0
    ):

        return "SPARSE_OBSTACLE_POINTS"

    # Possible ground over-rejection flag.
    if (
        diagnostic["ground_fraction"]
        >= GROUND_POINT_FRACTION_FLAG
        and diagnostic["obstacle_cell_points"]
        == 0
    ):

        return "GROUND_OVERREJECTION_FLAG"

    if (
        diagnostic["unresolved_reference"]
        > 0
    ):

        return "NO_TERRAIN_REFERENCE"

    if (
        diagnostic["height_failed"] > 0
        and diagnostic["obstacle_cell_points"] == 0
    ):

        return "HEIGHT_THRESHOLD"

    if (
        diagnostic["no_obstacle_elevation"] > 0
        and diagnostic["obstacle_cell_points"] == 0
    ):

        return "NO_OBSTACLE_ELEVATION"

    return "NO_OBSTACLE_CELLS"


# ============================================================
# Compact object line
# ============================================================

def format_object_line(
    gt_object,
    diagnostic,
    best_candidate,
    reason,
):

    coverage = (
        best_candidate["coverage"]
        if best_candidate is not None
        else diagnostic["best_component_coverage"]
    )

    purity = (
        best_candidate["purity"]
        if best_candidate is not None
        else 0.0
    )

    return (
        f"{gt_object.object_id:2d} "
        f"{gt_object.semantic_name:<14} "
        f"{gt_object.distance:5.1f}m "
        f"pts={gt_object.point_count:4d} "
        f"obs={diagnostic['obstacle_cell_points']:4d} "
        f"cov={coverage * 100:5.1f}% "
        f"pur={purity * 100:5.1f}% "
        f"{reason}"
    )


# ============================================================
# Main
# ============================================================

def main():

    print("=" * 70)
    print("ORBIT - MISSED OBJECT ANALYSIS")
    print("=" * 70)

    base = (
        Path("data")
        / "semantic_kitti"
        / "sequences"
        / SEQUENCE
    )

    total_gt = 0
    total_detected = 0

    failure_counts = Counter()
    distance_failures = defaultdict(Counter)
    class_failures = defaultdict(Counter)

    frame_stats = []

    detected_coverages = []
    detected_ious = []
    detected_purities = []

    stage_totals = Counter()
    stage_object_counts = Counter()

    difficult_objects = []

    # ========================================================
    # Frames
    # ========================================================

    for frame_number in range(
        START_FRAME,
        END_FRAME + 1,
    ):

        frame_name = f"{frame_number:06d}"

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

        if (
            not velodyne_path.exists()
            or not label_path.exists()
        ):

            print(
                f"{frame_name} | SKIPPED"
            )
            continue

        # ----------------------------------------------------
        # Load
        # ----------------------------------------------------

        points = load_semantic_kitti_frame(
            velodyne_path
        )

        semantic, instance = load_labels(
            label_path
        )

        if len(points) != len(semantic):

            print(
                f"{frame_name} | SKIPPED: "
                "point/label mismatch"
            )
            continue

        # ----------------------------------------------------
        # Ground truth
        # ----------------------------------------------------

        gt_objects = (
            build_ground_truth_objects(
                points,
                semantic,
                instance,
            )
        )

        # ----------------------------------------------------
        # ORBIT pipeline exactly ONCE
        # ----------------------------------------------------

        pipeline = build_orbit_frame(
            points
        )

        proposals = pipeline["proposals"]

        # ----------------------------------------------------
        # Fast overlap preparation
        # ----------------------------------------------------

        gt_owner = build_gt_owner_map(
            len(points),
            gt_objects,
        )

        candidates_by_gt = (
            build_candidate_table(
                proposals,
                gt_objects,
                gt_owner,
            )
        )

        (
            matches,
            matched_gt,
        ) = match_one_to_one(
            candidates_by_gt,
            gt_objects,
        )

        match_by_gt = {
            item["gt_index"]: item
            for item in matches
        }

        frame_detected = 0
        frame_failures = Counter()

        # ----------------------------------------------------
        # Diagnose each GT object
        # ----------------------------------------------------

        for gt_index, gt_object in enumerate(
            gt_objects
        ):

            total_gt += 1

            diagnostic = diagnose_gt_object(
                gt_object,
                points,
                pipeline,
            )

            accepted = (
                gt_index in matched_gt
            )

            best_candidate = None

            gt_candidates = candidates_by_gt.get(
                gt_index,
                []
            )

            if gt_candidates:
                best_candidate = gt_candidates[0]

            reason = classify_failure(
                diagnostic,
                best_candidate,
                accepted,
            )

            bucket = distance_bucket(
                gt_object.distance
            )

            class_failures[
                gt_object.semantic_name
            ][reason] += 1

            if accepted:

                frame_detected += 1
                total_detected += 1

                match = match_by_gt[
                    gt_index
                ]

                detected_coverages.append(
                    match["coverage"]
                )

                detected_ious.append(
                    match["iou"]
                )

                detected_purities.append(
                    match["purity"]
                )

            else:

                failure_counts[reason] += 1
                distance_failures[
                    bucket
                ][reason] += 1

                difficult_objects.append({
                    "frame": frame_name,
                    "gt": gt_object,
                    "diagnostic": diagnostic,
                    "candidate": best_candidate,
                    "reason": reason,
                })

            frame_failures[reason] += 1

            # ------------------------------------------------
            # Aggregate stage retention
            # ------------------------------------------------

            stage_totals["gt_points"] += (
                gt_object.point_count
            )

            stage_totals["ground_points"] += (
                diagnostic["ground_points"]
            )

            stage_totals["non_ground_points"] += (
                diagnostic["non_ground_points"]
            )

            stage_totals["obstacle_cell_points"] += (
                diagnostic["obstacle_cell_points"]
            )

            stage_totals["final_component_points"] += (
                diagnostic["final_component_points"]
            )

        # ----------------------------------------------------
        # Frame stats
        # ----------------------------------------------------

        frame_gt = len(gt_objects)

        recall = (
            frame_detected / frame_gt
            if frame_gt
            else 0.0
        )

        frame_stats.append({
            "frame": frame_name,
            "gt": frame_gt,
            "detected": frame_detected,
            "proposals": len(proposals),
            "raw_components": len(
                pipeline["raw_components"]
            ),
            "filtered_components": len(
                pipeline["filtered_components"]
            ),
            "merged_components": len(
                pipeline["merged_components"]
            ),
            "recall": recall,
        })

        print(
            f"FRAME {frame_name} | "
            f"GT={frame_gt} | "
            f"Detected={frame_detected} | "
            f"Recall={recall * 100:.1f}% | "
            f"Props={len(proposals)} | "
            f"Components="
            f"{len(pipeline['merged_components'])}"
        )

    # ========================================================
    # Overall summary
    # ========================================================

    total_missed = (
        total_gt - total_detected
    )

    recall = (
        total_detected / total_gt
        if total_gt
        else 0.0
    )

    print("\n" + "=" * 70)
    print("ORBIT EVALUATION SUMMARY")
    print("=" * 70)

    print(
        f"GT objects : {total_gt}"
    )

    print(
        f"Detected   : {total_detected}"
    )

    print(
        f"Missed     : {total_missed}"
    )

    print(
        f"Recall     : {recall * 100:.1f}%"
    )

    # ========================================================
    # Failure taxonomy
    # ========================================================

    print("\n" + "-" * 70)
    print("FAILURE TAXONOMY")
    print("-" * 70)

    for reason, count in (
        failure_counts.most_common()
    ):

        percentage = (
            count / max(total_missed, 1)
        ) * 100.0

        print(
            f"{reason:<30} "
            f"{count:3d} "
            f"({percentage:5.1f}%)"
        )

    # ========================================================
    # Distance analysis
    # ========================================================

    print("\n" + "-" * 70)
    print("DISTANCE ANALYSIS")
    print("-" * 70)

    for bucket in DISTANCE_BUCKETS:

        # Reconstruct GT count from frame objects is expensive;
        # use failure counts + accepted detections below by frame.
        # For this compact report, show failure composition.
        failures = distance_failures.get(
            bucket,
            Counter(),
        )

        if not failures:

            print(
                f"{bucket:<10} no misses"
            )

            continue

        parts = [
            f"{reason}={count}"
            for reason, count in failures.most_common()
        ]

        print(
            f"{bucket:<10} "
            + " | ".join(parts)
        )

    # ========================================================
    # Class analysis
    # ========================================================

    print("\n" + "-" * 70)
    print("FAILURES BY CLASS")
    print("-" * 70)

    for class_name in sorted(
        class_failures
    ):

        counts = class_failures[
            class_name
        ]

        parts = [
            f"{reason}={count}"
            for reason, count in counts.most_common()
            if reason != "DETECTED"
        ]

        if parts:

            print(
                f"{class_name:<15} "
                + " | ".join(parts)
            )

    # ========================================================
    # Pipeline stage retention
    # ========================================================

    print("\n" + "-" * 70)
    print("GT-ASSOCIATED PIPELINE RETENTION")
    print("-" * 70)

    gt_points = stage_totals[
        "gt_points"
    ]

    stages = (
        ("GT points", gt_points),
        ("After ground", stage_totals["non_ground_points"]),
        ("Obstacle cells", stage_totals["obstacle_cell_points"]),
        ("Final component", stage_totals["final_component_points"]),
    )

    for name, count in stages:

        percentage = (
            count / gt_points * 100.0
            if gt_points
            else 0.0
        )

        print(
            f"{name:<18} "
            f"{count:8d} "
            f"({percentage:5.1f}%)"
        )

    # ========================================================
    # Detected quality
    # ========================================================

    if detected_coverages:

        print("\n" + "-" * 70)
        print("DETECTED OBJECT QUALITY")
        print("-" * 70)

        print(
            f"Coverage : "
            f"{np.mean(detected_coverages) * 100:.1f}%"
        )

        print(
            f"Point IoU: "
            f"{np.mean(detected_ious) * 100:.1f}%"
        )

        print(
            f"Purity   : "
            f"{np.mean(detected_purities) * 100:.1f}%"
        )

    # ========================================================
    # Hardest missed objects
    # ========================================================

    print("\n" + "-" * 70)
    print("HARDEST MISSED OBJECTS")
    print("-" * 70)

    # Most important misses first:
    # zero coverage, then lowest coverage.
    difficult_objects.sort(
        key=lambda item: (
            item["candidate"]["coverage"]
            if item["candidate"] is not None
            else item["diagnostic"][
                "best_component_coverage"
            ],
            item["gt"].distance,
        )
    )

    for item in difficult_objects[:15]:

        print(
            format_object_line(
                item["gt"],
                item["diagnostic"],
                item["candidate"],
                item["reason"],
            )
        )

    # ========================================================
    # Frame summary
    # ========================================================

    print("\n" + "-" * 70)
    print("FRAME SUMMARY")
    print("-" * 70)

    for result in frame_stats:

        print(
            f"{result['frame']} | "
            f"{result['detected']:2d}/"
            f"{result['gt']:2d} | "
            f"{result['recall'] * 100:5.1f}% | "
            f"props={result['proposals']:3d} | "
            f"merged={result['merged_components']:3d}"
        )

    print("\n" + "=" * 70)
    print("ANALYSIS COMPLETE")
    print("=" * 70)


if __name__ == "__main__":
    main()