"""
ORBIT Phase 1B — Component Lineage Diagnostic (v2)

Purpose
-------
For GT object pairs that Phase 1 (Hypothesis B) has already shown are
CONTESTED — i.e. two GT objects whose best qualifying proposal is the
SAME proposal — trace exactly which pipeline stage first merges their
underlying adaptive cells:

    obstacle cells
        v
    RAW connected components   <- cells_touch() / height gate in
                                   connected_components()
        v
    FILTERED components        <- filter_components() minimum-size gate
        v
    MERGED components          <- merge_components() / should_merge_components()
        v
    final proposals

Answering "at which stage do they first share a component" tells you
which specific function is responsible for the under-segmentation:

  - share a RAW component      -> connected_components()'s adjacency/
                                   height-gate is too permissive; this is
                                   NOT a merge-threshold problem at all.
  - separate through FILTERED,
    share at MERGED             -> merge_components() / should_merge_components()
                                   is doing the merging.
  - never share                 -> contention is happening some other way
                                   (e.g. footprint recovery reconstructing
                                   overlapping point sets from genuinely
                                   separate components) — inspect proposal
                                   construction instead.

This script is diagnostic-only. It does NOT modify ORBIT thresholds or
detector code.

Given the confirmed finding from phase1_diagnostics.py that GT #210 and
GT #504 contest a proposal in EVERY analyzed frame (11/11), this script
by default sweeps ALL available frames and auto-discovers contested
pairs per frame from the candidate table, rather than requiring a
manually maintained instance-ID list for one frame at a time.

Run:
    python -m src.evaluation.phase1_component_lineage

Optional:
    ORBIT_PHASE1_LINEAGE_FRAMES=2,3,5     comma-separated frame numbers
    ORBIT_PHASE1_LINEAGE_FRAME=5          single frame (overridden by FRAMES)
    ORBIT_PHASE1_LINEAGE_INSTANCES=210,504,215,217,218
                                           force specific instance IDs
                                           instead of auto-discovery
"""

from __future__ import annotations

import os
from pathlib import Path
from collections import defaultdict, Counter
from typing import Optional

import numpy as np

from src.mapping.adaptive_grid import AdaptiveGrid
from src.evaluation.analyze_missed_objects import (
    load_labels,
    build_ground_truth_objects,
    build_orbit_frame,
    build_gt_owner_map,
    build_candidate_table,
    match_one_to_one,
    diagnose_gt_object,
    point_cell_key,
    MINIMUM_COVERAGE,
    MINIMUM_PURITY,
)

SEQUENCE = "00"
DEFAULT_FRAMES = tuple(range(10))


# ============================================================
# Config helpers
# ============================================================

def env_int_list(name: str) -> Optional[list[int]]:
    value = os.environ.get(name)
    if not value or not value.strip():
        return None
    out = []
    for item in value.split(","):
        item = item.strip()
        if item:
            try:
                out.append(int(item))
            except ValueError:
                pass
    return out or None


def selected_frames() -> list[int]:
    frames = env_int_list("ORBIT_PHASE1_LINEAGE_FRAMES")
    if frames:
        return frames

    single = os.environ.get("ORBIT_PHASE1_LINEAGE_FRAME")
    if single and single.strip():
        try:
            return [int(single)]
        except ValueError:
            pass

    return list(DEFAULT_FRAMES)


def forced_instance_ids() -> Optional[list[int]]:
    return env_int_list("ORBIT_PHASE1_LINEAGE_INSTANCES")


def load_points(path: Path) -> np.ndarray:
    raw = np.fromfile(path, dtype=np.float32)
    scan = raw.reshape(-1, 4)
    return scan[:, :3].astype(np.float64, copy=False)


def cell_key(cell) -> tuple[int, int, int]:
    return (int(cell.level), int(cell.ix), int(cell.iy))


# ============================================================
# Vectorized per-frame point -> GT-cell grouping
#
# Replaces target_cells()'s per-point Python loop, and computes it
# ONCE per GT object instead of once per print_stage() call.
# ============================================================

def build_target_cells_for_all_gt(
    gt_objects: list,
    points: np.ndarray,
    range_mask: np.ndarray,
) -> dict[int, dict[tuple, list[int]]]:
    """
    Returns {gt_index: {(level, ix, iy): [point_index, ...]}}
    computed once per GT object per frame.
    """
    result = {}

    for gt_index, gt in enumerate(gt_objects):
        indices = gt.point_indices
        indices = indices[range_mask[indices]]

        grouped: dict[tuple, list[int]] = defaultdict(list)
        for idx in indices:
            key = point_cell_key(points[idx, 0], points[idx, 1])
            if key is not None:
                grouped[key].append(int(idx))

        result[gt_index] = grouped

    return result


# ============================================================
# Contested-pair discovery (mirrors Hypothesis B in
# phase1_diagnostics.py, kept local to avoid cross-module coupling)
# ============================================================

def find_contested_pairs(candidates_by_gt: dict, gt_objects: list) -> list[tuple[int, int, int]]:
    """
    Returns [(proposal_index, gt_index_a, gt_index_b), ...] for every
    proposal that is a QUALIFYING candidate (meets coverage/purity
    thresholds) for more than one GT object. Only returns pairs, even
    if more than two GT objects contest a single proposal.
    """
    proposal_to_gts = defaultdict(list)

    for gt_index, candidates in candidates_by_gt.items():
        for c in candidates:
            if c["coverage"] >= MINIMUM_COVERAGE and c["purity"] >= MINIMUM_PURITY:
                proposal_to_gts[c["proposal_index"]].append(gt_index)

    pairs = []
    for proposal_index, gt_indices in proposal_to_gts.items():
        if len(gt_indices) < 2:
            continue
        gt_indices = sorted(gt_indices)
        for i in range(len(gt_indices)):
            for j in range(i + 1, len(gt_indices)):
                pairs.append((proposal_index, gt_indices[i], gt_indices[j]))

    return pairs


# ============================================================
# Per-stage component overlap
# ============================================================

def component_overlap_ids(
    components: list,
    target_keys: set,
) -> set[int]:
    """Indices of components (within `components`) that touch any of target_keys."""
    ids = set()
    for index, component in enumerate(components):
        for cell in component.cells:
            if cell_key(cell) in target_keys:
                ids.add(index)
                break
    return ids


def first_shared_stage(
    pipeline: dict,
    target_keys_a: set,
    target_keys_b: set,
) -> tuple[str, Optional[int]]:
    """
    Returns (stage_name, component_index) for the earliest stage at
    which the two GT objects' cell footprints land in the SAME
    component. stage_name is one of "RAW", "FILTERED", "MERGED",
    or "NEVER".
    """
    for stage_name, key in (
        ("RAW", "raw_components"),
        ("FILTERED", "filtered_components"),
        ("MERGED", "merged_components"),
    ):
        components = pipeline[key]
        ids_a = component_overlap_ids(components, target_keys_a)
        ids_b = component_overlap_ids(components, target_keys_b)
        shared = ids_a & ids_b

        if shared:
            return stage_name, next(iter(shared))

    return "NEVER", None


# ============================================================
# Reporting for one contested pair, one frame
# ============================================================

def report_pair(
    frame_name: str,
    gt_a,
    gt_b,
    target_keys_a: set,
    target_keys_b: set,
    target_points_a: dict,
    target_points_b: dict,
    obstacle_count_by_key: dict,
    pipeline: dict,
) -> str:
    print(f"\nGT #{gt_a.instance_id} <-> GT #{gt_b.instance_id}  "
          f"(dist {gt_a.distance:.1f}m / {gt_b.distance:.1f}m)")

    for name, keys, points_map in (
        ("GT " + str(gt_a.instance_id), target_keys_a, target_points_a),
        ("GT " + str(gt_b.instance_id), target_keys_b, target_points_b),
    ):
        # "real GT points" — actual point membership of this GT object,
        # NOT to be confused with a cell's total obstacle_count below,
        # which may include points belonging to OTHER objects sharing
        # the same cell footprint.
        real_gt_points_in_footprint = sum(len(v) for v in points_map.values())
        cell_obstacle_points_total = sum(
            obstacle_count_by_key.get(k, 0) for k in keys
        )
        print(
            f"  {name:<10} footprint_cells={len(keys):4d}  "
            f"real_GT_points={real_gt_points_in_footprint:5d}  "
            f"cell_obstacle_points_total={cell_obstacle_points_total:5d}"
        )

    stage, component_index = first_shared_stage(pipeline, target_keys_a, target_keys_b)

    if stage == "NEVER":
        print("  -> Never share a component through MERGED. Contention is not "
              "explained by component-level under-segmentation; inspect proposal "
              "point-recovery / footprint reconstruction instead.")
    else:
        print(f"  -> FIRST SHARED at stage: {stage} (component index {component_index})")

        if stage == "RAW":
            print("     Root cause candidate: connected_components() adjacency/height "
                  "gate (cells_touch / obstacle_height tolerance) — NOT merge_components().")
        elif stage == "FILTERED":
            print("     Root cause candidate: filtering changed nothing about separation; "
                  "still merged at RAW, filtering only removed size. Same as RAW case.")
        else:  # MERGED
            print("     Root cause candidate: merge_components() / should_merge_components() "
                  "— gap/height/dimension thresholds are joining two real objects.")

    return stage


# ============================================================
# Main
# ============================================================

def main():
    root = Path("data") / "semantic_kitti" / "sequences" / SEQUENCE
    frames = selected_frames()
    forced_ids = forced_instance_ids()

    print("=" * 78)
    print("ORBIT - PHASE 1B COMPONENT LINEAGE DIAGNOSTIC (v2)")
    print("=" * 78)
    print(f"Frames: {', '.join(f'{f:06d}' for f in frames)}")
    if forced_ids:
        print(f"Forced instance IDs (skipping auto-discovery): {forced_ids}")
    else:
        print("Contested pairs auto-discovered per frame from the candidate table.")

    stage_tally: Counter = Counter()
    pair_stage_by_instance: dict[tuple[int, int], list[str]] = defaultdict(list)

    for frame_number in frames:
        frame_name = f"{frame_number:06d}"
        velodyne_path = root / "velodyne" / f"{frame_name}.bin"
        label_path = root / "labels" / f"{frame_name}.label"

        if not velodyne_path.exists() or not label_path.exists():
            print(f"\n{frame_name} | SKIPPED - missing file")
            continue

        points = load_points(velodyne_path)
        semantic, instance = load_labels(label_path)

        if len(points) != len(semantic):
            print(f"\n{frame_name} | SKIPPED - point/label mismatch")
            continue

        gt_objects = build_ground_truth_objects(points, semantic, instance)
        if not gt_objects:
            print(f"\n{frame_name} | SKIPPED - no GT objects")
            continue

        pipeline = build_orbit_frame(points)
        range_mask = pipeline["range_mask"]

        gt_owner = build_gt_owner_map(len(points), gt_objects)
        candidates_by_gt = build_candidate_table(pipeline["proposals"], gt_objects, gt_owner)
        matches, matched_gt = match_one_to_one(candidates_by_gt, gt_objects)

        # Build obstacle_count-by-cell ONCE per frame (fixes the
        # per-stage-per-target rebuild in the original script).
        obstacle_count_by_key = {
            cell_key(c): int(c.obstacle_count) for c in pipeline["obstacle_cells"]
        }

        # Build GT->cell grouping ONCE per GT object per frame (fixes
        # the 4x-per-target recomputation in the original script).
        target_cells_by_gt = build_target_cells_for_all_gt(gt_objects, points, range_mask)

        gt_by_instance = {int(gt.instance_id): i for i, gt in enumerate(gt_objects)}

        if forced_ids:
            pairs = []
            forced_indices = [gt_by_instance[i] for i in forced_ids if i in gt_by_instance]
            for i in range(len(forced_indices)):
                for j in range(i + 1, len(forced_indices)):
                    pairs.append((None, forced_indices[i], forced_indices[j]))
        else:
            pairs = find_contested_pairs(candidates_by_gt, gt_objects)

        print(f"\n{'-' * 78}\nFRAME {frame_name}\n{'-' * 78}")
        print(f"GT objects: {len(gt_objects)}  Matched: {len(matched_gt)}  "
              f"Contested pairs: {len(pairs)}")

        if not pairs:
            continue

        for proposal_index, gt_index_a, gt_index_b in pairs:
            gt_a = gt_objects[gt_index_a]
            gt_b = gt_objects[gt_index_b]

            keys_a = set(target_cells_by_gt[gt_index_a])
            keys_b = set(target_cells_by_gt[gt_index_b])

            stage = report_pair(
                frame_name,
                gt_a,
                gt_b,
                keys_a,
                keys_b,
                target_cells_by_gt[gt_index_a],
                target_cells_by_gt[gt_index_b],
                obstacle_count_by_key,
                pipeline,
            )

            stage_tally[stage] += 1
            pair_key = (int(gt_a.instance_id), int(gt_b.instance_id))
            pair_stage_by_instance[pair_key].append(stage)

    # ========================================================
    # Aggregate verdict
    # ========================================================

    print("\n" + "=" * 78)
    print("PHASE 1B - AGGREGATE VERDICT ACROSS ALL ANALYZED FRAMES")
    print("=" * 78)

    if not stage_tally:
        print("No contested pairs found in the analyzed frames.")
    else:
        total = sum(stage_tally.values())
        for stage, count in stage_tally.most_common():
            print(f"  {stage:<10} {count:3d} ({count / total * 100:5.1f}%)")

        print("\nPer recurring instance-ID pair:")
        for (id_a, id_b), stages in sorted(pair_stage_by_instance.items()):
            counts = Counter(stages)
            summary = ", ".join(f"{s}={c}" for s, c in counts.most_common())
            print(f"  GT {id_a} <-> GT {id_b}: {summary} (n={len(stages)} frames)")

        dominant_stage, dominant_count = stage_tally.most_common(1)[0]
        if dominant_count / total > 0.7:
            if dominant_stage == "RAW":
                print("\nCONCLUSION: contention is overwhelmingly established at the RAW "
                      "connected-component stage. merge_components() is not the cause — "
                      "investigate cells_touch()/height-gate tolerance in "
                      "connected_components() instead.")
            elif dominant_stage == "MERGED":
                print("\nCONCLUSION: contention is overwhelmingly introduced during "
                      "merge_components(). Investigate should_merge_components() gap/"
                      "height/dimension thresholds for this distance band.")
            elif dominant_stage == "NEVER":
                print("\nCONCLUSION: contention is NOT explained by component-level "
                      "under-segmentation. Investigate proposal point-recovery / footprint "
                      "reconstruction, since components stay separate through MERGED.")
        else:
            print("\nCONCLUSION: stage is inconsistent across frames/pairs — investigate "
                  "per-pair rather than assuming one root cause.")

    print("\n" + "=" * 78)
    print("PHASE 1B DIAGNOSTICS COMPLETE")
    print("=" * 78)


if __name__ == "__main__":
    main()