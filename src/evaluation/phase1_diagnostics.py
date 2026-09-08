"""
ORBIT Phase 1 Diagnostics (v2)

Purpose
-------
Controlled, read-only experiments for three structural hypotheses:

A) FOOTPRINT CONTAMINATION
   Footprint-based proposal recovery is XY-only and pulls in ground /
   unrelated points, which is expected to explain high-coverage /
   low-purity proposals (production reason: MERGED_OR_LOW_PURITY, and
   any matched-but-impure object).

B) MATCHING CONTENTION
   MATCHING_FAILURE cases (a qualifying candidate exists but the GT
   object is still unmatched) are caused by another GT object winning
   the same proposal in the greedy one-to-one matcher.

C) COMPONENT-FILTERING LOSS
   Some objects labeled NO_OBSTACLE_CELLS / LOW_COMPONENT_COVERAGE by
   the production analyzer actually have real obstacle-cell evidence
   that gets killed by `filter_components`'s minimum-cell/point
   thresholds, or lost during merge-cell association. This script also
   applies a one-line relabel fix for that known mislabeling (see
   `classify_failure_fixed` below) without touching the production
   analyzer file.

This script:
  - imports the REAL pipeline builder and failure classifier from
    analyze_missed_objects.py, so results map 1:1 onto your existing
    failure taxonomy instead of an ad hoc reselection of "interesting"
    objects.
  - does NOT modify the ORBIT detector or any thresholds.
  - runs once across the full available sequence (same cost as your
    existing analyzer run) but only prints detailed diagnostics for a
    capped number of objects per hypothesis, plus an aggregate verdict.

Run from the ORBIT project root:

    python -m src.evaluation.phase1_diagnostics

Optional environment variables:
    ORBIT_PHASE1_FRAMES=2,3,7        (comma-separated frame numbers)
    ORBIT_PHASE1_FRAME=2             (single frame, overridden by FRAMES)
    ORBIT_PHASE1_MAX_PER_HYPOTHESIS=15
    ORBIT_PHASE1_JSON_OUT=phase1_results.json
"""

from __future__ import annotations

import json
import os
from collections import defaultdict, Counter
from pathlib import Path
from typing import Optional

import numpy as np

from src.mapping.adaptive_grid import AdaptiveGrid

# Reuse the REAL pipeline + failure taxonomy. This guarantees phase 1
# results are computed on exactly the same pipeline and exactly the
# same failure labels as your production analyzer, instead of a
# parallel re-implementation that can silently drift.
from src.evaluation.analyze_missed_objects import (
    load_labels,
    build_ground_truth_objects,
    build_orbit_frame,
    build_gt_owner_map,
    build_candidate_table,
    match_one_to_one,
    diagnose_gt_object,
    classify_failure,
    distance_bucket,
    point_cell_key,
    MINIMUM_COVERAGE,
    MINIMUM_PURITY,
    MINIMUM_GT_POINTS,
)


SEQUENCE = "00"
DEFAULT_FRAMES = tuple(range(10))  # full available sequence: 000000-000009

Z_BAND_EPSILON = 0.05

RECOVERY_MODES = ("baseline_xy", "non_ground", "z_band")


# ============================================================
# Config helpers
# ============================================================

def env_int(name: str, default: Optional[int]) -> Optional[int]:
    value = os.environ.get(name)
    if not value or not value.strip():
        return default
    try:
        return int(value)
    except ValueError:
        return default


def selected_frames() -> list[int]:
    requested = os.environ.get("ORBIT_PHASE1_FRAMES")
    if requested:
        out = []
        for item in requested.split(","):
            item = item.strip()
            if item:
                try:
                    out.append(int(item))
                except ValueError:
                    pass
        if out:
            return out

    single = env_int("ORBIT_PHASE1_FRAME", None)
    if single is not None:
        return [single]

    return list(DEFAULT_FRAMES)


# ============================================================
# Known analyzer bug fix (relabel only, no threshold changes)
# ============================================================

def classify_failure_fixed(diagnostic, best_candidate, accepted):
    """
    Wraps the production classify_failure() and relabels one known
    mislabeling: objects where real obstacle-cell evidence exists
    (obstacle_cell_points > 0) but no final component captured any of
    it (final_component_points == 0) currently fall through to the
    generic NO_OBSTACLE_CELLS bucket. That conflates "no evidence was
    ever found" with "evidence was found and then filtered/lost" —
    two different root causes needing two different fixes.

    This does not change any threshold or detector behavior. It only
    corrects the label attached to an already-computed diagnostic.
    """
    reason = classify_failure(diagnostic, best_candidate, accepted)

    if (
        reason == "NO_OBSTACLE_CELLS"
        and diagnostic["obstacle_cell_points"] > 0
        and diagnostic["final_component_points"] == 0
    ):
        return "COMPONENT_FILTERED_OUT"

    return reason


# ============================================================
# Fast per-frame point -> adaptive-cell assignment
#
# Replaces the O(cells x N) "boolean mask over the whole point cloud,
# per cell, per candidate, per mode" approach from the original script
# with a single O(N) assignment pass, reused for every hypothesis.
# ============================================================

def assign_points_to_cells(points_in_range: np.ndarray) -> dict:
    """
    Returns {(level, ix, iy): local_index_array} where local_index_array
    indexes into `points_in_range` (NOT the original frame array).
    Uses the same distance-band / resolution logic as point_cell_key().
    """
    if len(points_in_range) == 0:
        return {}

    x = points_in_range[:, 0]
    y = points_in_range[:, 1]
    dist_sq = x * x + y * y

    band_edges = (100.0, 625.0, 2500.0, 10000.0)  # matches point_cell_key
    cell_to_indices: dict = defaultdict(list)

    lower = 0.0
    for level, upper in enumerate(band_edges):
        band_mask = (dist_sq >= lower) & (dist_sq < upper)
        lower = upper

        if not np.any(band_mask):
            continue

        resolution = AdaptiveGrid.LEVELS[level]
        idx = np.flatnonzero(band_mask)
        ix = np.floor(x[idx] / resolution).astype(np.int64)
        iy = np.floor(y[idx] / resolution).astype(np.int64)

        for local_i, cell_ix, cell_iy in zip(idx.tolist(), ix.tolist(), iy.tolist()):
            cell_to_indices[(level, cell_ix, cell_iy)].append(local_i)

    return {key: np.asarray(v, dtype=np.int64) for key, v in cell_to_indices.items()}


def cell_key(cell) -> tuple:
    return (int(cell.level), int(cell.ix), int(cell.iy))


# ============================================================
# Hypothesis A — footprint contamination
# ============================================================

def recover_variant_point_indices(
    proposal,
    points: np.ndarray,
    ground_mask: np.ndarray,
    filtered_indices: np.ndarray,
    cell_to_local_indices: dict,
    mode: str,
) -> np.ndarray:
    """
    Rebuild a proposal's point membership using the SAME cell footprints
    ORBIT already computed, but with an added constraint per `mode`:

      baseline_xy  current behavior (XY footprint only, no z/ground test)
      non_ground   drop points ORBIT's own RANSAC ground mask flags as ground
      z_band       keep only points whose z falls within
                   [ground_elevation, obstacle_elevation] of the owning cell
    """
    if mode not in RECOVERY_MODES:
        raise ValueError(f"Unknown recovery mode: {mode}")

    selected: list[int] = []

    for cell in proposal.cells:
        local_idx = cell_to_local_indices.get(cell_key(cell))
        if local_idx is None or len(local_idx) == 0:
            continue

        global_idx = filtered_indices[local_idx]

        if mode == "baseline_xy":
            keep = np.ones(len(local_idx), dtype=bool)

        elif mode == "non_ground":
            keep = ~ground_mask[global_idx]

        else:  # z_band
            ground_z = cell.ground_elevation
            obstacle_z = cell.obstacle_elevation

            if ground_z is None or obstacle_z is None:
                continue

            z = points[global_idx, 2]
            low = float(ground_z) - Z_BAND_EPSILON
            high = float(obstacle_z) + Z_BAND_EPSILON
            keep = (z >= low) & (z <= high)

        if np.any(keep):
            selected.extend(global_idx[keep].tolist())

    if not selected:
        return np.empty(0, dtype=np.int64)

    return np.unique(np.asarray(selected, dtype=np.int64))


def proposal_metrics_for_gt(proposal_indices: np.ndarray, gt_object) -> dict:
    if len(proposal_indices) == 0:
        return {"overlap": 0, "proposal_size": 0, "coverage": 0.0, "purity": 0.0, "iou": 0.0}

    gt_set = set(gt_object.point_indices.tolist())
    prop_set = set(proposal_indices.tolist())

    overlap = len(gt_set & prop_set)
    proposal_size = len(prop_set)
    gt_size = gt_object.point_count

    coverage = overlap / gt_size if gt_size else 0.0
    purity = overlap / proposal_size if proposal_size else 0.0
    union = proposal_size + gt_size - overlap
    iou = overlap / union if union else 0.0

    return {
        "overlap": overlap,
        "proposal_size": proposal_size,
        "coverage": coverage,
        "purity": purity,
        "iou": iou,
    }


def run_hypothesis_a(
    frame_name: str,
    pool: list[dict],
    pipeline: dict,
    points: np.ndarray,
    cell_to_local_indices: dict,
    max_objects: int,
    aggregate: dict,
) -> None:
    print(f"\n{'=' * 78}\nHYPOTHESIS A — FOOTPRINT CONTAMINATION | FRAME {frame_name}\n{'=' * 78}")

    filtered_indices = pipeline["filtered_indices"]
    ground_mask = pipeline["ground_mask"]
    proposals = pipeline["proposals"]

    # Worst purity first: the objects most likely to demonstrate the effect.
    pool = sorted(pool, key=lambda item: item["candidate"]["purity"])[:max_objects]

    if not pool:
        print("No candidate objects with coverage/purity worth testing on this frame.")
        return

    for item in pool:
        gt = item["gt"]
        candidate = item["candidate"]
        proposal = proposals[candidate["proposal_index"]]

        results = {
            mode: recover_variant_point_indices(
                proposal, points, ground_mask, filtered_indices, cell_to_local_indices, mode
            )
            for mode in RECOVERY_MODES
        }
        metrics = {mode: proposal_metrics_for_gt(idx, gt) for mode, idx in results.items()}

        m0, m1, m2 = metrics["baseline_xy"], metrics["non_ground"], metrics["z_band"]

        print(
            f"\nGT #{gt.instance_id}  dist={gt.distance:.1f}m  GTpts={gt.point_count}  "
            f"reason={item['reason']}"
        )
        print(f"  CURRENT    pts={m0['proposal_size']:4d} cov={m0['coverage']*100:6.1f}% "
              f"pur={m0['purity']*100:6.1f}% IoU={m0['iou']*100:6.1f}%")
        print(f"  NON-GROUND pts={m1['proposal_size']:4d} cov={m1['coverage']*100:6.1f}% "
              f"pur={m1['purity']*100:6.1f}% IoU={m1['iou']*100:6.1f}%")
        print(f"  Z-BAND     pts={m2['proposal_size']:4d} cov={m2['coverage']*100:6.1f}% "
              f"pur={m2['purity']*100:6.1f}% IoU={m2['iou']*100:6.1f}%")

        purity_gain = (m1["purity"] - m0["purity"]) * 100.0
        iou_gain = (m1["iou"] - m0["iou"]) * 100.0
        print(f"  Δ NON-GROUND: purity {purity_gain:+.1f}pp, IoU {iou_gain:+.1f}pp")

        coverage_retained = m1["coverage"] >= max(MINIMUM_COVERAGE, m0["coverage"] - 0.10)

        if purity_gain > 10.0 and coverage_retained:
            verdict = "STRONG SUPPORT"
        elif purity_gain > 5.0:
            verdict = "PARTIAL SUPPORT"
        else:
            verdict = "WEAK / NO SUPPORT"

        print(f"  VERDICT: {verdict}")

        aggregate["tested"] += 1
        aggregate["purity_gains"].append(purity_gain)
        aggregate["iou_gains"].append(iou_gain)
        aggregate["verdicts"][verdict] += 1


# ============================================================
# Hypothesis B — matching contention
# ============================================================

def build_contested_proposals(candidates_by_gt: dict) -> dict:
    proposal_to_gts = defaultdict(list)

    for gt_index, candidates in candidates_by_gt.items():
        for c in candidates:
            if c["coverage"] >= MINIMUM_COVERAGE and c["purity"] >= MINIMUM_PURITY:
                proposal_to_gts[c["proposal_index"]].append(
                    (gt_index, c["coverage"], c["purity"], c["iou"])
                )

    return {p: v for p, v in proposal_to_gts.items() if len(v) > 1}


def run_hypothesis_b(
    frame_name: str,
    matching_failure_pool: list[dict],
    candidates_by_gt: dict,
    matched_gt: set,
    gt_objects: list,
    aggregate: dict,
) -> None:
    print(f"\n{'=' * 78}\nHYPOTHESIS B — MATCHING CONTENTION | FRAME {frame_name}\n{'=' * 78}")

    contested = build_contested_proposals(candidates_by_gt)

    if not matching_failure_pool:
        print("No MATCHING_FAILURE objects on this frame.")
        return

    if not contested:
        print(f"{len(matching_failure_pool)} MATCHING_FAILURE object(s) present, "
              "but no qualifying proposal is contested by multiple GT objects.")
        aggregate["matching_failure_total"] += len(matching_failure_pool)
        return

    explained = 0

    for item in matching_failure_pool:
        gt = item["gt"]
        candidate = item["candidate"]
        owners = contested.get(candidate["proposal_index"], [])

        aggregate["matching_failure_total"] += 1

        if len(owners) > 1:
            explained += 1
            aggregate["matching_failure_explained"] += 1
            competitor_ids = [
                gt_objects[i].instance_id for i, *_ in owners if i != item["gt_index"]
            ]
            print(
                f"GT #{gt.instance_id} dist={gt.distance:.1f}m — proposal "
                f"#{candidate['proposal_index']} also claimed by GT "
                f"{competitor_ids} -> CONTENTION CONFIRMED"
            )
        else:
            print(
                f"GT #{gt.instance_id} dist={gt.distance:.1f}m — its qualifying "
                f"proposal is NOT contested -> contention does not explain this case"
            )

    print(f"\n{explained}/{len(matching_failure_pool)} MATCHING_FAILURE cases on this "
          f"frame explained by proposal contention.")


# ============================================================
# Hypothesis C — component-filtering loss
# ============================================================

def target_cells_for_gt(gt_object, points, range_mask):
    grouped = defaultdict(list)
    indices = gt_object.point_indices
    indices = indices[range_mask[indices]]

    for idx in indices:
        key = point_cell_key(points[idx, 0], points[idx, 1])
        if key is not None:
            grouped[key].append(int(idx))

    return grouped


def classify_component_filtering(gt_object, points, pipeline) -> dict:
    """
    Localizes exactly where a GT object's obstacle evidence was lost
    relative to the component pipeline stages: never reached obstacle
    cells, reached obstacle cells but no raw component ever formed,
    a raw component formed but filter_components() removed it, or it
    survived filtering but didn't make it into a merged component.
    """
    range_mask = pipeline["range_mask"]
    obstacle_cells = pipeline["obstacle_cells"]
    raw_components = pipeline["raw_components"]
    filtered_components = pipeline["filtered_components"]
    merged_components = pipeline["merged_components"]

    target = target_cells_for_gt(gt_object, points, range_mask)
    gt_keys = set(target)
    obstacle_keys = {cell_key(c) for c in obstacle_cells}
    obstacle_keys_for_gt = gt_keys & obstacle_keys

    def keys_in(components):
        keys = set()
        for component in components:
            for cell in component.cells:
                k = cell_key(cell)
                if k in gt_keys:
                    keys.add(k)
        return keys

    raw_keys = keys_in(raw_components)
    filtered_keys = keys_in(filtered_components)
    merged_keys = keys_in(merged_components)

    obstacle_points = sum(len(target[k]) for k in obstacle_keys_for_gt)

    if obstacle_points == 0:
        classification = "NO_OBSTACLE_EVIDENCE"
    elif not raw_keys:
        classification = "OBSTACLE_CELLS_BUT_NO_RAW_COMPONENT"
    elif not filtered_keys:
        classification = "COMPONENT_FILTERED_OUT"
    elif not merged_keys:
        classification = "FILTERED_COMPONENT_LOST_BEFORE_MERGE"
    else:
        classification = "EVIDENCE_REACHED_COMPONENT_STAGE"

    return {
        "classification": classification,
        "gt_cells": len(gt_keys),
        "obstacle_cells": len(obstacle_keys_for_gt),
        "obstacle_points": obstacle_points,
        "raw_cells": len(raw_keys),
        "filtered_cells": len(filtered_keys),
        "merged_cells": len(merged_keys),
    }


def run_hypothesis_c(
    frame_name: str,
    pool: list[dict],
    points: np.ndarray,
    pipeline: dict,
    max_objects: int,
    aggregate: Counter,
) -> None:
    print(f"\n{'=' * 78}\nHYPOTHESIS C — COMPONENT FILTERING | FRAME {frame_name}\n{'=' * 78}")

    if not pool:
        print("No NO_OBSTACLE_CELLS / COMPONENT_FILTERED_OUT / LOW_COMPONENT_COVERAGE "
              "objects on this frame.")
        return

    for item in pool[:max_objects]:
        gt = item["gt"]
        result = classify_component_filtering(gt, points, pipeline)

        print(f"\nGT #{gt.instance_id}  dist={gt.distance:.1f}m  GTpts={gt.point_count}  "
              f"production_reason={item['reason']}")
        print(f"  obstacle cells : {result['obstacle_cells']}  "
              f"obstacle pts: {result['obstacle_points']}")
        print(f"  raw cells      : {result['raw_cells']}")
        print(f"  filtered cells : {result['filtered_cells']}")
        print(f"  merged cells   : {result['merged_cells']}")
        print(f"  VERDICT        : {result['classification']}")

        aggregate[result["classification"]] += 1


# ============================================================
# Main
# ============================================================

def main():
    root = Path("data") / "semantic_kitti" / "sequences" / SEQUENCE
    frames = selected_frames()
    max_per_hypothesis = env_int("ORBIT_PHASE1_MAX_PER_HYPOTHESIS", 15) or 15
    json_out = os.environ.get("ORBIT_PHASE1_JSON_OUT")

    print("=" * 78)
    print("ORBIT — PHASE 1 CONTROLLED DIAGNOSTICS (v2)")
    print("=" * 78)
    print("No detector thresholds or pipeline code are being modified.")
    print(f"Frames: {', '.join(f'{f:06d}' for f in frames)}")

    agg_a = {"tested": 0, "purity_gains": [], "iou_gains": [], "verdicts": Counter()}
    agg_b = {"matching_failure_total": 0, "matching_failure_explained": 0}
    agg_c: Counter = Counter()

    for frame_number in frames:
        frame_name = f"{frame_number:06d}"
        velodyne_path = root / "velodyne" / f"{frame_name}.bin"
        label_path = root / "labels" / f"{frame_name}.label"

        if not velodyne_path.exists() or not label_path.exists():
            print(f"\n{frame_name} | SKIPPED — missing file")
            continue

        points = _load_points(velodyne_path)
        semantic, instance = load_labels(label_path)

        if len(points) != len(semantic):
            print(f"\n{frame_name} | SKIPPED — point/label mismatch")
            continue

        gt_objects = build_ground_truth_objects(points, semantic, instance)
        if not gt_objects:
            print(f"\n{frame_name} | SKIPPED — no GT objects")
            continue

        pipeline = build_orbit_frame(points)
        cell_to_local_indices = assign_points_to_cells(pipeline["points_in_range"])

        gt_owner = build_gt_owner_map(len(points), gt_objects)
        candidates_by_gt = build_candidate_table(pipeline["proposals"], gt_objects, gt_owner)
        matches, matched_gt = match_one_to_one(candidates_by_gt, gt_objects)

        print(f"\n{'-' * 78}\nFRAME {frame_name}\n{'-' * 78}")
        print(f"GT objects: {len(gt_objects)}  Proposals: {len(pipeline['proposals'])}  "
              f"Matched: {len(matched_gt)}")

        # ----------------------------------------------------
        # Classify every GT object using the REAL production
        # taxonomy (bug-fixed), then route into hypothesis pools.
        # ----------------------------------------------------

        pool_a, pool_b, pool_c = [], [], []

        for gt_index, gt in enumerate(gt_objects):
            diagnostic = diagnose_gt_object(gt, points, pipeline)
            accepted = gt_index in matched_gt
            candidates = candidates_by_gt.get(gt_index, [])
            best_candidate = candidates[0] if candidates else None

            reason = classify_failure_fixed(diagnostic, best_candidate, accepted)

            entry = {
                "gt_index": gt_index,
                "gt": gt,
                "candidate": best_candidate,
                "reason": reason,
                "diagnostic": diagnostic,
            }

            # A: any object (matched or not) with a real candidate whose
            # purity looks suspicious relative to its coverage.
            if best_candidate is not None and best_candidate["coverage"] >= 0.5 \
                    and best_candidate["purity"] < 0.85:
                pool_a.append(entry)

            # B: exactly the production MATCHING_FAILURE bucket.
            if reason == "MATCHING_FAILURE":
                pool_b.append(entry)

            # C: exactly the buckets Problem #3 in the doc is about.
            if reason in ("NO_OBSTACLE_CELLS", "COMPONENT_FILTERED_OUT", "LOW_COMPONENT_COVERAGE"):
                pool_c.append(entry)

        run_hypothesis_a(frame_name, pool_a, pipeline, points, cell_to_local_indices,
                          max_per_hypothesis, agg_a)
        run_hypothesis_b(frame_name, pool_b, candidates_by_gt, matched_gt, gt_objects, agg_b)
        run_hypothesis_c(frame_name, pool_c, points, pipeline, max_per_hypothesis, agg_c)

    # ========================================================
    # Aggregate verdict — the part the original script lacked
    # ========================================================

    print("\n" + "=" * 78)
    print("PHASE 1 — AGGREGATE VERDICT ACROSS ALL ANALYZED FRAMES")
    print("=" * 78)

    print("\n[Hypothesis A — footprint contamination]")
    if agg_a["tested"]:
        mean_purity_gain = float(np.mean(agg_a["purity_gains"]))
        mean_iou_gain = float(np.mean(agg_a["iou_gains"]))
        print(f"  Objects tested        : {agg_a['tested']}")
        print(f"  Mean purity gain (non-ground filter): {mean_purity_gain:+.1f}pp")
        print(f"  Mean IoU gain    (non-ground filter): {mean_iou_gain:+.1f}pp")
        for verdict, count in agg_a["verdicts"].most_common():
            print(f"  {verdict:<20} {count}")
        if mean_purity_gain > 10.0:
            print("  CONCLUSION: contamination hypothesis is SUPPORTED — recovery "
                  "should exclude ground / apply a z-band constraint.")
        else:
            print("  CONCLUSION: contamination effect is small on this evidence — "
                  "look elsewhere (merging, adaptive-cell boundaries) before patching recovery.")
    else:
        print("  No qualifying candidates found to test.")

    print("\n[Hypothesis B — matching contention]")
    total_b = agg_b["matching_failure_total"]
    explained_b = agg_b["matching_failure_explained"]
    if total_b:
        pct = explained_b / total_b * 100.0
        print(f"  MATCHING_FAILURE cases: {total_b}")
        print(f"  Explained by contention: {explained_b} ({pct:.1f}%)")
        if pct > 50.0:
            print("  CONCLUSION: contention is the DOMINANT driver of MATCHING_FAILURE — "
                  "fix under-segmentation/merging before touching evaluator thresholds.")
        elif explained_b > 0:
            print("  CONCLUSION: contention explains SOME but not most cases — "
                  "investigate remaining cases individually.")
        else:
            print("  CONCLUSION: contention does NOT explain observed MATCHING_FAILURE "
                  "cases on this evidence — look at candidate ranking/tie-breaking instead.")
    else:
        print("  No MATCHING_FAILURE cases found in the analyzed frames.")

    print("\n[Hypothesis C — component-filtering loss]")
    if agg_c:
        total_c = sum(agg_c.values())
        for classification, count in agg_c.most_common():
            print(f"  {classification:<38} {count:3d} ({count/total_c*100:5.1f}%)")
        filtered_fraction = agg_c.get("COMPONENT_FILTERED_OUT", 0) / total_c
        if filtered_fraction > 0.3:
            print("  CONCLUSION: a meaningful share of 'no detection' cases had real "
                  "obstacle evidence that filter_components() removed — this is a "
                  "range-adaptive threshold question, not a 'no evidence' problem.")
    else:
        print("  No relevant objects found in the analyzed frames.")

    if json_out:
        summary = {
            "frames": frames,
            "hypothesis_a": {
                "tested": agg_a["tested"],
                "mean_purity_gain_pp": float(np.mean(agg_a["purity_gains"])) if agg_a["tested"] else None,
                "mean_iou_gain_pp": float(np.mean(agg_a["iou_gains"])) if agg_a["tested"] else None,
                "verdicts": dict(agg_a["verdicts"]),
            },
            "hypothesis_b": dict(agg_b),
            "hypothesis_c": dict(agg_c),
        }
        Path(json_out).write_text(json.dumps(summary, indent=2))
        print(f"\nSummary written to {json_out}")

    print("\n" + "=" * 78)
    print("PHASE 1 DIAGNOSTICS COMPLETE — interpret as hypothesis tests, not thresholds.")
    print("=" * 78)


def _load_points(velodyne_path: Path) -> np.ndarray:
    """Load XYZ only (mirrors load_semantic_kitti_frame in object_detection.py)."""
    raw = np.fromfile(velodyne_path, dtype=np.float32)
    scan = raw.reshape(-1, 4)
    return scan[:, :3].astype(np.float64, copy=False)


if __name__ == "__main__":
    main()