"""
ORBIT - proposal merging diagnostics.

Existing geometric association (`associate_components`) and semantic
fragment merging (`SemanticObjectAssociator.merge_fragments`) can join
multiple connected components or hypotheses into one detection.

This tool reports those joins and, when SemanticKITTI instance labels
are available, flags likely over-merges: one detection covering more
than one ground-truth instance.

It does not change association thresholds. It only inspects the current
pipeline.
"""

from __future__ import annotations

import argparse
import json
import sys
from collections import defaultdict
from pathlib import Path

import numpy as np

SRC_DIR = Path(__file__).resolve().parents[1]
REPO_ROOT = SRC_DIR.parent

if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

from common.paths import SEMANTIC_KITTI_ROOT, SYNTHETIC_SCENE
from common.types import MergeAnalysis, MergeEvent
from mapping.adaptive_grid import AdaptiveGrid
from mapping.semantic_grid import SemanticGrid


# ============================================================
# CELL INDEXING
# ============================================================

def cell_key_for_xy(x, y):
    """Return (level, ix, iy) using AdaptiveGrid range rings."""

    return SemanticGrid.point_cell(float(x), float(y))


def build_instance_index(points, instances, labels=None):
    """
    Map adaptive cell keys to SemanticKITTI instance / semantic IDs.

    Instance 0 is ignored (no instance).
    """

    xyz = np.asarray(points, dtype=np.float64)
    instances = np.asarray(instances)

    if labels is None:
        labels = np.zeros(len(xyz), dtype=np.int64)
    else:
        labels = np.asarray(labels)

    instance_map = defaultdict(set)
    semantic_map = defaultdict(set)

    for i in range(len(xyz)):
        key = cell_key_for_xy(xyz[i, 0], xyz[i, 1])

        if key is None:
            continue

        instance_id = int(instances[i])
        semantic_id = int(labels[i])

        if instance_id > 0:
            instance_map[key].add(instance_id)

        if semantic_id > 0:
            semantic_map[key].add(semantic_id)

    return instance_map, semantic_map


def ids_for_cells(cells, lookup):
    found = set()

    for cell in cells:
        key = (int(cell.level), int(cell.ix), int(cell.iy))
        found.update(lookup.get(key, ()))

    return sorted(found)


def box_instance_ids(min_x, max_x, min_y, max_y, points, instances):
    """Instance IDs of labelled points inside an XY box."""

    xyz = np.asarray(points)
    instances = np.asarray(instances)

    mask = (
        (xyz[:, 0] >= min_x)
        & (xyz[:, 0] <= max_x)
        & (xyz[:, 1] >= min_y)
        & (xyz[:, 1] <= max_y)
        & (instances > 0)
    )

    if not np.any(mask):
        return []

    return sorted(set(int(v) for v in instances[mask]))


# ============================================================
# GEOMETRIC PATH
# ============================================================

def run_geometric_pipeline(points, ground_mask=None):
    from perception.connected_components import (
        connected_components,
        detect_ground,
        filter_components,
    )
    from perception.object_association import associate_components
    from perception.terrain_obstacle_detection import extract_obstacle_cells

    points = np.asarray(points, dtype=np.float64)

    if ground_mask is None:
        ground_mask = detect_ground(points)

    grid = AdaptiveGrid()
    grid.build(points, ground_mask)

    obstacle_cells = extract_obstacle_cells(
        grid,
        minimum_height=0.15,
        minimum_obstacle_points=3,
    )

    raw_components = connected_components(obstacle_cells)
    filtered = filter_components(
        raw_components,
        minimum_cells=3,
        minimum_points=5,
    )
    objects = associate_components(filtered)

    return {
        "grid": grid,
        "raw_components": raw_components,
        "filtered_components": filtered,
        "objects": objects,
    }


def geometric_merge_events(objects, instance_map=None, semantic_map=None):
    events = []

    for obj in objects:
        if len(obj.components) < 2:
            continue

        width, length, height = obj.dimensions
        x, y = obj.center

        all_cells = []
        for component in obj.components:
            all_cells.extend(component.cells)

        instance_ids = []
        semantic_ids = []

        if instance_map is not None:
            instance_ids = ids_for_cells(all_cells, instance_map)

        if semantic_map is not None:
            semantic_ids = ids_for_cells(all_cells, semantic_map)

        notes = []
        overmerged = None

        if instance_ids:
            overmerged = len(instance_ids) > 1
            if overmerged:
                notes.append(
                    "detection covers multiple SemanticKITTI instances"
                )
        else:
            notes.append(
                "merged from multiple connected components "
                "(no instance labels to confirm over-merge)"
            )

        if len(semantic_ids) > 1:
            notes.append(
                "merged cells contain multiple semantic classes"
            )

        events.append(
            MergeEvent(
                object_id=int(obj.object_id),
                classification=str(obj.classification),
                component_ids=[
                    int(component.component_id)
                    for component in obj.components
                ],
                component_count=len(obj.components),
                point_count=int(obj.point_count),
                cell_count=int(obj.cell_count),
                center=(float(x), float(y)),
                dimensions=(
                    float(width),
                    float(length),
                    float(height),
                ),
                distance=float(obj.distance),
                instance_ids=instance_ids,
                semantic_ids=semantic_ids,
                overmerged=overmerged,
                notes=notes,
            )
        )

    return events


# ============================================================
# SEMANTIC PATH
# ============================================================

class RecordingAssociator:
    """
    Thin wrapper around SemanticObjectAssociator that keeps the
    pre-merge hypothesis list.
    """

    def __init__(self):
        from perception.semantic_object_association import (
            SemanticObjectAssociator,
        )

        self._associator = SemanticObjectAssociator()
        self.pre_merge = []
        self.post_merge = []

    def associate(self, semantic_grid):
        original = self._associator.merge_fragments

        def wrapped(objects):
            self.pre_merge = list(objects)
            merged = original(objects)
            self.post_merge = list(merged)
            return merged

        self._associator.merge_fragments = wrapped
        result = self._associator.associate(semantic_grid)
        self._associator.merge_fragments = original
        return result


def run_semantic_pipeline(points, labels, ground_mask=None):
    xyz = np.asarray(points, dtype=np.float64)[:, :3]

    if ground_mask is None:
        try:
            grid = AdaptiveGrid()
            ground_mask = grid.detect_ground(xyz)
        except (AttributeError, TypeError):
            ground_mask = xyz[:, 2] < 0.0

    grid = AdaptiveGrid()
    grid.build(xyz, ground_mask)

    semantic_grid = SemanticGrid(grid)
    semantic_grid.build(xyz, labels)

    associator = RecordingAssociator()
    objects = associator.associate(semantic_grid)

    # RecordingAssociator.associate already ran merge_fragments.
    # Fallback if wrapping failed.
    if not associator.post_merge:
        associator.post_merge = list(objects)

    return {
        "grid": grid,
        "semantic_grid": semantic_grid,
        "pre_merge": associator.pre_merge,
        "post_merge": associator.post_merge,
        "objects": objects,
    }


def semantic_merge_events(
    pre_merge,
    post_merge,
    points=None,
    instances=None,
):
    """
    A semantic merge event is a post-merge hypothesis whose point
    count and footprint cannot be explained by a single pre-merge
    object. We match by bounding-box containment / overlap.
    """

    events = []

    used_pre = set()

    for post in post_merge:
        children = []

        for index, pre in enumerate(pre_merge):
            if index in used_pre:
                continue

            if pre.semantic_id != post.semantic_id:
                continue

            contained = (
                pre.min_x >= post.min_x - 1e-6
                and pre.max_x <= post.max_x + 1e-6
                and pre.min_y >= post.min_y - 1e-6
                and pre.max_y <= post.max_y + 1e-6
            )

            if contained:
                children.append(pre)
                used_pre.add(index)

        if len(children) < 2:
            continue

        instance_ids = []
        if points is not None and instances is not None:
            instance_ids = box_instance_ids(
                post.min_x,
                post.max_x,
                post.min_y,
                post.max_y,
                points,
                instances,
            )

        notes = [
            f"merged {len(children)} same-class hypotheses"
        ]

        overmerged = None
        if instance_ids:
            overmerged = len(instance_ids) > 1
            if overmerged:
                notes.append(
                    "merged box covers multiple SemanticKITTI instances"
                )

        events.append(
            MergeEvent(
                object_id=int(post.object_id),
                classification=str(post.semantic_class),
                component_ids=[
                    int(child.object_id) for child in children
                ],
                component_count=len(children),
                point_count=int(post.points),
                cell_count=int(post.cells),
                center=(float(post.center_x), float(post.center_y)),
                dimensions=(
                    float(post.width),
                    float(post.length),
                    0.0,
                ),
                distance=float(post.distance()),
                instance_ids=instance_ids,
                semantic_ids=[int(post.semantic_id)],
                overmerged=overmerged,
                notes=notes,
            )
        )

    return events


# ============================================================
# ANALYSIS
# ============================================================

def analyze_frame(
    points,
    mode="geometric",
    ground_mask=None,
    labels=None,
    instances=None,
    source="unknown",
    frame=None,
):
    xyz = np.asarray(points)
    if xyz.ndim != 2 or xyz.shape[1] < 3:
        raise ValueError("points must have shape (N, 3) or (N, 4)")

    xyz3 = xyz[:, :3]

    instance_map = None
    semantic_map = None

    if instances is not None:
        instance_map, semantic_map = build_instance_index(
            xyz3,
            instances,
            labels=labels,
        )

    if mode == "geometric":
        result = run_geometric_pipeline(xyz3, ground_mask=ground_mask)
        events = geometric_merge_events(
            result["objects"],
            instance_map=instance_map,
            semantic_map=semantic_map,
        )
        raw = len(result["raw_components"])
        filtered = len(result["filtered_components"])
        final = len(result["objects"])

    elif mode == "semantic":
        if labels is None:
            raise ValueError("semantic mode requires labels")

        result = run_semantic_pipeline(
            xyz3,
            labels,
            ground_mask=ground_mask,
        )
        events = semantic_merge_events(
            result["pre_merge"],
            result["post_merge"],
            points=xyz3,
            instances=instances,
        )
        raw = len(result["pre_merge"])
        filtered = len(result["pre_merge"])
        final = len(result["post_merge"])

    else:
        raise ValueError(f"Unknown mode: {mode}")

    overmerged = sum(1 for event in events if event.overmerged)

    return MergeAnalysis(
        source=source,
        mode=mode,
        frame=None if frame is None else str(frame),
        input_points=len(xyz3),
        raw_components=raw,
        filtered_components=filtered,
        final_objects=final,
        merged_objects=len(events),
        overmerged_objects=overmerged,
        events=events,
    )


def print_analysis(analysis: MergeAnalysis):
    print("=" * 70)
    print("ORBIT - PROPOSAL MERGING ANALYSIS")
    print("=" * 70)

    print(f"\nSource:              {analysis.source}")
    print(f"Mode:                {analysis.mode}")

    if analysis.frame is not None:
        print(f"Frame:               {analysis.frame}")

    print(f"Input points:        {analysis.input_points:,}")
    print(f"Raw components:      {analysis.raw_components:,}")
    print(f"Filtered components: {analysis.filtered_components:,}")
    print(f"Final objects:       {analysis.final_objects:,}")
    print(f"Merged detections:   {analysis.merged_objects:,}")
    print(f"Likely over-merges:  {analysis.overmerged_objects:,}")

    print("\nMERGE EVENTS")
    print("-" * 70)

    if not analysis.events:
        print("  No multi-component merges detected.")
        print("=" * 70)
        return

    for event in analysis.events:
        flag = ""
        if event.overmerged:
            flag = "  [OVER-MERGE]"

        print(
            f"\nObject #{event.object_id}  "
            f"{event.classification}{flag}"
        )
        print(
            f"  Components:   {event.component_ids} "
            f"({event.component_count})"
        )
        print(
            f"  Center:       "
            f"({event.center[0]:.2f}, {event.center[1]:.2f}) m"
        )
        print(f"  Distance:     {event.distance:.2f} m")
        print(
            f"  Dimensions:   "
            f"{event.dimensions[0]:.2f} × "
            f"{event.dimensions[1]:.2f} × "
            f"{event.dimensions[2]:.2f} m"
        )
        print(f"  Points/cells: {event.point_count:,} / {event.cell_count}")

        if event.instance_ids:
            print(f"  GT instances: {event.instance_ids}")

        if event.semantic_ids:
            print(f"  Semantic IDs: {event.semantic_ids}")

        for note in event.notes:
            print(f"  Note:         {note}")

    print("\n" + "=" * 70)
    print("PROPOSAL MERGING ANALYSIS COMPLETE")
    print("=" * 70)


# ============================================================
# CLI
# ============================================================

def get_args():
    parser = argparse.ArgumentParser(
        description=(
            "Diagnose object-proposal merges in the current "
            "ORBIT perception pipeline"
        )
    )

    parser.add_argument(
        "--source",
        choices=("synthetic", "kitti"),
        default="synthetic",
        help="Input cloud (synthetic PLY or SemanticKITTI frame)",
    )
    parser.add_argument(
        "--mode",
        choices=("geometric", "semantic"),
        default="geometric",
        help="Which association path to inspect",
    )
    parser.add_argument(
        "--sequence",
        type=str,
        default="00",
    )
    parser.add_argument(
        "--frame",
        type=int,
        default=0,
    )
    parser.add_argument(
        "--dataset-root",
        type=str,
        default=str(SEMANTIC_KITTI_ROOT),
    )
    parser.add_argument(
        "--output",
        type=str,
        default=None,
        help="Optional JSON path for the analysis record",
    )

    return parser.parse_args()


def load_inputs(args):
    if args.source == "synthetic":
        if args.mode == "semantic":
            raise ValueError(
                "semantic mode requires SemanticKITTI labels; "
                "use --source kitti"
            )

        import open3d as o3d

        if not SYNTHETIC_SCENE.exists():
            raise FileNotFoundError(
                f"Synthetic scene not found:\n{SYNTHETIC_SCENE}\n"
                "Generate it with: python src/synthetic_scene.py"
            )

        cloud = o3d.io.read_point_cloud(str(SYNTHETIC_SCENE))
        points = np.asarray(cloud.points)

        return {
            "points": points,
            "labels": None,
            "instances": None,
            "source": str(SYNTHETIC_SCENE),
            "frame": None,
        }

    from datasets.kitti_loader import KittiLoader

    loader = KittiLoader(args.dataset_root, sequence=args.sequence)
    frame = f"{args.frame:06d}"
    points, labels, instances = loader.load_frame(frame, load_labels=True)

    if args.mode == "semantic" and labels is None:
        raise RuntimeError(f"No semantic labels for frame {frame}")

    return {
        "points": points,
        "labels": labels,
        "instances": instances,
        "source": f"{args.dataset_root} seq={args.sequence}",
        "frame": frame,
    }


def main():
    args = get_args()
    payload = load_inputs(args)

    analysis = analyze_frame(
        payload["points"],
        mode=args.mode,
        labels=payload["labels"],
        instances=payload["instances"],
        source=payload["source"],
        frame=payload["frame"],
    )

    print_analysis(analysis)

    if args.output:
        output_path = Path(args.output)
        output_path.parent.mkdir(parents=True, exist_ok=True)
        output_path.write_text(
            json.dumps(analysis.to_dict(), indent=2)
        )
        print(f"\nWrote {output_path}")


if __name__ == "__main__":
    main()
