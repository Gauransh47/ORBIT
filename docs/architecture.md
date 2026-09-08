# ORBIT architecture (current implementation)

This document describes **code that exists in this repository**.

Official iDEX / DRDO goals, and the gap to that expected system, are
in [project-context.md](project-context.md). Do not read the sections
below as a claim that deep learning, real-time operation, or finished
dynamic-object classification are done.

ORBIT today is a CPU-side LiDAR **prototype**. It builds a
**range-adaptive 2.5D map**, then forms object hypotheses from that
map. Optional later stages attach SemanticKITTI **ground-truth**
labels, track objects over time using KITTI poses, and copy confirmed
tracks into a world model.

It is a collection of runnable scripts under `src/`, not a packaged
library and **not** a trained neural network. Semantic class names on
the KITTI path come from the dataset, not from an ORBIT segmenter.

## Current purpose of this codebase

- Represent 100 m of LiDAR as a sparse grid whose cell size grows with
  range (implemented rings: 5 cm / 10 cm / 25 cm / 50 cm).
- Separate ground from obstacles using a RANSAC plane (or a `z < 0`
  fallback on some KITTI scripts).
- Group obstacle cells into object proposals and optionally merge
  fragments that look like one physical object.
- On SemanticKITTI, overlay native semantic IDs on the same cells,
  form class-aware hypotheses, track them, and store confirmed tracks.

Two perception paths exist and are **not fully unified**:

| Path | Typical input | Object formation |
|------|---------------|------------------|
| Geometric | `data/synthetic_scene.ply` or KITTI XYZ | Obstacle cells → connected components → `associate_components` |
| Semantic (GT overlay) | SemanticKITTI `.bin` + `.label` | `SemanticGrid` → `SemanticObjectAssociator` |
| DBSCAN sidecar | same clouds | `ObjectDetector` in `object_detection.py` (not used by the tracker) |

## Adaptive 2.5D grid

Implemented in `src/mapping/adaptive_grid.py`.

Points with horizontal range ≥ 100 m are dropped. Remaining points
are binned by range ring:

| Ring | Range | Cell size |
|------|-------|-----------|
| 0 | 0–10 m | 5 cm |
| 1 | 10–25 m | 10 cm |
| 2 | 25–50 m | 25 cm |
| 3 | 50–100 m | 50 cm |

Each occupied cell (`AdaptiveCell`) stores counts and min/sum/max Z
for **ground** and **obstacle** points separately. `semantic_class` is
one of `GROUND`, `OBSTACLE`, or `MIXED`. Elevation properties are
means of the corresponding Z values.

`AdaptiveGrid.build` is vectorized (sort + `reduceat`). A dictionary
`grid.cells[(level, ix, iy)]` is filled afterwards for the rest of
the stack. `insert_point` still exists for tests and single-point
insertion.

`src/mapping/semantic_grid.py` attaches SemanticKITTI class IDs to
the same cell keys. Comments in that file call it an evaluation /
visualization layer. It is **not** a learned semantic engine and does
not satisfy the official “perception model” component.

## Perception pipeline (geometric)

`src/perception/orbit_perception.py` (`OrbitPerception.process`) is
the combined geometric engine:

1. Keep points with XY range < 100 m.
2. RANSAC ground plane (Open3D `segment_plane`, 8 cm, 1000 iterations).
3. `AdaptiveGrid.build`.
4. `extract_obstacle_cells` — cells with local obstacle height
   `obstacle_z - ground_z` above a threshold (default 15 cm) and a
   minimum obstacle point count. Cells **without ground** are skipped,
   so pure-obstacle columns are not proposals.
5. Resolution-aware connected components
   (`src/perception/connected_components.py`).
6. Size filtering, then `associate_components`
   (`src/perception/object_association.py`) which may merge nearby
   components into one `ObjectGroup`.
7. Geometric class (`POLE` / `WALL` / `VEHICLE-LIKE` / `OBSTACLE`) and
   a hand-tuned confidence score. These are **not** the official
   static/dynamic categories (pedestrian, vehicle, …).

The same stages can be run individually via each module’s `__main__`.

`src/perception/object_detection.py` is a **separate** distance-aware
DBSCAN detector (scikit-learn) on non-ground points. It is visualized
by `object_detection_viewer.py` and is not wired into `OrbitPerception`
or `OrbitTracker`.

## Terrain / obstacle detection

`src/perception/terrain_obstacle_detection.py` defines `ObstacleCell`
and `extract_obstacle_cells`. Height is relative to the **cell’s own
ground elevation**, not a global plane. That is the “terrain-aware”
part of the geometric pipeline.

## Visualization

| Module | What it shows |
|--------|----------------|
| `visualization/adaptive_map.py` | Matplotlib polygons at each cell’s true resolution |
| `visualization/elevation_map.py` | Top-down scatter of ground / obstacle elevation |
| `visualization/adaptive_grid_viewer.py` | Open3D boxes at Cartesian cell centers, colored by resolution |
| `visualization/object_detection_viewer.py` | Open3D boxes for DBSCAN proposals |
| `ground_detection.py` | Colored ground vs non-ground Open3D view |
| `synthetic_scene.py` | Generates and optionally displays the synthetic cloud |
| `visualize.py` | Early random-cloud Open3D smoke test (“SIH LiDAR Prototype”) |

Viewers that call `open3d.visualization.draw_geometries` or
`matplotlib.pyplot.show` need a display. This is **not** the official
real-time dashboard; scripts are run one-shot on a saved cloud or
KITTI frame.

## Tracking and world model

`src/tracking/orbit_tracker.py` is the multi-frame KITTI loop:

1. Load a frame (`KittiLoader`).
2. Build adaptive + semantic grids (ground via missing
   `AdaptiveGrid.detect_ground`, so the code falls back to `z < 0`).
3. `SemanticObjectAssociator.associate`.
4. Convert detection XY into a persistent frame using
   `EgoMotionCompensator` (`poses.txt` + `calib.txt` `Tr`).
5. Greedy one-to-one matching (class, distance, dimensions).
6. `OrbitWorldModel.update` on **confirmed** tracks.

`src/world_model/orbit_world_model.py` stores a dict of `WorldObject`
records (position, velocity, motion state copied from the tracker).
It does not run its own motion classifier on update, and it does not
delete objects when tracks die.

## Evaluation and diagnostics

This tree does **not** contain object-level batch evaluators such as
`object_level_evaluation.py`, `batch_object_level_evaluation.py`,
`analyze_gt_loss.py`, `analyze_missed_objects.py`,
`phase1_component_lineage.py`, `phase1_diagnostics.py`, or
`trace_gt_object.py`.

What exists:

- `src/evaluation/analyze_proposal_merging.py` — reports detections
  assembled from multiple connected components (geometric path) or
  multiple hypotheses (semantic `merge_fragments`). With SemanticKITTI
  instance IDs, a merge that covers more than one instance is flagged
  as a likely over-merge.
- `src/datasets/inspect_kitti.py` / `run_orbit_kitti.py` — inspect a
  real frame or run **geometric** `OrbitPerception` on it (labels are
  printed, not used for matching).
- `src/benchmark.py`, `src/profile_orbit.py`,
  `src/performance/profile_pipeline.py` — grid vs uniform 5 cm cost
  and stage timings on the machine that runs them. Not an official
  accuracy or real-time evidence package.

## Data flow (current)

```
LiDAR points (PLY or SemanticKITTI .bin)
        │
        ├─ range filter (< 100 m)
        ├─ ground mask (RANSAC or z < 0 fallback)
        └─ AdaptiveGrid (2.5D cells)
                │
                ├─ geometric: ObstacleCell → components → ObjectGroup
                │                 └─ OrbitPerception result dict
                │
                └─ semantic: SemanticGrid (GT labels)
                                └─ ObjectHypothesis
                                        └─ OrbitTracker (world XY)
                                                └─ OrbitWorldModel
```

## Shared types

Runtime types stay next to their owners (`AdaptiveCell`,
`ObjectComponent`, `ObjectGroup`, `ObjectHypothesis`, `Track`,
`WorldObject`). `src/common/` only holds path helpers and JSON-friendly
merge-diagnosis records so evaluation does not re-import perception
dataclasses into every report.

Intended system and remaining gaps: [project-context.md](project-context.md).
