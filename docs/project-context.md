# Project context

This page maps the official iDEX / DRDO problem statement onto the
repository. It is the place to look for **intent vs status**.

Code-level pipelines, modules, and run commands are in
[architecture.md](architecture.md) and [development.md](development.md).

---

## 1. Project objective / intended system

**Title:** Adaptive Variable Resolution 2.5D LiDAR Mapping for Dynamic
Environment Perception

**Organization:** DRDO — Department of Defence Production / iDEX  
**Theme:** Smart Vehicles

Autonomous navigation needs precise surroundings. Dense 3D LiDAR is
rich but expensive to process; a flat 2D occupancy grid drops height
needed for curbs, potholes, terrain, and overhangs. The intended
approach is **foveated mapping**: fine detail near the vehicle for
safety, coarser cells farther away to save compute and memory.

### Primary goal

A software framework that turns raw LiDAR into a **variable-resolution
2.5D grid** that acts as an **elevation map with semantic information**
tied to space.

### Primary tasks (official)

1. **Terrain analysis** — drivable vs non-drivable surfaces.
2. **Object detection** — static (walls, poles) and dynamic
   (pedestrians, vehicles), with classification.
3. **Adaptive spatial representation** — cell size grows with range,
   without spatial alignment errors, projection errors, or silent
   data loss when going from 3D LiDAR into 2.5D.

### Expected solution components (official)

| Component | Intended role |
|-----------|----------------|
| Deep learning / semantic perception | Segment LiDAR into terrain, static obstacles, and moving objects. PointNet++ and sparse CNNs are **examples**, not mandatory. |
| Variable-resolution grid engine | Project **classified** 3D points into a 2.5D grid: high resolution near the sensor (the statement cites ~5 cm within ~10 m as the target direction) and coarser farther out (the statement cites ~50 cm out to ~100 m). |
| Real-time visualization / dashboard | Show the 2.5D map, terrain, obstacles, and objects with clear colour coding, and demonstrate variable resolution vs a uniform fine grid. |
| Performance evaluation | Latency, FPS, memory, detection/classification accuracy, and behaviour vs distance. |

These are **targets**. They are not a description of the current repo.

---

## 2. Current implementation

The repository is a **CPU Python prototype**: scripts under `src/`,
NumPy + Open3D, no trained network, no ROS, no packaged runtime.

### What exists and matches the intended direction

**Variable-resolution 2.5D grid** (`src/mapping/adaptive_grid.py`)
is implemented, not merely planned. Occupied cells use Cartesian
keys `(level, ix, iy)` with these **actual** rings:

| Ring | Horizontal range | Cell size |
|------|------------------|-----------|
| 0 | 0–10 m | 5 cm |
| 1 | 10–25 m | 10 cm |
| 2 | 25–50 m | 25 cm |
| 3 | 50–100 m | 50 cm |

Points at ≥ 100 m XY range are dropped. Each cell stores separate
ground vs obstacle Z statistics (count, sum, min, max). That is a
sparse **elevation-style** map, not a full 3D voxel volume.

Those ring sizes **happen to match** the statement’s example
near/far resolutions. That is current code, not a claim that the
full expected system is done.

**Terrain-ish analysis** is geometric: Open3D RANSAC plane (8 cm)
in most synthetic pipelines, or `z < 0` on some KITTI scripts when
`AdaptiveGrid.detect_ground` is missing. Cells are labelled
`GROUND` / `OBSTACLE` / `MIXED`. Obstacle height is
`obstacle_z - ground_z` **inside the same cell**.

**Object proposals** are geometric clustering (connected components
+ merge) or a separate DBSCAN sidecar. Classes such as `POLE`,
`WALL`, `VEHICLE-LIKE` are **shape heuristics**, not a learned
detector. SemanticKITTI **ground-truth labels** can be pasted onto
cells (`SemanticGrid`) and clustered (`SemanticObjectAssociator`).
That is **not** ORBIT semantic segmentation.

**Visualization** is a set of Open3D / Matplotlib scripts (map
polygons, elevation scatter, grid boxes, detection boxes). There is
no integrated real-time dashboard.

**Performance scripts** time `AdaptiveGrid` vs a uniform 5 cm dict
(`benchmark.py`) and `OrbitPerception` stages. They report latency /
FPS for whatever machine runs them. They are not a claim of
operational real-time or of accuracy metrics.

**Tracking / world model** (KITTI poses) keep object identities over
frames using GT semantics plus greedy matching. Motion state is
rule-based in world/LiDAR-frame 0 coordinates.

### What does not exist (do not document as implemented)

- PointNet++, sparse CNNs, or any learned LiDAR segmentation
- ORBIT-produced per-point semantic labels
- A production real-time dashboard
- Published latency/FPS/memory/accuracy evidence against a required
  budget
- A finished dynamic-vs-static classifier that does not rely on
  SemanticKITTI moving-* IDs (and those IDs are currently **excluded**
  from `OBJECT_CLASSES`)
- Guarantees that 3D→2.5D projection has no alignment or information
  loss (cells average Z; instance/point identity is not stored in
  the grid)

---

## 3. Remaining work / gap to the expected solution

| Official requirement | Gap in this repository |
|----------------------|-------------------------|
| Semantic segmentation model | No training code, weights, or inference. KITTI labels are external GT. |
| Classified points → grid | Grid is filled from XYZ + a **ground mask**, not from a learned class per point. `SemanticGrid` is an overlay. |
| Drivable vs non-drivable terrain | RANSAC / `z < 0` is not curb/pothole/overhang terrain analysis. Mixed cells and missing ground in a cell drop pure-obstacle columns from geometric proposals. |
| Static vs dynamic objects | Geometric classes do not encode motion. Tracker motion labels need multi-frame KITTI and still miss moving-* semantic IDs 252–259. |
| No projection / alignment / data loss | Range rings can split one object across resolutions. Association and merge are heuristic. Over-merge is diagnosed, not solved. Cell stats discard raw points. |
| Real-time visualization dashboard | Separate offline viewers; GUI needs a display; no live vehicle loop. |
| Variable vs uniform demonstration | `benchmark.py` compares cell counts/time for uniform 5 cm vs adaptive **on a synthetic cloud**, not a dashboard A/B study. |
| Evaluation: latency, FPS, memory, accuracy vs range | Partial timing only. No official accuracy protocol, no distance-binned detection metrics, no memory study beyond the uniform-grid dict experiment. |
| Software framework | Runnable scripts with mixed import styles; not an installable, tested product API. |

Useful existing work to **keep** while closing gaps: the vectorized
adaptive grid, terrain-relative obstacle height, resolution-aware
connected components, and the (unfinished) world-coordinate tracker.

A reasonable order toward the official solution, without pretending
it is done:

1. Replace RANSAC / `z < 0` with a terrain model that can express
   drivability and local height defects.
2. Attach **ORBIT** semantics (learned or otherwise) **before** grid
   insertion, instead of KITTI GT after the fact.
3. Unify geometric and semantic object paths; include moving classes;
   evaluate with instance-aware metrics vs range.
4. One visualization path that shows terrain, obstacles, objects, and
   variable vs uniform cost.
5. Record latency, FPS, memory, and accuracy as first-class outputs.

Until those exist in the tree, describe ORBIT as a **prototype grid
and geometric perception stack** aimed at the iDEX problem, not as
the completed expected solution.
