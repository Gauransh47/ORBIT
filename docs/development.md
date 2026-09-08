# ORBIT development

How to work on the **current** prototype. Official problem statement
and remaining gaps: [project-context.md](project-context.md).

This is not a packaged product. There is no `pyproject.toml`.
Deep-learning training/inference is not part of the setup below
because it is not in the repository.

## Project structure

```
ORBIT/
  docs/                  # this documentation
  src/
    common/              # path helpers + diagnostic records only
    datasets/            # SemanticKITTI / KITTI I/O + nuScenes mini loader
    evaluation/          # merge diagnostics
    mapping/             # adaptive + semantic grids
    perception/          # ground, obstacles, clustering, association
    performance/         # OrbitPerception stage timing
    tracking/            # ego motion + temporal tracker
    visualization/       # matplotlib / Open3D viewers
    world_model/         # persistent tracks
    pipeline.py          # RANSAC + AdaptiveGrid demo
    synthetic_scene.py   # writes data/synthetic_scene.ply
    ...
  tests/                 # pytest modules (src/ must be on PYTHONPATH)
  data/                  # gitignored; not shipped
```

There is no `pyproject.toml`, `setup.py`, or root `README`.
`src/` is not an installed package; scripts prepend `src` to
`sys.path` or expect `PYTHONPATH=src` from the repo root.

Import styles are mixed and matter:

- Most geometric scripts: `from mapping...` / `from perception...`
  after inserting `src/` on `sys.path`.
- Tracker / semantic grid / semantic associator:
  `from src.mapping...` (repo root must be on `PYTHONPATH`).
- `src/datasets/inspect_kitti.py`: `from kitti_loader import ...`
  (run with cwd `src/datasets` or it will fail).

New tools should call `common.paths.ensure_src_on_path()` and use
`mapping` / `perception` imports.

## Environment and setup

Python 3 with:

| Package | Required by |
|---------|-------------|
| numpy | everything |
| open3d | I/O, RANSAC, 3D viewers, most `__main__` demos |
| matplotlib | maps, Visual Intelligence dashboard |
| scikit-learn | `object_detection.py` only (DBSCAN) |
| pytest | tests |

Install those packages in a virtualenv. `data/` is listed in
`.gitignore`.

**Synthetic cloud** (needed by geometric demos):

```bash
mkdir -p data
PYTHONPATH=src python src/synthetic_scene.py
```

This writes `data/synthetic_scene.ply` and opens an Open3D window
when a display is available.

**SemanticKITTI** (needed by tracker / semantic scripts), expected at:

```
data/semantic_kitti/sequences/<seq>/
  velodyne/*.bin
  labels/*.label          # optional for geometry-only
  poses.txt               # required by ego motion / tracker
  calib.txt               # required by EgoMotionCompensator
  times.txt               # optional; 0.10 s fallback otherwise
```

## How to run main components

Run from the **repository root** unless noted. Open3D GUI scripts need
a display.

Geometric / synthetic (after generating the PLY):

```bash
PYTHONPATH=src python src/pipeline.py
PYTHONPATH=src python src/perception/orbit_perception.py
PYTHONPATH=src python src/perception/terrain_obstacle_detection.py
PYTHONPATH=src python src/perception/connected_components.py
PYTHONPATH=src python src/perception/object_association.py
PYTHONPATH=src python src/mapping/adaptive_grid.py
PYTHONPATH=src python src/visualization/adaptive_map.py
PYTHONPATH=src python src/visualization/elevation_map.py
PYTHONPATH=src python src/visualization/adaptive_grid_viewer.py
```

End-to-end prototype + Visual Intelligence dashboard:

```bash
PYTHONPATH=src python src/orbit_system.py --source kitti --sequence 00 --start 0 --end 9 --dashboard
PYTHONPATH=src python src/orbit_system.py --source kitti --sequence 00 --start 0 --end 9 --save-figures output/dashboard
```

See [dashboard.md](dashboard.md) for colour meanings and panel sources.
nuScenes v1.0-mini (dataset **outside** the repo): [nuscenes.md](nuscenes.md).

DBSCAN sidecar:

```bash
PYTHONPATH=src python src/perception/object_detection.py
PYTHONPATH=src python src/visualization/object_detection_viewer.py
```

KITTI (dataset must exist):

```bash
cd src/datasets && python inspect_kitti.py --sequence 00 --frame 0
PYTHONPATH=src python src/datasets/run_orbit_kitti.py --sequence 00 --frame 0
PYTHONPATH=. python src/mapping/semantic_grid.py
PYTHONPATH=. python src/perception/semantic_object_association.py --sequence 00 --frame 0
PYTHONPATH=. python src/tracking/ego_motion.py
PYTHONPATH=. python src/tracking/orbit_tracker.py --sequence 00 --start 0 --end 10
```

Profiling / benchmark (synthetic PLY):

```bash
PYTHONPATH=src python src/benchmark.py
PYTHONPATH=src python src/profile_orbit.py
PYTHONPATH=src python src/performance/profile_pipeline.py
```

## Evaluation scripts

Proposal merging (geometric path on the synthetic scene):

```bash
PYTHONPATH=src python src/evaluation/analyze_proposal_merging.py \
  --source synthetic --mode geometric
```

Geometric merges on a KITTI frame, using instance IDs when labels exist:

```bash
PYTHONPATH=src python src/evaluation/analyze_proposal_merging.py \
  --source kitti --mode geometric --sequence 00 --frame 0 \
  --output /tmp/orbit_merges.json
```

Semantic fragment-merge path (requires labels):

```bash
PYTHONPATH=src python src/evaluation/analyze_proposal_merging.py \
  --source kitti --mode semantic --sequence 00 --frame 0
```

`--mode semantic` inspects `SemanticObjectAssociator.merge_fragments`.
`--mode geometric` inspects `associate_components` after connected
components.

JSON written by `--output` is a `MergeAnalysis` record
(`src/common/types.py`).

## Tests

```bash
PYTHONPATH=src python -m pytest tests/
```

Current tests:

- `tests/test_adaptive_grid.py` — range rings, cell indices,
  `build` / `insert_point`, parent lookup.
- `tests/test_proposal_merging.py` — cell indexing, instance overlap,
  geometric multi-component merge reporting.

Tests do not download KITTI. They do not cover tracking, ego motion,
or the world model.

## Conventions

- Prefer extending the existing pipeline over adding a second
  representation. The live grid key is `(level, ix, iy)` in Cartesian
  coordinates, not polar `(ring, bin, sector)`.
- Ground detection is duplicated (Open3D RANSAC in several modules).
  `AdaptiveGrid` has **no** `detect_ground` method; KITTI scripts that
  call it catch `AttributeError` and use `z < 0`.
- Do not treat geometric classes (`POLE`, `VEHICLE-LIKE`, …) as
  SemanticKITTI IDs; they are separate vocabularies.
- Moving SemanticKITTI classes (IDs 252–259) are **not** in
  `OBJECT_CLASSES` in `semantic_object_association.py`, so they never
  become hypotheses for the tracker.
- `data/` and `venv/` stay untracked.
- Keep evaluation tools read-only with respect to association
  thresholds unless the change is an explicit perception patch.
- When writing docs or comments, separate **intended iDEX/DRDO
  capabilities** from **what the code does now**. Do not describe
  PointNet++, sparse CNNs, real-time dashboards, or learned
  segmentation as present unless they are added to the tree.
