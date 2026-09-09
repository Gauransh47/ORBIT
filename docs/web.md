# ORBIT website — architecture and phases

The Matplotlib dashboard stays. This document is the web layer only.

## Inspection (Python side)

`PipelineState` (`src/orbit_system.py`) is the export source:

| Field | Use on the web |
|-------|----------------|
| `frame_index` | Frame navigation |
| `points` | Live ego LiDAR (downsample for JSON) |
| `metrics` | Cards: `mapped_points`, `ground_points`, `adaptive_cells`, `obstacle_cells`, `proposals`, `live_tracks`, `confirmed_tracks`, `latency_ms`, `world_frame` |
| `ego_xy`, `ego_heading_rad`, `pose_source` | Trajectory / CURRENT |
| `world_points` | Optional world cloud subsample |
| `tracks` | `track_id`, `class_name`, `confirmed`, `hits`, `position` |
| `grid.cells` | AdaptiveCell: `center`, `resolution`, `semantic_class`, `ground_elevation` |
| `proposals` | Optional boxes in ego frame |

World frame is **LiDAR frame 0**, never nuScenes global.

Safest export: a new module `src/web_export/export_orbit_data.py` that
calls `OrbitSystem.process_frame` (same as CLI) and writes JSON. Do not
reimplement perception. Downsample visualization clouds; keep metric
counts from the full pipeline.

## Target directories

```
ORBIT/
  src/                    # Python pipeline (untouched in website phases)
  src/web_export/         # Phase 3 exporter
  web/                    # Vite + React
  exported_data/          # gitignored dumps; copy subset to web/public/data
  docs/web.md
```

## Design philosophy (Phase 2)

The site is a **spatial intelligence experience**, not a rearranged Python
dashboard. Dark near-black field, cyan for spatial cues, green for terrain,
warm accents for obstacles. Typography uses `clamp()`, CSS Grid, and Flexbox.
Decorative Canvas/SVG sits behind or beside copy; content is never absolutely
positioned for layout.

Landing sketches are labeled conceptual. They are not scene-0061 output.

## Phase plan

1. **Foundation** — Vite app, theme, nav, routes, placeholders.
2. **Landing** — cinematic hero, glance cards, scroll pipeline,
   conceptual demo / map / planning previews, tech foundation.
3. **Exporter** — real PipelineState JSON (`src/web_export/`).
4. **Interactive demo** — PREV/NEXT explorer from JSON.
5. **Dataset-aware explorer** — registry, nuScenes / KITTI / synthetic.
6. **2.5D map explorer** — dedicated `/map` adaptive grid.
7. **Path planning demonstration (this phase)** — website A* on exported cells.
8. **Visual polish** — contrast, lighting, legends (not this phase).

Do not invent metrics in the demo once JSON exists.

## Phase 2 additions

- Immersive hero: Canvas LiDAR field with cursor parallax, scan sweep,
  trajectory fragment, contrast veils, pause when off-screen, fewer particles
  when `prefers-reduced-motion`.
- Glance cards with conceptual sketches (LiDAR, grid, objects, world).
- Scroll-linked pipeline story (seven stages; navigation marked planned).
- Demo / map / planning preview sections with honest CTAs into existing routes.
- Module-style top nav (horizontal scroll on narrow viewports).
- Framer Motion section reveals.

## Current website limitations

- The landing hero sketches remain conceptual (not exported JSON).
- Path planning on `/planning` is a **website A\* demonstration**, not a Python
  ORBIT runtime planner.
- Visual contrast / color grading is deferred to Phase 8.
- `web/public/data/**/*.json` (except `datasets.json`) is not committed; copy
  from `exported_data/`.
- SemanticKITTI stays listed as “Not exported yet” until JSON exists.

## What remains after Phase 3

Interactive Demo is implemented in Phase 4 below.

# Phase 3 — Real Data Export

The website visualizes exported ORBIT pipeline results. It does not execute
perception, mapping, tracking, or world-model algorithms in the browser.

## Architecture

```
Python ORBIT Pipeline
        ↓
OrbitSystem.process_frame → PipelineState
        ↓
src/web_export/export_orbit_data.py
        ↓
exported_data/<scene>/*.json
        ↓
React website (later phases)
```

The exporter calls the existing orchestrator. It does not import
`src/visualization/dashboard.py` and does not reimplement RANSAC, the
adaptive grid, the tracker, or the world model.

## PipelineState → JSON mapping

| PipelineState | JSON |
|---------------|------|
| `frame_index` | `frame_index` |
| `ego_xy`, `ego_heading_rad`, `pose_source`, `T_ego_to_world` | `pose` |
| `metrics` | `metrics` (plus `point_count_full` / `point_count_exported`) |
| `points` (range-filtered ego XYZ) | `points` (downsampled), `points_frame`: `current_lidar` |
| same indices × `T_ego_to_world` | `world_points`, `world_points_frame`: `lidar_frame_0` |
| `tracks` | `tracks` (world XY, `dimensions_xy`, hits, confirmed, …) |
| `grid.cells` | `adaptive_cells` |
| `obstacle_cells` | `obstacle_cells` |
| `proposals` | `proposals` (`frame`: `current_lidar`) |
| `world_objects` | `world_objects` including `history_xy` |
| `ground` | `ground.method`, `inlier_count`, `plane_model` (not the full mask) |
| `retired_ids` | `retired_ids` |

World frame is **lidar_frame_0** (`WORLD_REFERENCE_FRAME = 0`). No silent
conversion to nuScenes global.

`view_model` is not exported (dashboard-only). Track objects do not store a
full XY polyline; `motion_history` on a track is recent **speeds**, exported
as `motion_history_speed`. Position history lives on world objects as
`history_xy`.

## Exporter command

```bash
PYTHONPATH=src python -m web_export.export_orbit_data \
  --source nuscenes \
  --dataset-root "D:\ORBIT_DATA\nuscenes" \
  --scene scene-0061 \
  --start 0 \
  --end 19 \
  --output exported_data \
  --max-points 5000
```

`--dataset-root` may be omitted if `ORBIT_NUSCENES_ROOT` is set.

KITTI:

```bash
PYTHONPATH=src python -m web_export.export_orbit_data \
  --source kitti --dataset-root data/semantic_kitti \
  --sequence 00 --start 0 --end 4 --output exported_data
```

Synthetic PLY (`data/synthetic_scene.ply`):

```bash
PYTHONPATH=src python -m web_export.export_orbit_data \
  --source synthetic --output exported_data
```

## Output directory

```
exported_data/<scene-id>/
  manifest.json
  trajectory.json
  frame_0000.json
  ...
```

JSON dumps are gitignored. `exported_data/README.md` is the placeholder.

## Point downsampling

Evenly spaced original indices:

`np.round(np.linspace(0, N-1, max_points))`

No RNG. Geometry is a subset of real mapped XYZ. `metrics.point_count_full`
is `len(PipelineState.points)` after range filter; `point_count_exported` is
the JSON array length. Adaptive cells are written in full (no cell downsample).

## What is not exported

- Full unfiltered raw scan (`input_points` is in metrics only)
- Per-point ground mask
- Dashboard `view_model`
- Invented 3D boxes, GT classes, or path-planning routes
- nuScenes `sample_annotation` (the loader never reads it)

# Phase 4 — Interactive ORBIT Explorer

`/demo` is a visualization client. It does not execute perception, mapping,
tracking, or the world model.

## Architecture

```
web/public/data/scene-0061/manifest.json     (fetched once)
web/public/data/scene-0061/trajectory.json   (fetched once)
web/public/data/scene-0061/frame_XXXX.json   (fetched on demand, LRU cache)
        ↓
React + R3F explorer
```

Copy:

```bash
mkdir -p web/public/data/scene-0061
cp exported_data/scene-0061/*.json web/public/data/scene-0061/
cd web && npm run dev
```

Open http://localhost:5173/demo

# Phase 5 — Multi-dataset explorer

The Interactive Demo is dataset-aware. It still only **visualizes** exported
JSON.

## Registry

`web/public/data/datasets.json` describes datasets (`nuscenes`,
`semantic-kitti`, `synthetic`) and their collections (scene / sequence /
environment). The client probes each listed path for `manifest.json`. If the
file is missing or is HTML (SPA fallback), the collection is **not available**.
No placeholder detections are invented.

## Directory layout

Preferred:

```
web/public/data/datasets.json
web/public/data/nuscenes/scene-0061/manifest.json
web/public/data/semantic-kitti/sequence-00/…
web/public/data/synthetic/environment-01/…
```

Legacy (still supported):

```
web/public/data/scene-0061/manifest.json
```

## URL

```
/demo?dataset=nuscenes&scene=scene-0061
/demo?dataset=semantic-kitti&sequence=00
/demo?dataset=synthetic&environment=environment-01
```

Legacy `/demo?scene=scene-0061` is rewritten to include `dataset` when the
collection is found.

Invalid dataset or scene → error state, no silent fallback.

## Capabilities

Inferred from the loaded frame + `trajectory.json` (and optional declared
`capabilities` on a dataset in the registry):

`has_points`, `has_world_points`, `has_trajectory`, `has_adaptive_grid`,
`has_tracks`, `has_objects`, `has_proposals`, `has_semantic_labels`,
`has_ground_data`, `has_path_data`.

Unavailable view modes are disabled (e.g. Objects if no tracks/objects/proposals
were exported).

## Adding a dataset

1. Export with `python -m web_export.export_orbit_data --source …`
2. Copy JSON into `web/public/data/<dataset-id>/<collection-id>/`
3. Add an entry to `datasets.json` with `paths` to probe
4. Do not add fake frames

SemanticKITTI later: `--source kitti`, folder `semantic-kitti/sequence-00`,
`collection_label: sequence`. Synthetic later: `--source synthetic`,
`synthetic/environment-01`.


## Data loading and cache

1. Fetch `manifest.json`.
2. Fetch `trajectory.json` if present.
3. Fetch only the current frame JSON.
4. `FrameCache` keeps the last 12 frames in memory (LRU). Revisit uses cache.

Frames are never imported into the Vite bundle.

## Visualization modes

| Mode | JSON fields |
|------|-------------|
| LiDAR | `points` (`points_frame`), proposal footprints from `proposals` (XY + width/length) |
| World | `world_points`, ego from `pose.ego_xy`, optional `trajectory.samples` |
| Adaptive Grid | `adaptive_cells` (`center`, `resolution`, `semantic_class`, elevations) |
| Objects | `tracks` and extra `world_objects` as ground-plane footprints (`position`, `dimensions_xy`) |

No invented 3D AABBs. Grid cells are in the current LiDAR XY of that frame.

## Local

```bash
cd web
npm install
npm run dev
npm run build
```

# Phase 6 — Dedicated 2.5D Adaptive Grid Map Explorer

`/map` is a visualization client for exported adaptive grids. It does not
run mapping, occupancy, or terrain algorithms in the browser.

## Architecture

```
datasets.json → probe manifest.json
        ↓
manifest.json + trajectory.json (once)
        ↓
frame_XXXX.json on demand (FrameCache LRU, 12 frames)
        ↓
InstancedMesh of exported adaptive_cells
```

URL query is the same as Phase 5:

```
/map?dataset=nuscenes&scene=scene-0061
/map?dataset=semantic-kitti&sequence=00
/map?dataset=synthetic&environment=environment-01
```

Only collections with a real `manifest.json` become selectable. Listed but
missing exports show **Not exported yet**. No cells, elevations, obstacles,
or tracks are invented.

## Exported fields visualized

| JSON | Use |
|------|-----|
| `adaptive_cells.center` | Cell XY (LiDAR / world XY of that frame) |
| `adaptive_cells.resolution` | Footprint size (adaptive nature) |
| `adaptive_cells.semantic_class` | Semantic mode (GROUND / MIXED / OBSTACLE when present) |
| `adaptive_cells.ground_elevation`, `z_mean` | Terrain height; flat grid if neither is present |
| `adaptive_cells.obstacle_elevation`, counts, `level`, `ix`/`iy` | Inspector / extrusion when present |
| `obstacle_cells` | Optional overlay (never synthesized) |
| `trajectory.json` `samples` | Optional ego trajectory overlay |
| `tracks`, `world_objects` | Optional object footprints |
| `world_points` | Optional world cloud overlay |
| `pose.ego_xy` | Ego marker |
| `metrics.adaptive_cells`, `live_tracks` | Info strip counts |
| `world_points_frame` / `manifest.world_frame` | Map frame label (`lidar_frame_0`) |

## Visualization modes

1. **Terrain** — colormap from real elevations; flat fallback if none.
2. **Semantic** — color by exported `semantic_class` only.
3. **Resolution** — color by exported `resolution` bins (fine → coarse).
4. **Obstacles** — highlight cells whose exported class is OBSTACLE / MIXED.

Camera: orbit rotate, zoom, pan, plus Iso / Top / Side presets.

Frame navigation: **− / +**, Previous / Next, optional Play. Not a slider.

## Performance

- On-demand frame fetch; LRU `FrameCache` (12).
- Adaptive cells rendered as one `InstancedMesh`.
- Geometries and materials disposed on unmount.
- World-points overlay is off by default.
- The `/map` route is lazy-loaded so Three.js is not on every page.

## Missing data

| Condition | UI |
|-----------|----|
| Dataset listed, no JSON | “This dataset has not been exported yet.” |
| Collection unavailable | Error from registry probe; no fake scene |
| Missing `manifest.json` / frame file | Retryable load error |
| Empty / missing `adaptive_cells` | “Adaptive grid data is not available for this export.” |
| No elevation fields | Flat grid + “Elevation unavailable.” |
| No `trajectory.json` | Trajectory overlay disabled |
| Empty `obstacle_cells` | Overlay disabled |

Path planning is not part of this phase. Visual color polish is deferred to Phase 8.

# Phase 7 — Path planning demonstration + synthetic map crash

## Path planning (`/planning`)

This page is **not** an ORBIT runtime planner. The Python pipeline still does
not emit a driving path. The website runs A* on **exported** environment JSON
and labels the route as computed in the browser.

```
exported adaptive_cells + obstacle_cells + pose.ego_xy
        ↓
website occupancy graph (GROUND free; OBSTACLE / MIXED / obstacle_cells blocked)
        ↓
A* (Euclidean heuristic, cell-center graph)
        ↓
polyline drawn only if a path exists
```

URL query matches Phase 5 (`dataset`, `scene` / `sequence` / `environment`).
One-frame collections (typical synthetic export) are supported: Previous/Next
are disabled, Play is disabled.

### Exported fields used

| Field | Role |
|-------|------|
| `adaptive_cells.center`, `resolution`, `semantic_class` | Graph nodes / adjacency / traversability |
| `adaptive_cells.obstacle_count` | Blocked if > 0 |
| `obstacle_cells` | Extra blocked footprints |
| `pose.ego_xy` | Default start |
| `metrics.adaptive_cells` | Counts in the info strip |
| `world_points_frame` / `manifest.world_frame` | Map frame label |

Tracks and world objects are displayed only as optional map overlays elsewhere;
they are not a planning cost layer.

### Adaptive grid → planning graph

Each exported cell is a node. Two **unblocked** cells are neighbors if their
axis-aligned footprints touch (centers within `(res_i + res_j) / 2`). That
preserves varying resolution without rasterizing a new uniform grid in the
browser. Unmapped space is not a node, so the planner cannot invent free space.

Blocked if any of: `semantic_class` is `OBSTACLE` or `MIXED`; `obstacle_count > 0`;
footprint overlaps an exported `obstacle_cell`.

Start and goal snap to the nearest **traversable** cell within about two cell
widths. Click-to-set goal (default) or start. If A* fails, **no polyline is
drawn**.

## Synthetic `/map` crash (root cause)

Inspected a real `--source synthetic` export (`scene_id: synthetic`,
`frame_indices: [0]`, `frame_0000.json` ≈ 70 MB):

- Schema matches nuScenes: `center`, `resolution`, `semantic_class`, elevations.
- **151,924** `adaptive_cells` (no NaN/missing centers), 690 `obstacle_cells`.
- Single trajectory sample. Valid `pose.ego_xy`.

The crash was **not** a missing-field schema mismatch. Phase 6 created one
`InstancedMesh` instance per exported cell and walked every cell for camera
bounds / coloring. ~1.5×10⁵ boxes plus a 70 MB `JSON.parse` froze/crashed the
tab. One-frame navigation was already valid (`FRAME 00 / 00`) but easy to
overlook.

### Fix

- Deterministic even-index visualization sample (`MAX_RENDER_CELLS = 28000`).
  Metrics and planning still use the **full** export.
- Skip non-finite cells/camera targets.
- Normalize missing arrays on load.
- Render error boundary.
- Label one-frame exports.

Visual contrast / black cells vs blue background is **not** changed here
(Phase 8).

Copy synthetic JSON (gitignored) to `web/public/data/synthetic/` so
`/map?dataset=synthetic&environment=environment-01` can probe `manifest.json`.


