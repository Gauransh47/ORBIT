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
3. **Exporter (this phase)** — real PipelineState JSON (`src/web_export/`).
4. **Interactive demo** — PREV/NEXT, titled views from JSON.
5. **2.5D explorer** — R3F rotate/zoom/views on exported cells.
6. **Path planning page** — still a planned extension unless the Python
   runtime adds a planner.

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

- Demo and map pages are still placeholders (Phase 4 / 5).
- The landing hero and sketches are conceptual, not exported PipelineState.
- Path planning remains a planned extension.

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

## What remains for Phase 4

Load these JSON files in the Interactive Demo (PREV/NEXT, titled views).
Do not invent metrics if a file is missing.
