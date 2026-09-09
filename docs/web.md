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
  src/                    # Python pipeline (untouched in Phase 1)
  src/web_export/         # Phase 3 exporter
  web/                    # Vite + React (this phase)
  exported_data/          # gitignored dumps; copy subset to web/public/data
  docs/web.md
```

## Phase plan

1. **Foundation (this PR)** — Vite app, theme, nav, landing, placeholders.
2. Landing polish — more immersive hero / pipeline on home.
3. Exporter — real PipelineState JSON.
4. Interactive demo — PREV/NEXT, titled views from JSON.
5. 2.5D explorer — R3F rotate/zoom/views.
6. Path planning page — labeled planned extension (skeleton already linked).

Do not invent metrics in the demo once JSON exists.
