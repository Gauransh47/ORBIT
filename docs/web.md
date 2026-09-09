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
  src/web_export/         # Phase 3 exporter (not implemented yet)
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
2. **Landing (this phase)** — cinematic hero, glance cards, scroll pipeline,
   conceptual demo / map / planning previews, tech foundation.
3. **Exporter** — real PipelineState JSON (`src/web_export/`).
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

## Current limitations

- No `web_export` module.
- Demo and map pages are still placeholders.
- Hero and sketches are synthetic, not exported PipelineState.
- Path planning remains a planned extension.

## What remains for Phase 3

Implement `src/web_export/export_orbit_data.py` to write
`exported_data/<scene>/metadata.json`, `trajectory.json`, and
`frames/frame_XXX.json` from real `OrbitSystem.process_frame` output.
Do not invent detections or metrics.
