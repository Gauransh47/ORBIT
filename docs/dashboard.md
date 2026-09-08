# ORBIT Visual Intelligence Dashboard

This dashboard renders **real `PipelineState` objects** from
`OrbitSystem.process_frame`. It does not invent detections, tracks,
or metrics.

It is a **prototype visualization**, not a production navigation
product. It does **not** display SemanticKITTI ground-truth semantics
or a learned traversability / segmentation model.

## Run

```bash
# KITTI sequence 00, frames 0–9, interactive dashboard
PYTHONPATH=src python src/orbit_system.py \
    --source kitti --sequence 00 --start 0 --end 9 --dashboard

# equivalent: --show now opens the same dashboard
PYTHONPATH=src python src/orbit_system.py \
    --source kitti --sequence 00 --start 0 --end 9 --show

# 3D LiDAR coloured by RANSAC ground vs non-ground
PYTHONPATH=src python src/orbit_system.py \
    --source kitti --sequence 00 --start 0 --end 9 \
    --dashboard --lidar-color terrain

# PNG frames (no GUI)
PYTHONPATH=src python src/orbit_system.py \
    --source kitti --sequence 00 --start 0 --end 9 \
    --save-figures output/dashboard

# static architecture diagram (not live data)
PYTHONPATH=src python src/orbit_system.py \
    --save-architecture output/orbit_architecture.png

# synthetic PLY
PYTHONPATH=src python src/orbit_system.py --source synthetic --dashboard
```

Dependencies: `numpy`, `open3d`, `matplotlib` (see `requirements.txt`).
No Streamlit or Plotly is required.

Interactive playback: after all selected frames are processed, a
**Frame** slider steps through stored pipeline states. Tracking
identity is whatever the tracker produced while those frames ran
in order.

## Panels (all fed by real pipeline outputs)

| Panel | Source |
|-------|--------|
| 3D LiDAR | `state.points` XYZ. Colour **elevation**: point Z (`viridis`). Colour **terrain**: `state.ground.mask` (RANSAC inliers vs remainder). |
| BEV | Same points + `state.proposals` boxes + `state.tracks` + `WorldObject.history` trails. Origin is LiDAR frame 0. |
| Elevation map | Adaptive-grid cells with `ground_elevation` (mean ground Z in the cell). |
| Traversability | `AdaptiveCell.semantic_class` only: GROUND / MIXED / OBSTACLE. |
| Tracking | Live / confirmed / world counts from the tracker and world model. IDs labelled for nearest confirmed tracks (capped). |
| Metrics | `state.metrics` from `OrbitSystem` (points, cells, proposals, tracks, latency). |
| Timeline | Live-track count per processed frame. |

### Object colours

Geometric detector classes (not KITTI names):

- VEHICLE-LIKE — blue
- WALL — gold
- POLE — yellow
- OBSTACLE — red

Labels are **selective**: nearest confirmed tracks (max 8) and
highest-confidence proposals (max 5). Every proposal still has a
box; IDs are not drawn on all of them.

### Traversability derivation (not learned)

| Cell class | Meaning in this prototype | Colour |
|------------|---------------------------|--------|
| GROUND | Only RANSAC-ground points in the cell | Green (treat as traversable) |
| MIXED | Ground and obstacle points in the same cell | Yellow (caution) |
| OBSTACLE | Only non-ground points in the cell | Red (blocked) |

This is a **deterministic display** of the existing 2.5D cell
classification. It is not drivability, not a neural model, and not
SemanticKITTI terrain labels.

## Limitations

- Dashboard Matplotlib 3D is a subsample of the cloud (25k points) for
  speed; metrics still report full mapped counts.
- `--show` opens the dashboard, not the old single scatter plot.
  `visualization.pipeline_view.render_topdown` remains for tests.
- KITTI mode still uses geometric perception only (no GT labels).
- Ego-motion world frame is LiDAR frame 0.
