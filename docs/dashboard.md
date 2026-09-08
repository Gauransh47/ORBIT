# ORBIT Visual Intelligence Dashboard V2

This dashboard renders **real `PipelineState` objects** from
`OrbitSystem.process_frame`. It does not invent detections, tracks,
trajectories, or metrics.

It is a **prototype visualization**, not a production navigation
product. It does **not** display SemanticKITTI ground-truth semantics
or a learned traversability / segmentation model.

## Two coordinate systems

| View | Frame | What you should see |
|------|--------|---------------------|
| LIVE LiDAR SENSOR | Current LiDAR frame. Sensor at `(0, 0)`. | The scan from this frame only. Nearby geometry moves relative to the sensor as the slider changes. |
| GLOBAL WORLD MODEL | LiDAR **frame 0** (same world the tracker uses). | Environment stays put. The ego marker and trajectory move. |

`state.points` is already in the current sensor frame (range-filtered XYZ).
The live panel plots that cloud as-is.

`state.world_points` is a deterministic subsample of the same scan
transformed into LiDAR frame 0. The world panel **accumulates** those
clouds from frame 0 through the selected slider index.

## Ego trajectory source (not fabricated)

When `--source kitti`, `OrbitSystem` attaches `EgoMotionCompensator`
(`sequences/<seq>/poses.txt` + `calib.txt` `Tr`).

When `--source nuscenes`, it attaches `NuScenesEgoMotion` (LIDAR_TOP
keyframe `calibrated_sensor` + `ego_pose`). See [nuscenes.md](nuscenes.md).

For each frame:

1. `T = ego_motion.transform(frame, 0)` — current LiDAR → LiDAR frame 0  
   (same transform the tracker applies to detections).
2. `ego_xy` = translation of the sensor origin `(0,0,0)`.
3. `ego_heading_rad` = `atan2` of LiDAR **+X** (forward) after `T`.

`pose_source` on each state is one of:

- `kitti_poses_lidar_to_frame0` — KITTI odometry was loaded
- `nuscenes_ego_pose_lidar_to_frame0` — nuScenes ego/lidar extras
- `identity_no_odometry` — synthetic / no compensator (trajectory stays at origin)
- `identity_empty_poses` — compensator present but no pose rows

The dashboard never interpolates or invents motion. If KITTI poses
are missing, the global path will not move — that is correct.

## Run

```bash
# KITTI sequence 00, frames 0–9, interactive dashboard
PYTHONPATH=src python src/orbit_system.py \
    --source kitti --sequence 00 --start 0 --end 9 --dashboard

# equivalent: --show
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

# synthetic PLY (identity pose — no KITTI trajectory)
PYTHONPATH=src python src/orbit_system.py --source synthetic --dashboard
```

Dependencies: `numpy`, `open3d`, `matplotlib` (see `requirements.txt`).

The **Frame** slider restyles every panel from stored states
(live scan, world ego marker, terrain, tracks, metrics, timeline).

## Panels

| Panel | Source |
|-------|--------|
| Live LiDAR | `state.points` in the current ego frame. Default colour: point Z (`turbo`). Optional `--lidar-color terrain`: `state.ground.mask`. Sensor marker at origin; +X forward. Confirmed tracks inverse-transformed into the ego frame. |
| Global world | Accumulated `state.world_points` (frame 0) + confirmed tracks + ego trajectory from `state.ego_xy`. |
| Terrain elevation | Square patches from `AdaptiveCell.ground_elevation` and `resolution`. |
| Terrain structure | `AdaptiveCell.semantic_class` only: GROUND / MIXED / OBSTACLE. Not drivability. |
| Status | Large figures from `state.metrics`. |
| Tracking | Live / confirmed / world counts; at most 8 confirmed long-lived tracks. |
| Timeline | Real `live_tracks` and `confirmed_tracks` per processed frame. |

### Cell class colours (geometric, not learned)

| Cell class | Meaning in this prototype | Colour |
|------------|---------------------------|--------|
| GROUND | Only RANSAC-ground points in the cell | Green |
| MIXED | Ground and obstacle points in the same cell | Amber |
| OBSTACLE | Only non-ground points in the cell | Red |

## Limitations

- Live and world clouds are subsampled for speed; metrics still report full mapped counts.
- Terrain panels are in the **current ego grid**, not a fused global DEM.
- Synthetic source has no odometry; the ego path stays at the origin.
- Heading assumes KITTI LiDAR +X is forward.
- No learned semantics, PointNet++, sparse CNN, or drivability model.
- `--show` opens this dashboard. `visualization.pipeline_view.render_topdown` remains for tests.
