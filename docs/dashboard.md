# ORBIT Visual Intelligence Dashboard V3

This dashboard renders **real `PipelineState` objects** from
`OrbitSystem.process_frame`. It does not invent detections, tracks,
trajectories, or metrics.

It is a **prototype visualization**, not a production HUD. Cell class
is geometric GROUND / MIXED / OBSTACLE — not learned drivability.

## Layout

Two **hero** panels dominate the top:

| Panel | Frame | Story |
|-------|--------|--------|
| LIVE LiDAR | Current sensor XYZ, origin at `(0,0)` | What the LiDAR sees now |
| GLOBAL WORLD MODEL | LiDAR **frame 0** | Where the vehicle has travelled |

Below: terrain elevation patches, geometric cell class, motion timeline,
and compact metric cards from `state.metrics`. TRAVEL is the path length
of real `ego_xy` samples — not a hardcoded scene-0061 distance.

ORBIT world is **LiDAR frame 0**, not nuScenes global. The header REF
line states that explicitly.

## Run

```bash
PYTHONPATH=src python src/orbit_system.py \
    --source kitti --sequence 00 --start 0 --end 9 --dashboard
```

```powershell
python src/orbit_system.py `
    --source nuscenes `
    --dataset-root "D:\ORBIT_DATA\nuscenes" `
    --scene scene-0061 `
    --start 0 `
    --end 19 `
    --dashboard
```

PNG export: `--save-figures output/dashboard`

Optional live colour: `--lidar-color terrain` (RANSAC ground mask).

The **Frame** slider restyles every panel from stored states.

## Limitations

- Clouds are subsampled for speed; metrics still use full counts.
- Terrain panels are the **current ego grid**, not a fused global DEM.
- Speed is not shown (timestamps are not on `PipelineState`; we do not invent it).
- No learned semantics.
