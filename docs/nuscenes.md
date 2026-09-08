# nuScenes v1.0-mini adapter

ORBIT can ingest **nuScenes v1.0-mini LIDAR_TOP keyframes** as a second
dataset backend. KITTI / SemanticKITTI remains unchanged.

The extract **must stay outside Git**. Do not copy it into `data/`, do
not symlink it into the repo, and do not commit blobs.

## Dataset location

On the development PC the official mini extract is:

```text
D:\ORBIT_DATA\nuscenes\
  samples\
  sweeps\
  maps\
  v1.0-mini\
```

Pass that path at runtime:

```powershell
python src/orbit_system.py `
    --source nuscenes `
    --dataset-root "D:\ORBIT_DATA\nuscenes" `
    --scene scene-0061 `
    --start 0 `
    --end 9

python src/orbit_system.py `
    --source nuscenes `
    --dataset-root "D:\ORBIT_DATA\nuscenes" `
    --scene scene-0061 `
    --start 0 `
    --end 9 `
    --dashboard
```

Alternatively set `ORBIT_NUSCENES_ROOT` to the extract. There is no
hardcoded `D:` default in code.

List scene names from `v1.0-mini/scene.json` (`name` field, e.g.
`scene-0061`). If `--scene` is omitted, ORBIT uses the first name in
sorted order.

## What is implemented

- **Keyframe-only** LIDAR_TOP: `scene.first_sample_token` →
  `sample.next` → `last_sample_token`. Each sample contributes its
  keyframe `LIDAR_TOP` (`is_key_frame` true).
- Integer ORBIT frames `0 .. N-1`.
- `.pcd.bin` as float32 with five columns `x, y, z, intensity, ring`.
  Perception receives XYZ only.
- Ego motion from `calibrated_sensor` + `ego_pose` (not annotations).

**Sweeps are not played back.** `sweeps/` files are ignored so the
stream stays comparable to KITTI frame indices. The loader is structured
so a later `sample_data.prev/next` chain can be added without changing
perception.

## Coordinate frames

nuScenes chain (real dataset extras):

```text
LIDAR_TOP  →  calibrated_sensor  →  ego vehicle  →  ego_pose  →  global
```

Quaternion order is `(w, x, y, z)`.

ORBIT’s world / reference frame is still **the first processed LiDAR
keyframe (index 0)**, not nuScenes global:

```text
T_frame0_from_current =
    inv(T_global_from_lidar(0))
    @ T_global_from_lidar(current)
```

Dashboard trajectory `ego_xy` is that transform’s origin in frame 0.

## What is not used

- `sample_annotation.json` (no GT boxes as ORBIT detections)
- lidarseg / panoptic (not in mini by default; unused even if present)
- SemanticKITTI labels
- Map rasters under `maps/`

This is the same geometric prototype as KITTI: RANSAC ground, adaptive
grid, geometric proposals, tracker, world model.

## Tests

Loader tests use **tiny synthetic JSON + fake `.pcd.bin` files** under
pytest’s `tmp_path`. They never read `D:\ORBIT_DATA`.
