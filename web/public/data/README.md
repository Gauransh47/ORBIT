# Frontend exported data

Runtime JSON. Not bundled into JavaScript.

Vite serves **only** `web/public/` as the site root. Exports must live under
`web/public/data/…` (URL `/data/…`) **or** on a static host configured with
`VITE_ORBIT_DATA_URL` (same relative paths). See `web/README.md`.

## Registry

`datasets.json` (committed) lists datasets and collection ids. Availability is
**probed** by fetching `manifest.json` — missing exports are “Not exported yet”.

## Preferred layout

```
web/public/data/
  datasets.json
  nuscenes/scene-0061/{manifest,trajectory,frame_*.json}
  semantic-kitti/sequence-00/…
  synthetic/environment-01/…
```

## Backwards compatible (Phase 3/4)

```
web/public/data/scene-0061/
```

still works. The registry lists both paths for scene-0061.

## Copy nuScenes

```bash
mkdir -p web/public/data/nuscenes/scene-0061
cp exported_data/scene-0061/*.json web/public/data/nuscenes/scene-0061/
# or legacy:
mkdir -p web/public/data/scene-0061
cp exported_data/scene-0061/*.json web/public/data/scene-0061/
```
