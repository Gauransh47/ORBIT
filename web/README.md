# ORBIT website

Separate React application. It does **not** replace
`src/visualization/dashboard.py`.

The Interactive Demo (`/demo`) visualizes exported JSON only.

## Local

```bash
mkdir -p web/public/data/nuscenes/scene-0061
cp exported_data/scene-0061/*.json web/public/data/nuscenes/scene-0061/
# legacy also works: web/public/data/scene-0061/

cd web
npm install
npm run dev
```

Open http://localhost:5173/demo?dataset=nuscenes&scene=scene-0061

Dedicated 2.5D map explorer: http://localhost:5173/map?dataset=nuscenes&scene=scene-0061

That page visualizes exported `adaptive_cells` (center, resolution,
semantic_class, elevations). Large synthetic grids are subsampled for drawing
only (full JSON is unchanged).

Path planning: http://localhost:5173/planning?dataset=nuscenes&scene=scene-0061

`/planning` is a **browser A\* demonstration** on exported GROUND cells.
Choose Start, then Destination, then Locate path. Route playback visualizes
that computed path only. It is not the Python ORBIT runtime planner and not
live vehicle control.

The map explorer includes honest export metrics (point/cell counts, resolution
distribution) when those fields exist. It does not invent FPS, latency, or
model accuracy.

This website is a prototype visualization client for the intended DRDO/iDEX
system. PointNet++ and sparse CNN perception are planned future work and are
not implemented.

Synthetic export (one frame, often >100k cells):

```bash
PYTHONPATH=src python -m web_export.export_orbit_data --source synthetic --output exported_data
mkdir -p web/public/data/synthetic
cp exported_data/synthetic/*.json web/public/data/synthetic/
```

```bash
npm run build
npm run preview
```

Dataset registry: `web/public/data/datasets.json` (see `docs/web.md` Phase 5).

## Vercel

In the Vercel project:

| Setting | Value |
|---------|--------|
| Root directory | `web` |
| Build command | `npm run build` |
| Output directory | `dist` |
| Environment variables | optional `VITE_ORBIT_DATA_URL` (see below) |

`vercel.json` rewrites unmatched routes to `index.html` (`/`, `/demo`, `/map`, `/planning`).

Committed static data is only `public/data/datasets.json` plus the small `scene-fixture` export when present. Full scene JSON is gitignored.

### Local `/data` (default)

`npm run dev` / a Vercel build **without** `VITE_ORBIT_DATA_URL` fetches:

```text
/data/datasets.json
/data/scene-fixture/manifest.json
/data/nuscenes/scene-0061/…
```

from `web/public/data/`. Collections without a real `manifest.json` stay **Not exported yet**. Copy exporter output into `public/data/` as above for a full local scene.

### Production: externally hosted exports

To serve real gitignored exports in production, upload the **same directory layout** as `web/public/data/` to any static HTTPS host (R2, S3, Vercel Blob public URL, etc.). Then set a **build-time** env var on Vercel:

| Name | Example |
|------|---------|
| `VITE_ORBIT_DATA_URL` | `https://your-dataset-host.example.com/orbit-data` |

Vite inlines this at `npm run build`. After changing it, **redeploy**. Do not put credentials in this variable; the host must be publicly readable JSON.

The client tries the external origin first, then same-origin `/orbit-data/…` (Vite/Vercel proxy), then `/data/…`. `scene-fixture` on Vercel still works if the host is missing that collection.

Public R2 (`*.r2.dev`) often omits CORS headers, so a direct `fetch` fails in the browser even though the URL opens in a tab. `web/vercel.json` rewrites `/orbit-data/:path*` to the public R2 origin; keep that destination in sync with `VITE_ORBIT_DATA_URL`. Local `npm run dev` uses the Vite proxy when the env var is set.

Expected host layout (same paths as the exporter / `datasets.json`):

```text
<VITE_ORBIT_DATA_URL>/
  datasets.json
  scene-fixture/{manifest,trajectory,frame_*.json}
  nuscenes/scene-0061/{manifest,trajectory,frame_*.json}
  scene-0061/                          # legacy probe path
  synthetic/environment-01/{manifest,trajectory,frame_*.json}
  synthetic/                           # legacy probe path
  semantic-kitti/sequence-00/…         # when exported
```

Each collection still needs `manifest.json` with a `files` array; frames are `frame_XXXX.json` listed there.

### CORS

Direct `fetch` to R2 still needs `Access-Control-Allow-Origin` for the Vercel site (and `http://localhost:5173` if you fetch R2 from local). If the bucket does not send CORS headers, the same-origin `/orbit-data` proxy is used instead. Optional bucket CORS:

```text
Access-Control-Allow-Origin: https://orbit-bice-six.vercel.app
Access-Control-Allow-Methods: GET, HEAD
Access-Control-Allow-Headers: Content-Type
```

## Stack

React, TypeScript, Vite, Tailwind, Framer Motion, Three.js / R3F / Drei.
