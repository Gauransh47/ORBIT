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

`/planning` runs website-side A* on exported GROUND cells. It does not claim
an ORBIT runtime planner.

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

Root directory: `web`. Copy exported JSON into `web/public/data/` so
`/data/datasets.json` and scene manifests are static files.

## Stack

React, TypeScript, Vite, Tailwind, Framer Motion, Three.js / R3F / Drei.
