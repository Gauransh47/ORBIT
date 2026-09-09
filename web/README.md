# ORBIT website

Separate React application. It does **not** replace
`src/visualization/dashboard.py`.

Phase 4 is the Interactive ORBIT Explorer (`/demo`). It only visualizes
exported JSON. It does not run the Python pipeline.

## Local

```bash
# After Phase 3 export:
mkdir -p web/public/data/scene-0061
cp exported_data/scene-0061/*.json web/public/data/scene-0061/

cd web
npm install
npm run dev
```

Open http://localhost:5173/demo

```bash
npm run build
npm run preview
```

## Vercel

Root directory: `web`

Build command: `npm run build`

Output directory: `dist`

SPA rewrites are in `vercel.json`.

Copy exported JSON to `web/public/data/scene-0061/` so `/data/scene-0061/manifest.json`
is served statically. Do not bundle frames into JS.

## Stack

React, TypeScript, Vite, Tailwind, Framer Motion, Three.js / R3F / Drei.
