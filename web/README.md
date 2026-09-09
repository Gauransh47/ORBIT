# ORBIT website

Separate React application. It does **not** replace
`src/visualization/dashboard.py`.

Phase 2 is the immersive landing experience. Real PipelineState JSON is Phase 3.

## Local

```bash
cd web
npm install
npm run dev
```

Open http://localhost:5173

```bash
npm run build
npm run preview
```

## Vercel

Root directory: `web`

Build command: `npm run build`

Output directory: `dist`

SPA rewrites are in `vercel.json`.

Exported JSON (Phase 3+) should be copied to `web/public/data/` for static hosting.

## Stack

React, TypeScript, Vite, Tailwind, Framer Motion, Canvas hero.
Three.js / R3F / Recharts remain installed for later phases.
