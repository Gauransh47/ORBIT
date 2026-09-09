# ORBIT website (Phase 1)

Separate React application. It does **not** replace
`src/visualization/dashboard.py`.

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

Exported JSON (later) should be copied to `web/public/data/` for static hosting.

## Stack

React, TypeScript, Vite, Tailwind, Framer Motion.
Three.js / R3F / Recharts are installed for later phases.
