# ORBIT

**Adaptive Variable Resolution 2.5D LiDAR Mapping for Dynamic Environment Perception**

Prototype / proof-of-concept toward the intended DRDO / iDEX Smart Vehicles system.

PointNet++ and Sparse CNN perception are **future work**. They are **not implemented**.

---

## 1. Project information

| | |
|---|---|
| **Project title** | ORBIT |
| **Problem title** | Adaptive Variable Resolution 2.5D Lidar Mapping for Dynamic Environment Perception |
| **Organization** | DRDO — Department of Defence Production / iDEX |
| **Theme** | Smart Vehicles |
| **Category** | Software |
| **Status** | Prototype / proof-of-concept |

---

## 2. Problem statement

Dense 3D LiDAR gives rich spatial information but is computationally expensive. Traditional 2D occupancy maps drop height that matters for curbs, potholes, terrain, and overhangs.

ORBIT explores a **foveated 2.5D** representation: higher spatial detail near the sensor, coarser cells farther away, while keeping elevation and geometric structure that a flat grid would lose.

Official task directions:

1. Terrain analysis (drivable vs non-drivable surfaces)
2. Object detection (static obstacles and dynamic objects)
3. Adaptive spatial representation (non-uniform resolution)

---

## 3. Overview

```text
Dataset / LiDAR
        ↓
ORBIT processing pipeline (Python)
        ↓
PipelineState
        ↓
JSON export
        ↓
Interactive web visualization
```

The browser **does not** run perception, mapping, or tracking. It visualizes exported JSON.

---

## 4. Current prototype

**Implemented / demonstrated**

- LiDAR-derived environment processing (geometric)
- Adaptive variable-resolution 2.5D grid
- Elevation / terrain representation from exported cell fields
- Obstacle representation (`semantic_class`, `obstacle_count`, `obstacle_cells`)
- Tracking / object footprints from exported `tracks` and `world_objects`
- Dataset-aware explorer (nuScenes, SemanticKITTI, ORBIT Synthetic)
- Interactive Demo, 2.5D Map, and foveated-structure overlay
- Website A* planning demonstration on exported occupancy
- Metrics derived only from export counts

**Not implemented**

- PointNet++, Sparse CNN, or any learned LiDAR segmentation
- Learned drivable-terrain classification
- Production Python runtime planner
- Live vehicle control
- Official FPS / latency / memory / accuracy evaluation

---

## 5. Architecture

```text
nuScenes / SemanticKITTI / synthetic LiDAR
        ↓
OrbitSystem.process_frame
        ↓
Geometric perception (RANSAC, clustering) + AdaptiveGrid + tracker
        ↓
PipelineState
        ├── Matplotlib dashboard (engineering console)
        └── web_export → JSON
                    ↓
              React website (visualization only)
                    ↓
              Browser A* demonstration
```

World frame is **LiDAR frame 0**, not nuScenes global.

---

## 6. Adaptive spatial representation

The Python mapper bins occupied space into cells whose size grows with range (`src/mapping/adaptive_grid.py`). Fine cells carry higher spatial detail near the sensor; coarser cells cover farther ranges.

The website draws the **exported** `resolution` values. Foveation rings use the **actual range extent** of each exported resolution around ego. Legends do not invent 10 m / 100 m radii unless those extents exist in the JSON.

When the Python mapper ran with its default rings, cell sizes are 5 cm / 10 cm / 25 cm / 50 cm over 0–10 / 10–25 / 25–50 / 50–100 m. That is mapper configuration, not a performance claim.

---

## 7. Dynamic environment representation

Exported tracks and world objects may include `track_id`, geometric `class_name`, `position`, `dimensions_xy`, `velocity_xy`, `motion_state`, hits, and related fields **when the pipeline wrote them**.

The website shows those fields in an object inspector and optional ID labels. It does not invent velocity, classes, or motion. `class_name` and `motion_state` are geometric/exported labels, not learned categories.

---

## 8. Path planning demonstration

```text
exported adaptive_cells + obstacle_cells
        ↓
browser occupancy graph
        ↓
Start → Destination → Locate path
        ↓
A* (browser)
        ↓
route + playback only if PATH FOUND
```

This is a **website-side planning demonstration using exported ORBIT environment data**. It is **not** the production ORBIT runtime planner and not live navigation.

---

## 9. Datasets

| Dataset | Architecture | Public git / typical deploy |
|---------|--------------|-----------------------------|
| nuScenes | Scenes (`scene-0061`, `scene-fixture`, …) | Small `scene-fixture` JSON may be committed; full scenes stay local |
| SemanticKITTI | Sequences | Listed until exported JSON exists — no invented sequence |
| ORBIT Synthetic | Environments | Often one large frame; **not** committed (too large) |

Registry: `web/public/data/datasets.json`. Missing `manifest.json` → honest “Not exported yet”.

Copy a full export locally:

```bash
mkdir -p web/public/data/nuscenes/scene-0061
cp exported_data/scene-0061/*.json web/public/data/nuscenes/scene-0061/
```

---

## 10. Web platform

| Route | Contents |
|-------|----------|
| `/` | Landing |
| `/demo` | Interactive Demo (LiDAR, world, grid, objects) |
| `/map` | 2.5D Map (terrain / semantic / resolution / obstacles, adaptive structure) |
| `/planning` | Start / Destination / Locate path / playback |
| `/pipeline`, `/about`, `/technology` | System description |

---

## 11. Metrics

Shown only when JSON fields exist: input points, exported points, adaptive cells, point-to-cell ratio, resolution levels, fine/coarse counts, live tracks, frame index, exported frames.

Not shown: accuracy, FPS, latency, memory savings.

---

## 12. Technology stack

- **Pipeline:** Python 3, NumPy, Open3D, Matplotlib, scikit-learn (DBSCAN sidecar)
- **Tests:** pytest
- **Website:** React, TypeScript, Vite, Tailwind CSS, Framer Motion, Three.js / React Three Fiber
- **Export:** `python -m web_export.export_orbit_data`
- **Deploy target:** Vercel (static Vite build)

---

## 13. Repository structure

```text
ORBIT/
├── README.md
├── SUBMISSION_GUIDE.md
├── requirements.txt
├── docs/                      # technical documentation
├── assets/screenshots/        # prototype screenshots
├── submission/                # PPT and demo-video placeholders
├── src/                       # Python ORBIT pipeline
├── tests/
├── web/                       # Vite + React visualization
│   ├── vercel.json
│   └── public/data/           # datasets.json (+ optional small fixture)
└── exported_data/             # local exporter output (JSON gitignored)
```

---

## 14. Installation

### Website (visualization)

```bash
cd web
npm install
npm run dev
```

Open http://localhost:5173/ — also `/demo`, `/map`, `/planning`.

```bash
cd web
npm run build
npm run preview
```

### Python pipeline

```bash
python3 -m venv venv
source venv/bin/activate          # Windows: venv\Scripts\activate
pip install -r requirements.txt
PYTHONPATH=src python -m pytest tests/
```

Dataset-dependent pipeline commands: [docs/development.md](docs/development.md).

---

## 15. Deploy (Vercel)

| Setting | Value |
|---------|--------|
| Root directory | `web` |
| Build command | `npm run build` |
| Output directory | `dist` |
| Environment variables | none required |

`web/vercel.json` rewrites unknown paths to `index.html` so `/demo`, `/map`, and `/planning` work as an SPA.

A public deploy includes `datasets.json` and, if present, the small `scene-fixture` export. Large nuScenes / synthetic frames stay gitignored. Reviewers who need a full scene should run locally after copying JSON into `web/public/data/` as above.

---

## 16. Screenshots

| | |
|---|---|
| Landing | [assets/screenshots/01-landing.png](assets/screenshots/01-landing.png) |
| Interactive Demo | [assets/screenshots/02-interactive-demo.png](assets/screenshots/02-interactive-demo.png) |
| 2.5D Map | [assets/screenshots/03-adaptive-grid-map.png](assets/screenshots/03-adaptive-grid-map.png) |
| Synthetic / foveation | [assets/screenshots/04-synthetic-foveation.png](assets/screenshots/04-synthetic-foveation.png) |
| Path planning | [assets/screenshots/05-path-planning.png](assets/screenshots/05-path-planning.png) |

Notes: [assets/screenshots/README.md](assets/screenshots/README.md). PPT and demo video: [submission/](submission/).

---

## 17. Documentation

| Document | Contents |
|----------|----------|
| [docs/project-context.md](docs/project-context.md) | Official statement vs current status |
| [docs/architecture.md](docs/architecture.md) | Python architecture |
| [docs/development.md](docs/development.md) | Commands and tests |
| [docs/web.md](docs/web.md) | Website and JSON export |
| [docs/dashboard.md](docs/dashboard.md) | Matplotlib dashboard |
| [docs/nuscenes.md](docs/nuscenes.md) | nuScenes adapter |
| [web/README.md](web/README.md) | Website run notes |
| [SUBMISSION_GUIDE.md](SUBMISSION_GUIDE.md) | SIH packaging |

---

## 18. Future work

- PointNet++ and Sparse CNN (learned LiDAR perception)
- Learned semantic segmentation and terrain / drivable classification
- Improved learned object classification
- Production runtime planner
- Real-time vehicle integration
- Formal latency, FPS, memory, and accuracy evaluation

---

## 19. Prototype status

ORBIT is a **prototype / proof-of-concept**. It demonstrates an adaptive variable-resolution **2.5D spatial representation** from LiDAR-derived / exported data, plus interactive visualization and a browser planning demonstration.

It is not a completed DRDO/iDEX production system. Deep learning remains planned future work.
