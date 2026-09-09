# ORBIT documentation

ORBIT is a **prototype / proof-of-concept** toward an iDEX / DRDO Smart
Vehicles problem: **adaptive variable-resolution 2.5D LiDAR mapping**.
Learned LiDAR perception (PointNet++, sparse CNNs) is intended future
work, not current code.

These documents separate **what the official problem asks for** from
**what this repository actually implements**. Intended capabilities
are never described as finished unless they exist in the code.

| Document | Contents |
|----------|----------|
| [Project context](project-context.md) | Official statement, current status, gap to the expected solution |
| [Architecture](architecture.md) | How the **current** code is structured and how data flows |
| [Development](development.md) | Setup, how to run scripts, tests, conventions |
| [Dashboard](dashboard.md) | Visual Intelligence dashboard (real pipeline outputs) |
| [nuScenes](nuscenes.md) | v1.0-mini LIDAR_TOP adapter (external dataset) |
| [Website](web.md) | Separate React/Vite presentation layer (Phase 1) |

There is no packaged Python distribution and no root README.
Application code lives under `src/`.
