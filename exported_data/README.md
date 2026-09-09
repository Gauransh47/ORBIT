# exported_data

Written by `python -m web_export.export_orbit_data`. Do not commit scene dumps
(JSON is gitignored).

Layout:

```
exported_data/<scene-id>/
  manifest.json
  trajectory.json
  frame_0000.json
  frame_0001.json
  ...
```

Copy a visualization subset into `web/public/data/` for Vercel (Phase 4+).
