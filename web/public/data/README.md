# Frontend scene data

The explorer loads JSON at runtime from this folder. It does **not** bundle
frames into the JavaScript build.

## scene-0061 (default)

Copy the Phase 3 export here:

```text
exported_data/scene-0061/manifest.json
exported_data/scene-0061/trajectory.json
exported_data/scene-0061/frame_0000.json
…
exported_data/scene-0061/frame_0019.json
        ↓
web/public/data/scene-0061/
```

From the repo root:

```bash
mkdir -p web/public/data/scene-0061
cp exported_data/scene-0061/manifest.json \
   exported_data/scene-0061/trajectory.json \
   exported_data/scene-0061/frame_*.json \
   web/public/data/scene-0061/
```

Then open `/demo`.

JSON files are gitignored (they are large). This README is the placeholder.

Optional other scene:

`/demo?scene=your-scene-id` loads `/data/<id>/manifest.json`.
