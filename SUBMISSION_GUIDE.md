# SIH submission packaging

This repository follows the NSUT SIH reference layout where practical:

- Overview → `README.md`
- Technical docs → `docs/`
- Screenshots → `assets/screenshots/`
- Presentation and demo-video **links/files** → `submission/`
- Python pipeline → `src/`
- Website → `web/`

Do not copy sample CropGuard content from the reference template. Keep ORBIT’s application layout (`src/`, `web/`) unchanged.

## Before submitting the GitHub link

1. Repository is public (or otherwise accessible to reviewers).
2. `README.md` describes the **current prototype**, not future deep learning as done.
3. Screenshots in `assets/screenshots/` match the current UI (recapture if needed).
4. Add the final PPT in `submission/` **or** a viewer link in `submission/PRESENTATION.md`.
5. Add a demo-video URL in `submission/DEMO.md` if you record one.
6. Deploy the website on Vercel if a public URL is required (root directory `web`). Do not commit huge frame JSON.
7. Do not commit secrets, `.env` files, `venv/`, `node_modules/`, or raw datasets under `data/`.

## Public website data

Large exports are gitignored. A public Vercel deploy will show **Not exported yet** for collections without JSON. The small `web/public/data/scene-fixture/` export may be committed so reviewers can open a real (tiny) scene. Full nuScenes / synthetic grids should be demonstrated locally or via a separately hosted data drop — not by stuffing 70+ MB frames into git.
