from pathlib import Path
import sys


SRC_DIR = Path(__file__).resolve().parents[1]
REPO_ROOT = SRC_DIR.parent

SYNTHETIC_SCENE = REPO_ROOT / "data" / "synthetic_scene.ply"
SEMANTIC_KITTI_ROOT = REPO_ROOT / "data" / "semantic_kitti"


def ensure_src_on_path():
    """
    Insert src/ on sys.path so scripts can use package-style imports
    such as `from mapping.adaptive_grid import AdaptiveGrid`.

    Existing modules already do this locally. New tools should call
    this helper instead of copying the boilerplate.
    """

    src = str(SRC_DIR)

    if src not in sys.path:
        sys.path.insert(0, src)

    return SRC_DIR
