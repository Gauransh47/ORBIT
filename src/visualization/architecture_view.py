"""
Static architecture diagram of the implemented ORBIT prototype.

This is not live pipeline output.
"""

from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch

from visualization.color_schemes import ACCENT, BG, EDGE, MUTED, PANEL, TEXT


STAGES = [
    "LiDAR input",
    "Preprocessing (range < 100 m)",
    "Ground estimation (RANSAC)",
    "Adaptive 2.5D grid",
    "Terrain / obstacle cells",
    "Geometric object proposals",
    "Ego-motion → LiDAR frame 0",
    "Multi-object tracker",
    "Persistent world model",
    "Visualization dashboard",
]


def render_architecture_diagram(save_path=None, show=False):
    fig, ax = plt.subplots(figsize=(8.5, 11), facecolor=BG)
    ax.set_facecolor(BG)
    ax.set_xlim(0, 10)
    ax.set_ylim(0, 12)
    ax.axis("off")

    ax.text(
        5, 11.5,
        "ORBIT prototype data flow",
        ha="center", color=TEXT, fontsize=14, fontweight="bold",
    )
    ax.text(
        5, 11.05,
        "Implemented geometric pipeline  ·  not a learned semantic stack",
        ha="center", color=MUTED, fontsize=8,
    )

    y = 10.3
    for name in STAGES:
        box = FancyBboxPatch(
            (2.2, y - 0.55),
            5.6, 0.7,
            boxstyle="round,pad=0.04,rounding_size=0.12",
            facecolor=PANEL,
            edgecolor=ACCENT if name == STAGES[-1] else EDGE,
            linewidth=1.2,
        )
        ax.add_patch(box)
        ax.text(5, y - 0.20, name, ha="center", va="center", color=TEXT, fontsize=10)
        if y > 1.2:
            ax.annotate(
                "",
                xy=(5, y - 0.72),
                xytext=(5, y - 0.55),
                arrowprops=dict(arrowstyle="-", color=MUTED, lw=1.0),
            )
        y -= 0.95

    if save_path:
        Path(save_path).parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(save_path, facecolor=fig.get_facecolor(), bbox_inches="tight")

    if show:
        plt.show()
    else:
        plt.close(fig)

    return fig
