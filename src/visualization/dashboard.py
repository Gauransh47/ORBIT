"""
ORBIT Visual Intelligence Dashboard.

Consumes a sequence of real PipelineState objects. Does not rerun
ground estimation, grid construction, or detection.
"""

from pathlib import Path
from typing import List, Optional, Sequence

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec
from matplotlib.lines import Line2D
from matplotlib.patches import Rectangle

from visualization.color_schemes import (
    ACCENT,
    BG,
    EDGE,
    ELEVATION_CMAP,
    MUTED,
    ORIGIN,
    PANEL,
    TERRAIN_GROUND,
    TERRAIN_NON_GROUND,
    TEXT,
    TRACK_CONFIRMED,
    TRACK_TRAIL,
    TRACK_UNCONFIRMED,
    TRAV_BLOCKED,
    TRAV_CAUTION,
    TRAV_SAFE,
)
from visualization.dashboard_data import (
    MAX_BEV_POINTS,
    MAX_LIDAR_POINTS,
    cell_records,
    elevation_layer,
    elevation_values,
    select_proposal_labels,
    select_track_labels,
    subsample_indices,
    terrain_point_labels,
    track_trails,
    traversability_layer,
    proposal_style,
)


TRAV_FACE = {
    0: TRAV_SAFE,
    1: TRAV_CAUTION,
    2: TRAV_BLOCKED,
}


def _style_axes(ax, title, three_d=False):
    ax.set_facecolor(PANEL)
    ax.set_title(title, color=TEXT, fontsize=10, pad=8, loc="left", fontweight="medium")
    ax.tick_params(colors=MUTED, labelsize=7)
    for spine in ax.spines.values():
        spine.set_color(EDGE)
    if three_d:
        ax.xaxis.pane.fill = False
        ax.yaxis.pane.fill = False
        ax.zaxis.pane.fill = False
        ax.xaxis.pane.set_edgecolor(EDGE)
        ax.yaxis.pane.set_edgecolor(EDGE)
        ax.zaxis.pane.set_edgecolor(EDGE)
        ax.tick_params(axis="z", colors=MUTED, labelsize=7)
    ax.set_xlabel(ax.get_xlabel(), color=MUTED, fontsize=8)
    ax.set_ylabel(ax.get_ylabel(), color=MUTED, fontsize=8)


def _draw_lidar(ax, state, mode="elevation"):
    ax.cla()
    points = np.asarray(state.points)
    if len(points) == 0:
        _style_axes(ax, "3D LiDAR", three_d=True)
        return

    idx = subsample_indices(len(points), MAX_LIDAR_POINTS)
    sample = points[idx]
    x, y, z = sample[:, 0], sample[:, 1], sample[:, 2]

    if mode == "terrain" and state.ground.mask is not None:
        labels = terrain_point_labels(state.ground.mask)[idx]
        colors = np.where(labels == 0, TERRAIN_GROUND, TERRAIN_NON_GROUND)
        ax.scatter(x, y, z, c=colors, s=0.35, linewidths=0, alpha=0.7, depthshade=False)
        title = "3D LiDAR  ·  RANSAC ground / non-ground"
    else:
        ax.scatter(
            x, y, z,
            c=z,
            cmap=ELEVATION_CMAP,
            s=0.35,
            linewidths=0,
            alpha=0.75,
            depthshade=False,
        )
        title = "3D LiDAR  ·  elevation (Z)"

    ax.view_init(elev=22, azim=-70)
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_zlabel("Z (m)")
    _style_axes(ax, title, three_d=True)


def _draw_bev(ax, state):
    ax.cla()
    points = np.asarray(state.points)
    if len(points):
        idx = subsample_indices(len(points), MAX_BEV_POINTS, seed=1)
        sample = points[idx]
        if state.ground.mask is not None and len(state.ground.mask) == len(points):
            labels = terrain_point_labels(state.ground.mask)[idx]
            colors = np.where(labels == 0, TERRAIN_GROUND, TERRAIN_NON_GROUND)
            ax.scatter(
                sample[:, 0], sample[:, 1],
                c=list(colors), s=0.4, linewidths=0, alpha=0.35, zorder=1,
            )
        else:
            ax.scatter(
                sample[:, 0], sample[:, 1],
                c=elevation_values(sample), cmap=ELEVATION_CMAP,
                s=0.4, linewidths=0, alpha=0.35, zorder=1,
            )

    for proposal in state.proposals:
        x, y = proposal.center
        color = proposal_style(proposal.classification)
        rect = Rectangle(
            (x - proposal.width / 2.0, y - proposal.length / 2.0),
            proposal.width,
            proposal.length,
            fill=False,
            edgecolor=color,
            linewidth=1.1,
            zorder=4,
        )
        ax.add_patch(rect)

    for trail in track_trails(state.world_objects):
        ax.plot(
            trail[:, 0],
            trail[:, 1],
            color=TRACK_TRAIL,
            linewidth=1.0,
            alpha=0.85,
            zorder=5,
        )

    for track in state.tracks:
        px, py = float(track.position[0]), float(track.position[1])
        color = TRACK_CONFIRMED if track.confirmed else TRACK_UNCONFIRMED
        ax.scatter(px, py, c=color, s=18 if track.confirmed else 10, zorder=6)

    for track in select_track_labels(state.tracks):
        ax.annotate(
            f"#{track.track_id}",
            (float(track.position[0]), float(track.position[1])),
            color=TEXT,
            fontsize=7,
            xytext=(4, 4),
            textcoords="offset points",
            zorder=7,
        )

    for proposal in select_proposal_labels(state.proposals):
        ax.annotate(
            f"{proposal.classification} {proposal.confidence:.0%}",
            (proposal.center_x, proposal.center_y),
            color=MUTED,
            fontsize=6,
            xytext=(4, -9),
            textcoords="offset points",
            zorder=7,
        )

    ax.scatter([0], [0], marker="x", c=ORIGIN, s=36, zorder=8)
    ax.set_aspect("equal")
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    _style_axes(ax, "BEV world map  ·  LiDAR frame 0")

    legend = [
        Line2D([0], [0], color=TERRAIN_GROUND, marker="o", linestyle="", label="ground"),
        Line2D([0], [0], color=TERRAIN_NON_GROUND, marker="o", linestyle="", label="non-ground"),
        Line2D([0], [0], color=TRACK_CONFIRMED, marker="o", linestyle="", label="confirmed track"),
        Line2D([0], [0], color="#4C9BE8", marker="s", linestyle="", markerfacecolor="none", label="VEHICLE-LIKE"),
        Line2D([0], [0], color="#D4A017", marker="s", linestyle="", markerfacecolor="none", label="WALL"),
        Line2D([0], [0], color="#C9C15A", marker="s", linestyle="", markerfacecolor="none", label="POLE"),
        Line2D([0], [0], color="#D65A5A", marker="s", linestyle="", markerfacecolor="none", label="OBSTACLE"),
    ]
    ax.legend(
        handles=legend,
        loc="upper right",
        fontsize=6,
        framealpha=0.35,
        facecolor=PANEL,
        edgecolor=EDGE,
        labelcolor=TEXT,
    )


def _draw_elevation(ax, state, fig):
    ax.cla()
    records = cell_records(state.grid)
    xs, ys, zs = elevation_layer(records)
    if len(xs) == 0:
        ax.text(0.5, 0.5, "No ground elevation cells", color=MUTED, ha="center", va="center", transform=ax.transAxes)
        _style_axes(ax, "Elevation map")
        return

    scatter = ax.scatter(
        xs, ys, c=zs, cmap=ELEVATION_CMAP, s=3.5, linewidths=0, alpha=0.95
    )
    ax.set_aspect("equal")
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    _style_axes(ax, "Elevation map  ·  cell ground Z")
    cbar = fig.colorbar(scatter, ax=ax, fraction=0.046, pad=0.04)
    cbar.set_label("ground elevation (m)", color=MUTED, fontsize=7)
    cbar.ax.yaxis.set_tick_params(color=MUTED, labelsize=6)
    plt.setp(cbar.ax.yaxis.get_ticklabels(), color=MUTED)


def _draw_traversability(ax, state):
    ax.cla()
    records = cell_records(state.grid)
    xs, ys, codes, sizes = traversability_layer(records)
    if len(xs) == 0:
        ax.text(0.5, 0.5, "No occupied cells", color=MUTED, ha="center", va="center", transform=ax.transAxes)
        _style_axes(ax, "Traversability (prototype)")
        return

    colors = [TRAV_FACE[int(code)] for code in codes]
    ax.scatter(xs, ys, c=colors, s=np.clip(sizes, 1.5, 18), linewidths=0, alpha=0.9)
    ax.set_aspect("equal")
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    _style_axes(ax, "Traversability  ·  GROUND / MIXED / OBSTACLE")
    legend = [
        Line2D([0], [0], color=TRAV_SAFE, marker="s", linestyle="", label="GROUND · traversable"),
        Line2D([0], [0], color=TRAV_CAUTION, marker="s", linestyle="", label="MIXED · caution"),
        Line2D([0], [0], color=TRAV_BLOCKED, marker="s", linestyle="", label="OBSTACLE · blocked"),
    ]
    ax.legend(
        handles=legend,
        loc="upper right",
        fontsize=6,
        framealpha=0.35,
        facecolor=PANEL,
        edgecolor=EDGE,
        labelcolor=TEXT,
    )


def _draw_metrics(ax, state, sequence, source, frame_count, retired_total):
    ax.cla()
    ax.set_xlim(0, 1)
    ax.set_ylim(0, 1)
    ax.axis("off")
    ax.set_facecolor(PANEL)
    m = state.metrics

    ax.text(0.04, 0.94, "SYSTEM METRICS", color=ACCENT, fontsize=9, fontweight="bold", va="top")
    rows = [
        ("Source", f"{source}  seq {sequence}"),
        ("Frame", f"{state.frame_index}  /  {frame_count - 1 if frame_count else state.frame_index}"),
        ("Mapped points", f"{m.get('mapped_points', 0):,}"),
        ("Ground inliers", f"{m.get('ground_points', 0):,}"),
        ("Adaptive cells", f"{m.get('adaptive_cells', 0):,}"),
        ("Obstacle cells", f"{m.get('obstacle_cells', 0):,}"),
        ("Proposals", f"{m.get('proposals', 0)}"),
        ("Live tracks", f"{m.get('live_tracks', 0)}"),
        ("Confirmed", f"{m.get('confirmed_tracks', 0)}"),
        ("World objects", f"{m.get('world_objects', 0)}"),
        ("Retired this frame", str(m.get("retired_ids", []))),
        ("Retired (session)", str(retired_total)),
        ("Latency", f"{m.get('latency_ms', 0):.1f} ms"),
        ("World frame", str(m.get("world_frame", "lidar_frame_0"))),
    ]
    y = 0.84
    for label, value in rows:
        ax.text(0.04, y, label, color=MUTED, fontsize=7, va="top")
        ax.text(0.96, y, value, color=TEXT, fontsize=7, va="top", ha="right")
        y -= 0.055


def _draw_tracking(ax, state):
    ax.cla()
    ax.set_xlim(0, 1)
    ax.set_ylim(0, 1)
    ax.axis("off")
    ax.set_facecolor(PANEL)
    ax.text(0.04, 0.94, "TRACKING / WORLD MODEL", color=ACCENT, fontsize=9, fontweight="bold", va="top")

    confirmed = [t for t in state.tracks if t.confirmed]
    ax.text(
        0.04, 0.84,
        f"Live {len(state.tracks)}   Confirmed {len(confirmed)}   World {len(state.world_objects)}",
        color=TEXT, fontsize=8, va="top",
    )

    lines = select_track_labels(state.tracks, max_labels=10)
    y = 0.74
    if not lines:
        ax.text(0.04, y, "No live tracks", color=MUTED, fontsize=8, va="top")
        return

    ax.text(0.04, y, "Nearest labelled tracks", color=MUTED, fontsize=7, va="top")
    y -= 0.07
    for track in lines:
        flag = "CONF" if track.confirmed else "tent"
        ax.text(
            0.04, y,
            f"#{track.track_id:03d}  {track.class_name:<12}  "
            f"{flag}  hits={track.hits}  miss={track.missed}",
            color=TRACK_CONFIRMED if track.confirmed else MUTED,
            fontsize=7,
            family="monospace",
            va="top",
        )
        y -= 0.065
        if y < 0.06:
            break


def _draw_timeline(ax, states, current_index):
    ax.cla()
    ax.set_facecolor(PANEL)
    frames = [s.frame_index for s in states]
    counts = [s.metrics.get("live_tracks", 0) for s in states]
    ax.plot(frames, counts, color=ACCENT, linewidth=1.4)
    ax.scatter(frames, counts, c=ACCENT, s=18, zorder=3)
    cur = states[current_index].frame_index
    ax.axvline(cur, color=TEXT, linestyle="--", linewidth=0.8, alpha=0.7)
    ax.set_xlabel("frame", color=MUTED, fontsize=8)
    ax.set_ylabel("live tracks", color=MUTED, fontsize=8)
    _style_axes(ax, "Frame timeline  ·  live tracks (real tracker counts)")


def render_dashboard(
    states: Sequence,
    index: int = -1,
    sequence: str = "00",
    source: str = "kitti",
    lidar_mode: str = "elevation",
    show: bool = False,
    save_path: Optional[str] = None,
    fig=None,
):
    """
    Draw one dashboard page for states[index].

    `states` must be outputs of OrbitSystem.process_frame.
    """

    if not states:
        raise ValueError("render_dashboard requires at least one PipelineState")

    if index < 0:
        index = len(states) + index
    index = max(0, min(index, len(states) - 1))
    state = states[index]

    retired_total = sorted(
        {
            int(tid)
            for item in states[: index + 1]
            for tid in item.retired_ids
        }
    )

    created = fig is None
    if fig is None:
        fig = plt.figure(figsize=(16.5, 10.2), dpi=120, facecolor=BG)

    fig.clear()
    fig.patch.set_facecolor(BG)
    gs = GridSpec(
        3, 3,
        figure=fig,
        height_ratios=[1.2, 1.05, 0.38],
        hspace=0.38,
        wspace=0.28,
        left=0.05,
        right=0.98,
        top=0.90,
        bottom=0.07,
    )

    fig.suptitle(
        "ORBIT  —  Autonomous Terrain Intelligence System",
        color=TEXT,
        fontsize=15,
        fontweight="bold",
        x=0.05,
        ha="left",
        y=0.97,
    )
    fig.text(
        0.05, 0.935,
        f"{source.upper()} sequence {sequence}   |   "
        f"frame {state.frame_index}   |   "
        f"LIVE PIPELINE STATUS   |   "
        f"prototype  ·  geometric perception  ·  no learned semantics",
        color=MUTED,
        fontsize=8,
        ha="left",
    )

    ax_3d = fig.add_subplot(gs[0, 0], projection="3d")
    ax_bev = fig.add_subplot(gs[0, 1])
    ax_met = fig.add_subplot(gs[0, 2])
    ax_el = fig.add_subplot(gs[1, 0])
    ax_tv = fig.add_subplot(gs[1, 1])
    ax_trk = fig.add_subplot(gs[1, 2])
    ax_time = fig.add_subplot(gs[2, :])

    for panel in (ax_bev, ax_met, ax_el, ax_tv, ax_trk, ax_time):
        panel.set_facecolor(PANEL)

    _draw_lidar(ax_3d, state, mode=lidar_mode)
    _draw_bev(ax_bev, state)
    _draw_metrics(ax_met, state, sequence, source, len(states), retired_total)
    _draw_elevation(ax_el, state, fig)
    _draw_traversability(ax_tv, state)
    _draw_tracking(ax_trk, state)
    _draw_timeline(ax_time, states, index)

    if save_path:
        Path(save_path).parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(save_path, facecolor=fig.get_facecolor(), bbox_inches="tight")

    if show:
        plt.show()
    elif created and not save_path:
        plt.close(fig)

    return fig


def launch_dashboard(
    states: Sequence,
    sequence: str = "00",
    source: str = "kitti",
    lidar_mode: str = "elevation",
    show: bool = True,
):
    """Interactive slider over already processed pipeline states."""

    from matplotlib.widgets import Slider

    fig = plt.figure(figsize=(16.5, 10.2), dpi=120, facecolor=BG)

    def draw(index):
        render_dashboard(
            states,
            index=int(index),
            sequence=sequence,
            source=source,
            lidar_mode=lidar_mode,
            show=False,
            fig=fig,
        )
        if len(states) > 1:
            slider_ax = fig.add_axes([0.22, 0.012, 0.56, 0.028], facecolor=PANEL)
            slider = Slider(
                slider_ax,
                "Frame",
                0,
                len(states) - 1,
                valinit=int(index),
                valstep=1,
                color=ACCENT,
            )
            slider.label.set_color(TEXT)
            slider.valtext.set_color(TEXT)
            slider.on_changed(draw)
            fig._orbit_slider = slider

    draw(len(states) - 1)

    if show:
        plt.show()
    return fig


def save_dashboard_frames(
    states: Sequence,
    output_dir,
    sequence: str = "00",
    source: str = "kitti",
    lidar_mode: str = "elevation",
):
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    paths = []
    for i, state in enumerate(states):
        path = output_dir / f"orbit_dashboard_{source}_{sequence}_{state.frame_index:06d}.png"
        fig = render_dashboard(
            states,
            index=i,
            sequence=sequence,
            source=source,
            lidar_mode=lidar_mode,
            show=False,
            save_path=str(path),
        )
        plt.close(fig)
        paths.append(path)
    return paths
