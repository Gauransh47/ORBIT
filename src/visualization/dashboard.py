"""
ORBIT Visual Intelligence Dashboard V2.

Consumes a sequence of real PipelineState objects. Does not rerun
ground estimation, grid construction, or detection.

Two coordinate systems:

    LIVE LiDAR SENSOR  — current scan in the current LiDAR frame
                         (sensor at the origin).
    GLOBAL WORLD MODEL — LiDAR frame 0. Ego trajectory from KITTI
                         poses (or identity if no odometry).
"""

from pathlib import Path
from typing import Optional, Sequence

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.collections import PatchCollection
from matplotlib.gridspec import GridSpec
from matplotlib.lines import Line2D
from matplotlib.patches import Rectangle

from visualization.color_schemes import (
    ACCENT,
    BG,
    CELL_GROUND,
    CELL_MIXED,
    CELL_OBSTACLE,
    CURRENT,
    EDGE,
    EGO,
    EGO_FILL,
    ELEVATION_CMAP,
    MUTED,
    PANEL,
    START,
    TERRAIN_GROUND,
    TERRAIN_NON_GROUND,
    TEXT,
    TRACK_CONFIRMED,
    TRACK_TRAIL,
    TRACK_UNCONFIRMED,
    TRAJECTORY,
    TRAJECTORY_MARK,
)
from visualization.dashboard_data import (
    MAX_CELL_PATCHES,
    MAX_LIDAR_POINTS,
    MAX_TRACK_LIST,
    accumulated_world_cloud,
    cell_records,
    class_patches,
    elevation_patches,
    extract_headings,
    extract_trajectory,
    pose_source_of,
    select_track_labels,
    subsample_indices,
    subsample_records,
    terrain_point_labels,
    track_trails,
    world_to_ego_xy,
)


CELL_FACE = {
    0: CELL_GROUND,
    1: CELL_MIXED,
    2: CELL_OBSTACLE,
}


def _style_axes(ax, title, three_d=False):
    ax.set_facecolor(PANEL)
    ax.set_title(title, color=TEXT, fontsize=11, pad=9, loc="left", fontweight="bold")
    ax.tick_params(colors=MUTED, labelsize=7)
    ax.grid(True, color=EDGE, linewidth=0.4, alpha=0.55)
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
        ax.grid(False)
    ax.set_xlabel(ax.get_xlabel(), color=MUTED, fontsize=8)
    ax.set_ylabel(ax.get_ylabel(), color=MUTED, fontsize=8)


def _panel_meta(ax, text):
    ax.text(
        0.01, 0.01, text,
        transform=ax.transAxes,
        color=MUTED,
        fontsize=7,
        va="bottom",
        ha="left",
    )


def _vehicle_poly_xy(x, y, heading, length=3.6, width=1.7):
    c, s = np.cos(heading), np.sin(heading)
    hx, hy = length / 2.0, width / 2.0
    local = np.array(
        [
            [hx, hy],
            [hx, -hy],
            [-hx * 0.7, -hy],
            [-hx, 0.0],
            [-hx * 0.7, hy],
        ],
        dtype=np.float64,
    )
    rot = np.array([[c, -s], [s, c]])
    return local @ rot.T + np.array([x, y])


def _draw_live_lidar(ax, state, mode="elevation"):
    ax.cla()
    points = np.asarray(state.points)
    title = "LIVE LiDAR SENSOR\nCURRENT EGO FRAME"

    if len(points) == 0:
        _style_axes(ax, title, three_d=True)
        return

    idx = subsample_indices(len(points), MAX_LIDAR_POINTS, seed=int(state.frame_index))
    sample = points[idx]
    x, y, z = sample[:, 0], sample[:, 1], sample[:, 2]

    if mode == "terrain" and state.ground.mask is not None:
        labels = terrain_point_labels(state.ground.mask)[idx]
        colors = np.where(labels == 0, TERRAIN_GROUND, TERRAIN_NON_GROUND)
        ax.scatter(x, y, z, c=list(colors), s=0.28, linewidths=0, alpha=0.72, depthshade=False)
        meta = "RANSAC ground / non-ground"
    else:
        ax.scatter(
            x, y, z,
            c=z,
            cmap=ELEVATION_CMAP,
            s=0.28,
            linewidths=0,
            alpha=0.78,
            depthshade=False,
            vmin=np.percentile(z, 2) if len(z) else None,
            vmax=np.percentile(z, 98) if len(z) else None,
        )
        meta = "elevation (sensor Z)"

    T = getattr(state, "T_ego_to_world", None)
    if T is not None:
        for track in select_track_labels(state.tracks, max_labels=6, confirmed_only=True):
            xy = world_to_ego_xy(np.asarray(track.position[:2], dtype=np.float64), T)
            ax.scatter(
                [xy[0]], [xy[1]], [0.4],
                c=TRACK_CONFIRMED, s=18, zorder=9, depthshade=False,
            )

    ax.scatter([0], [0], [0], c=EGO, s=55, marker="o", zorder=10, depthshade=False)
    ax.quiver(0, 0, 0, 5.0, 0.0, 0.0, color=EGO, arrow_length_ratio=0.18, linewidth=1.6)

    ax.view_init(elev=24, azim=-72)
    span = 35.0
    ax.set_xlim(-span, span)
    ax.set_ylim(-span, span)
    ax.set_zlim(-4.0, 12.0)
    ax.set_xlabel("X forward (m)")
    ax.set_ylabel("Y (m)")
    ax.set_zlabel("Z (m)")
    _style_axes(ax, title, three_d=True)
    ax.text2D(0.01, 0.02, f"sensor @ (0,0)  ·  {meta}", transform=ax.transAxes, color=MUTED, fontsize=7)


def _draw_world_map(ax, states, index):
    ax.cla()
    state = states[index]
    title = "GLOBAL WORLD MODEL\nLiDAR FRAME 0 REFERENCE"

    cloud = accumulated_world_cloud(states, index)
    if len(cloud):
        z = cloud[:, 2] if cloud.shape[1] >= 3 else np.zeros(len(cloud))
        ax.scatter(
            cloud[:, 0], cloud[:, 1],
            c=z,
            cmap=ELEVATION_CMAP,
            s=0.22,
            linewidths=0,
            alpha=0.28,
            zorder=1,
        )

    for trail in track_trails(state.world_objects):
        ax.plot(trail[:, 0], trail[:, 1], color=TRACK_TRAIL, linewidth=1.0, alpha=0.7, zorder=4)

    confirmed = [t for t in state.tracks if t.confirmed]
    for track in confirmed:
        px, py = float(track.position[0]), float(track.position[1])
        ax.scatter(px, py, c=TRACK_CONFIRMED, s=22, zorder=6, linewidths=0)

    for track in select_track_labels(state.tracks, max_labels=6, confirmed_only=True):
        ax.annotate(
            f"#{track.track_id:03d}",
            (float(track.position[0]), float(track.position[1])),
            color=TEXT,
            fontsize=7,
            xytext=(5, 4),
            textcoords="offset points",
            zorder=7,
        )

    traj = extract_trajectory(states, up_to=index)
    headings = extract_headings(states, up_to=index)
    if len(traj) >= 1:
        if len(traj) >= 2:
            ax.plot(
                traj[:, 0], traj[:, 1],
                color=TRAJECTORY, linewidth=2.0, solid_capstyle="round", zorder=8,
            )
        ax.scatter(
            traj[:, 0], traj[:, 1],
            c=TRAJECTORY_MARK, s=12, zorder=9, linewidths=0, alpha=0.85,
        )
        ax.scatter(
            [traj[0, 0]], [traj[0, 1]],
            c=START, s=90, marker="o", zorder=11, edgecolors=BG, linewidths=0.6,
        )
        ax.annotate(
            "START",
            (traj[0, 0], traj[0, 1]),
            color=START,
            fontsize=8,
            fontweight="bold",
            xytext=(6, 8),
            textcoords="offset points",
            zorder=12,
        )

        cx, cy = traj[-1]
        heading = float(headings[-1]) if len(headings) else 0.0
        poly = _vehicle_poly_xy(cx, cy, heading)
        ax.fill(poly[:, 0], poly[:, 1], color=EGO_FILL, zorder=12)
        ax.plot(poly[:, 0], poly[:, 1], color=EGO, linewidth=1.4, zorder=13)
        ax.annotate(
            "",
            xy=(cx + 6.0 * np.cos(heading), cy + 6.0 * np.sin(heading)),
            xytext=(cx, cy),
            arrowprops=dict(arrowstyle="-|>", color=CURRENT, lw=1.6),
            zorder=14,
        )
        ax.annotate(
            "CURRENT",
            (cx, cy),
            color=CURRENT,
            fontsize=8,
            fontweight="bold",
            xytext=(8, -12),
            textcoords="offset points",
            zorder=14,
        )

    ax.set_aspect("equal")
    ax.autoscale()
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    _style_axes(ax, title)
    src = getattr(state, "pose_source", "unknown")
    _panel_meta(ax, f"ego trajectory  ·  {src}")

    legend = [
        Line2D([0], [0], color=TRAJECTORY, linewidth=2, label="ego path"),
        Line2D([0], [0], color=START, marker="o", linestyle="", label="start"),
        Line2D([0], [0], color=EGO, marker="s", linestyle="", label="current"),
        Line2D([0], [0], color=TRACK_CONFIRMED, marker="o", linestyle="", label="confirmed track"),
    ]
    ax.legend(
        handles=legend,
        loc="upper right",
        fontsize=7,
        framealpha=0.4,
        facecolor=PANEL,
        edgecolor=EDGE,
        labelcolor=TEXT,
    )


def _add_cell_patches(ax, xs, ys, resolutions, facecolors_or_array, cmap=None, vmin=None, vmax=None):
    patches = []
    for x, y, r in zip(xs, ys, resolutions):
        patches.append(Rectangle((x - r / 2.0, y - r / 2.0), r, r))
    collection = PatchCollection(
        patches,
        linewidths=0.05,
        edgecolors="none",
        alpha=0.92,
    )
    if cmap is not None:
        collection.set_cmap(cmap)
        collection.set_array(np.asarray(facecolors_or_array, dtype=np.float64))
        if vmin is not None:
            collection.set_clim(vmin, vmax)
    else:
        collection.set_facecolor(list(facecolors_or_array))
    ax.add_collection(collection)
    return collection


def _draw_elevation(ax, state, fig):
    ax.cla()
    records = subsample_records(cell_records(state.grid), MAX_CELL_PATCHES, seed=7)
    xs, ys, res, zs = elevation_patches(records)
    title = "TERRAIN ELEVATION MODEL\nGROUND HEIGHT (Z)"
    if len(xs) == 0:
        ax.text(0.5, 0.5, "No ground elevation cells", color=MUTED, ha="center", va="center", transform=ax.transAxes)
        _style_axes(ax, title)
        return

    vmin, vmax = np.percentile(zs, 2), np.percentile(zs, 98)
    if not np.isfinite(vmin) or vmin == vmax:
        vmin, vmax = float(np.min(zs)), float(np.max(zs) + 1e-3)
    coll = _add_cell_patches(ax, xs, ys, res, zs, cmap=ELEVATION_CMAP, vmin=vmin, vmax=vmax)
    ax.set_aspect("equal")
    ax.autoscale()
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    _style_axes(ax, title)
    cbar = fig.colorbar(coll, ax=ax, fraction=0.046, pad=0.03)
    cbar.set_label("ground Z (m)", color=MUTED, fontsize=7)
    cbar.ax.yaxis.set_tick_params(color=MUTED, labelsize=6)
    plt.setp(cbar.ax.yaxis.get_ticklabels(), color=MUTED)
    _panel_meta(ax, "AdaptiveCell.ground_elevation  ·  current ego grid")


def _draw_terrain_class(ax, state):
    ax.cla()
    records = subsample_records(cell_records(state.grid), MAX_CELL_PATCHES, seed=8)
    xs, ys, res, codes = class_patches(records)
    title = "TERRAIN STRUCTURE\nGEOMETRIC CELL CLASS"
    if len(xs) == 0:
        ax.text(0.5, 0.5, "No occupied cells", color=MUTED, ha="center", va="center", transform=ax.transAxes)
        _style_axes(ax, title)
        return

    faces = [CELL_FACE[int(code)] for code in codes]
    _add_cell_patches(ax, xs, ys, res, faces)
    ax.set_aspect("equal")
    ax.autoscale()
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    _style_axes(ax, title)
    legend = [
        Line2D([0], [0], color=CELL_GROUND, marker="s", linestyle="", label="GROUND"),
        Line2D([0], [0], color=CELL_MIXED, marker="s", linestyle="", label="MIXED"),
        Line2D([0], [0], color=CELL_OBSTACLE, marker="s", linestyle="", label="OBSTACLE"),
    ]
    ax.legend(
        handles=legend,
        loc="upper right",
        fontsize=7,
        framealpha=0.4,
        facecolor=PANEL,
        edgecolor=EDGE,
        labelcolor=TEXT,
    )
    _panel_meta(ax, "AdaptiveCell.semantic_class  ·  not learned drivability")


def _metric_block(ax, x, y, label, value, size=16):
    ax.text(x, y, label, color=MUTED, fontsize=7, va="top", transform=ax.transAxes)
    ax.text(x, y - 0.055, value, color=TEXT, fontsize=size, fontweight="bold", va="top", transform=ax.transAxes)


def _draw_status(ax, state, sequence, source, frame_count):
    ax.cla()
    ax.set_xlim(0, 1)
    ax.set_ylim(0, 1)
    ax.axis("off")
    ax.set_facecolor(PANEL)
    ax.set_title("SYSTEM STATUS", color=TEXT, fontsize=11, loc="left", fontweight="bold", pad=8)
    m = state.metrics
    last = frame_count - 1 if frame_count else state.frame_index

    _metric_block(ax, 0.04, 0.90, "FRAME", f"{state.frame_index} / {last}", 18)
    _metric_block(ax, 0.52, 0.90, "LATENCY", f"{m.get('latency_ms', 0):.0f} ms", 16)
    _metric_block(ax, 0.04, 0.68, "POINT CLOUD", f"{m.get('mapped_points', 0):,}", 15)
    _metric_block(ax, 0.52, 0.68, "GROUND", f"{m.get('ground_points', 0):,}", 15)
    _metric_block(ax, 0.04, 0.46, "TERRAIN GRID", f"{m.get('adaptive_cells', 0):,}", 15)
    _metric_block(ax, 0.52, 0.46, "OBSTACLE CELLS", f"{m.get('obstacle_cells', 0):,}", 15)
    _metric_block(ax, 0.04, 0.24, "PROPOSALS", str(m.get("proposals", 0)), 15)
    _metric_block(ax, 0.52, 0.24, "SOURCE", f"{source} {sequence}", 12)

    ax.text(
        0.04, 0.05,
        f"pose  {getattr(state, 'pose_source', 'unknown')}",
        color=MUTED, fontsize=7, va="bottom", transform=ax.transAxes,
    )


def _draw_tracking(ax, state):
    ax.cla()
    ax.set_xlim(0, 1)
    ax.set_ylim(0, 1)
    ax.axis("off")
    ax.set_facecolor(PANEL)
    ax.set_title("TRACKING", color=TEXT, fontsize=11, loc="left", fontweight="bold", pad=8)

    confirmed = [t for t in state.tracks if t.confirmed]
    _metric_block(ax, 0.04, 0.92, "LIVE TRACKS", str(len(state.tracks)), 16)
    _metric_block(ax, 0.38, 0.92, "CONFIRMED", str(len(confirmed)), 16)
    _metric_block(ax, 0.70, 0.92, "WORLD OBJECTS", str(len(state.world_objects)), 14)

    ax.text(0.04, 0.68, "TRACK", color=MUTED, fontsize=7, transform=ax.transAxes)
    ax.text(0.28, 0.68, "CLASS", color=MUTED, fontsize=7, transform=ax.transAxes)
    ax.text(0.70, 0.68, "HITS", color=MUTED, fontsize=7, transform=ax.transAxes)

    rows = select_track_labels(state.tracks, max_labels=MAX_TRACK_LIST, confirmed_only=True)
    if not rows:
        rows = select_track_labels(state.tracks, max_labels=MAX_TRACK_LIST)
    if not rows:
        ax.text(0.04, 0.58, "No live tracks", color=MUTED, fontsize=9, transform=ax.transAxes)
        return

    y = 0.60
    for track in rows:
        color = TRACK_CONFIRMED if track.confirmed else TRACK_UNCONFIRMED
        ax.text(0.04, y, f"#{track.track_id:03d}", color=color, fontsize=9, family="monospace", transform=ax.transAxes)
        ax.text(0.28, y, str(track.class_name), color=TEXT, fontsize=9, family="monospace", transform=ax.transAxes)
        ax.text(0.70, y, str(track.hits), color=TEXT, fontsize=9, family="monospace", transform=ax.transAxes)
        y -= 0.07


def _draw_timeline(ax, states, current_index):
    ax.cla()
    ax.set_facecolor(PANEL)
    frames = [s.frame_index for s in states]
    live = [s.metrics.get("live_tracks", 0) for s in states]
    conf = [s.metrics.get("confirmed_tracks", 0) for s in states]
    ax.plot(frames, live, color=ACCENT, linewidth=1.6, label="live")
    ax.plot(frames, conf, color=TRACK_CONFIRMED, linewidth=1.4, linestyle="--", label="confirmed")
    ax.scatter(frames, live, c=ACCENT, s=16, zorder=3)
    cur = states[current_index].frame_index
    ax.axvline(cur, color=CURRENT, linestyle="-", linewidth=1.0, alpha=0.85)
    ax.set_xlabel("frame", color=MUTED, fontsize=8)
    ax.set_ylabel("tracks", color=MUTED, fontsize=8)
    _style_axes(ax, "FRAME TIMELINE")
    ax.legend(
        loc="upper right",
        fontsize=7,
        framealpha=0.35,
        facecolor=PANEL,
        edgecolor=EDGE,
        labelcolor=TEXT,
    )


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
    """Draw one dashboard page for states[index] (real PipelineState only)."""

    if not states:
        raise ValueError("render_dashboard requires at least one PipelineState")

    if index < 0:
        index = len(states) + index
    index = max(0, min(index, len(states) - 1))
    state = states[index]

    created = fig is None
    if fig is None:
        fig = plt.figure(figsize=(20.5, 12.4), dpi=110, facecolor=BG)

    fig.clear()
    fig.patch.set_facecolor(BG)
    gs = GridSpec(
        3, 3,
        figure=fig,
        height_ratios=[1.28, 1.05, 0.78],
        width_ratios=[1.18, 1.18, 0.82],
        hspace=0.42,
        wspace=0.28,
        left=0.04,
        right=0.985,
        top=0.88,
        bottom=0.09,
    )

    fig.text(
        0.04, 0.965,
        "ORBIT",
        color=EGO,
        fontsize=18,
        fontweight="bold",
        ha="left",
        va="top",
    )
    fig.text(
        0.04, 0.932,
        "AUTONOMOUS TERRAIN INTELLIGENCE SYSTEM",
        color=TEXT,
        fontsize=12,
        ha="left",
        va="top",
    )
    fig.text(
        0.04, 0.905,
        "LIVE PERCEPTION   ·   GLOBAL MAPPING   ·   TERRAIN ANALYSIS"
        "     geometric prototype  ·  no learned semantics",
        color=MUTED,
        fontsize=8,
        ha="left",
        va="top",
    )
    fig.text(
        0.985, 0.965,
        f"{source.upper()}  {sequence}    frame {state.frame_index}",
        color=TEXT,
        fontsize=11,
        ha="right",
        va="top",
    )
    fig.text(
        0.985, 0.932,
        pose_source_of(states[: index + 1]),
        color=MUTED,
        fontsize=8,
        ha="right",
        va="top",
    )

    ax_live = fig.add_subplot(gs[0, 0], projection="3d")
    ax_world = fig.add_subplot(gs[0, 1])
    ax_status = fig.add_subplot(gs[0, 2])
    ax_el = fig.add_subplot(gs[1, 0])
    ax_cls = fig.add_subplot(gs[1, 1])
    ax_trk = fig.add_subplot(gs[1, 2])
    ax_time = fig.add_subplot(gs[2, :])

    for panel in (ax_world, ax_status, ax_el, ax_cls, ax_trk, ax_time):
        panel.set_facecolor(PANEL)

    _draw_live_lidar(ax_live, state, mode=lidar_mode)
    _draw_world_map(ax_world, states, index)
    _draw_status(ax_status, state, sequence, source, len(states))
    _draw_elevation(ax_el, state, fig)
    _draw_terrain_class(ax_cls, state)
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

    fig = plt.figure(figsize=(20.5, 12.4), dpi=110, facecolor=BG)

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
            slider_ax = fig.add_axes([0.22, 0.018, 0.56, 0.028], facecolor=PANEL)
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
