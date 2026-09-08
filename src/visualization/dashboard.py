"""
ORBIT Visual Intelligence Dashboard V3.

Consumes real PipelineState objects only. Does not rerun perception.
Hero panels: live ego LiDAR (current frame) and world map (LiDAR frame 0).
"""

from pathlib import Path
from typing import Optional, Sequence

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.collections import PatchCollection
from matplotlib.gridspec import GridSpec, GridSpecFromSubplotSpec
from matplotlib.patches import FancyBboxPatch, Rectangle

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
    LIVE_DOT,
    MUTED,
    PANEL,
    PROPOSAL_COLORS,
    START,
    TERRAIN_GROUND,
    TERRAIN_NON_GROUND,
    TEXT,
    TRACK_CONFIRMED,
    TRACK_TRAIL,
    TRACK_UNCONFIRMED,
    TRAJECTORY,
    TRAJECTORY_MARK,
    proposal_color,
)
from visualization.dashboard_data import (
    MAX_CELL_PATCHES,
    MAX_LIDAR_POINTS,
    accumulated_world_cloud,
    cell_records,
    class_patches,
    compact_count,
    elevation_patches,
    extract_headings,
    extract_trajectory,
    pose_caption,
    select_track_labels,
    source_badge,
    subsample_indices,
    subsample_records,
    terrain_point_labels,
    track_class_counts,
    track_trails,
    travel_metres,
    xy_bounds,
)


CELL_FACE = {0: CELL_GROUND, 1: CELL_MIXED, 2: CELL_OBSTACLE}

FIGSIZE = (22.0, 13.2)
FIGDPI = 120


def _chrome(ax, three_d=False):
    ax.set_facecolor(PANEL)
    ax.tick_params(colors=MUTED, labelsize=6, width=0.4)
    ax.grid(True, color=EDGE, linewidth=0.35, alpha=0.45)
    for spine in ax.spines.values():
        spine.set_color(EDGE)
        spine.set_linewidth(0.9)
    if three_d:
        ax.grid(False)


def _title_block(ax, title, subtitle):
    ax.set_title("")
    ax.text(
        0.0, 1.045, title,
        transform=ax.transAxes, color=TEXT, fontsize=11,
        fontweight="bold", ha="left", va="bottom",
    )
    ax.text(
        0.0, 1.012, subtitle,
        transform=ax.transAxes, color=MUTED, fontsize=7.5,
        ha="left", va="bottom",
    )


def _vehicle_poly_xy(x, y, heading, length=4.2, width=1.9):
    c, s = np.cos(heading), np.sin(heading)
    hx, hy = length / 2.0, width / 2.0
    local = np.array(
        [[hx, hy], [hx, -hy], [-hx * 0.65, -hy], [-hx, 0.0], [-hx * 0.65, hy]],
        dtype=np.float64,
    )
    rot = np.array([[c, -s], [s, c]])
    return local @ rot.T + np.array([x, y])


def _set_equal_limits(ax, x0, x1, y0, y1):
    ax.set_xlim(x0, x1)
    ax.set_ylim(y0, y1)
    ax.set_aspect("equal", adjustable="box")


def _draw_live_lidar(ax, state, mode="elevation"):
    ax.cla()
    _chrome(ax)
    _title_block(ax, "LIVE LiDAR", "EGO-CENTRIC SENSOR VIEW")
    ax.set_xlabel("X forward (m)", color=MUTED, fontsize=7)
    ax.set_ylabel("Y (m)", color=MUTED, fontsize=7)

    points = np.asarray(state.points)
    if len(points) == 0:
        ax.text(0.5, 0.5, "No points", color=MUTED, ha="center", va="center", transform=ax.transAxes)
        return

    idx = subsample_indices(len(points), MAX_LIDAR_POINTS, seed=int(state.frame_index))
    sample = points[idx]
    x, y, z = sample[:, 0], sample[:, 1], sample[:, 2]

    if mode == "terrain" and state.ground.mask is not None:
        labels = terrain_point_labels(state.ground.mask)[idx]
        colors = np.where(labels == 0, TERRAIN_GROUND, TERRAIN_NON_GROUND)
        ax.scatter(x, y, c=list(colors), s=5.2, linewidths=0, alpha=0.75, rasterized=True)
    else:
        zspan = float(np.percentile(z, 97) - np.percentile(z, 3)) if len(z) else 0.0
        colour = z if zspan >= 0.30 else np.hypot(x, y)
        vmin, vmax = np.percentile(colour, 3), np.percentile(colour, 97)
        if not np.isfinite(vmin) or vmin == vmax:
            vmin, vmax = float(np.min(colour)), float(np.max(colour) + 1e-3)
        ax.scatter(
            x, y, c=colour, cmap=ELEVATION_CMAP, s=5.5, linewidths=0,
            alpha=0.82, vmin=vmin, vmax=vmax, rasterized=True,
        )

    for proposal in getattr(state, "proposals", []) or []:
        cx, cy = proposal.center
        color = proposal_color(proposal.classification)
        ax.add_patch(
            Rectangle(
                (cx - proposal.width / 2.0, cy - proposal.length / 2.0),
                proposal.width,
                proposal.length,
                fill=False,
                edgecolor=color,
                linewidth=0.9,
                alpha=0.85,
                zorder=5,
            )
        )

    poly = _vehicle_poly_xy(0.0, 0.0, 0.0, length=4.0, width=1.8)
    ax.fill(poly[:, 0], poly[:, 1], color=EGO_FILL, zorder=8)
    ax.plot(np.append(poly[:, 0], poly[0, 0]), np.append(poly[:, 1], poly[0, 1]), color=EGO, lw=1.6, zorder=9)
    ax.annotate(
        "",
        xy=(6.5, 0.0),
        xytext=(0.0, 0.0),
        arrowprops=dict(arrowstyle="-|>", color=EGO, lw=1.8, mutation_scale=12),
        zorder=10,
    )
    ax.scatter([0], [0], c=EGO, s=28, zorder=11, edgecolors=BG, linewidths=0.4)

    extras = [0.0, 6.5]
    x0, x1, y0, y1 = xy_bounds(
        np.concatenate([x, extras]),
        np.concatenate([y, np.zeros_like(extras)]),
        pad_frac=0.10,
        min_span=18.0,
    )
    _set_equal_limits(ax, x0, x1, y0, y1)


def _draw_world_map(ax, states, index):
    ax.cla()
    _chrome(ax)
    _title_block(ax, "GLOBAL WORLD MODEL", "LIDAR FRAME 0 REFERENCE")
    ax.set_xlabel("X (m)", color=MUTED, fontsize=7)
    ax.set_ylabel("Y (m)", color=MUTED, fontsize=7)

    state = states[index]
    cloud = accumulated_world_cloud(states, index)
    xs_fit, ys_fit = [], []

    if len(cloud):
        z = cloud[:, 2] if cloud.shape[1] >= 3 else np.zeros(len(cloud))
        ax.scatter(
            cloud[:, 0], cloud[:, 1],
            c=z, cmap=ELEVATION_CMAP, s=1.6, linewidths=0,
            alpha=0.32, rasterized=True, zorder=1,
        )
        xs_fit.append(cloud[:, 0])
        ys_fit.append(cloud[:, 1])

    for trail in track_trails(state.world_objects):
        ax.plot(trail[:, 0], trail[:, 1], color=TRACK_TRAIL, linewidth=1.0, alpha=0.65, zorder=3)

    for track in state.tracks:
        px, py = float(track.position[0]), float(track.position[1])
        color = TRACK_CONFIRMED if track.confirmed else TRACK_UNCONFIRMED
        ax.scatter(px, py, c=color, s=26 if track.confirmed else 12, zorder=6, linewidths=0)

    labelled = select_track_labels(state.tracks, max_labels=6, confirmed_only=True)
    for track in labelled:
        cls = str(track.class_name).replace("VEHICLE-LIKE", "VEHICLE")
        ax.annotate(
            f"#{track.track_id:03d}\n{cls}",
            (float(track.position[0]), float(track.position[1])),
            color=TEXT,
            fontsize=6.5,
            ha="left",
            va="bottom",
            xytext=(6, 5),
            textcoords="offset points",
            zorder=7,
        )

    traj = extract_trajectory(states, up_to=index)
    headings = extract_headings(states, up_to=index)
    if len(traj):
        xs_fit.append(traj[:, 0])
        ys_fit.append(traj[:, 1])
        if len(traj) >= 2:
            ax.plot(traj[:, 0], traj[:, 1], color=TRAJECTORY, linewidth=2.8, solid_capstyle="round", zorder=8)
        ax.scatter(traj[:, 0], traj[:, 1], c=TRAJECTORY_MARK, s=10, zorder=9, linewidths=0, alpha=0.9)
        ax.scatter([traj[0, 0]], [traj[0, 1]], c=START, s=110, marker="o", zorder=12, edgecolors=BG, linewidths=0.7)
        ax.annotate(
            "START",
            (traj[0, 0], traj[0, 1]),
            color=START, fontsize=8, fontweight="bold",
            xytext=(8, 10), textcoords="offset points", zorder=13,
        )
        cx, cy = float(traj[-1, 0]), float(traj[-1, 1])
        heading = float(headings[-1]) if len(headings) else 0.0
        span = 20.0
        if len(traj) >= 2:
            span = max(12.0, float(np.linalg.norm(traj[-1] - traj[0])) * 0.08)
        length = np.clip(span, 4.0, 9.0)
        poly = _vehicle_poly_xy(cx, cy, heading, length=length, width=length * 0.42)
        ax.fill(poly[:, 0], poly[:, 1], color=EGO_FILL, zorder=14)
        ax.plot(np.append(poly[:, 0], poly[0, 0]), np.append(poly[:, 1], poly[0, 1]), color=EGO, lw=1.8, zorder=15)
        ax.annotate(
            "",
            xy=(cx + length * 1.6 * np.cos(heading), cy + length * 1.6 * np.sin(heading)),
            xytext=(cx, cy),
            arrowprops=dict(arrowstyle="-|>", color=CURRENT, lw=1.8, mutation_scale=11),
            zorder=16,
        )
        ax.annotate(
            "CURRENT",
            (cx, cy),
            color=CURRENT, fontsize=8, fontweight="bold",
            xytext=(10, -14), textcoords="offset points", zorder=16,
        )

    if xs_fit:
        x0, x1, y0, y1 = xy_bounds(
            np.concatenate(xs_fit),
            np.concatenate(ys_fit),
            pad_frac=0.16,
            min_span=16.0,
        )
        _set_equal_limits(ax, x0, x1, y0, y1)
    else:
        _set_equal_limits(ax, -10, 10, -10, 10)


def _add_cell_patches(ax, xs, ys, resolutions, facecolors_or_array, cmap=None, vmin=None, vmax=None):
    xs = np.asarray(xs, dtype=np.float64)
    ys = np.asarray(ys, dtype=np.float64)
    resolutions = np.asarray(resolutions, dtype=np.float64)
    span = max(float(np.ptp(xs) if len(xs) else 1.0), float(np.ptp(ys) if len(ys) else 1.0), 8.0)
    floor = span / 220.0
    patches = []
    for x, y, r in zip(xs, ys, resolutions):
        draw = max(float(r), floor)
        patches.append(Rectangle((x - draw / 2.0, y - draw / 2.0), draw, draw))
    collection = PatchCollection(patches, linewidths=0.0, edgecolors="none", alpha=0.9)
    if cmap is not None:
        collection.set_cmap(cmap)
        collection.set_array(np.asarray(facecolors_or_array, dtype=np.float64))
        collection.set_clim(vmin, vmax)
    else:
        collection.set_facecolor(list(facecolors_or_array))
    ax.add_collection(collection)
    return collection


def _draw_elevation(ax, state, fig):
    ax.cla()
    _chrome(ax)
    _title_block(ax, "TERRAIN ELEVATION", "GROUND HEIGHT  (Z)")
    ax.set_xlabel("X (m)", color=MUTED, fontsize=7)
    ax.set_ylabel("Y (m)", color=MUTED, fontsize=7)

    records = subsample_records(cell_records(state.grid), MAX_CELL_PATCHES, seed=7)
    xs, ys, res, zs = elevation_patches(records)
    if len(xs) == 0:
        ax.text(0.5, 0.5, "No ground cells", color=MUTED, ha="center", va="center", transform=ax.transAxes)
        return

    vmin, vmax = np.percentile(zs, 5), np.percentile(zs, 95)
    if not np.isfinite(vmin) or abs(vmax - vmin) < 1e-6:
        vmin, vmax = float(np.min(zs)), float(np.max(zs) + 1e-3)
    coll = _add_cell_patches(ax, xs, ys, res, zs, cmap=ELEVATION_CMAP, vmin=vmin, vmax=vmax)
    x0, x1, y0, y1 = xy_bounds(xs, ys, pad_frac=0.05, min_span=6.0)
    _set_equal_limits(ax, x0, x1, y0, y1)
    cbar = fig.colorbar(coll, ax=ax, fraction=0.035, pad=0.02, shrink=0.82)
    cbar.set_ticks([vmin, vmax])
    cbar.set_ticklabels(["LOW", "HIGH"])
    cbar.ax.tick_params(colors=MUTED, labelsize=6)
    cbar.outline.set_edgecolor(EDGE)


def _draw_terrain_class(ax, state):
    ax.cla()
    _chrome(ax)
    _title_block(ax, "TERRAIN STRUCTURE", "geometric class  ·  not learned drivability")
    ax.set_xlabel("X (m)", color=MUTED, fontsize=7)
    ax.set_ylabel("Y (m)", color=MUTED, fontsize=7)

    records = subsample_records(cell_records(state.grid), MAX_CELL_PATCHES, seed=8)
    xs, ys, res, codes = class_patches(records)
    if len(xs) == 0:
        ax.text(0.5, 0.5, "No cells", color=MUTED, ha="center", va="center", transform=ax.transAxes)
        return

    faces = [CELL_FACE[int(code)] for code in codes]
    _add_cell_patches(ax, xs, ys, res, faces)
    x0, x1, y0, y1 = xy_bounds(xs, ys, pad_frac=0.05, min_span=6.0)
    _set_equal_limits(ax, x0, x1, y0, y1)


def _card(ax, x, y, w, h, label, value):
    box = FancyBboxPatch(
        (x, y), w, h,
        boxstyle="round,pad=0.008,rounding_size=0.02",
        facecolor="#161C28",
        edgecolor=EDGE,
        linewidth=0.7,
        transform=ax.transAxes,
        clip_on=False,
        zorder=2,
    )
    ax.add_patch(box)
    ax.text(
        x + 0.04, y + h * 0.70, label, color=MUTED, fontsize=6.5, va="center",
        transform=ax.transAxes, zorder=5, clip_on=False,
    )
    ax.text(
        x + 0.04, y + h * 0.32, value, color=TEXT, fontsize=13, fontweight="bold", va="center",
        transform=ax.transAxes, zorder=5, clip_on=False,
    )


def _draw_status(ax, states, index):
    ax.cla()
    ax.set_xlim(0, 1)
    ax.set_ylim(0, 1)
    ax.axis("off")
    ax.set_facecolor(PANEL)
    for spine in ax.spines.values():
        spine.set_color(EDGE)

    state = states[index]
    m = state.metrics
    last = max(s.frame_index for s in states)
    travel = travel_metres(states, up_to=index)
    confirmed = sum(1 for t in state.tracks if t.confirmed)

    ax.text(0.04, 0.965, "SYSTEM", color=TEXT, fontsize=10, fontweight="bold", va="top", transform=ax.transAxes)

    cards = [
        ("FRAME", f"{state.frame_index} / {last}"),
        ("TRAVEL", f"{travel:.1f} m"),
        ("LATENCY", f"{float(m.get('latency_ms', 0)):.0f} ms"),
        ("POINTS", compact_count(m.get("mapped_points", 0))),
        ("GROUND", compact_count(m.get("ground_points", 0))),
        ("CELLS", compact_count(m.get("adaptive_cells", 0))),
        ("OBSTACLE", compact_count(m.get("obstacle_cells", 0))),
        ("PROPOSALS", str(int(m.get("proposals", 0)))),
    ]

    cols, w, h, gap_x, gap_y = 2, 0.45, 0.125, 0.03, 0.016
    x0, y0 = 0.04, 0.82
    for i, (lab, val) in enumerate(cards):
        c = i % cols
        r = i // cols
        _card(ax, x0 + c * (w + gap_x), y0 - r * (h + gap_y), w, h, lab, val)

    ax.text(0.04, 0.20, f"LIVE  {len(state.tracks)}", color=ACCENT, fontsize=10, fontweight="bold", transform=ax.transAxes)
    ax.text(0.52, 0.20, f"CONFIRMED  {confirmed}", color=TRACK_CONFIRMED, fontsize=10, fontweight="bold", transform=ax.transAxes)

    counts = track_class_counts(state.tracks)
    bar_top = 0.145
    ax.text(0.04, bar_top + 0.03, "CLASS MIX", color=MUTED, fontsize=6.5, transform=ax.transAxes)
    if not counts:
        return
    max_n = max(n for _, n in counts) or 1
    y = bar_top - 0.01
    for name, n in counts[:4]:
        color = PROPOSAL_COLORS.get(name, TRACK_CONFIRMED)
        ax.text(0.04, y, name.replace("VEHICLE-LIKE", "VEHICLE"), color=MUTED, fontsize=6, va="center", transform=ax.transAxes)
        ax.add_patch(
            Rectangle((0.38, y - 0.012), 0.50 * (n / max_n), 0.024, transform=ax.transAxes, facecolor=color, edgecolor="none", clip_on=False)
        )
        ax.text(0.90, y, str(n), color=TEXT, fontsize=6.5, va="center", ha="right", transform=ax.transAxes)
        y -= 0.038


def _draw_timeline(ax, states, current_index):
    ax.cla()
    _chrome(ax)
    _title_block(ax, "FRAME TIMELINE", "LIVE  ·  CONFIRMED")
    frames = [s.frame_index for s in states]
    live = [s.metrics.get("live_tracks", 0) for s in states]
    conf = [s.metrics.get("confirmed_tracks", 0) for s in states]
    ax.fill_between(frames, live, color=ACCENT, alpha=0.12)
    ax.plot(frames, live, color=ACCENT, linewidth=2.0, label="live")
    ax.plot(frames, conf, color=TRACK_CONFIRMED, linewidth=1.6, label="confirmed")
    cur = states[current_index].frame_index
    ax.axvline(cur, color=CURRENT, linewidth=1.4, alpha=0.9)
    ax.scatter([cur], [live[current_index]], c=LIVE_DOT, s=36, zorder=5, edgecolors=BG, linewidths=0.4)
    if frames:
        ax.annotate("START", (frames[0], live[0]), color=START, fontsize=7, xytext=(0, 8), textcoords="offset points", ha="left")
        ax.annotate("CURRENT", (cur, live[current_index]), color=CURRENT, fontsize=7, xytext=(6, 8), textcoords="offset points")
    ax.set_xlabel("frame", color=MUTED, fontsize=7)
    ax.set_ylabel("tracks", color=MUTED, fontsize=7)
    ax.legend(loc="upper right", fontsize=7, frameon=False, labelcolor=TEXT)


def _draw_header(fig, state, states, index, source, sequence):
    last = max(s.frame_index for s in states)
    travel = travel_metres(states, up_to=index)
    fig.text(0.045, 0.975, "ORBIT", color=EGO, fontsize=22, fontweight="bold", va="top", ha="left")
    fig.text(0.045, 0.948, "Autonomous Terrain Intelligence", color=TEXT, fontsize=11, va="top", ha="left")
    fig.text(0.045, 0.924, source_badge(source, sequence), color=MUTED, fontsize=8.5, va="top", ha="left")

    fig.text(0.98, 0.975, f"FRAME  {state.frame_index}  /  {last}", color=TEXT, fontsize=14, fontweight="bold", va="top", ha="right")
    fig.text(0.98, 0.948, "●  PROCESSING", color=LIVE_DOT, fontsize=10, va="top", ha="right")
    fig.text(
        0.98, 0.924,
        f"REF  {pose_caption(getattr(state, 'pose_source', ''))}    TRAVEL  {travel:.1f} m",
        color=MUTED, fontsize=8.5, va="top", ha="right",
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
    if not states:
        raise ValueError("render_dashboard requires at least one PipelineState")

    if index < 0:
        index = len(states) + index
    index = max(0, min(index, len(states) - 1))
    state = states[index]

    created = fig is None
    if fig is None:
        fig = plt.figure(figsize=FIGSIZE, dpi=FIGDPI, facecolor=BG)

    fig.clear()
    fig.patch.set_facecolor(BG)
    gs = GridSpec(
        3, 2,
        figure=fig,
        height_ratios=[2.42, 1.12, 1.02],
        width_ratios=[1.0, 1.0],
        hspace=0.38,
        wspace=0.22,
        left=0.045,
        right=0.985,
        top=0.875,
        bottom=0.085,
    )
    bottom = GridSpecFromSubplotSpec(
        1, 5, subplot_spec=gs[2, :], wspace=0.28, width_ratios=[1.15, 1.15, 1.15, 0.95, 0.95],
    )

    _draw_header(fig, state, states, index, source, sequence)

    ax_live = fig.add_subplot(gs[0, 0])
    ax_world = fig.add_subplot(gs[0, 1])
    ax_el = fig.add_subplot(gs[1, 0])
    ax_cls = fig.add_subplot(gs[1, 1])
    ax_time = fig.add_subplot(bottom[0, :3])
    ax_stat = fig.add_subplot(bottom[0, 3:])

    _draw_live_lidar(ax_live, state, mode=lidar_mode)
    _draw_world_map(ax_world, states, index)
    _draw_elevation(ax_el, state, fig)
    _draw_terrain_class(ax_cls, state)
    _draw_timeline(ax_time, states, index)
    _draw_status(ax_stat, states, index)

    if save_path:
        Path(save_path).parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(save_path, facecolor=fig.get_facecolor(), dpi=FIGDPI)

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
    from matplotlib.widgets import Slider

    fig = plt.figure(figsize=FIGSIZE, dpi=FIGDPI, facecolor=BG)

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
            slider_ax = fig.add_axes([0.18, 0.018, 0.50, 0.024], facecolor=PANEL)
            slider = Slider(
                slider_ax, "FRAME", 0, len(states) - 1,
                valinit=int(index), valstep=1, color=ACCENT,
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
            states, index=i, sequence=sequence, source=source,
            lidar_mode=lidar_mode, show=False, save_path=str(path),
        )
        plt.close(fig)
        paths.append(path)
    return paths
