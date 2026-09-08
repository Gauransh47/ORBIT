"""
Visualization that consumes OrbitSystem pipeline state.

Does not run RANSAC, grid construction, or object detection.
"""

from typing import Any, Dict, Optional


def view_model_from_state(state) -> Dict[str, Any]:
    """Serializable view of one processed frame."""

    proposals = []
    for proposal in state.proposals:
        proposals.append(
            {
                "id": proposal.proposal_id,
                "center": proposal.center,
                "width": proposal.width,
                "length": proposal.length,
                "classification": proposal.classification,
                "confidence": proposal.confidence,
            }
        )

    tracks = []
    for track in state.tracks:
        tracks.append(
            {
                "track_id": int(track.track_id),
                "class_name": str(track.class_name),
                "position": (
                    float(track.position[0]),
                    float(track.position[1]),
                ),
                "confirmed": bool(track.confirmed),
                "missed": int(track.missed),
                "hits": int(track.hits),
            }
        )

    world_objects = []
    for obj in state.world_objects:
        world_objects.append(
            {
                "track_id": int(obj.track_id),
                "class_name": str(obj.class_name),
                "position": (
                    float(obj.position[0]),
                    float(obj.position[1]),
                ),
                "motion_state": str(obj.motion_state),
            }
        )

    return {
        "frame_index": state.frame_index,
        "proposals": proposals,
        "tracks": tracks,
        "world_objects": world_objects,
        "metrics": dict(state.metrics),
        "retired_ids": list(state.retired_ids),
        "world_frame": "lidar_frame_0",
    }


def render_topdown(state, ax=None, show=False):
    """
    Draw proposals (rectangles) and track IDs in XY.

    Returns the matplotlib Axes. Requires matplotlib.
    """

    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle

    created_fig = False
    if ax is None:
        _, ax = plt.subplots(figsize=(8, 8))
        created_fig = True

    for proposal in state.proposals:
        x, y = proposal.center
        rect = Rectangle(
            (x - proposal.width / 2.0, y - proposal.length / 2.0),
            proposal.width,
            proposal.length,
            fill=False,
            edgecolor="tab:red",
            linewidth=1.0,
        )
        ax.add_patch(rect)

    for track in state.tracks:
        ax.scatter(
            track.position[0],
            track.position[1],
            c="tab:blue",
            s=20,
            zorder=3,
        )
        ax.annotate(
            f"#{track.track_id}",
            (track.position[0], track.position[1]),
            fontsize=8,
            color="tab:blue",
        )

    ax.scatter([0], [0], marker="x", c="k", s=40, label="sensor")
    ax.set_aspect("equal")
    ax.set_xlabel("X (m) — LiDAR frame 0 / prototype world")
    ax.set_ylabel("Y (m)")
    ax.set_title(f"ORBIT prototype  frame {state.frame_index}")
    ax.grid(True, alpha=0.3)

    if show:
        plt.show()
    elif created_fig:
        plt.close(ax.figure)

    return ax
