"""
ORBIT Visual Intelligence colour language.

Colours encode pipeline quantities, not decoration.
Geometric classes are detector / grid labels, not SemanticKITTI GT
and not a learned traversability model.
"""

# Surfaces — dark scientific, not pure black
BG = "#0C1016"
PANEL = "#141A24"
EDGE = "#2C3545"
TEXT = "#E6EDF5"
MUTED = "#8A93A6"
ACCENT = "#3EC8FF"
ORIGIN = "#F4F1EA"

# Ego / motion
EGO = "#7DFFF5"
EGO_FILL = "#1A3A44"
TRAJECTORY = "#3D78E8"
TRAJECTORY_MARK = "#6AA0FF"
CURRENT = "#FFFFFF"
START = "#F0C14A"

# LiDAR terrain (RANSAC mask): ground vs non-ground
TERRAIN_GROUND = "#2FBF9A"
TERRAIN_NON_GROUND = "#E07050"

# Geometric ObjectProposal.classification
PROPOSAL_COLORS = {
    "VEHICLE-LIKE": "#4C9BE8",
    "WALL": "#D4A017",
    "POLE": "#C9C15A",
    "OBSTACLE": "#D65A5A",
}
PROPOSAL_FALLBACK = "#A0A8B8"

# Geometric AdaptiveCell.semantic_class (not learned drivability)
CELL_GROUND = "#2F9E6B"
CELL_MIXED = "#C9A227"
CELL_OBSTACLE = "#C44747"
CELL_UNKNOWN = "#3A4250"

# Back-compat aliases used by older tests / architecture diagram
TRAV_SAFE = CELL_GROUND
TRAV_CAUTION = CELL_MIXED
TRAV_BLOCKED = CELL_OBSTACLE
TRAV_UNKNOWN = CELL_UNKNOWN

TRAV_LABELS = {
    0: "GROUND",
    1: "MIXED",
    2: "OBSTACLE",
}

ELEVATION_CMAP = "turbo"
TRACK_CONFIRMED = "#8FD4FF"
TRACK_UNCONFIRMED = "#5A6578"
TRACK_TRAIL = "#3D6F99"


def proposal_color(classification: str) -> str:
    return PROPOSAL_COLORS.get(str(classification), PROPOSAL_FALLBACK)
