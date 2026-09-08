"""
ORBIT dashboard colour language.

Colours encode pipeline quantities, not decoration.
Geometric classes are detector labels, not SemanticKITTI GT.
"""

# Surfaces
BG = "#0B0D10"
PANEL = "#12151C"
EDGE = "#2A3140"
TEXT = "#E8EDF4"
MUTED = "#8B93A7"
ACCENT = "#5BA3F5"
ORIGIN = "#F4F1EA"

# LiDAR terrain (RANSAC mask): ground vs non-ground
TERRAIN_GROUND = "#3FAE7A"
TERRAIN_NON_GROUND = "#C45C4A"

# Geometric ObjectProposal.classification
PROPOSAL_COLORS = {
    "VEHICLE-LIKE": "#4C9BE8",
    "WALL": "#D4A017",
    "POLE": "#C9C15A",
    "OBSTACLE": "#D65A5A",
}
PROPOSAL_FALLBACK = "#A0A8B8"

# Deterministic traversability from AdaptiveCell.semantic_class
TRAV_SAFE = "#2F9E6B"
TRAV_CAUTION = "#C9A227"
TRAV_BLOCKED = "#C44747"
TRAV_UNKNOWN = "#3A4250"

TRAV_LABELS = {
    0: "traversable (GROUND cells)",
    1: "caution (MIXED cells)",
    2: "blocked (OBSTACLE cells)",
}

ELEVATION_CMAP = "viridis"
TRACK_CONFIRMED = "#7EC8FF"
TRACK_UNCONFIRMED = "#6B7385"
TRACK_TRAIL = "#3D6F99"


def proposal_color(classification: str) -> str:
    return PROPOSAL_COLORS.get(str(classification), PROPOSAL_FALLBACK)
