"""
ORBIT Visual Intelligence colour language (Dashboard V3).

Colours encode pipeline quantities, not decoration.
Geometric classes are detector / grid labels, not learned drivability.
"""

from matplotlib.colors import LinearSegmentedColormap

# Surfaces
BG = "#070B12"
PANEL = "#10151F"
EDGE = "#2A3344"
TEXT = "#E8EEF6"
MUTED = "#7E8899"
ACCENT = "#3DDCFF"
ORIGIN = "#F4F1EA"

# Ego / motion
EGO = "#5CFFF0"
EGO_FILL = "#12383C"
TRAJECTORY = "#3B82F6"
TRAJECTORY_MARK = "#7EB0FF"
CURRENT = "#FFFFFF"
START = "#F5C542"
LIVE_DOT = "#3DFF9A"

# LiDAR terrain (RANSAC mask)
TERRAIN_GROUND = "#2FBF9A"
TERRAIN_NON_GROUND = "#E07050"

# Geometric ObjectProposal / track class
PROPOSAL_COLORS = {
    "VEHICLE-LIKE": "#3DDCFF",
    "WALL": "#E8893A",
    "POLE": "#A78BFA",
    "OBSTACLE": "#F05252",
}
PROPOSAL_FALLBACK = "#8A93A4"

CELL_GROUND = "#1FA97A"
CELL_MIXED = "#D4A017"
CELL_OBSTACLE = "#D44545"
CELL_UNKNOWN = "#3A4250"

TRAV_SAFE = CELL_GROUND
TRAV_CAUTION = CELL_MIXED
TRAV_BLOCKED = CELL_OBSTACLE
TRAV_UNKNOWN = CELL_UNKNOWN

TRAV_LABELS = {
    0: "GROUND",
    1: "MIXED",
    2: "OBSTACLE",
}

ELEVATION_CMAP = LinearSegmentedColormap.from_list(
    "orbit_elevation",
    ["#3B6BFF", "#19D0E0", "#3EE89A", "#F4E14C", "#FF8E24", "#FF4E3A"],
)

TRACK_CONFIRMED = "#9AE7FF"
TRACK_UNCONFIRMED = "#4B5568"
TRACK_TRAIL = "#2F5F88"


def proposal_color(classification: str) -> str:
    return PROPOSAL_COLORS.get(str(classification), PROPOSAL_FALLBACK)
