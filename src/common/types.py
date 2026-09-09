"""
Shared diagnostic records.

Perception runtime types (AdaptiveCell, ObjectComponent, ObjectGroup,
ObjectHypothesis, Track, WorldObject) stay in the modules that own
them so existing imports are unchanged.

These records are JSON-friendly summaries used by evaluation tools.
"""

from dataclasses import asdict, dataclass, field
from typing import Any, Dict, List, Optional, Tuple


CellKey = Tuple[int, int, int]


@dataclass
class MergeEvent:
    """One detection assembled from more than one proposal/component."""

    object_id: int
    classification: str
    component_ids: List[int]
    component_count: int
    point_count: int
    cell_count: int
    center: Tuple[float, float]
    dimensions: Tuple[float, float, float]
    distance: float
    instance_ids: List[int] = field(default_factory=list)
    semantic_ids: List[int] = field(default_factory=list)
    overmerged: Optional[bool] = None
    notes: List[str] = field(default_factory=list)

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)


@dataclass
class MergeAnalysis:
    """Summary of proposal-merging behaviour on one frame."""

    source: str
    mode: str
    frame: Optional[str]
    input_points: int
    raw_components: int
    filtered_components: int
    final_objects: int
    merged_objects: int
    overmerged_objects: int
    events: List[MergeEvent] = field(default_factory=list)

    def to_dict(self) -> Dict[str, Any]:
        payload = asdict(self)
        return payload
