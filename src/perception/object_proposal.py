"""
Canonical frame-level object proposal.

This is the geometric detector output consumed by the tracker adapter.
It is independent of SemanticKITTI labels and of the DBSCAN sidecar
in object_detection.py.
"""

from dataclasses import dataclass, field
from typing import Tuple


@dataclass
class ObjectProposal:
    proposal_id: int
    center: Tuple[float, float]
    width: float
    length: float
    height: float
    classification: str
    confidence: float
    distance: float
    point_count: int
    cell_count: int
    max_height: float
    mean_height: float
    density: float
    component_count: int = 1
    extra: dict = field(default_factory=dict)

    @property
    def center_x(self) -> float:
        return float(self.center[0])

    @property
    def center_y(self) -> float:
        return float(self.center[1])

    @property
    def semantic_class(self) -> str:
        """Tracker match key: geometric class name."""
        return self.classification

    def as_dict(self) -> dict:
        return {
            "proposal_id": self.proposal_id,
            "center": self.center,
            "width": self.width,
            "length": self.length,
            "height": self.height,
            "classification": self.classification,
            "confidence": self.confidence,
            "distance": self.distance,
            "point_count": self.point_count,
            "cell_count": self.cell_count,
            "max_height": self.max_height,
            "mean_height": self.mean_height,
            "density": self.density,
            "component_count": self.component_count,
        }
