"""
Adapt geometric ObjectProposal objects into tracker detections.

The tracker reads:
    center_x, center_y, width, length, semantic_class, confidence

Prototype world frame: LiDAR frame 0 (see tracking.orbit_tracker
WORLD_REFERENCE_FRAME). Existing tracks are not transformed again.
"""

from dataclasses import dataclass
from typing import List

from perception.object_proposal import ObjectProposal


@dataclass
class TrackerDetection:
    center_x: float
    center_y: float
    width: float
    length: float
    semantic_class: str
    confidence: float
    proposal_id: int


def proposals_to_detections(
    proposals: List[ObjectProposal],
) -> List[TrackerDetection]:
    detections = []

    for proposal in proposals:
        detections.append(
            TrackerDetection(
                center_x=float(proposal.center_x),
                center_y=float(proposal.center_y),
                width=float(proposal.width),
                length=float(proposal.length),
                semantic_class=str(proposal.classification),
                confidence=float(proposal.confidence),
                proposal_id=int(proposal.proposal_id),
            )
        )

    return detections
