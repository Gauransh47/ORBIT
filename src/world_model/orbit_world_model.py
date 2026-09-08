"""
ORBIT - Persistent World Model

Consumes temporal tracker tracks and converts them into
a persistent representation of the surrounding scene.

The World Model is deliberately separated from detection
and tracking.

Pipeline:

LiDAR
  -> Semantic Grid
  -> Object Association
  -> Temporal Tracking
  -> WORLD MODEL
"""

from dataclasses import dataclass, field
from typing import Dict, List

import numpy as np


# ============================================================
# CONFIGURATION
# ============================================================

STATIC_SPEED_THRESHOLD = 0.15
MOVING_SPEED_THRESHOLD = 0.50

MAX_HISTORY = 20

MIN_TRACK_HITS = 2


# ============================================================
# WORLD OBJECT
# ============================================================

@dataclass
class WorldObject:

    track_id: int

    class_name: str

    position: np.ndarray

    velocity: np.ndarray

    dimensions: np.ndarray

    confidence: float

    hits: int

    missed: int

    age: int

    first_frame: int

    last_frame: int

    confirmed: bool

    motion_state: str = "UNCERTAIN"

    history: List[np.ndarray] = field(
        default_factory=list
    )

    # --------------------------------------------------------

    @property
    def distance(self) -> float:

        return float(
            np.linalg.norm(
                self.position
            )
        )

    # --------------------------------------------------------

    @property
    def speed(self) -> float:

        return float(
            np.linalg.norm(
                self.velocity
            )
        )

    # --------------------------------------------------------

    @property
    def persistence(self) -> float:

        if self.age <= 0:
            return 0.0

        return min(
            1.0,
            self.hits / float(self.age)
        )

    # --------------------------------------------------------

    def update_from_track(
        self,
        track,
        frame_index: int,
    ):

        self.position = np.asarray(
            track.position,
            dtype=np.float32,
        ).copy()

        self.velocity = np.asarray(
            track.velocity,
            dtype=np.float32,
        ).copy()

        self.dimensions = np.asarray(
            track.dimensions,
            dtype=np.float32,
        ).copy()

        self.confidence = float(
            track.confidence
        )

        self.hits = int(
            track.hits
        )

        self.missed = int(
            track.missed
        )

        self.age = int(
            track.age
        )

        self.confirmed = bool(
            track.confirmed
        )

        self.last_frame = frame_index

        self.history.append(
            self.position.copy()
        )

        if len(self.history) > MAX_HISTORY:
            self.history.pop(0)

        # The temporal tracker is the source of truth
        # for motion classification.
        self.motion_state = str(
            getattr(
                track,
                "motion_state",
                "UNCERTAIN",
            )
        )

        if self.missed > 0:
            self.motion_state = "UNCERTAIN"


# ============================================================
# MOTION CLASSIFICATION
# ============================================================

def classify_motion(
    speed: float,
    hits: int,
    missed: int,
) -> str:

    if missed > 0:
        return "UNCERTAIN"

    if hits < MIN_TRACK_HITS:
        return "UNCERTAIN"

    if speed < STATIC_SPEED_THRESHOLD:
        return "STATIC"

    if speed >= MOVING_SPEED_THRESHOLD:
        return "MOVING"

    return "UNCERTAIN"


# ============================================================
# WORLD MODEL
# ============================================================

class OrbitWorldModel:

    def __init__(self):

        self.objects: Dict[int, WorldObject] = {}

        self.frame_index = -1

        self.frames_processed = 0

    # --------------------------------------------------------

    def update(
        self,
        tracks,
        frame_index: int,
    ):

        self.frame_index = int(frame_index)

        self.frames_processed += 1

        seen_ids = set()

        for track in tracks:

            track_id = int(
                track.track_id
            )

            seen_ids.add(track_id)

            # ------------------------------------------------
            # Existing object
            # ------------------------------------------------

            if track_id in self.objects:

                world_object = self.objects[
                    track_id
                ]

                world_object.update_from_track(
                    track,
                    frame_index,
                )

            # ------------------------------------------------
            # New object
            # ------------------------------------------------

            else:

                position = np.asarray(
                    track.position,
                    dtype=np.float32,
                ).copy()

                velocity = np.asarray(
                    track.velocity,
                    dtype=np.float32,
                ).copy()

                dimensions = np.asarray(
                    track.dimensions,
                    dtype=np.float32,
                ).copy()

                world_object = WorldObject(

                    track_id=track_id,

                    class_name=str(
                        track.class_name
                    ),

                    position=position,

                    velocity=velocity,

                    dimensions=dimensions,

                    confidence=float(
                        track.confidence
                    ),

                    hits=int(
                        track.hits
                    ),

                    missed=int(
                        track.missed
                    ),

                    age=int(
                        track.age
                    ),

                    first_frame=int(
                        track.first_frame
                    ),

                    last_frame=int(
                        track.last_frame
                    ),

                    confirmed=bool(
                        track.confirmed
                    ),

                    motion_state=str(
                        getattr(
                            track,
                            "motion_state",
                            "UNCERTAIN",
                        )
                    ),

                    history=[
                        position.copy()
                    ],
                )

                self.objects[track_id] = world_object

        # ----------------------------------------------------
        # Objects not seen this frame
        # ----------------------------------------------------

        for track_id, obj in self.objects.items():

            if track_id not in seen_ids:

                # The tracker already owns the actual
                # missed count. This is only a world-model
                # fallback for objects absent from input.
                obj.motion_state = "UNCERTAIN"


    def sync(
        self,
        tracks,
        frame_index: int,
    ):
        """
        Mirror the tracker's live/coasted tracks.

        Tracks the tracker has pruned are removed from the world
        model so they do not remain as permanent UNCERTAIN ghosts.
        """

        live_ids = {
            int(track.track_id)
            for track in tracks
        }

        self.update(tracks, frame_index)

        retired = []

        for track_id in list(self.objects.keys()):
            if track_id not in live_ids:
                del self.objects[track_id]
                retired.append(track_id)

        return retired

    # --------------------------------------------------------

    def active_objects(
        self,
        confirmed_only: bool = True,
    ) -> List[WorldObject]:

        objects = list(
            self.objects.values()
        )

        if confirmed_only:

            objects = [
                obj
                for obj in objects
                if obj.confirmed
            ]

        return sorted(
            objects,
            key=lambda obj: obj.distance,
        )

    # --------------------------------------------------------

    def objects_by_class(
        self,
        class_name: str,
    ) -> List[WorldObject]:

        return [
            obj
            for obj in self.objects.values()
            if obj.class_name == class_name
        ]

    # --------------------------------------------------------

    def moving_objects(self) -> List[WorldObject]:

        return [
            obj
            for obj in self.objects.values()
            if obj.motion_state == "MOVING"
        ]

    # --------------------------------------------------------

    def static_objects(self) -> List[WorldObject]:

        return [
            obj
            for obj in self.objects.values()
            if obj.motion_state == "STATIC"
        ]

    # --------------------------------------------------------

    def nearest(
        self,
        count: int = 10,
    ) -> List[WorldObject]:

        return self.active_objects(
            confirmed_only=True
        )[:count]

    # --------------------------------------------------------

    def summary(self) -> dict:

        objects = self.active_objects()

        classes = {}

        for obj in objects:

            classes[obj.class_name] = (
                classes.get(
                    obj.class_name,
                    0,
                ) + 1
            )

        moving = sum(
            obj.motion_state == "MOVING"
            for obj in objects
        )

        static = sum(
            obj.motion_state == "STATIC"
            for obj in objects
        )

        uncertain = sum(
            obj.motion_state == "UNCERTAIN"
            for obj in objects
        )

        return {

            "frame": self.frame_index,

            "frames_processed":
                self.frames_processed,

            "total_objects":
                len(objects),

            "moving":
                moving,

            "static":
                static,

            "uncertain":
                uncertain,

            "classes":
                classes,
        }


# ============================================================
# DISPLAY
# ============================================================

def print_world_object(
    obj: WorldObject,
):

    x = obj.position[0]
    y = obj.position[1]

    print(
        f"  #{obj.track_id:03d} "
        f"{obj.class_name:<18} "
        f"| {obj.motion_state:<9} "
        f"| conf={obj.confidence * 100:5.1f}% "
        f"| pos=({x:7.2f}, {y:7.2f}) "
        f"| speed={obj.speed:5.2f} "
        f"| hits={obj.hits:2d} "
        f"| age={obj.age:2d}"
    )


def print_world_summary(
    world: OrbitWorldModel,
):

    summary = world.summary()

    print()
    print("=" * 70)

    print(
        "ORBIT WORLD MODEL"
    )

    print("=" * 70)

    print(
        f"\nFrame:             "
        f"{summary['frame']:06d}"
    )

    print(
        f"Frames processed:  "
        f"{summary['frames_processed']}"
    )

    print(
        f"Persistent objects:"
        f" {summary['total_objects']}"
    )

    print(
        f"Moving:            "
        f"{summary['moving']}"
    )

    print(
        f"Static:            "
        f"{summary['static']}"
    )

    print(
        f"Uncertain:         "
        f"{summary['uncertain']}"
    )

    print(
        "\nCLASS DISTRIBUTION"
    )

    print("-" * 70)

    for class_name, count in sorted(
        summary["classes"].items()
    ):

        print(
            f"  {class_name:<22}"
            f"{count:>4}"
        )


def print_nearest(
    world: OrbitWorldModel,
    count: int = 15,
):

    objects = world.nearest(count)

    print(
        "\nNEAREST WORLD OBJECTS"
    )

    print("-" * 70)

    if not objects:

        print(
            "  No confirmed objects."
        )

        return

    for obj in objects:

        print_world_object(obj)


# ============================================================
# SERIALIZATION
# ============================================================

def world_object_to_dict(
    obj: WorldObject,
) -> dict:

    return {

        "track_id":
            obj.track_id,

        "class":
            obj.class_name,

        "position": [
            float(obj.position[0]),
            float(obj.position[1]),
        ],

        "velocity": [
            float(obj.velocity[0]),
            float(obj.velocity[1]),
        ],

        "speed":
            obj.speed,

        "distance":
            obj.distance,

        "dimensions": [
            float(obj.dimensions[0]),
            float(obj.dimensions[1]),
        ],

        "confidence":
            obj.confidence,

        "hits":
            obj.hits,

        "missed":
            obj.missed,

        "age":
            obj.age,

        "first_frame":
            obj.first_frame,

        "last_frame":
            obj.last_frame,

        "motion_state":
            obj.motion_state,

        "persistence":
            obj.persistence,

        "confirmed":
            obj.confirmed,

        "history": [
            [
                float(p[0]),
                float(p[1]),
            ]
            for p in obj.history
        ],
    }


# ============================================================
# SIMPLE TEST
# ============================================================

if __name__ == "__main__":

    print("=" * 70)

    print(
        "ORBIT - WORLD MODEL MODULE"
    )

    print("=" * 70)

    print(
        "\nWorld model initialized."
    )

    print(
        "Waiting for tracker input..."
    )

    print(
        "\nExpected tracker attributes:"
    )

    print("  track_id")
    print("  class_name")
    print("  position")
    print("  velocity")
    print("  dimensions")
    print("  confidence")
    print("  hits")
    print("  missed")
    print("  age")
    print("  first_frame")
    print("  last_frame")
    print("  confirmed")

    print(
        "\nWORLD MODEL READY"
    )

    print("=" * 70)