"""
ORBIT - TEMPORAL OBJECT TRACKER

World-coordinate temporal tracker for SemanticKITTI.

Key design:
    - Every detection is converted from LiDAR coordinates into
      a single persistent world/reference coordinate system.
    - Tracks are stored ONLY in world coordinates.
    - Ego motion is NOT applied to existing tracks during matching.
    - Therefore vehicle/LiDAR motion is separated from object motion.
    - Static objects should remain nearly stationary in world space.
"""

from pathlib import Path
import argparse

import numpy as np

from src.datasets.kitti_loader import KittiLoader
from src.mapping.adaptive_grid import AdaptiveGrid
from src.mapping.semantic_grid import SemanticGrid
from src.perception.semantic_object_association import SemanticObjectAssociator
from src.tracking.ego_motion import EgoMotionCompensator

import src.world_model.orbit_world_model


# ============================================================
# CONFIGURATION
# ============================================================

# Motion thresholds are in WORLD coordinates and metres/second.
# Static objects should have only small residual world-space motion.
STATIC_SPEED_THRESHOLD = 0.15
MOVING_SPEED_THRESHOLD = 0.50

# Motion must persist across multiple observations before an object
# is declared MOVING. This prevents one noisy association from
# making a building/pole/trunk appear to move.
MOVING_CONFIRMATION_FRAMES = 3
STATIC_CONFIRMATION_FRAMES = 3
MOTION_NOISE_THRESHOLD = 0.15

DYNAMIC_CLASSES = {
    "car", "moving-car",
    "bicycle", "moving-bicyclist",
    "motorcycle", "moving-motorcyclist",
    "person", "moving-person",
    "bus", "moving-bus",
    "truck", "moving-truck",
    "other-vehicle", "moving-other-vehicle",
    "on-rails", "moving-on-rails",
}

STATIC_CLASSES = {
    "building", "wall", "fence",
    "traffic-sign", "pole", "trunk",
    "terrain", "vegetation",
}

# Safe fallback for datasets without times.txt. SemanticKITTI/KITTI
# odometry is normally recorded at about 10 Hz. We still report that
# the fallback is being used rather than silently pretending it is exact.
DEFAULT_FRAME_DT = 0.10

DATASET_ROOT = Path("data/semantic_kitti")
DEFAULT_SEQUENCE = "00"

MAX_MATCH_DISTANCE = 3.0
MAX_DIMENSION_DIFFERENCE = 4.0

MAX_MISSED_FRAMES = 2
MIN_CONFIRMATION_HITS = 2

# Reference/world frame. KITTI pose 0 is identity in this dataset.
WORLD_REFERENCE_FRAME = 0


# ============================================================
# DETECTION HELPERS
# ============================================================

def detection_position(detection):
    """Return detection center as XY float32."""
    return np.array(
        [
            detection.center_x,
            detection.center_y,
        ],
        dtype=np.float32,
    )


def detection_dimensions(detection):
    """Return detection width/length as float32."""
    return np.array(
        [
            detection.width,
            detection.length,
        ],
        dtype=np.float32,
    )


# ============================================================
# TRACK
# ============================================================

class Track:

    def __init__(
        self,
        track_id,
        detection,
        frame_index,
        world_position,
    ):
        self.track_id = track_id

        self.class_name = detection.semantic_class

        self.position = np.asarray(
            world_position,
            dtype=np.float32,
        ).copy()

        self.previous_position = self.position.copy()

        # World-space velocity in metres/second.
        self.velocity = np.zeros(
            2,
            dtype=np.float32,
        )

        self.dimensions = detection_dimensions(detection)

        self.confidence = float(detection.confidence)

        self.hits = 1
        self.missed = 0
        self.age = 1

        self.first_frame = frame_index
        self.last_frame = frame_index

        self.confirmed = (
            MIN_CONFIRMATION_HITS <= 1
        )

        self.motion_state = (
            "STATIC"
            if self.class_name in STATIC_CLASSES
            else "UNKNOWN"
        )

        self.raw_speed = 0.0
        self.dt = DEFAULT_FRAME_DT
        self.timestamp = None

        # Temporal evidence for robust motion classification.
        self.moving_streak = 0
        self.stationary_streak = 0
        self.motion_history = []

    # ========================================================
    # UPDATE
    # ========================================================

    def update(
        self,
        detection,
        frame_index,
        world_position,
        dt=DEFAULT_FRAME_DT,
        timestamp=None,
    ):
        new_position = np.asarray(
            world_position,
            dtype=np.float32,
        )

        displacement = (
            new_position - self.position
        )

        # Convert displacement into a physical velocity.
        # dt is the actual elapsed time between observations.
        dt = float(dt)
        if not np.isfinite(dt) or dt <= 1e-6:
            dt = DEFAULT_FRAME_DT

        instantaneous_velocity = displacement / dt

        # Smooth velocity in metres/second.
        self.velocity = (
            0.7 * self.velocity
            + 0.3 * instantaneous_velocity
        )

        self.raw_speed = float(
            np.linalg.norm(instantaneous_velocity)
        )
        self.dt = dt
        self.timestamp = timestamp

        self.previous_position = (
            self.position.copy()
        )

        self.position = new_position.copy()

        self.dimensions = detection_dimensions(
            detection
        )

        self.confidence = float(
            detection.confidence
        )

        self.class_name = detection.semantic_class

        self.hits += 1
        self.missed = 0
        self.age += 1
        self.last_frame = frame_index

        if self.hits >= MIN_CONFIRMATION_HITS:
            self.confirmed = True

        self.update_motion_state()

    # ========================================================
    # MOTION CLASSIFICATION
    # ========================================================

    def update_motion_state(self):
        """
        Robust temporal motion classification in WORLD coordinates.

        Static semantic classes are never promoted directly to MOVING.
        Dynamic classes require persistent motion across several frames.
        Small residual motion is treated as measurement noise.
        """

        speed = float(np.linalg.norm(self.velocity))

        # Keep a short history for diagnostics.
        self.motion_history.append(speed)
        if len(self.motion_history) > 8:
            self.motion_history.pop(0)

        # --------------------------------------------------------
        # STATIC CLASSES
        # --------------------------------------------------------
        if self.class_name in STATIC_CLASSES:
            self.moving_streak = 0

            if speed <= MOTION_NOISE_THRESHOLD:
                self.stationary_streak += 1
                self.motion_state = "STATIC"

                # Suppress tiny residual world-space drift.
                self.velocity[:] = 0.0
            else:
                self.stationary_streak = 0

                # A building/pole/etc. is not called MOVING from
                # one noisy measurement.
                self.motion_state = "UNCERTAIN"

            return

        # --------------------------------------------------------
        # KNOWN DYNAMIC CLASSES
        # --------------------------------------------------------
        if self.class_name in DYNAMIC_CLASSES:
            if speed >= MOVING_SPEED_THRESHOLD:
                self.moving_streak += 1
                self.stationary_streak = 0

                if self.moving_streak >= MOVING_CONFIRMATION_FRAMES:
                    self.motion_state = "MOVING"
                elif self.motion_state == "MOVING":
                    self.motion_state = "MOVING"
                else:
                    self.motion_state = "UNCERTAIN"

            elif speed <= MOTION_NOISE_THRESHOLD:
                self.stationary_streak += 1
                self.moving_streak = 0

                if self.stationary_streak >= STATIC_CONFIRMATION_FRAMES:
                    self.motion_state = "STATIC"
                elif self.motion_state == "MOVING":
                    self.motion_state = "MOVING"
                else:
                    self.motion_state = "UNCERTAIN"

            else:
                self.moving_streak = max(0, self.moving_streak - 1)
                self.stationary_streak = max(0, self.stationary_streak - 1)

                if self.motion_state not in {"MOVING", "STATIC"}:
                    self.motion_state = "UNCERTAIN"

            return

        # --------------------------------------------------------
        # UNKNOWN / OTHER OBJECTS
        # --------------------------------------------------------
        if speed >= MOVING_SPEED_THRESHOLD:
            self.moving_streak += 1
            self.stationary_streak = 0

            if self.moving_streak >= MOVING_CONFIRMATION_FRAMES:
                self.motion_state = "MOVING"
            else:
                self.motion_state = "UNCERTAIN"

        elif speed <= MOTION_NOISE_THRESHOLD:
            self.stationary_streak += 1
            self.moving_streak = 0

            if self.stationary_streak >= STATIC_CONFIRMATION_FRAMES:
                self.motion_state = "STATIC"
            else:
                self.motion_state = "UNCERTAIN"

        else:
            self.moving_streak = max(0, self.moving_streak - 1)
            self.stationary_streak = max(0, self.stationary_streak - 1)
            self.motion_state = "UNCERTAIN"

    # ========================================================
    # MISSED
    # ========================================================

    def mark_missed(self):
        self.missed += 1
        self.age += 1

    # ========================================================
    # PREDICTION
    # ========================================================

    def predicted_position(self, dt=None):
        if dt is None:
            dt = self.dt

        dt = float(dt)
        if not np.isfinite(dt) or dt <= 1e-6:
            dt = DEFAULT_FRAME_DT

        return self.position + self.velocity * dt

    # ========================================================
    # DISTANCE
    # ========================================================

    def distance_to(self, world_position, dt=None):
        world_position = np.asarray(
            world_position,
            dtype=np.float32,
        )

        return float(
            np.linalg.norm(
                self.predicted_position(dt)
                - world_position
            )
        )

    # ========================================================
    # DIMENSION DIFFERENCE
    # ========================================================

    def dimension_difference(self, detection):
        dimensions = detection_dimensions(
            detection
        )

        return float(
            np.linalg.norm(
                self.dimensions - dimensions
            )
        )


# ============================================================
# TRACKER
# ============================================================

class OrbitTracker:

    def __init__(
        self,
        ego_motion=None,
        world_reference_frame=WORLD_REFERENCE_FRAME,
    ):
        self.tracks = []
        self.next_track_id = 1

        self.total_matches = 0
        self.total_new_tracks = 0
        self.total_lost_tracks = 0

        self.ego_motion = ego_motion
        self.world_reference_frame = int(
            world_reference_frame
        )

        self.current_frame = None
        self.timestamps = None
        self.using_fallback_dt = False

    # ========================================================
    # WORLD COORDINATE CONVERSION
    # ========================================================

    def to_world_position(
        self,
        position,
        source_frame,
    ):
        """
        Convert an XY LiDAR detection into the persistent
        world/reference coordinate system.

        We intentionally do NOT transform the existing tracks.
        Every new detection is independently brought into the
        same world frame.

        The object center has no Z estimate in the hypothesis
        object, so z=0 is used consistently.
        """

        position = np.asarray(
            position,
            dtype=np.float64,
        )

        if position.shape != (2,):
            raise ValueError(
                "Expected XY position with shape (2,), "
                f"got {position.shape}"
            )

        if self.ego_motion is None:
            return position.astype(np.float32)

        xyz = np.array(
            [
                position[0],
                position[1],
                0.0,
            ],
            dtype=np.float64,
        )

        world_xyz = (
            self.ego_motion.transform_point(
                xyz,
                source_frame=int(source_frame),
                target_frame=self.world_reference_frame,
            )
        )

        return world_xyz[:2].astype(np.float32)

    # ========================================================
    # TIMESTAMP CONFIGURATION
    # ========================================================

    def set_timestamps(self, timestamps):
        """Attach dataset timestamps (seconds) to the tracker."""
        values = np.asarray(timestamps, dtype=np.float64).reshape(-1)
        if len(values) == 0:
            self.timestamps = None
            self.using_fallback_dt = True
            return

        if np.any(~np.isfinite(values)):
            raise ValueError("timestamps contain non-finite values")

        self.timestamps = values
        self.using_fallback_dt = False

    # ========================================================
    # TIME DELTA
    # ========================================================

    def time_delta(self, source_frame, target_frame):
        """Return elapsed seconds between two frame indices."""
        if self.timestamps is None:
            return DEFAULT_FRAME_DT * abs(
                int(target_frame) - int(source_frame)
            )

        source_frame = int(source_frame)
        target_frame = int(target_frame)

        if (
            source_frame < 0
            or target_frame < 0
            or source_frame >= len(self.timestamps)
            or target_frame >= len(self.timestamps)
        ):
            return DEFAULT_FRAME_DT * abs(
                target_frame - source_frame
            )

        dt = float(
            self.timestamps[target_frame]
            - self.timestamps[source_frame]
        )

        if not np.isfinite(dt) or dt <= 1e-6:
            return DEFAULT_FRAME_DT * max(
                1, abs(target_frame - source_frame)
            )

        return dt

    # ========================================================
    # CREATE TRACK
    # ========================================================

    def create_track(
        self,
        detection,
        frame_index,
        world_position,
    ):
        track = Track(
            self.next_track_id,
            detection,
            frame_index,
            world_position,
        )

        self.next_track_id += 1
        self.tracks.append(track)
        self.total_new_tracks += 1

        return track

    # ========================================================
    # MATCH SCORE
    # ========================================================

    def match_score(
        self,
        track,
        detection,
        world_position,
        dt=DEFAULT_FRAME_DT,
    ):
        # Semantic consistency.
        if (
            track.class_name
            != detection.semantic_class
        ):
            return None

        # Position in the SAME world coordinate system.
        distance = track.distance_to(
            world_position,
            dt=dt,
        )

        if distance > MAX_MATCH_DISTANCE:
            return None

        # Dimension consistency.
        dimension_difference = (
            track.dimension_difference(
                detection
            )
        )

        if (
            dimension_difference
            > MAX_DIMENSION_DIFFERENCE
        ):
            return None

        distance_score = (
            distance / MAX_MATCH_DISTANCE
        )

        dimension_score = (
            dimension_difference
            / MAX_DIMENSION_DIFFERENCE
        )

        confidence_score = (
            1.0 - float(detection.confidence)
        )

        return (
            0.65 * distance_score
            + 0.25 * dimension_score
            + 0.10 * confidence_score
        )

    # ========================================================
    # ASSOCIATE
    # ========================================================

    def associate(
        self,
        detections,
        frame_index,
        actual_frame=None,
    ):
        if actual_frame is None:
            actual_frame = frame_index

        actual_frame = int(actual_frame)
        self.current_frame = actual_frame

        # ----------------------------------------------------
        # Convert ALL detections into world coordinates.
        # ----------------------------------------------------

        world_positions = [
            self.to_world_position(
                detection_position(detection),
                actual_frame,
            )
            for detection in detections
        ]

        # ----------------------------------------------------
        # First frame.
        # ----------------------------------------------------

        if not self.tracks:
            for detection, world_position in zip(
                detections,
                world_positions,
            ):
                self.create_track(
                    detection,
                    actual_frame,
                    world_position,
                )

            return

        # ----------------------------------------------------
        # Candidate matches.
        #
        # IMPORTANT:
        # Existing tracks are already in world coordinates.
        # Do NOT compensate them again.
        # ----------------------------------------------------

        candidates = []

        for track_index, track in enumerate(
            self.tracks
        ):
            for detection_index, detection in enumerate(
                detections
            ):
                world_position = world_positions[
                    detection_index
                ]

                dt = self.time_delta(
                    track.last_frame,
                    actual_frame,
                )

                score = self.match_score(
                    track,
                    detection,
                    world_position,
                    dt=dt,
                )

                if score is not None:
                    candidates.append(
                        (
                            score,
                            track_index,
                            detection_index,
                        )
                    )

        candidates.sort(
            key=lambda x: x[0]
        )

        # ----------------------------------------------------
        # Greedy one-to-one assignment.
        # ----------------------------------------------------

        matched_tracks = set()
        matched_detections = set()
        matches = []

        for (
            score,
            track_index,
            detection_index,
        ) in candidates:

            if track_index in matched_tracks:
                continue

            if detection_index in matched_detections:
                continue

            matched_tracks.add(track_index)
            matched_detections.add(detection_index)

            matches.append(
                (
                    track_index,
                    detection_index,
                    score,
                )
            )

        # ----------------------------------------------------
        # Update matched tracks.
        # ----------------------------------------------------

        for (
            track_index,
            detection_index,
            score,
        ) in matches:

            track = self.tracks[track_index]
            detection = detections[detection_index]
            world_position = world_positions[
                detection_index
            ]

            dt = self.time_delta(
                track.last_frame,
                actual_frame,
            )

            timestamp = (
                None
                if self.timestamps is None
                else float(self.timestamps[actual_frame])
            )

            track.update(
                detection,
                actual_frame,
                world_position,
                dt=dt,
                timestamp=timestamp,
            )

            self.total_matches += 1

        # ----------------------------------------------------
        # Mark unmatched tracks.
        # ----------------------------------------------------

        for track_index, track in enumerate(
            self.tracks
        ):
            if track_index not in matched_tracks:
                track.mark_missed()

        # ----------------------------------------------------
        # Create tracks for unmatched detections.
        # ----------------------------------------------------

        for detection_index, detection in enumerate(
            detections
        ):
            if detection_index not in matched_detections:

                self.create_track(
                    detection,
                    actual_frame,
                    world_positions[
                        detection_index
                    ],
                )

        # ----------------------------------------------------
        # Remove dead tracks.
        # ----------------------------------------------------

        alive_tracks = []

        for track in self.tracks:

            if track.missed <= MAX_MISSED_FRAMES:
                alive_tracks.append(track)

            else:
                self.total_lost_tracks += 1

        self.tracks = alive_tracks

    # ========================================================
    # ACTIVE TRACKS
    # ========================================================

    def active_tracks(self):
        return [
            track
            for track in self.tracks
            if track.confirmed
        ]


# ============================================================
# PERCEPTION
# ============================================================

def extract_detections(
    dataset,
    frame,
    associator,
):
    points, labels, instances = (
        dataset.load_frame(
            frame,
            load_labels=True,
        )
    )

    if labels is None:
        raise RuntimeError(
            f"No semantic labels for frame {frame}"
        )

    xyz = points[:, :3]

    print(
        f"Input points: {len(xyz):,}"
    )

    print(
        "\nRunning geometric ground detection..."
    )

    grid = AdaptiveGrid()

    try:
        ground_mask = grid.detect_ground(xyz)

    except (
        AttributeError,
        TypeError,
    ):
        ground_mask = xyz[:, 2] < 0.0

    grid.build(
        xyz,
        ground_mask,
    )

    print(
        "Building semantic grid..."
    )

    semantic_grid = SemanticGrid(grid)

    semantic_grid.build(
        xyz,
        labels,
    )

    print(
        "Associating semantic objects..."
    )

    objects = associator.associate(
        semantic_grid
    )

    return objects


# ============================================================
# PRINT TRACKS
# ============================================================

def print_tracks(tracker):
    active = tracker.active_tracks()

    print(
        f"\nACTIVE TRACKS: {len(active)}"
    )

    print("-" * 70)

    if not active:
        print(
            "  No confirmed tracks yet."
        )
        return

    for track in sorted(
        active,
        key=lambda t: np.linalg.norm(
            t.position
        ),
    ):

        distance = float(
            np.linalg.norm(
                track.position
            )
        )

        speed = float(
            np.linalg.norm(
                track.velocity
            )
        )

        print(
            f"  Track #{track.track_id:03d} | "
            f"{track.class_name:<18} | "
            f"state={track.motion_state:<9} | "
            f"conf={track.confidence * 100:5.1f}% | "
            f"world=("
            f"{track.position[0]:7.2f}, "
            f"{track.position[1]:7.2f}"
            f") | "
            f"dist={distance:6.2f}m | "
            f"vel={speed:6.2f}m/s | "
            f"hits={track.hits:2d} | "
            f"missed={track.missed}"
        )


# ============================================================
# ARGUMENTS
# ============================================================

def get_args():
    parser = argparse.ArgumentParser(
        description=(
            "ORBIT world-coordinate temporal tracker"
        )
    )

    parser.add_argument(
        "--sequence",
        type=str,
        default=DEFAULT_SEQUENCE,
    )

    parser.add_argument(
        "--start",
        type=int,
        default=0,
    )

    parser.add_argument(
        "--end",
        type=int,
        default=None,
    )

    return parser.parse_args()


# ============================================================
# TIMESTAMPS
# ============================================================

def load_sequence_timestamps(dataset_root, sequence):
    """Load KITTI/Sequence timestamps from times.txt."""
    path = (
        Path(dataset_root)
        / "sequences"
        / str(sequence).zfill(2)
        / "times.txt"
    )

    if not path.exists():
        return None

    values = np.loadtxt(
        path,
        dtype=np.float64,
    )

    values = np.asarray(values, dtype=np.float64).reshape(-1)

    if len(values) == 0:
        return None

    return values


# ============================================================
# MAIN
# ============================================================

def main():

    args = get_args()

    sequence = str(
        args.sequence
    ).zfill(2)

    print("=" * 70)
    print(
        "ORBIT - WORLD-COORDINATE TEMPORAL TRACKER"
    )
    print("=" * 70)

    dataset = KittiLoader(
        DATASET_ROOT,
        sequence=sequence,
    )

    available_frames = dataset.frame_ids()

    if not available_frames:
        raise RuntimeError(
            "No LiDAR frames found."
        )

    start = max(
        0,
        args.start,
    )

    if start >= len(available_frames):
        raise ValueError(
            f"Start frame {start} is outside "
            f"available range."
        )

    if args.end is None:
        end = len(available_frames) - 1
    else:
        end = min(
            args.end,
            len(available_frames) - 1,
        )

    if end < start:
        raise ValueError(
            "End frame must be >= start frame."
        )

    frames = available_frames[
        start : end + 1
    ]

    print(
        f"\nSequence:     {sequence}"
    )

    print(
        f"Frames:       {len(frames)}"
    )

    print(
        f"Range:        "
        f"{frames[0]} → {frames[-1]}"
    )

    # ========================================================
    # EGO MOTION
    # ========================================================

    ego_motion = EgoMotionCompensator(
        DATASET_ROOT,
        sequence=sequence,
    )

    print(
        f"\nLoaded poses: "
        f"{len(ego_motion.poses)}"
    )

    timestamps = load_sequence_timestamps(
        DATASET_ROOT,
        sequence,
    )

    if timestamps is None:
        print(
            f"Timestamps:    unavailable "
            f"(using {DEFAULT_FRAME_DT:.2f}s fallback)"
        )
    else:
        print(
            f"Timestamps:    {len(timestamps)} loaded"
        )

    print(
        f"World reference frame: "
        f"{WORLD_REFERENCE_FRAME:06d}"
    )

    # ========================================================
    # TRACKER
    # ========================================================

    tracker = OrbitTracker(
        ego_motion=ego_motion,
        world_reference_frame=WORLD_REFERENCE_FRAME,
    )

    if timestamps is not None:
        tracker.set_timestamps(timestamps)

    associator = SemanticObjectAssociator()

    # ========================================================
    # WORLD MODEL
    # ========================================================

    world_model = (
        src.world_model.orbit_world_model
        .OrbitWorldModel()
    )

    # ========================================================
    # PROCESS FRAMES
    # ========================================================

    previous_frame = None

    for frame in frames:

        actual_frame = int(frame)

        print(
            "\n" + "=" * 70
        )

        print(
            f"FRAME {frame}"
        )

        print(
            "=" * 70
        )

        detections = extract_detections(
            dataset,
            frame,
            associator,
        )

        print(
            f"Detections:    "
            f"{len(detections)}"
        )

        # ----------------------------------------------------
        # Ego motion diagnostics.
        # This is INFORMATION ONLY.
        # The tracker does not move existing world tracks.
        # ----------------------------------------------------

        if previous_frame is not None:

            T_current_to_world = (
                ego_motion.transform(
                    actual_frame,
                    WORLD_REFERENCE_FRAME,
                )
            )

            translation = (
                T_current_to_world[:3, 3]
            )

            magnitude = float(
                np.linalg.norm(
                    translation
                )
            )

            relative = (
                ego_motion.transform(
                    actual_frame,
                    previous_frame,
                )
            )

            relative_translation = (
                relative[:3, 3]
            )

            relative_magnitude = float(
                np.linalg.norm(
                    relative_translation
                )
            )

            print(
                "\nEgo motion:"
            )

            print(
                f"  {previous_frame:06d}"
                f" -> "
                f"{actual_frame:06d}"
            )

            print(
                f"  Relative translation: "
                f"{relative_translation}"
            )

            print(
                f"  Relative magnitude:    "
                f"{relative_magnitude:.4f} m"
            )

            print(
                f"  World-pose translation: "
                f"{translation}"
            )

            print(
                f"  World-pose magnitude:   "
                f"{magnitude:.4f} m"
            )

        # ----------------------------------------------------
        # Physical frame interval.
        # ----------------------------------------------------
        if previous_frame is not None:
            dt = tracker.time_delta(
                previous_frame,
                actual_frame,
            )
            print(
                f"  Frame delta time:     {dt:.4f} s"
            )

        # ----------------------------------------------------
        # Tracking.
        # ----------------------------------------------------

        tracker.associate(
            detections,
            actual_frame,
            actual_frame=actual_frame,
        )

        print(
            f"\nMatches:       "
            f"{tracker.total_matches}"
        )

        print(
            f"New tracks:    "
            f"{tracker.total_new_tracks}"
        )

        print(
            f"Active tracks: "
            f"{len(tracker.active_tracks())}"
        )

        print_tracks(tracker)

        # ----------------------------------------------------
        # World model.
        # ----------------------------------------------------

        world_model.update(
            tracker.active_tracks(),
            actual_frame,
        )

        src.world_model.orbit_world_model.print_world_summary(
            world_model
        )

        src.world_model.orbit_world_model.print_nearest(
            world_model,
            count=10,
        )

        previous_frame = actual_frame

    # ========================================================
    # SUMMARY
    # ========================================================

    print(
        "\n" + "=" * 70
    )

    print(
        "TRACKING SUMMARY"
    )

    print("-" * 70)

    print(
        f"Frames processed: "
        f"{len(frames)}"
    )

    print(
        f"Total tracks:     "
        f"{tracker.next_track_id - 1}"
    )

    print(
        f"Total matches:    "
        f"{tracker.total_matches}"
    )

    print(
        f"New tracks:       "
        f"{tracker.total_new_tracks}"
    )

    print(
        f"Lost tracks:      "
        f"{tracker.total_lost_tracks}"
    )

    print(
        f"Currently active: "
        f"{len(tracker.active_tracks())}"
    )

    print(
        "\n" + "=" * 70
    )

    print(
        "ORBIT TRACKING COMPLETE"
    )

    print("=" * 70)

    # ========================================================
    # FINAL WORLD MODEL
    # ========================================================

    print(
        "\n" + "=" * 70
    )

    print(
        "FINAL WORLD MODEL"
    )

    print("-" * 70)

    src.world_model.orbit_world_model.print_world_summary(
        world_model
    )

    src.world_model.orbit_world_model.print_nearest(
        world_model,
        count=15,
    )


if __name__ == "__main__":
    main()