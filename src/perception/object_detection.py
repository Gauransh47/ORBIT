import numpy as np
import open3d as o3d

from dataclasses import dataclass
from sklearn.cluster import DBSCAN


# ============================================================
# ORBIT - DISTANCE AWARE OBJECT DETECTION
# ============================================================


# ------------------------------------------------------------
# Object proposal
# ------------------------------------------------------------

@dataclass
class ObjectProposal:

    cluster_id: int
    points: np.ndarray

    classification: str

    center: tuple

    dimensions: tuple

    distance: float

    verticality: float

    density: float


# ============================================================
# Ground detection
# ============================================================

def detect_ground(points, distance_threshold=0.08):

    """
    Detect the dominant ground plane.

    Returns
    -------
    ground_mask : np.ndarray
        Boolean mask where True = ground.
    """

    points = np.asarray(points)

    if len(points) < 3:
        return np.zeros(
            len(points),
            dtype=bool
        )

    cloud = o3d.geometry.PointCloud()

    cloud.points = (
        o3d.utility.Vector3dVector(points)
    )

    plane_model, inliers = (
        cloud.segment_plane(
            distance_threshold=distance_threshold,
            ransac_n=3,
            num_iterations=1000
        )
    )

    ground_mask = np.zeros(
        len(points),
        dtype=bool
    )

    ground_mask[inliers] = True

    return ground_mask


# ============================================================
# Distance-aware detector
# ============================================================

class ObjectDetector:

    """
    ORBIT distance-aware geometric object detector.

    Clustering resolution follows ORBIT's adaptive
    spatial hierarchy:

        0-10m     -> 5cm
        10-25m    -> 10cm
        25-50m    -> 25cm
        50-100m   -> 50cm
    """

    RESOLUTION_ZONES = [
        (0.0, 10.0, 0.05),
        (10.0, 25.0, 0.10),
        (25.0, 50.0, 0.25),
        (50.0, 100.0, 0.50),
    ]

    def __init__(
        self,
        cluster_tolerance=0.35,
        min_points=30,
        max_range=100.0
    ):

        # Kept for compatibility with previous code.
        self.cluster_tolerance = (
            cluster_tolerance
        )

        self.min_points = min_points
        self.max_range = max_range

    # --------------------------------------------------------
    # Determine adaptive resolution
    # --------------------------------------------------------

    def get_resolution(self, distance):

        for r_min, r_max, resolution in (
            self.RESOLUTION_ZONES
        ):

            if (
                r_min
                <= distance
                < r_max
            ):
                return resolution

        return None

    # --------------------------------------------------------
    # Distance-aware clustering
    # --------------------------------------------------------

    def _cluster_zone(
        self,
        points,
        resolution,
        cluster_offset
    ):

        if len(points) == 0:
            return []

        # ----------------------------------------------------
        # Adaptive voxelization
        #
        # This is critical for performance.
        #
        # Instead of running DBSCAN on hundreds of thousands
        # of raw points, we first compress the cloud according
        # to ORBIT's current spatial resolution.
        # ----------------------------------------------------

        voxel_indices = np.floor(
            points / resolution
        ).astype(np.int64)

        unique_voxels, inverse = np.unique(
            voxel_indices,
            axis=0,
            return_inverse=True
        )

        # Calculate centroid of every voxel.
        voxel_points = np.zeros(
            (
                len(unique_voxels),
                3
            ),
            dtype=np.float64
        )

        np.add.at(
            voxel_points,
            inverse,
            points
        )

        voxel_counts = np.bincount(
            inverse
        )

        voxel_points /= (
            voxel_counts[:, None]
        )

        # ----------------------------------------------------
        # Adaptive DBSCAN tolerance
        # ----------------------------------------------------

        # A few resolution cells provide enough tolerance
        # to connect points belonging to the same object.
        eps = max(
            resolution * 2.5,
            0.10
        )

        labels = DBSCAN(
            eps=eps,
            min_samples=max(
                3,
                min(
                    8,
                    self.min_points // 10
                )
            ),
            algorithm="ball_tree",
            n_jobs=-1
        ).fit_predict(
            voxel_points
        )

        clusters = []

        for label in np.unique(labels):

            if label == -1:
                continue

            voxel_mask = (
                labels == label
            )

            cluster_voxels = (
                voxel_points[
                    voxel_mask
                ]
            )

            # Recover original points belonging
            # to this cluster.
            original_mask = np.isin(
                inverse,
                np.where(
                    voxel_mask
                )[0]
            )

            cluster_points = (
                points[
                    original_mask
                ]
            )

            if (
                len(cluster_points)
                < self.min_points
            ):
                continue

            clusters.append(
                (
                    cluster_offset + label,
                    cluster_points
                )
            )

        return clusters

    # --------------------------------------------------------
    # Detect
    # --------------------------------------------------------

    def detect(self, points):

        points = np.asarray(
            points,
            dtype=np.float64
        )

        if len(points) == 0:
            return []

        # ----------------------------------------------------
        # Keep only ORBIT's 100m operating range
        # ----------------------------------------------------

        distance = np.sqrt(
            points[:, 0] ** 2
            + points[:, 1] ** 2
        )

        range_mask = (
            distance < self.max_range
        )

        points = points[
            range_mask
        ]

        distance = distance[
            range_mask
        ]

        if len(points) == 0:
            return []

        # ----------------------------------------------------
        # Divide points into ORBIT resolution zones
        # ----------------------------------------------------

        zone_clusters = []

        cluster_offset = 0

        for r_min, r_max, resolution in (
            self.RESOLUTION_ZONES
        ):

            zone_mask = (
                (distance >= r_min)
                & (distance < r_max)
            )

            zone_points = points[
                zone_mask
            ]

            if len(zone_points) == 0:
                continue

            print(
                f"  Zone "
                f"{r_min:.0f}-{r_max:.0f}m "
                f"| resolution="
                f"{resolution * 100:.0f}cm "
                f"| points="
                f"{len(zone_points):,}"
            )

            clusters = self._cluster_zone(
                zone_points,
                resolution,
                cluster_offset
            )

            zone_clusters.extend(
                clusters
            )

            cluster_offset += (
                len(clusters) + 1000
            )

        # ----------------------------------------------------
        # Convert clusters into object proposals
        # ----------------------------------------------------

        proposals = []

        for cluster_id, cluster_points in (
            zone_clusters
        ):

            proposal = (
                self._create_proposal(
                    cluster_id,
                    cluster_points
                )
            )

            if proposal is not None:
                proposals.append(
                    proposal
                )

        # ----------------------------------------------------
        # Sort by distance
        # ----------------------------------------------------

        proposals.sort(
            key=lambda obj: obj.distance
        )

        # Re-number objects for clean presentation.
        for i, proposal in enumerate(
            proposals,
            start=1
        ):

            proposal.cluster_id = i

        return proposals

    # ========================================================
    # Object feature extraction
    # ========================================================

    def _create_proposal(
        self,
        cluster_id,
        points
    ):

        if len(points) < self.min_points:
            return None

        # ----------------------------------------------------
        # Bounding dimensions
        # ----------------------------------------------------

        minimum = np.min(
            points,
            axis=0
        )

        maximum = np.max(
            points,
            axis=0
        )

        dimensions = (
            maximum - minimum
        )

        dx = dimensions[0]
        dy = dimensions[1]
        dz = dimensions[2]

        # ----------------------------------------------------
        # Center
        # ----------------------------------------------------

        center = (
            np.mean(
                points[:, 0]
            ),
            np.mean(
                points[:, 1]
            ),
            np.mean(
                points[:, 2]
            )
        )

        # ----------------------------------------------------
        # Distance from LiDAR
        # ----------------------------------------------------

        distance = np.sqrt(
            center[0] ** 2
            + center[1] ** 2
        )

        # ----------------------------------------------------
        # Verticality
        #
        # Height / horizontal width.
        #
        # High value = pole-like structure.
        # ----------------------------------------------------

        horizontal_size = max(
            min(dx, dy),
            0.05
        )

        verticality = (
            dz
            / horizontal_size
        )

        # ----------------------------------------------------
        # Density
        # ----------------------------------------------------

        volume = max(
            dx * dy * dz,
            1e-6
        )

        density = (
            len(points)
            / volume
        )

        # ----------------------------------------------------
        # Classification
        # ----------------------------------------------------

        classification = (
            self._classify(
                dx,
                dy,
                dz,
                verticality,
                len(points)
            )
        )

        return ObjectProposal(
            cluster_id=cluster_id,
            points=points,
            classification=classification,
            center=center,
            dimensions=(
                dx,
                dy,
                dz
            ),
            distance=distance,
            verticality=verticality,
            density=density
        )

    # ========================================================
    # Geometric classification
    # ========================================================

    def _classify(
        self,
        dx,
        dy,
        dz,
        verticality,
        point_count
    ):

        horizontal_max = max(
            dx,
            dy
        )

        horizontal_min = min(
            dx,
            dy
        )

        # ----------------------------------------------------
        # POLE
        # ----------------------------------------------------

        if (
            dz >= 2.0
            and horizontal_max <= 0.8
            and verticality >= 3.5
        ):

            return "POLE"

        # ----------------------------------------------------
        # WALL
        # ----------------------------------------------------

        if (
            dz >= 1.0
            and horizontal_max >= 1.5
            and horizontal_min <= 2.5
            and verticality >= 0.35
        ):

            return "WALL"

        # ----------------------------------------------------
        # VEHICLE-LIKE
        # ----------------------------------------------------

        if (
            1.0 <= horizontal_max <= 5.0
            and 1.0 <= horizontal_min <= 3.0
            and 0.5 <= dz <= 2.5
            and point_count >= 100
        ):

            return "VEHICLE_LIKE"

        # ----------------------------------------------------
        # PERSON-LIKE
        # ----------------------------------------------------

        if (
            1.2 <= dz <= 2.2
            and horizontal_max <= 1.0
            and point_count >= 30
        ):

            return "PERSON_LIKE"

        # ----------------------------------------------------
        # Generic obstacle
        # ----------------------------------------------------

        return "OBSTACLE"


# ============================================================
# Standalone test
# ============================================================

def main():

    print("=" * 70)
    print(
        "ORBIT - DISTANCE AWARE OBJECT DETECTION"
    )
    print("=" * 70)

    cloud = o3d.io.read_point_cloud(
        "data/synthetic_scene.ply"
    )

    points = np.asarray(
        cloud.points
    )

    print(
        f"\nInput points: "
        f"{len(points):,}"
    )

    # --------------------------------------------------------
    # Ground
    # --------------------------------------------------------

    print(
        "\nDetecting ground..."
    )

    ground_mask = detect_ground(
        points
    )

    ground_points = points[
        ground_mask
    ]

    non_ground_points = points[
        ~ground_mask
    ]

    print(
        f"Ground points: "
        f"{len(ground_points):,}"
    )

    print(
        f"Non-ground points: "
        f"{len(non_ground_points):,}"
    )

    # --------------------------------------------------------
    # Detector
    # --------------------------------------------------------

    detector = ObjectDetector(
        min_points=30,
        max_range=100.0
    )

    print(
        "\nDistance-aware clustering:"
    )

    proposals = detector.detect(
        non_ground_points
    )

    # --------------------------------------------------------
    # Results
    # --------------------------------------------------------

    print(
        f"\nDetected objects: "
        f"{len(proposals)}"
    )

    print(
        "\nObject proposals:"
    )

    print(
        "-" * 70
    )

    for proposal in proposals:

        dx, dy, dz = (
            proposal.dimensions
        )

        print(
            f"\nObject #{proposal.cluster_id}"
        )

        print(
            f"  Classification: "
            f"{proposal.classification}"
        )

        print(
            f"  Points: "
            f"{len(proposal.points):,}"
        )

        print(
            f"  Dimensions: "
            f"{dx:.2f}m × "
            f"{dy:.2f}m × "
            f"{dz:.2f}m"
        )

        print(
            f"  Distance: "
            f"{proposal.distance:.2f}m"
        )

        print(
            f"  Verticality: "
            f"{proposal.verticality:.2f}"
        )

        print(
            f"  Density: "
            f"{proposal.density:.2f}"
        )

    # --------------------------------------------------------
    # Summary
    # --------------------------------------------------------

    summary = {}

    for proposal in proposals:

        label = proposal.classification

        summary[label] = (
            summary.get(label, 0)
            + 1
        )

    print(
        "\n" + "=" * 70
    )

    print(
        "CLASS SUMMARY"
    )

    print(
        "=" * 70
    )

    for label, count in sorted(
        summary.items()
    ):

        print(
            f"  {label:<20} "
            f"{count}"
        )

    print(
        "\n" + "=" * 70
    )

    print(
        "ORBIT DISTANCE-AWARE "
        "DETECTION COMPLETE"
    )

    print(
        "=" * 70
    )


if __name__ == "__main__":
    main()