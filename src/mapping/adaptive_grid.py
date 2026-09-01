import numpy as np
import open3d as o3d


# ======================================================================
# ADAPTIVE CELL
# ======================================================================

class AdaptiveCell:

    __slots__ = (
        "level",
        "resolution",
        "ix",
        "iy",
        "point_count",
        "ground_count",
        "ground_z_sum",
        "ground_z_min",
        "ground_z_max",
        "obstacle_count",
        "obstacle_z_sum",
        "obstacle_z_min",
        "obstacle_z_max",
    )

    def __init__(
        self,
        level,
        resolution,
        ix,
        iy,
    ):

        self.level = int(level)
        self.resolution = float(resolution)

        self.ix = int(ix)
        self.iy = int(iy)

        self.point_count = 0

        self.ground_count = 0
        self.ground_z_sum = 0.0
        self.ground_z_min = np.inf
        self.ground_z_max = -np.inf

        self.obstacle_count = 0
        self.obstacle_z_sum = 0.0
        self.obstacle_z_min = np.inf
        self.obstacle_z_max = -np.inf

    # ------------------------------------------------------------------
    # Single point API
    # ------------------------------------------------------------------

    def add_point(
        self,
        z,
        is_ground=True,
    ):

        self.point_count += 1

        z = float(z)

        if is_ground:

            self.ground_count += 1
            self.ground_z_sum += z

            if z < self.ground_z_min:
                self.ground_z_min = z

            if z > self.ground_z_max:
                self.ground_z_max = z

        else:

            self.obstacle_count += 1
            self.obstacle_z_sum += z

            if z < self.obstacle_z_min:
                self.obstacle_z_min = z

            if z > self.obstacle_z_max:
                self.obstacle_z_max = z

    # ------------------------------------------------------------------
    # Bulk statistics
    # ------------------------------------------------------------------

    def add_statistics(
        self,
        point_count,
        ground_count,
        ground_z_sum,
        ground_z_min,
        ground_z_max,
        obstacle_count,
        obstacle_z_sum,
        obstacle_z_min,
        obstacle_z_max,
    ):

        self.point_count = int(point_count)

        self.ground_count = int(
            ground_count
        )

        self.ground_z_sum = float(
            ground_z_sum
        )

        self.ground_z_min = float(
            ground_z_min
        )

        self.ground_z_max = float(
            ground_z_max
        )

        self.obstacle_count = int(
            obstacle_count
        )

        self.obstacle_z_sum = float(
            obstacle_z_sum
        )

        self.obstacle_z_min = float(
            obstacle_z_min
        )

        self.obstacle_z_max = float(
            obstacle_z_max
        )

    # ------------------------------------------------------------------
    # Statistics
    # ------------------------------------------------------------------

    @property
    def ground_elevation(self):

        if self.ground_count == 0:
            return None

        return (
            self.ground_z_sum
            / self.ground_count
        )

    @property
    def obstacle_elevation(self):

        if self.obstacle_count == 0:
            return None

        return (
            self.obstacle_z_sum
            / self.obstacle_count
        )

    @property
    def z_mean(self):

        if self.point_count == 0:
            return 0.0

        return (
            self.ground_z_sum
            + self.obstacle_z_sum
        ) / self.point_count

    @property
    def z_min(self):

        if self.point_count == 0:
            return 0.0

        return min(
            self.ground_z_min,
            self.obstacle_z_min,
        )

    @property
    def z_max(self):

        if self.point_count == 0:
            return 0.0

        return max(
            self.ground_z_max,
            self.obstacle_z_max,
        )

    @property
    def semantic_class(self):

        if self.ground_count == 0:
            return "OBSTACLE"

        if self.obstacle_count == 0:
            return "GROUND"

        return "MIXED"

    @property
    def occupancy(self):

        return self.point_count > 0

    @property
    def center(self):

        return (
            self.ix * self.resolution
            + self.resolution / 2.0,

            self.iy * self.resolution
            + self.resolution / 2.0,
        )


# ======================================================================
# ADAPTIVE GRID
# ======================================================================

class AdaptiveGrid:

    BASE_RESOLUTION = 0.05

    LEVELS = {
        0: 0.05,
        1: 0.10,
        2: 0.25,
        3: 0.50,
    }

    RINGS = (
        (0.0, 10.0, 0),
        (10.0, 25.0, 1),
        (25.0, 50.0, 2),
        (50.0, 100.0, 3),
    )

    MAX_RANGE = 100.0

    def __init__(self):

        # Compatibility dictionary.
        #
        # The final cells are generated from compact NumPy
        # aggregates rather than point-by-point insertion.
        self.cells = {}

        self.mapped_points = 0

        # Compact arrays.
        self._levels = np.empty(
            0,
            dtype=np.int8,
        )

        self._ix = np.empty(
            0,
            dtype=np.int64,
        )

        self._iy = np.empty(
            0,
            dtype=np.int64,
        )

        self._point_count = np.empty(
            0,
            dtype=np.int32,
        )

        self._ground_count = np.empty(
            0,
            dtype=np.int32,
        )

        self._ground_sum = np.empty(
            0,
            dtype=np.float64,
        )

        self._ground_min = np.empty(
            0,
            dtype=np.float64,
        )

        self._ground_max = np.empty(
            0,
            dtype=np.float64,
        )

        self._obstacle_count = np.empty(
            0,
            dtype=np.int32,
        )

        self._obstacle_sum = np.empty(
            0,
            dtype=np.float64,
        )

        self._obstacle_min = np.empty(
            0,
            dtype=np.float64,
        )

        self._obstacle_max = np.empty(
            0,
            dtype=np.float64,
        )

    # ==================================================================
    # LEVEL
    # ==================================================================

    def get_level(
        self,
        distance,
    ):

        if distance < 10.0:
            return 0

        if distance < 25.0:
            return 1

        if distance < 50.0:
            return 2

        if distance < 100.0:
            return 3

        return None

    # ==================================================================
    # WORLD → CELL
    # ==================================================================

    def world_to_cell(
        self,
        x,
        y,
        level,
    ):

        resolution = self.LEVELS[
            level
        ]

        return (
            int(
                np.floor(
                    x / resolution
                )
            ),
            int(
                np.floor(
                    y / resolution
                )
            ),
        )

    # ==================================================================
    # SINGLE POINT INSERTION
    # ==================================================================

    def insert_point(
        self,
        x,
        y,
        z,
        is_ground=True,
    ):

        distance_sq = (
            x * x
            + y * y
        )

        if distance_sq >= (
            self.MAX_RANGE
            * self.MAX_RANGE
        ):
            return False

        if distance_sq < 100.0:
            level = 0

        elif distance_sq < 625.0:
            level = 1

        elif distance_sq < 2500.0:
            level = 2

        else:
            level = 3

        resolution = self.LEVELS[
            level
        ]

        ix, iy = self.world_to_cell(
            x,
            y,
            level,
        )

        key = (
            level,
            ix,
            iy,
        )

        cell = self.cells.get(
            key
        )

        if cell is None:

            cell = AdaptiveCell(
                level,
                resolution,
                ix,
                iy,
            )

            self.cells[key] = cell

        cell.add_point(
            z,
            is_ground,
        )

        self.mapped_points += 1

        return True

    # ==================================================================
    # VECTOR AGGREGATION
    # ==================================================================

    @staticmethod
    def _aggregate_level(
        x,
        y,
        z,
        ground,
        level,
        resolution,
    ):

        if len(x) == 0:
            return None

        # --------------------------------------------------------------
        # Cell coordinates
        # --------------------------------------------------------------

        ix = np.floor(
            x / resolution
        ).astype(
            np.int64
        )

        iy = np.floor(
            y / resolution
        ).astype(
            np.int64
        )

        # --------------------------------------------------------------
        # Sort by cell
        # --------------------------------------------------------------

        order = np.lexsort(
            (iy, ix)
        )

        ix = ix[order]
        iy = iy[order]
        z = z[order]
        ground = ground[order]

        n = len(ix)

        # --------------------------------------------------------------
        # Cell boundaries
        # --------------------------------------------------------------

        boundary = np.empty(
            n,
            dtype=bool,
        )

        boundary[0] = True

        if n > 1:

            boundary[1:] = (
                (ix[1:] != ix[:-1])
                |
                (iy[1:] != iy[:-1])
            )

        starts = np.flatnonzero(
            boundary
        )

        cell_count = len(
            starts
        )

        ends = np.empty(
            cell_count,
            dtype=np.int64,
        )

        if cell_count > 1:

            ends[:-1] = starts[1:]

        ends[-1] = n

        counts = (
            ends - starts
        )

        # --------------------------------------------------------------
        # Semantic counts
        # --------------------------------------------------------------

        ground_int = (
            ground.astype(
                np.int32
            )
        )

        ground_counts = (
            np.add.reduceat(
                ground_int,
                starts,
            )
        )

        obstacle_counts = (
            counts
            - ground_counts
        )

        # --------------------------------------------------------------
        # Sums
        # --------------------------------------------------------------

        ground_z = np.where(
            ground,
            z,
            0.0,
        )

        obstacle_z = np.where(
            ~ground,
            z,
            0.0,
        )

        ground_sums = (
            np.add.reduceat(
                ground_z,
                starts,
            )
        )

        obstacle_sums = (
            np.add.reduceat(
                obstacle_z,
                starts,
            )
        )

        # --------------------------------------------------------------
        # Min / max
        # --------------------------------------------------------------

        ground_min_values = np.where(
            ground,
            z,
            np.inf,
        )

        ground_max_values = np.where(
            ground,
            z,
            -np.inf,
        )

        obstacle_min_values = np.where(
            ~ground,
            z,
            np.inf,
        )

        obstacle_max_values = np.where(
            ~ground,
            z,
            -np.inf,
        )

        ground_mins = (
            np.minimum.reduceat(
                ground_min_values,
                starts,
            )
        )

        ground_maxs = (
            np.maximum.reduceat(
                ground_max_values,
                starts,
            )
        )

        obstacle_mins = (
            np.minimum.reduceat(
                obstacle_min_values,
                starts,
            )
        )

        obstacle_maxs = (
            np.maximum.reduceat(
                obstacle_max_values,
                starts,
            )
        )

        return (
            ix[starts],
            iy[starts],
            counts,
            ground_counts,
            ground_sums,
            ground_mins,
            ground_maxs,
            obstacle_counts,
            obstacle_sums,
            obstacle_mins,
            obstacle_maxs,
        )

    # ==================================================================
    # BUILD
    # ==================================================================

    def build(
        self,
        points,
        ground_mask=None,
    ):

        points = np.asarray(
            points,
            dtype=np.float64,
        )

        if points.ndim != 2:

            raise ValueError(
                "points must have shape (N, 3)"
            )

        if points.shape[1] != 3:

            raise ValueError(
                "points must have shape (N, 3)"
            )

        n = len(points)

        if ground_mask is None:

            ground_mask = np.ones(
                n,
                dtype=bool,
            )

        else:

            ground_mask = np.asarray(
                ground_mask,
                dtype=bool,
            )

            if len(ground_mask) != n:

                raise ValueError(
                    "ground_mask length must "
                    "match points"
                )

        # --------------------------------------------------------------
        # Reset
        # --------------------------------------------------------------

        self.cells.clear()

        self.mapped_points = 0

        # Empty cloud
        if n == 0:
            return points

        x = points[:, 0]
        y = points[:, 1]
        z = points[:, 2]

        # --------------------------------------------------------------
        # Range filtering
        # --------------------------------------------------------------

        distance_sq = (
            x * x
            + y * y
        )

        valid = (
            distance_sq
            < self.MAX_RANGE
            * self.MAX_RANGE
        )

        if not np.any(valid):

            return points[
                valid
            ]

        x = x[valid]
        y = y[valid]
        z = z[valid]

        ground = ground_mask[
            valid
        ]

        distance_sq = distance_sq[
            valid
        ]

        self.mapped_points = len(x)

        # --------------------------------------------------------------
        # Resolution levels
        # --------------------------------------------------------------

        levels = np.empty(
            len(x),
            dtype=np.int8,
        )

        levels[
            distance_sq < 100.0
        ] = 0

        levels[
            (distance_sq >= 100.0)
            & (distance_sq < 625.0)
        ] = 1

        levels[
            (distance_sq >= 625.0)
            & (distance_sq < 2500.0)
        ] = 2

        levels[
            distance_sq >= 2500.0
        ] = 3

        # --------------------------------------------------------------
        # Compact result containers
        # --------------------------------------------------------------

        compact = []

        # --------------------------------------------------------------
        # Aggregate every level
        # --------------------------------------------------------------

        for level in range(4):

            mask = (
                levels == level
            )

            if not np.any(mask):
                continue

            result = (
                self._aggregate_level(
                    x[mask],
                    y[mask],
                    z[mask],
                    ground[mask],
                    level,
                    self.LEVELS[level],
                )
            )

            if result is None:
                continue

            compact.append(
                (
                    level,
                    result,
                )
            )

        # --------------------------------------------------------------
        # Build compact arrays
        # --------------------------------------------------------------

        total_cells = sum(
            len(result[0])
            for _, result in compact
        )

        if total_cells == 0:
            return points[valid]

        self._levels = np.empty(
            total_cells,
            dtype=np.int8,
        )

        self._ix = np.empty(
            total_cells,
            dtype=np.int64,
        )

        self._iy = np.empty(
            total_cells,
            dtype=np.int64,
        )

        self._point_count = np.empty(
            total_cells,
            dtype=np.int32,
        )

        self._ground_count = np.empty(
            total_cells,
            dtype=np.int32,
        )

        self._ground_sum = np.empty(
            total_cells,
            dtype=np.float64,
        )

        self._ground_min = np.empty(
            total_cells,
            dtype=np.float64,
        )

        self._ground_max = np.empty(
            total_cells,
            dtype=np.float64,
        )

        self._obstacle_count = np.empty(
            total_cells,
            dtype=np.int32,
        )

        self._obstacle_sum = np.empty(
            total_cells,
            dtype=np.float64,
        )

        self._obstacle_min = np.empty(
            total_cells,
            dtype=np.float64,
        )

        self._obstacle_max = np.empty(
            total_cells,
            dtype=np.float64,
        )

        # --------------------------------------------------------------
        # Fill arrays
        # --------------------------------------------------------------

        offset = 0

        for level, result in compact:

            (
                cell_ix,
                cell_iy,
                counts,
                ground_counts,
                ground_sums,
                ground_mins,
                ground_maxs,
                obstacle_counts,
                obstacle_sums,
                obstacle_mins,
                obstacle_maxs,
            ) = result

            count = len(
                cell_ix
            )

            end = (
                offset + count
            )

            self._levels[
                offset:end
            ] = level

            self._ix[
                offset:end
            ] = cell_ix

            self._iy[
                offset:end
            ] = cell_iy

            self._point_count[
                offset:end
            ] = counts

            self._ground_count[
                offset:end
            ] = ground_counts

            self._ground_sum[
                offset:end
            ] = ground_sums

            self._ground_min[
                offset:end
            ] = ground_mins

            self._ground_max[
                offset:end
            ] = ground_maxs

            self._obstacle_count[
                offset:end
            ] = obstacle_counts

            self._obstacle_sum[
                offset:end
            ] = obstacle_sums

            self._obstacle_min[
                offset:end
            ] = obstacle_mins

            self._obstacle_max[
                offset:end
            ] = obstacle_maxs

            offset = end

        # --------------------------------------------------------------
        # Compatibility dictionary
        #
        # Construct once after vectorized aggregation.
        # --------------------------------------------------------------

        self._build_cell_view()

        return points[valid]

    # ==================================================================
    # COMPATIBILITY CELL VIEW
    # ==================================================================

    def _build_cell_view(self):

        levels = self._levels
        ix_values = self._ix
        iy_values = self._iy

        point_counts = self._point_count
        ground_counts = self._ground_count
        ground_sums = self._ground_sum
        ground_mins = self._ground_min
        ground_maxs = self._ground_max

        obstacle_counts = self._obstacle_count
        obstacle_sums = self._obstacle_sum
        obstacle_mins = self._obstacle_min
        obstacle_maxs = self._obstacle_max

        count = len(levels)

        cells = {}

        # Local references reduce attribute/dictionary lookup overhead
        levels_resolution = self.LEVELS

        for i in range(count):

            level = int(levels[i])
            ix = int(ix_values[i])
            iy = int(iy_values[i])

            # --------------------------------------------------------------
            # Fast construction
            #
            # Avoid:
            #   AdaptiveCell(...)
            #   cell.add_statistics(...)
            #
            # because this happens ~152k times.
            # --------------------------------------------------------------

            cell = AdaptiveCell.__new__(AdaptiveCell)

            cell.level = level
            cell.resolution = levels_resolution[level]

            cell.ix = ix
            cell.iy = iy

            cell.point_count = int(point_counts[i])

            cell.ground_count = int(ground_counts[i])
            cell.ground_z_sum = float(ground_sums[i])
            cell.ground_z_min = float(ground_mins[i])
            cell.ground_z_max = float(ground_maxs[i])

            cell.obstacle_count = int(obstacle_counts[i])
            cell.obstacle_z_sum = float(obstacle_sums[i])
            cell.obstacle_z_min = float(obstacle_mins[i])
            cell.obstacle_z_max = float(obstacle_maxs[i])

            cells[(level, ix, iy)] = cell

        self.cells = cells


    # ==================================================================
    # PARENT
    # ==================================================================

    def get_parent(
        self,
        cell,
    ):

        if cell.level >= 3:
            return None

        parent_level = (
            cell.level + 1
        )

        x, y = (
            cell.center
        )

        parent_ix, parent_iy = (
            self.world_to_cell(
                x,
                y,
                parent_level,
            )
        )

        return (
            parent_level,
            parent_ix,
            parent_iy,
        )

    # ==================================================================
    # STATISTICS
    # ==================================================================

    def statistics(self):

        level_counts = {
            level: 0
            for level in self.LEVELS
        }

        point_counts = {
            level: 0
            for level in self.LEVELS
        }

        # Use NumPy arrays instead of iterating over cells.
        for level in range(4):

            mask = (
                self._levels == level
            )

            if np.any(mask):

                level_counts[level] = int(
                    np.count_nonzero(mask)
                )

                point_counts[level] = int(
                    self._point_count[
                        mask
                    ].sum()
                )

        return {
            "total_cells":
                len(self._levels),

            "level_counts":
                level_counts,

            "point_counts":
                point_counts,

            "mapped_points":
                self.mapped_points,
        }


# ======================================================================
# STANDALONE EXECUTION
# ======================================================================

def main():

    print("=" * 70)
    print(
        "ORBIT - VECTORIZED ADAPTIVE 2.5D GRID"
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

    print(
        "\nDetecting ground..."
    )

    _, inliers = (
        cloud.segment_plane(
            distance_threshold=0.08,
            ransac_n=3,
            num_iterations=1000,
        )
    )

    ground_mask = np.zeros(
        len(points),
        dtype=bool,
    )

    ground_mask[
        inliers
    ] = True

    print(
        f"Ground points: "
        f"{ground_mask.sum():,}"
    )

    print(
        f"Non-ground points: "
        f"{(~ground_mask).sum():,}"
    )

    grid = AdaptiveGrid()

    mapped = grid.build(
        points,
        ground_mask,
    )

    stats = (
        grid.statistics()
    )

    print(
        f"\nMapped points: "
        f"{len(mapped):,}"
    )

    print(
        f"Occupied cells: "
        f"{stats['total_cells']:,}"
    )

    print(
        "\nResolution hierarchy:"
    )

    for level, resolution in (
        grid.LEVELS.items()
    ):

        print(
            f"  Level {level}: "
            f"{resolution * 100:.0f} cm"
            f" | cells="
            f"{stats['level_counts'][level]:,}"
            f" | points="
            f"{stats['point_counts'][level]:,}"
        )


if __name__ == "__main__":
    main()