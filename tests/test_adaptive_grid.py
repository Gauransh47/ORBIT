import sys
from pathlib import Path

import numpy as np

# Allow importing from src
sys.path.append(
    str(Path(__file__).resolve().parents[1] / "src")
)

from mapping.adaptive_grid import AdaptiveGrid


def test_resolution_zones():

    grid = AdaptiveGrid()

    test_cases = [
        (2.0, 0),
        (9.9, 0),
        (10.0, 1),
        (24.9, 1),
        (25.0, 2),
        (49.9, 2),
        (50.0, 3),
        (99.9, 3),
        (100.0, None)
    ]

    for distance, expected_level in test_cases:

        level = grid.get_level(distance)

        assert level == expected_level


def test_world_to_cell():

    grid = AdaptiveGrid()

    # 5 cm grid
    ix, iy = grid.world_to_cell(
        1.00,
        2.00,
        0
    )

    assert ix == 20
    assert iy == 40

    # 10 cm grid
    ix, iy = grid.world_to_cell(
        1.00,
        2.00,
        1
    )

    assert ix == 10
    assert iy == 20


def test_point_assignment():

    grid = AdaptiveGrid()

    points = np.array([
        [2.0, 0.0, 0.0],
        [12.0, 0.0, 0.0],
        [35.0, 0.0, 0.0],
        [60.0, 0.0, 0.0]
    ])

    grid.build(points)

    assert len(grid.cells) == 4


def test_outside_range():

    grid = AdaptiveGrid()

    # 101m away — outside ORBIT range
    grid.insert_point(
        101.0,
        0.0,
        0.0
    )

    assert len(grid.cells) == 0


def test_elevation_statistics():

    grid = AdaptiveGrid()

    grid.insert_point(
        2.0,
        0.0,
        0.0
    )

    grid.insert_point(
        2.01,
        0.01,
        1.0
    )

    # There should be one occupied cell
    assert len(grid.cells) == 1

    cell = next(
        iter(grid.cells.values())
    )

    assert cell.point_count == 2

    assert cell.z_min == 0.0
    assert cell.z_max == 1.0

    assert abs(
        cell.z_mean - 0.5
    ) < 1e-6


def test_parent_relationship():

    grid = AdaptiveGrid()

    grid.insert_point(
        2.0,
        2.0,
        0.0
    )

    cell = next(
        iter(grid.cells.values())
    )

    assert cell.level == 0

    parent = grid.get_parent(
        cell
    )

    assert parent is not None

    assert parent[0] == 1