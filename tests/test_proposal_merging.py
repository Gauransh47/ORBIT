import sys
from pathlib import Path

import numpy as np

sys.path.append(
    str(Path(__file__).resolve().parents[1] / "src")
)

from evaluation.analyze_proposal_merging import (
    box_instance_ids,
    cell_key_for_xy,
    geometric_merge_events,
    semantic_merge_events,
)
from perception.connected_components import ObjectComponent
from perception.object_association import associate_components
from perception.semantic_object_association import ObjectHypothesis
from perception.terrain_obstacle_detection import ObstacleCell
from common.types import MergeEvent


def _cell(level, ix, iy, resolution, height=1.0, points=10):
    return ObstacleCell(
        level=level,
        ix=ix,
        iy=iy,
        resolution=resolution,
        ground_elevation=0.0,
        obstacle_elevation=height,
        obstacle_height=height,
        obstacle_count=points,
    )


def test_cell_key_matches_range_rings():
    assert cell_key_for_xy(1.0, 0.0)[0] == 0
    assert cell_key_for_xy(12.0, 0.0)[0] == 1
    assert cell_key_for_xy(30.0, 0.0)[0] == 2
    assert cell_key_for_xy(60.0, 0.0)[0] == 3
    assert cell_key_for_xy(101.0, 0.0) is None


def test_box_instance_ids_ignores_unlabelled():
    points = np.array(
        [
            [0.0, 0.0, 0.0],
            [0.2, 0.1, 0.0],
            [5.0, 5.0, 0.0],
        ]
    )
    instances = np.array([0, 7, 7])

    found = box_instance_ids(-1.0, 1.0, -1.0, 1.0, points, instances)
    assert found == [7]


def test_associate_components_merges_adjacent_cells():
    left = ObjectComponent(0)
    left.add(_cell(0, 0, 0, 0.05, height=2.0, points=20))
    left.add(_cell(0, 1, 0, 0.05, height=2.0, points=20))
    left.add(_cell(0, 2, 0, 0.05, height=2.0, points=20))

    right = ObjectComponent(1)
    right.add(_cell(0, 4, 0, 0.05, height=2.0, points=20))
    right.add(_cell(0, 5, 0, 0.05, height=2.0, points=20))
    right.add(_cell(0, 6, 0, 0.05, height=2.0, points=20))

    objects = associate_components([left, right])

    assert len(objects) == 1
    assert len(objects[0].components) == 2

    events = geometric_merge_events(objects)
    assert len(events) == 1
    assert events[0].component_ids == [0, 1]
    assert events[0].overmerged is None


def test_geometric_overmerge_flag_with_instance_map():
    component_a = ObjectComponent(0)
    cell_a = _cell(0, 20, 0, 0.05)
    component_a.add(cell_a)

    component_b = ObjectComponent(1)
    cell_b = _cell(0, 21, 0, 0.05)
    component_b.add(cell_b)

    group = associate_components([component_a, component_b])[0]

    instance_map = {
        (0, 20, 0): {11},
        (0, 21, 0): {22},
    }

    events = geometric_merge_events(
        [group],
        instance_map=instance_map,
    )

    assert len(events) == 1
    assert events[0].overmerged is True
    assert events[0].instance_ids == [11, 22]


def _hypothesis(object_id, semantic_id, min_x, max_x, min_y, max_y, points=20):
    return ObjectHypothesis(
        object_id=object_id,
        semantic_id=semantic_id,
        semantic_class="car",
        confidence=0.9,
        entropy=0.1,
        cells=4,
        points=points,
        center_x=(min_x + max_x) / 2.0,
        center_y=(min_y + max_y) / 2.0,
        width=max_x - min_x,
        length=max_y - min_y,
        min_x=min_x,
        max_x=max_x,
        min_y=min_y,
        max_y=max_y,
    )


def test_semantic_merge_events_nested_boxes():
    pre = [
        _hypothesis(0, 10, 0.0, 1.0, 0.0, 1.0),
        _hypothesis(1, 10, 1.2, 2.2, 0.0, 1.0),
    ]
    post = [
        _hypothesis(0, 10, 0.0, 2.2, 0.0, 1.0, points=40),
    ]

    events = semantic_merge_events(pre, post)
    assert len(events) == 1
    assert events[0].component_count == 2
    assert events[0].component_ids == [0, 1]


def test_merge_event_to_dict():
    event = MergeEvent(
        object_id=1,
        classification="OBSTACLE",
        component_ids=[0, 1],
        component_count=2,
        point_count=10,
        cell_count=4,
        center=(1.0, 2.0),
        dimensions=(1.0, 1.0, 1.0),
        distance=2.236,
        instance_ids=[3, 4],
        semantic_ids=[10],
        overmerged=True,
        notes=["test"],
    )

    payload = event.to_dict()
    assert payload["overmerged"] is True
    assert payload["instance_ids"] == [3, 4]
