from aeromind_interfaces.msg import SemanticObject
from aeromind_perception.object_tracker_node import (
    _filter_objects,
    _normalize_class_name,
    _object_distance,
)


def _object(identifier, class_name, x, state="confirmed", dynamic=False, age=0.1):
    item = SemanticObject()
    item.id = identifier
    item.class_name = class_name
    item.position_valid = True
    item.position.x = x
    item.state = state
    item.dynamic = dynamic
    item.age_sec = age
    return item


def test_chinese_query_class_names_map_to_detector_classes():
    assert _normalize_class_name("人") == "person"
    assert _normalize_class_name("车辆") == "car"
    assert _normalize_class_name("bus") == "bus"


def test_current_query_filters_lifecycle_dynamic_and_freshness():
    items = [
        _object("person_1", "person", 1.0, dynamic=True),
        _object("person_2", "person", 2.0, state="tentative", dynamic=True),
        _object("car_1", "car", 3.0, dynamic=True),
        _object("person_3", "person", 4.0, dynamic=True, age=5.0),
    ]
    result = _filter_objects(
        items,
        class_name="person",
        dynamic_only=True,
        confirmed_only=True,
        fresh_within_sec=1.0,
    )
    assert [item.id for item in result] == ["person_1"]


def test_nearest_query_distance_is_three_dimensional():
    item = _object("person_1", "person", 3.0)
    item.position.y = 4.0
    assert _object_distance(item, (0.0, 0.0, 0.0)) == 5.0
