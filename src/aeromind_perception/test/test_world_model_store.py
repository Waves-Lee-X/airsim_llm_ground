import numpy as np
import pytest

from aeromind_perception.world_model_store import WorldModelStore


def _record(identifier, class_name, x, state="confirmed"):
    return {
        "id": identifier,
        "class_name": class_name,
        "confidence": 0.8,
        "position_valid": True,
        "position": (x, 2.0, 3.0),
        "velocity": (0.1, 0.0, 0.0),
        "dynamic": True,
        "state": state,
        "hit_count": 3,
        "miss_count": 0,
        "position_covariance": [0.1, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.1],
        "sources": ["/perception/detections", "/tf"],
    }


def test_store_records_and_filters_history(tmp_path):
    store = WorldModelStore(str(tmp_path / "world.db"), retention_hours=1.0)
    assert store.record("map", 100.0, [
        _record("person_0001", "person", 1.0),
        _record("car_0002", "car", 4.0),
    ]) == 2
    store.record("map", 102.0, [_record("person_0001", "person", 1.5)])

    person = store.query_history(class_name="person", limit=10)
    assert [item["observed_at"] for item in person] == [102.0, 100.0]
    assert person[0]["position_m"]["x"] == 1.5
    assert person[0]["frame_id"] == "map"
    assert store.count() == 3
    store.close()


def test_store_retention_removes_old_records(tmp_path):
    store = WorldModelStore(str(tmp_path / "world.db"), retention_hours=1.0)
    store.record("odom", 100.0, [_record("person_0001", "person", 1.0)])
    store.record("odom", 4000.0, [_record("person_0001", "person", 2.0)])
    assert store.cleanup(now=4000.0) == 1
    assert store.count() == 1
    store.close()


def test_history_freshness_uses_caller_timeline_for_bag_replay(tmp_path):
    store = WorldModelStore(str(tmp_path / "world.db"), retention_hours=1.0)
    store.record("map", 100.0, [_record("person_0001", "person", 1.0)])
    store.record("map", 105.0, [_record("person_0001", "person", 2.0)])
    rows = store.query_history(
        class_name="person", fresh_within_sec=2.0, now=106.0, limit=10
    )
    assert [item["observed_at"] for item in rows] == [105.0]
    store.close()


def test_track_ids_remain_unique_across_store_restarts(tmp_path):
    path = str(tmp_path / "world.db")
    first = WorldModelStore(path)
    assert first.allocate_track_id("person") == "person_000001"
    first.close()

    second = WorldModelStore(path)
    assert second.allocate_track_id("person") == "person_000002"
    assert second.allocate_track_id("car") == "car_000003"
    second.close()


def test_store_accepts_numpy_float32_covariance(tmp_path):
    store = WorldModelStore(str(tmp_path / "world.db"))
    record = _record("person_0001", "person", 1.0)
    record["position_covariance"] = np.asarray(
        [0.1, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.1],
        dtype=np.float32,
    )

    assert store.record("odom", 100.0, [record]) == 1
    history = store.query_history(object_id="person_0001")
    assert len(history[0]["position_covariance"]) == 9
    assert history[0]["position_covariance"][0] == pytest.approx(0.1)
    store.close()
