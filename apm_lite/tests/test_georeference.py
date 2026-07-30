from pathlib import Path

import pytest

from aeromind_apm_lite.common.coordinates import (
    CalibrationStatus,
    GeoReference,
    GeoReferenceConflict,
    GeoReferenceError,
    GeoReferenceStore,
    VehicleHome,
    Wgs84Position,
    default_georeference,
    load_georeference,
)


ROOT = Path(__file__).resolve().parents[1]


def surveyed_reference(calibration_id="zhengzhou-field-20260730-v1"):
    return GeoReference(
        calibration_id=calibration_id,
        status=CalibrationStatus.SURVEYED,
        map_origin_wgs84=Wgs84Position(
            latitude_deg=34.7472,
            longitude_deg=113.6253,
            altitude_m=112.4,
        ),
        map_x_heading_from_true_north_deg=28.0,
        airsim_origin_wgs84=Wgs84Position(
            latitude_deg=34.74718,
            longitude_deg=113.62525,
            altitude_m=111.9,
        ),
        vehicle_homes=tuple(
            VehicleHome(
                vehicle_id=index,
                map_position_m=(0.0, 3.0 * (index - 1), 0.0),
            )
            for index in range(1, 5)
        ),
    )


def test_example_georeference_loads_as_an_incomplete_draft():
    reference = load_georeference(
        ROOT / "configs/calibration/venue.example.yaml"
    )

    assert reference.status == CalibrationStatus.DRAFT
    assert not reference.is_complete
    assert len(reference.config_hash) == 64
    assert not reference.coordinate_round_trip_report()["ready"]


def test_surveyed_reference_has_stable_hash_and_round_trips_coordinates():
    reference = surveyed_reference()
    equivalent = GeoReference.model_validate(
        reference.model_dump(mode="json")
    )

    assert reference.config_hash == equivalent.config_hash
    report = reference.coordinate_round_trip_report()
    assert report["ready"]
    assert report["passed"]
    assert report["maximum_error_m"] < 0.01
    assert len(report["points"]) == 5
    assert report["points"][1]["airsim_ned_m"] != [0.0, 0.0, 0.0]


def test_surveyed_reference_requires_all_boundary_coordinates():
    draft = default_georeference().model_dump(mode="json")
    draft["status"] = "surveyed"

    with pytest.raises(ValueError, match="map_origin_wgs84"):
        GeoReference.model_validate(draft)


def test_duplicate_vehicle_homes_and_non_finite_values_are_rejected():
    payload = default_georeference().model_dump(mode="json")
    payload["vehicle_homes"][1]["vehicle_id"] = 1
    with pytest.raises(ValueError, match="duplicate vehicle_id"):
        GeoReference.model_validate(payload)

    payload = default_georeference().model_dump(mode="json")
    payload["vehicle_homes"][0]["map_position_m"][0] = float("nan")
    with pytest.raises(ValueError):
        GeoReference.model_validate(payload)


def test_store_persists_yaml_atomically_and_reloads_it(tmp_path):
    path = tmp_path / "venue.yaml"
    store = GeoReferenceStore(path)
    reference = surveyed_reference()

    saved = store.save(reference)
    reloaded = GeoReferenceStore(path)

    assert saved.config_hash == reference.config_hash
    assert reloaded.value == reference
    assert reloaded.public_payload()["immutable"]
    assert not list(tmp_path.glob("*.tmp"))


def test_surveyed_id_cannot_be_mutated_but_a_new_version_can_be_saved(tmp_path):
    store = GeoReferenceStore(tmp_path / "venue.yaml")
    original = surveyed_reference()
    store.save(original)
    changed = original.model_copy(update={"notes": "changed survey"})

    with pytest.raises(GeoReferenceConflict, match="new calibration_id"):
        store.save(changed)

    next_version = changed.model_copy(
        update={"calibration_id": "zhengzhou-field-20260730-v2"}
    )
    store.save(next_version)
    assert store.value.calibration_id.endswith("-v2")


def test_invalid_yaml_is_reported_as_a_georeference_error(tmp_path):
    path = tmp_path / "venue.yaml"
    path.write_text("vehicle_homes: [", encoding="utf-8")

    with pytest.raises(GeoReferenceError, match="failed to read"):
        load_georeference(path)
