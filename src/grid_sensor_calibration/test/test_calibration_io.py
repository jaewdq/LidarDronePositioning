"""Tests for the YAML input/output layer."""

from __future__ import annotations

import math

import numpy as np
import pytest
import yaml

from grid_sensor_calibration.calibration_io import (
    CalibrationIOError,
    NominalPose,
    SensorTransform,
    load_calibration_input,
    load_sensor_transforms,
    save_sensor_transforms,
    transform_from_estimate,
    update_calibration_file,
)
from grid_sensor_calibration.calibration_math import (
    InvalidShapeError,
    estimate_sensor_pose,
    grid_to_sensor,
    installation_error,
)

TARGETS = [[3.0, 3.0], [21.0, 3.0], [21.0, 12.0], [3.0, 12.0]]
SENSOR1_POSE = (12.20, -0.85, math.radians(2.0))


def _write(path, payload) -> str:
    """Dump ``payload`` to ``path`` as YAML and return the path as a string."""
    path.write_text(yaml.safe_dump(payload, sort_keys=False), encoding="utf-8")
    return str(path)


def _dataset(pose=SENSOR1_POSE, name="sensor1", nominal=(12.0, -1.0, 0.0)) -> dict:
    """Build a single-sensor calibration dataset dict."""
    sensor_points = grid_to_sensor(np.asarray(TARGETS), *pose)
    return {
        "sensor_name": name,
        "parent_frame": "grid",
        "child_frame": name,
        "nominal_pose": {"x": nominal[0], "y": nominal[1], "yaw": nominal[2]},
        "grid_points": TARGETS,
        "sensor_points": sensor_points.tolist(),
    }


# --------------------------------------------------------------------------
# Input files
# --------------------------------------------------------------------------


def test_load_single_sensor_dataset(tmp_path):
    path = _write(tmp_path / "sensor1.yaml", _dataset())

    dataset = load_calibration_input(path)

    assert dataset.sensor_name == "sensor1"
    assert dataset.child_frame == "sensor1"
    assert dataset.number_of_points == 4
    assert dataset.grid_points.shape == (4, 2)
    assert dataset.sensor_points.shape == (4, 2)
    assert dataset.nominal_pose == NominalPose(12.0, -1.0, 0.0)


def test_load_multi_sensor_dataset_requires_a_name(tmp_path):
    payload = {"sensors": {"sensor1": _dataset(), "sensor2": _dataset(name="sensor2")}}
    path = _write(tmp_path / "sensors.yaml", payload)

    with pytest.raises(CalibrationIOError):
        load_calibration_input(path)

    assert load_calibration_input(path, "sensor2").sensor_name == "sensor2"


def test_dataset_round_trip_recovers_the_true_pose(tmp_path):
    path = _write(tmp_path / "sensor1.yaml", _dataset())
    dataset = load_calibration_input(path)

    estimate = estimate_sensor_pose(dataset.sensor_points, dataset.grid_points)

    assert estimate.tx == pytest.approx(SENSOR1_POSE[0], abs=1e-9)
    assert estimate.ty == pytest.approx(SENSOR1_POSE[1], abs=1e-9)
    assert estimate.yaw == pytest.approx(SENSOR1_POSE[2], abs=1e-9)


def test_nominal_pose_accepts_degrees(tmp_path):
    payload = _dataset()
    payload["nominal_pose"] = {"x": 12.0, "y": 16.0, "yaw_deg": 90.0}
    dataset = load_calibration_input(_write(tmp_path / "sensor2.yaml", payload))
    assert dataset.nominal_pose.yaw == pytest.approx(math.pi / 2.0)


def test_missing_file_is_reported(tmp_path):
    with pytest.raises(CalibrationIOError):
        load_calibration_input(str(tmp_path / "does_not_exist.yaml"))


def test_mismatched_point_counts_are_rejected(tmp_path):
    payload = _dataset()
    payload["sensor_points"] = payload["sensor_points"][:3]
    with pytest.raises(InvalidShapeError):
        load_calibration_input(_write(tmp_path / "bad.yaml", payload))


def test_malformed_points_are_rejected(tmp_path):
    payload = _dataset()
    payload["sensor_points"][0] = [1.0, 2.0, 3.0]
    with pytest.raises(CalibrationIOError):
        load_calibration_input(_write(tmp_path / "bad.yaml", payload))


# --------------------------------------------------------------------------
# Result files
# --------------------------------------------------------------------------


def test_save_and_load_transforms_round_trip(tmp_path):
    path = str(tmp_path / "result.yaml")
    transforms = {
        "sensor1": SensorTransform("sensor1", 12.034, -0.982, 0.0064),
        "sensor2": SensorTransform("sensor2", 11.93, 16.08, -0.026),
    }
    save_sensor_transforms(path, transforms, {"width": 24.0, "height": 15.0})

    loaded = load_sensor_transforms(path)

    assert set(loaded) == {"sensor1", "sensor2"}
    for name, original in transforms.items():
        assert loaded[name].x == pytest.approx(original.x)
        assert loaded[name].y == pytest.approx(original.y)
        assert loaded[name].yaw == pytest.approx(original.yaw)
        assert loaded[name].parent_frame == "grid"
        assert loaded[name].child_frame == name


def test_flat_layout_is_also_accepted(tmp_path):
    path = _write(
        tmp_path / "flat.yaml",
        {
            "sensor1": {
                "parent_frame": "grid",
                "child_frame": "sensor1",
                "x": 12.034,
                "y": -0.982,
                "yaw": 0.0064,
                "calibration": {"rms_error": 0.012},
            }
        },
    )
    loaded = load_sensor_transforms(path)
    assert loaded["sensor1"].x == pytest.approx(12.034)
    assert loaded["sensor1"].calibration["rms_error"] == pytest.approx(0.012)


def test_update_preserves_other_sensors(tmp_path):
    path = str(tmp_path / "result.yaml")
    update_calibration_file(path, "sensor1", SensorTransform("sensor1", 1.0, 2.0, 0.1))
    update_calibration_file(path, "sensor2", SensorTransform("sensor2", 3.0, 4.0, 0.2))
    update_calibration_file(path, "sensor1", SensorTransform("sensor1", 5.0, 6.0, 0.3))

    loaded = load_sensor_transforms(path)

    assert loaded["sensor1"].x == pytest.approx(5.0)
    assert loaded["sensor2"].x == pytest.approx(3.0)
    assert len(loaded) == 2


def test_transform_from_estimate_stores_quality_and_errors(tmp_path):
    targets = np.asarray(TARGETS)
    sensor_points = grid_to_sensor(targets, *SENSOR1_POSE)
    estimate = estimate_sensor_pose(sensor_points, targets)
    error = installation_error(estimate.tx, estimate.ty, estimate.yaw, 12.0, -1.0, 0.0)

    transform = transform_from_estimate(
        "sensor1", estimate, nominal=NominalPose(12.0, -1.0, 0.0), error=error
    )
    path = str(tmp_path / "result.yaml")
    save_sensor_transforms(path, {"sensor1": transform})
    stored = yaml.safe_load(open(path, encoding="utf-8"))["sensors"]["sensor1"]

    assert stored["calibration"]["number_of_points"] == 4
    assert stored["calibration"]["method"] == "closed_form"
    assert stored["calibration"]["installation_error"]["dx"] == pytest.approx(0.20, abs=1e-6)
    assert stored["calibration"]["installation_error"]["dy"] == pytest.approx(0.15, abs=1e-6)
    assert stored["nominal_pose"]["x"] == pytest.approx(12.0)
    assert stored["yaw_deg"] == pytest.approx(2.0, abs=1e-6)


def test_empty_result_file_is_reported(tmp_path):
    path = _write(tmp_path / "result.yaml", {"grid": {"width": 24.0}})
    with pytest.raises(CalibrationIOError):
        load_sensor_transforms(path)


def test_shipped_example_configuration_is_consistent():
    """The datasets and the example result shipped in config/ must agree."""
    import os

    config_dir = os.path.join(os.path.dirname(os.path.dirname(__file__)), "config")
    if not os.path.isdir(config_dir):  # installed copy without sources
        pytest.skip("config directory not available")

    transforms = load_sensor_transforms(os.path.join(config_dir, "sensors_calibration.yaml"))
    for name in ("sensor1", "sensor2"):
        dataset = load_calibration_input(os.path.join(config_dir, f"{name}.yaml"))
        estimate = estimate_sensor_pose(dataset.sensor_points, dataset.grid_points)
        assert estimate.tx == pytest.approx(transforms[name].x, abs=1e-6)
        assert estimate.ty == pytest.approx(transforms[name].y, abs=1e-6)
        assert estimate.yaw == pytest.approx(transforms[name].yaw, abs=1e-6)
        assert estimate.rms_error < 0.02
