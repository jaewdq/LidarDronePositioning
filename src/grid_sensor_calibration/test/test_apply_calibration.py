"""Tests for writing a calibration result into the tracker YAML."""

from __future__ import annotations

import math

import numpy as np

from grid_sensor_calibration.apply_calibration_node import apply_to_yaml_text, tracker_values
from grid_sensor_calibration.calibration_io import (
    load_sensor_transforms,
    transform_from_estimate,
    update_calibration_file,
)
from grid_sensor_calibration.calibration_math import estimate_sensor_pose, grid_to_sensor

GRID = np.array([[0.0, 0.0], [3.0, 0.0], [0.0, 1.5], [3.0, 1.5]])
POSE = (1.48, -0.73, math.radians(-88.5))

TRACKER_YAML = """lidar_drone_tracker:
  ros__parameters:
    lidar1_x:      1.5
    lidar1_y:     -0.7
    lidar1_z:      1.2
    lidar1_roll:   0.0
    lidar1_pitch:  0.0
    lidar1_yaw:    -1.570796   # comment kept
    lidar2_x:      1.5
    lidar1_extrinsic_rmse: 0.02
    roi_x_min: -0.3
"""


def _saved(tmp_path, tilt=None):
    sensor = grid_to_sensor(GRID, *POSE)
    estimate = estimate_sensor_pose(sensor, GRID)
    transform = transform_from_estimate("sensor1", estimate, child_frame="lidar1", extra=tilt)
    path = tmp_path / "sensors_calibration.yaml"
    update_calibration_file(str(path), "sensor1", transform)
    return load_sensor_transforms(str(path))


def test_values_include_tilt_when_stored(tmp_path):
    values = tracker_values(_saved(tmp_path, {
        "sensor_z": 1.267, "sensor_roll": 0.010077, "sensor_pitch": -0.016381}))
    assert math.isclose(values["lidar1_x"], POSE[0], abs_tol=1e-6)
    assert math.isclose(values["lidar1_y"], POSE[1], abs_tol=1e-6)
    assert math.isclose(values["lidar1_yaw"], POSE[2], abs_tol=1e-6)
    assert values["lidar1_z"] == 1.267
    assert values["lidar1_pitch"] == -0.016381
    assert values["lidar1_extrinsic_rmse"] < 1e-6


def test_values_without_tilt_leave_z_roll_pitch_out(tmp_path):
    values = tracker_values(_saved(tmp_path))
    assert "lidar1_z" not in values and "lidar1_roll" not in values
    assert "lidar1_x" in values


def test_yaml_lines_replaced_in_place():
    values = {"lidar1_x": 1.48, "lidar1_yaw": -1.544616, "lidar1_z": 1.267}
    text, changes, missing = apply_to_yaml_text(TRACKER_YAML, values)
    assert "lidar1_x:      1.480\n" in text
    assert "lidar1_yaw:    -1.544616   # comment kept" in text
    assert "lidar1_z:      1.267\n" in text
    assert "lidar2_x:      1.5\n" in text        # other sensor untouched
    assert "roi_x_min: -0.3" in text
    assert ("lidar1_x", "1.5", "1.480") in changes
    assert missing == []
    assert text.count("\n") == TRACKER_YAML.count("\n")


def test_missing_key_reported_not_added():
    text, changes, missing = apply_to_yaml_text(TRACKER_YAML, {"lidar3_x": 1.0})
    assert text == TRACKER_YAML
    assert changes == [] and missing == ["lidar3_x"]
