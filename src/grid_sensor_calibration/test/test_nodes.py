"""Node-level tests: calibration node, static TF publisher, point transform.

They exercise the ROS wiring (parameters, YAML persistence, TF publication and
the two transform back-ends) without any hardware.
"""

from __future__ import annotations

import math

import numpy as np
import pytest
import yaml

rclpy = pytest.importorskip("rclpy", reason="ROS 2 Python runtime not available")
from geometry_msgs.msg import PointStamped, PoseStamped  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402
from tf2_ros.buffer import Buffer  # noqa: E402
from tf2_ros.transform_listener import TransformListener  # noqa: E402

from grid_sensor_calibration.calibration_io import load_sensor_transforms  # noqa: E402
from grid_sensor_calibration.calibration_math import (  # noqa: E402
    grid_to_sensor,
    sensor_to_grid,
)
from grid_sensor_calibration.sensor_adapter import create_adapter  # noqa: E402
from grid_sensor_calibration.sensor_calibration_node import (  # noqa: E402
    SensorCalibrationNode,
)

TARGETS = np.array([[3.0, 3.0], [21.0, 3.0], [21.0, 12.0], [3.0, 12.0]])
SENSOR1_POSE = (12.20, -0.85, math.radians(2.0))
SENSOR2_POSE = (11.93, 16.08, math.radians(-1.5))
_SPIN_TIMEOUT_SEC = 5.0


@pytest.fixture(scope="module", autouse=True)
def ros_context():
    """Initialise and shut down rclpy once for this module."""
    rclpy.init()
    yield
    rclpy.shutdown()


def _write_dataset(tmp_path, name, pose, nominal) -> str:
    """Write a synthetic calibration dataset and return its path."""
    payload = {
        "sensor_name": name,
        "parent_frame": "grid",
        "child_frame": name,
        "nominal_pose": {"x": nominal[0], "y": nominal[1], "yaw": nominal[2]},
        "grid_points": TARGETS.tolist(),
        "sensor_points": grid_to_sensor(TARGETS, *pose).tolist(),
    }
    path = tmp_path / f"{name}.yaml"
    path.write_text(yaml.safe_dump(payload, sort_keys=False), encoding="utf-8")
    return str(path)


def _params(**kwargs):
    """Build a parameter override list from keyword arguments."""
    return [Parameter(name, value=value) for name, value in kwargs.items()]


# --------------------------------------------------------------------------
# Calibration node
# --------------------------------------------------------------------------


def test_calibration_node_estimates_and_persists(tmp_path):
    dataset = _write_dataset(tmp_path, "sensor1", SENSOR1_POSE, (12.0, -1.0, 0.0))
    result_file = str(tmp_path / "sensors_calibration.yaml")
    node = SensorCalibrationNode(
        parameter_overrides=_params(
            sensor_name="sensor1",
            child_frame="sensor1",
            input_file=dataset,
            calibration_file=result_file,
            nominal_x=12.0,
            nominal_y=-1.0,
            auto_calibrate=False,
        )
    )
    try:
        estimate = node.calibrate()
    finally:
        node.destroy_node()

    assert estimate.tx == pytest.approx(SENSOR1_POSE[0], abs=1e-9)
    assert estimate.ty == pytest.approx(SENSOR1_POSE[1], abs=1e-9)
    assert estimate.yaw == pytest.approx(SENSOR1_POSE[2], abs=1e-9)
    assert estimate.number_of_points == 4

    stored = load_sensor_transforms(result_file)["sensor1"]
    assert stored.parent_frame == "grid"
    assert stored.child_frame == "sensor1"
    assert stored.x == pytest.approx(SENSOR1_POSE[0], abs=1e-9)
    assert stored.calibration["installation_error"]["dx"] == pytest.approx(0.20, abs=1e-6)
    assert stored.calibration["installation_error"]["dy"] == pytest.approx(0.15, abs=1e-6)


def test_calibration_node_keeps_sensors_independent(tmp_path):
    result_file = str(tmp_path / "sensors_calibration.yaml")
    for name, pose, nominal in (
        ("sensor1", SENSOR1_POSE, (12.0, -1.0)),
        ("sensor2", SENSOR2_POSE, (12.0, 16.0)),
    ):
        dataset = _write_dataset(tmp_path, name, pose, (*nominal, 0.0))
        node = SensorCalibrationNode(
            parameter_overrides=_params(
                sensor_name=name,
                child_frame=name,
                input_file=dataset,
                calibration_file=result_file,
                nominal_x=nominal[0],
                nominal_y=nominal[1],
                auto_calibrate=False,
            )
        )
        try:
            node.calibrate()
        finally:
            node.destroy_node()

    stored = load_sensor_transforms(result_file)
    assert set(stored) == {"sensor1", "sensor2"}
    assert stored["sensor2"].y == pytest.approx(SENSOR2_POSE[1], abs=1e-9)


def test_calibration_node_reports_missing_data(tmp_path):
    node = SensorCalibrationNode(
        parameter_overrides=_params(
            sensor_name="sensor9",
            calibration_file=str(tmp_path / "unused.yaml"),
            auto_calibrate=False,
        )
    )
    try:
        response = node._srv_run_calibration(None, type("R", (), {})())
        assert response.success is False
        assert "no calibration data" in response.message
    finally:
        node.destroy_node()


def test_calibration_node_live_capture_service(tmp_path):
    """Method B: capture measurements from the topic, then calibrate."""
    result_file = str(tmp_path / "sensors_calibration.yaml")
    node = SensorCalibrationNode(
        parameter_overrides=_params(
            sensor_name="sensor1",
            child_frame="sensor1",
            calibration_file=result_file,
            nominal_x=12.0,
            nominal_y=-1.0,
            auto_calibrate=False,
            capture_sample_count=5,
            grid_points=TARGETS.flatten().tolist(),
        )
    )
    response_type = type("R", (), {})
    try:
        measurements = grid_to_sensor(TARGETS, *SENSOR1_POSE)
        for measurement in measurements:
            for _ in range(5):  # several samples per target, they get averaged
                msg = PointStamped()
                msg.header.frame_id = "sensor1"
                msg.point.x, msg.point.y = float(measurement[0]), float(measurement[1])
                node._on_measurement(msg)
            assert node._srv_capture_point(None, response_type()).success is True

        # No fifth target configured -> capture must refuse.
        node._on_measurement(PointStamped())
        assert node._srv_capture_point(None, response_type()).success is False

        estimate = node.calibrate()
    finally:
        node.destroy_node()

    assert estimate.number_of_points == 4
    assert estimate.tx == pytest.approx(SENSOR1_POSE[0], abs=1e-9)
    assert estimate.yaw == pytest.approx(SENSOR1_POSE[2], abs=1e-9)


def test_calibration_node_applies_tilt_correction_to_captures(tmp_path):
    """Captured xyz points are pre-rotated by known roll/pitch before 2D fit."""
    tx, ty, yaw = 1.5, 3.5, math.radians(88.0)
    roll, pitch = math.radians(3.0), math.radians(-2.0)

    planar_sensor = grid_to_sensor(TARGETS, tx, ty, yaw)
    corrected = np.column_stack([
        planar_sensor[:, 0],
        planar_sensor[:, 1],
        np.linspace(-1.2, -0.2, TARGETS.shape[0]),
    ])
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    rx = np.array([
        [1.0, 0.0, 0.0],
        [0.0, cr, -sr],
        [0.0, sr, cr],
    ])
    ry = np.array([
        [cp, 0.0, sp],
        [0.0, 1.0, 0.0],
        [-sp, 0.0, cp],
    ])
    raw_captures = ((ry @ rx).T @ corrected.T).T

    node = SensorCalibrationNode(
        parameter_overrides=_params(
            sensor_name="sensor1",
            child_frame="sensor1",
            calibration_file=str(tmp_path / "sensors_calibration.yaml"),
            auto_calibrate=False,
            save_result=False,
            grid_points=TARGETS.flatten().tolist(),
            apply_tilt_correction=True,
            sensor_z=2.0,
            sensor_roll=roll,
            sensor_pitch=pitch,
        )
    )
    try:
        node._captured = [raw_captures[i] for i in range(raw_captures.shape[0])]
        assert node.apply_tilt_correction is True
        assert node.sensor_z == pytest.approx(2.0)
        assert node._captured_sensor_xy() == pytest.approx(planar_sensor, abs=1e-9)
        estimate = node.calibrate()
    finally:
        node.destroy_node()

    assert estimate.tx == pytest.approx(tx, abs=1e-9)
    assert estimate.ty == pytest.approx(ty, abs=1e-9)
    assert estimate.yaw == pytest.approx(yaw, abs=1e-9)
    assert estimate.rms_error < 1e-9


# --------------------------------------------------------------------------
# Static TF publisher (full ROS round trip)
# --------------------------------------------------------------------------


def test_point_stamped_adapter_reads_frame_and_position():
    adapter = create_adapter("point_stamped", default_frame_id="fallback")
    msg = PointStamped()
    msg.header.frame_id = "sensor2"
    msg.point.x, msg.point.y, msg.point.z = 1.5, -2.5, 0.75

    measurement = adapter.extract(msg)

    assert measurement.frame_id == "sensor2"
    assert measurement.x == pytest.approx(1.5)
    assert measurement.y == pytest.approx(-2.5)
    assert measurement.z == pytest.approx(0.75)
    assert measurement.to_point_stamped().header.frame_id == "sensor2"


def test_adapter_falls_back_to_the_default_frame():
    adapter = create_adapter("point_stamped", default_frame_id="sensor1")
    assert adapter.extract(PointStamped()).frame_id == "sensor1"


def test_pose_stamped_adapter_uses_the_position():
    adapter = create_adapter("pose_stamped", default_frame_id="sensor1")
    msg = PoseStamped()
    msg.pose.position.x, msg.pose.position.y = 3.0, 4.0
    measurement = adapter.extract(msg)
    assert (measurement.x, measurement.y) == pytest.approx((3.0, 4.0))


def test_adapter_rejects_non_finite_measurements():
    adapter = create_adapter("point_stamped", default_frame_id="sensor1")
    msg = PointStamped()
    msg.point.x = float("nan")
    assert adapter.extract(msg) is None


def test_unknown_adapter_is_rejected():
    with pytest.raises(ValueError):
        create_adapter("laser_scan")


# ==========================================================================
# Capture recovery: undo, indexed re-capture, reset backup, restart
# ==========================================================================


def _feed(node, xy, z=0.0, count=5):
    """Push several identical samples so a capture has something to average."""
    for _ in range(count):
        msg = PointStamped()
        msg.header.frame_id = "sensor1"
        msg.point.x, msg.point.y, msg.point.z = float(xy[0]), float(xy[1]), float(z)
        node._on_measurement(msg)


def _capture_node(tmp_path, **overrides):
    """Build a live-capture node writing its store under tmp_path."""
    params = dict(
        sensor_name="sensor1",
        child_frame="sensor1",
        calibration_file=str(tmp_path / "sensors_calibration.yaml"),
        auto_calibrate=False,
        capture_sample_count=5,
        grid_points=TARGETS.flatten().tolist(),
        capture_store=str(tmp_path / "sensor1_captures.yaml"),
    )
    params.update(overrides)
    return SensorCalibrationNode(parameter_overrides=_params(**params))


def test_undo_point_removes_only_the_last_capture(tmp_path):
    """'~/undo_point' pops one point and frees its slot for a re-capture."""
    node = _capture_node(tmp_path)
    reply = type("R", (), {})
    try:
        measurements = grid_to_sensor(TARGETS, *SENSOR1_POSE)
        for measurement in measurements:
            _feed(node, measurement)
            assert node._srv_capture_point(None, reply()).success is True
        assert len(node._captured) == 4

        assert node._srv_undo_point(None, reply()).success is True
        assert len(node._captured) == 3

        # The freed slot is the fourth target again, not a fifth one.
        _feed(node, measurements[3])
        assert node._srv_capture_point(None, reply()).success is True
        assert len(node._captured) == 4
        assert node.calibrate().rms_error < 1e-6

        node._captured.clear()
        assert node._srv_undo_point(None, reply()).success is False
    finally:
        node.destroy_node()


def test_capture_index_rewrites_one_slot_and_resets(tmp_path):
    """A bad target can be redone alone, leaving the other three untouched."""
    node = _capture_node(tmp_path)
    reply = type("R", (), {})
    try:
        measurements = grid_to_sensor(TARGETS, *SENSOR1_POSE)
        for index, measurement in enumerate(measurements):
            # Target 2 is captured badly, half a metre off.
            offset = np.array([0.5, 0.5]) if index == 2 else np.zeros(2)
            _feed(node, measurement + offset)
            assert node._srv_capture_point(None, reply()).success is True

        spoiled = node.calibrate().rms_error
        assert spoiled > 0.1

        # Redo just that one.
        node.set_parameters([Parameter("capture_index", Parameter.Type.INTEGER, 2)])
        _feed(node, measurements[2])
        response = node._srv_capture_point(None, reply())
        assert response.success is True
        assert "re-captured" in response.message
        assert len(node._captured) == 4

        # The index is one-shot: it must be back to append mode.
        assert node.get_parameter("capture_index").value == -1
        assert node.calibrate().rms_error < 1e-6
    finally:
        node.destroy_node()


def test_capture_index_refuses_to_leave_a_hole(tmp_path):
    """Writing past the end would break the positional correspondence."""
    node = _capture_node(tmp_path)
    reply = type("R", (), {})
    try:
        measurements = grid_to_sensor(TARGETS, *SENSOR1_POSE)
        _feed(node, measurements[0])
        assert node._srv_capture_point(None, reply()).success is True

        node.set_parameters([Parameter("capture_index", Parameter.Type.INTEGER, 3)])
        _feed(node, measurements[3])
        response = node._srv_capture_point(None, reply())
        assert response.success is False
        assert "hole" in response.message
        assert len(node._captured) == 1
    finally:
        node.destroy_node()


def test_reset_is_reversible_with_restore(tmp_path):
    """'~/reset' backs the captures up so it is no longer destructive."""
    node = _capture_node(tmp_path)
    reply = type("R", (), {})
    try:
        measurements = grid_to_sensor(TARGETS, *SENSOR1_POSE)
        for measurement in measurements:
            _feed(node, measurement)
            node._srv_capture_point(None, reply())
        before = np.vstack(node._captured).copy()

        response = node._srv_reset(None, reply())
        assert response.success is True
        assert "restore" in response.message
        assert len(node._captured) == 0

        assert node._srv_restore(None, reply()).success is True
        assert np.allclose(np.vstack(node._captured), before)
    finally:
        node.destroy_node()


def test_captures_survive_a_node_restart(tmp_path):
    """The store is reloaded, so a crash costs no walking."""
    reply = type("R", (), {})
    measurements = grid_to_sensor(TARGETS, *SENSOR1_POSE)

    node = _capture_node(tmp_path)
    try:
        for measurement in measurements[:3]:
            _feed(node, measurement, z=-1.25)
            node._srv_capture_point(None, reply())
    finally:
        node.destroy_node()

    revived = _capture_node(tmp_path)
    try:
        assert len(revived._captured) == 3
        # z is carried through, unlike before: it is what a ground-height
        # cross-check would need.
        assert revived._captured[0][2] == pytest.approx(-1.25)

        _feed(revived, measurements[3], z=-1.25)
        assert revived._srv_capture_point(None, reply()).success is True
        assert revived.calibrate().rms_error < 1e-6
    finally:
        revived.destroy_node()


def test_store_recorded_against_other_targets_is_refused(tmp_path):
    """Reloading against a different grid would mis-pair every measurement."""
    reply = type("R", (), {})
    node = _capture_node(tmp_path)
    try:
        for measurement in grid_to_sensor(TARGETS, *SENSOR1_POSE):
            _feed(node, measurement)
            node._srv_capture_point(None, reply())
    finally:
        node.destroy_node()

    moved = (TARGETS + np.array([1.0, 0.0])).flatten().tolist()
    revived = _capture_node(tmp_path, grid_points=moved)
    try:
        assert len(revived._captured) == 0
    finally:
        revived.destroy_node()


def test_status_flags_the_worst_point(tmp_path):
    """'~/status' names the outlier and prints the command to redo it."""
    node = _capture_node(tmp_path)
    reply = type("R", (), {})
    try:
        measurements = grid_to_sensor(TARGETS, *SENSOR1_POSE)
        for index, measurement in enumerate(measurements):
            offset = np.array([0.6, 0.0]) if index == 1 else np.zeros(2)
            _feed(node, measurement + offset)
            node._srv_capture_point(None, reply())

        response = node._srv_status(None, reply())
        assert response.success is True
        assert "rms if dropped" in response.message
        assert "dropping point 1" in response.message
        assert "capture_index 1" in response.message
    finally:
        node.destroy_node()
