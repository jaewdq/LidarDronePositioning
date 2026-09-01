"""TF2 direction tests: the ROS transform must equal ``Pg = R Ps + t``.

This is the failure mode the specification warns about (section 29): swapping
parent and child, or the sign of the yaw, still "runs" but silently mirrors
every measurement. Every test here pins the direction down numerically.
"""

from __future__ import annotations

import math

import numpy as np
import pytest

rclpy = pytest.importorskip("rclpy", reason="ROS 2 Python runtime not available")
tf2_geometry_msgs = pytest.importorskip("tf2_geometry_msgs")  # noqa: F401
from geometry_msgs.msg import PointStamped  # noqa: E402
from tf2_ros.buffer import Buffer  # noqa: E402

from grid_sensor_calibration.calibration_io import (  # noqa: E402
    SensorTransform,
    load_sensor_transforms,
    save_sensor_transforms,
)
from grid_sensor_calibration.calibration_math import (  # noqa: E402
    grid_to_sensor,
    sensor_to_grid,
    yaw_to_quaternion,
)
from grid_sensor_calibration.sensor_tf_publisher import transform_to_msg  # noqa: E402

GRID_FRAME = "grid"
SENSOR1_POSE = (12.20, -0.85, math.radians(2.0))
SENSOR2_POSE = (11.93, 16.08, math.radians(-1.5))
TEST_POINTS = np.array([[0.0, 0.0], [1.0, 0.0], [0.0, 1.0], [7.5, -3.25], [-11.0, 9.0]])


def _static_buffer(sensor_frame: str, tx: float, ty: float, yaw: float) -> Buffer:
    """Create a TF buffer holding one static ``grid -> sensor`` transform.

    Args:
        sensor_frame: Child frame id.
        tx: Sensor origin x in the grid frame [m].
        ty: Sensor origin y in the grid frame [m].
        yaw: Sensor yaw in the grid frame [rad].

    Returns:
        A populated :class:`tf2_ros.buffer.Buffer`.
    """
    transform = SensorTransform(
        child_frame=sensor_frame, x=tx, y=ty, yaw=yaw, parent_frame=GRID_FRAME
    )
    buffer = Buffer()
    buffer.set_transform_static(
        transform_to_msg(transform, rclpy.time.Time().to_msg()), "unit_test"
    )
    return buffer


def _tf2_point(buffer: Buffer, sensor_frame: str, xy) -> np.ndarray:
    """Transform a sensor-frame point into the grid frame through tf2.

    Args:
        buffer: Buffer holding the transform.
        sensor_frame: Frame the point is stamped in.
        xy: ``(x, y)`` in the sensor frame.

    Returns:
        ``(2,)`` array with the grid-frame coordinates.
    """
    point = PointStamped()
    point.header.frame_id = sensor_frame
    point.point.x = float(xy[0])
    point.point.y = float(xy[1])
    transformed = buffer.transform(point, GRID_FRAME)
    return np.array([transformed.point.x, transformed.point.y])


# --------------------------------------------------------------------------
# tf2 versus the direct matrix computation
# --------------------------------------------------------------------------


@pytest.mark.parametrize("pose", [SENSOR1_POSE, SENSOR2_POSE])
def test_tf2_matches_direct_computation(pose):
    tx, ty, yaw = pose
    buffer = _static_buffer("sensor1", tx, ty, yaw)
    for xy in TEST_POINTS:
        assert _tf2_point(buffer, "sensor1", xy) == pytest.approx(
            sensor_to_grid(xy, tx, ty, yaw), abs=1e-9
        )


def test_tf2_reproduces_the_specification_example():
    """Section 29: sensor at (10, 5, 90 deg), point (1, 0) -> grid (10, 6)."""
    buffer = _static_buffer("sensor1", 10.0, 5.0, math.radians(90.0))
    assert _tf2_point(buffer, "sensor1", (1.0, 0.0)) == pytest.approx([10.0, 6.0], abs=1e-9)


def test_tf2_translation_is_the_sensor_origin():
    tx, ty, yaw = SENSOR1_POSE
    buffer = _static_buffer("sensor1", tx, ty, yaw)
    assert _tf2_point(buffer, "sensor1", (0.0, 0.0)) == pytest.approx([tx, ty], abs=1e-9)


def test_swapped_parent_and_child_would_give_a_different_answer():
    """Guard: the inverse TF direction is *not* accidentally equivalent."""
    tx, ty, yaw = SENSOR1_POSE
    forward = sensor_to_grid([1.0, 0.0], tx, ty, yaw)
    inverted = grid_to_sensor([1.0, 0.0], tx, ty, yaw)
    assert not np.allclose(forward, inverted)


def test_yaw_quaternion_sign_convention_is_ccw():
    """A +90 deg TF must rotate the sensor +x axis onto the grid +y axis."""
    buffer = _static_buffer("sensor1", 0.0, 0.0, math.radians(90.0))
    assert _tf2_point(buffer, "sensor1", (1.0, 0.0)) == pytest.approx([0.0, 1.0], abs=1e-9)
    assert yaw_to_quaternion(math.radians(90.0))[2] > 0.0


# --------------------------------------------------------------------------
# Full chain: YAML -> TransformStamped -> tf2 -> point
# --------------------------------------------------------------------------


def test_yaml_to_tf2_chain_matches_direct_math(tmp_path):
    transforms = {
        "sensor1": SensorTransform("sensor1", *SENSOR1_POSE, parent_frame=GRID_FRAME),
        "sensor2": SensorTransform("sensor2", *SENSOR2_POSE, parent_frame=GRID_FRAME),
    }
    path = tmp_path / "sensors_calibration.yaml"
    save_sensor_transforms(str(path), transforms)

    loaded = load_sensor_transforms(str(path))
    buffer = Buffer()
    stamp = rclpy.time.Time().to_msg()
    for transform in loaded.values():
        buffer.set_transform_static(transform_to_msg(transform, stamp), "unit_test")

    for name, pose in (("sensor1", SENSOR1_POSE), ("sensor2", SENSOR2_POSE)):
        for xy in TEST_POINTS:
            assert _tf2_point(buffer, name, xy) == pytest.approx(
                sensor_to_grid(xy, *pose), abs=1e-9
            )


def test_both_sensors_agree_on_a_common_target():
    """The point of the whole exercise: one target, two sensors, one grid position."""
    target_grid = np.array([9.0, 6.0])
    measurement1 = grid_to_sensor(target_grid, *SENSOR1_POSE)
    measurement2 = grid_to_sensor(target_grid, *SENSOR2_POSE)

    buffer = Buffer()
    stamp = rclpy.time.Time().to_msg()
    for name, pose in (("sensor1", SENSOR1_POSE), ("sensor2", SENSOR2_POSE)):
        buffer.set_transform_static(
            transform_to_msg(
                SensorTransform(name, pose[0], pose[1], pose[2], parent_frame=GRID_FRAME),
                stamp,
            ),
            "unit_test",
        )

    assert _tf2_point(buffer, "sensor1", measurement1) == pytest.approx(target_grid, abs=1e-9)
    assert _tf2_point(buffer, "sensor2", measurement2) == pytest.approx(target_grid, abs=1e-9)
