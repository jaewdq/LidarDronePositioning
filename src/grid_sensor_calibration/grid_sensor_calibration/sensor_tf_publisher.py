"""Static TF publisher for the calibrated ``grid -> sensorN`` transforms.

Once a calibration exists there is no reason to recompute it, so this node
simply reads the calibration YAML and broadcasts one **static** transform per
sensor. The number of sensors comes from the file (or the ``sensors``
parameter), never from the code.

TF direction (the classic pitfall)
----------------------------------
``header.frame_id`` = ``grid`` (parent), ``child_frame_id`` = ``sensorN``
(child), translation = the sensor origin **expressed in the grid frame**,
rotation = yaw of the sensor x-axis w.r.t. the grid x-axis. With that
convention tf2 maps a point stamped in ``sensorN`` into ``grid`` as
``Pg = R(yaw) * Ps + t`` -- identical to
:func:`grid_sensor_calibration.calibration_math.sensor_to_grid`.
"""

from __future__ import annotations

from typing import Dict, List, Optional

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

from .calibration_io import CalibrationIOError, SensorTransform, load_sensor_transforms
from .calibration_math import yaw_to_quaternion

__all__ = ["SensorTfPublisher", "transform_to_msg", "main"]


def transform_to_msg(
    transform: SensorTransform, stamp, parent_frame: Optional[str] = None
) -> TransformStamped:
    """Convert a :class:`SensorTransform` into a ``TransformStamped``.

    Args:
        transform: Calibrated planar transform.
        stamp: ``builtin_interfaces/Time`` used for the header.
        parent_frame: Overrides the transform's own parent frame when given.

    Returns:
        The stamped transform with ``z = 0`` and a yaw-only quaternion.
    """
    msg = TransformStamped()
    msg.header.stamp = stamp
    msg.header.frame_id = parent_frame or transform.parent_frame
    msg.child_frame_id = transform.child_frame
    msg.transform.translation.x = float(transform.x)
    msg.transform.translation.y = float(transform.y)
    msg.transform.translation.z = float(transform.z)
    qx, qy, qz, qw = yaw_to_quaternion(transform.yaw)
    msg.transform.rotation.x = qx
    msg.transform.rotation.y = qy
    msg.transform.rotation.z = qz
    msg.transform.rotation.w = qw
    return msg


class SensorTfPublisher(Node):
    """Broadcasts every calibrated sensor transform as a static TF."""

    def __init__(self, **kwargs) -> None:
        """Read the calibration file and broadcast the static transforms.

        Args:
            **kwargs: Forwarded to ``rclpy.node.Node`` (e.g.
                ``parameter_overrides`` in unit tests).
        """
        super().__init__("sensor_tf_publisher", **kwargs)

        self.declare_parameter("calibration_file", "calibration_result.yaml")
        self.declare_parameter("parent_frame", "")
        self.declare_parameter("sensors", [""])

        calibration_file: str = self.get_parameter("calibration_file").value
        parent_override: str = self.get_parameter("parent_frame").value
        selected = [name for name in self.get_parameter("sensors").value if name]

        try:
            transforms: Dict[str, SensorTransform] = load_sensor_transforms(calibration_file)
        except CalibrationIOError as exc:
            self.get_logger().error(
                f"cannot publish sensor TFs: {exc}. Run the calibration first."
            )
            raise SystemExit(1) from exc

        if selected:
            missing = [name for name in selected if name not in transforms]
            if missing:
                self.get_logger().warn(
                    f"sensors {missing} are not present in '{calibration_file}'"
                )
            transforms = {
                name: tf for name, tf in transforms.items() if name in selected
            }
        if not transforms:
            self.get_logger().error("no sensor selected for TF publication")
            raise SystemExit(1)

        self._broadcaster = StaticTransformBroadcaster(self)
        stamp = self.get_clock().now().to_msg()
        messages = [
            transform_to_msg(tf, stamp, parent_override or None)
            for tf in transforms.values()
        ]
        self._broadcaster.sendTransform(messages)

        for name, tf in transforms.items():
            self.get_logger().info(
                f"static TF {parent_override or tf.parent_frame} -> {tf.child_frame} "
                f"[{name}]: x={tf.x:.3f} m, y={tf.y:.3f} m, yaw={tf.yaw_deg:.3f} deg"
            )
        self.get_logger().info(
            f"published {len(messages)} static transform(s) from '{calibration_file}'"
        )


def main(args: Optional[List[str]] = None) -> None:
    """Entry point for ``ros2 run grid_sensor_calibration sensor_tf_publisher``.

    Args:
        args: Optional command line arguments forwarded to ``rclpy``.
    """
    rclpy.init(args=args)
    node: Optional[SensorTfPublisher] = None
    try:
        node = SensorTfPublisher()
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
