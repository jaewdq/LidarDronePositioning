"""Transforms live sensor detections into the shared ``grid`` frame.

Each configured sensor gets one subscription and one publisher::

    /sensor1/target_point  (frame_id: sensor1)  ->  /sensor1/target_point_grid  (frame_id: grid)
    /sensor2/target_point  (frame_id: sensor2)  ->  /sensor2/target_point_grid  (frame_id: grid)

Two interchangeable back-ends are available and must agree numerically
(``test/test_transform.py`` asserts exactly that):

* ``transform_mode: tf2``    -- looks the transform up in the TF tree,
* ``transform_mode: direct`` -- applies ``Pg = R Ps + t`` from the calibration
  YAML without any TF traffic (handy for offline replay of recorded bags).

Every converted detection also goes to one common topic
(``fused_topic``) and through the :meth:`PointTransformNode.on_grid_detection`
hook, which is the intended attachment point for future target association,
multi-sensor fusion and Kalman filtering: those stages only ever see detections
already expressed in ``grid``.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Dict, List, Optional

import numpy as np
import rclpy
import tf2_geometry_msgs  # noqa: F401  (registers PointStamped with tf2)
from geometry_msgs.msg import PointStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import LookupException, TransformException  # noqa: F401
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from .calibration_io import CalibrationIOError, SensorTransform, load_sensor_transforms
from .calibration_math import sensor_to_grid
from .sensor_adapter import Measurement, MeasurementAdapter, create_adapter

__all__ = ["GridDetection", "SensorChannel", "PointTransformNode", "main"]

_TF2_MODE = "tf2"
_DIRECT_MODE = "direct"


@dataclass(frozen=True)
class GridDetection:
    """A detection after conversion into the common grid frame."""

    sensor_name: str
    frame_id: str          # always the target frame, e.g. "grid"
    stamp: Any             # builtin_interfaces/Time
    xy: np.ndarray         # (2,) position in the grid frame [m]

    def to_point_stamped(self) -> PointStamped:
        """Convert the detection into a ``PointStamped`` in the grid frame.

        Returns:
            The stamped point with ``z = 0``.
        """
        msg = PointStamped()
        msg.header.frame_id = self.frame_id
        msg.header.stamp = self.stamp
        msg.point.x = float(self.xy[0])
        msg.point.y = float(self.xy[1])
        msg.point.z = 0.0
        return msg


@dataclass
class SensorChannel:
    """Per-sensor wiring: adapter, frames, topics and publisher."""

    sensor_name: str
    source_frame: str
    input_topic: str
    output_topic: str
    adapter: MeasurementAdapter
    publisher: Any = None


class PointTransformNode(Node):
    """Republishes every sensor detection in the shared grid frame."""

    def __init__(self, **kwargs) -> None:
        """Declare parameters and build one channel per configured sensor.

        Args:
            **kwargs: Forwarded to ``rclpy.node.Node`` (e.g.
                ``parameter_overrides`` in unit tests).
        """
        super().__init__("point_transform_node", **kwargs)

        self.declare_parameter("sensors", ["sensor1", "sensor2"])
        self.declare_parameter("target_frame", "grid")
        self.declare_parameter("input_topic_template", "/{sensor}/target_point")
        self.declare_parameter("output_topic_template", "/{sensor}/target_point_grid")
        self.declare_parameter("input_message_type", "point_stamped")
        self.declare_parameter("transform_mode", _TF2_MODE)
        self.declare_parameter("calibration_file", "")
        self.declare_parameter("tf_timeout_sec", 0.2)
        self.declare_parameter("use_latest_transform", True)
        self.declare_parameter("fused_topic", "/targets/grid_points")
        self.declare_parameter("queue_size", 10)

        self.target_frame: str = self.get_parameter("target_frame").value
        self.transform_mode: str = str(self.get_parameter("transform_mode").value).lower()
        self.tf_timeout = Duration(seconds=float(self.get_parameter("tf_timeout_sec").value))
        self.use_latest_transform: bool = bool(
            self.get_parameter("use_latest_transform").value
        )
        queue_size = int(self.get_parameter("queue_size").value)

        self._transforms: Dict[str, SensorTransform] = {}
        if self.transform_mode == _DIRECT_MODE:
            self._load_direct_transforms()
        else:
            if self.transform_mode != _TF2_MODE:
                self.get_logger().warn(
                    f"unknown transform_mode '{self.transform_mode}', using '{_TF2_MODE}'"
                )
                self.transform_mode = _TF2_MODE
            self._buffer = Buffer()
            self._listener = TransformListener(self._buffer, self)

        fused_topic: str = self.get_parameter("fused_topic").value
        self.fused_publisher = (
            self.create_publisher(PointStamped, fused_topic, queue_size)
            if fused_topic
            else None
        )

        self.channels: Dict[str, SensorChannel] = {}
        adapter_name: str = self.get_parameter("input_message_type").value
        input_template: str = self.get_parameter("input_topic_template").value
        output_template: str = self.get_parameter("output_topic_template").value

        for sensor_name in [name for name in self.get_parameter("sensors").value if name]:
            source_frame = self._resolve_source_frame(sensor_name)
            channel = SensorChannel(
                sensor_name=sensor_name,
                source_frame=source_frame,
                input_topic=input_template.format(sensor=sensor_name),
                output_topic=output_template.format(sensor=sensor_name),
                adapter=create_adapter(adapter_name, default_frame_id=source_frame),
            )
            channel.publisher = self.create_publisher(
                PointStamped, channel.output_topic, queue_size
            )
            self.create_subscription(
                channel.adapter.message_type,
                channel.input_topic,
                self._make_callback(channel),
                queue_size,
            )
            self.channels[sensor_name] = channel
            self.get_logger().info(
                f"[{sensor_name}] {channel.input_topic} ({source_frame}) -> "
                f"{channel.output_topic} ({self.target_frame})"
            )

        if not self.channels:
            self.get_logger().error("no sensor configured; set the 'sensors' parameter")
        self.get_logger().info(f"transform_mode = {self.transform_mode}")

    # ------------------------------------------------------------------
    # Setup helpers
    # ------------------------------------------------------------------

    def _load_direct_transforms(self) -> None:
        """Load the calibration YAML used by the ``direct`` transform mode.

        Raises:
            SystemExit: If the calibration file is missing or unreadable, since
                direct mode cannot work without it.
        """
        calibration_file: str = self.get_parameter("calibration_file").value
        if not calibration_file:
            self.get_logger().error(
                "transform_mode 'direct' requires the 'calibration_file' parameter"
            )
            raise SystemExit(1)
        try:
            self._transforms = load_sensor_transforms(calibration_file)
        except CalibrationIOError as exc:
            self.get_logger().error(f"cannot load calibration: {exc}")
            raise SystemExit(1) from exc

    def _resolve_source_frame(self, sensor_name: str) -> str:
        """Return the TF frame of a sensor (its child frame, if calibrated).

        Args:
            sensor_name: Logical sensor name.

        Returns:
            The frame id used by that sensor's messages.
        """
        transform = self._transforms.get(sensor_name)
        return transform.child_frame if transform is not None else sensor_name

    def _make_callback(self, channel: SensorChannel):
        """Build the subscription callback bound to one sensor channel.

        Args:
            channel: The channel this callback belongs to.

        Returns:
            A callable suitable for ``create_subscription``.
        """

        def callback(msg: Any) -> None:
            measurement = channel.adapter.extract(msg)
            if measurement is None:
                self.get_logger().warn(
                    f"[{channel.sensor_name}] dropped a non-finite measurement",
                    throttle_duration_sec=5.0,
                )
                return
            detection = self.transform_measurement(channel, measurement)
            if detection is not None:
                self._publish(channel, detection)

        return callback

    # ------------------------------------------------------------------
    # Transformation
    # ------------------------------------------------------------------

    def transform_measurement(
        self, channel: SensorChannel, measurement: Measurement
    ) -> Optional[GridDetection]:
        """Convert one sensor-frame measurement into the grid frame.

        Args:
            channel: The originating sensor channel.
            measurement: Measurement expressed in the sensor frame.

        Returns:
            The :class:`GridDetection`, or ``None`` when the transform is not
            available yet.
        """
        if self.transform_mode == _DIRECT_MODE:
            return self._transform_direct(channel, measurement)
        return self._transform_tf2(channel, measurement)

    def _transform_direct(
        self, channel: SensorChannel, measurement: Measurement
    ) -> Optional[GridDetection]:
        """Apply ``Pg = R Ps + t`` from the calibration file.

        Args:
            channel: The originating sensor channel.
            measurement: Measurement expressed in the sensor frame.

        Returns:
            The grid-frame detection, or ``None`` if the sensor is uncalibrated.
        """
        transform = self._transforms.get(channel.sensor_name)
        if transform is None:
            self.get_logger().error(
                f"[{channel.sensor_name}] no calibrated transform available",
                throttle_duration_sec=5.0,
            )
            return None
        grid_xy = sensor_to_grid(measurement.xy, transform.x, transform.y, transform.yaw)
        return GridDetection(
            sensor_name=channel.sensor_name,
            frame_id=self.target_frame,
            stamp=measurement.stamp,
            xy=np.asarray(grid_xy, dtype=float),
        )

    def _transform_tf2(
        self, channel: SensorChannel, measurement: Measurement
    ) -> Optional[GridDetection]:
        """Transform through tf2 (``sensorN`` -> ``grid``).

        Args:
            channel: The originating sensor channel.
            measurement: Measurement expressed in the sensor frame.

        Returns:
            The grid-frame detection, or ``None`` when the lookup fails.
        """
        point = measurement.to_point_stamped()
        point.header.frame_id = channel.source_frame
        if self.use_latest_transform:
            point.header.stamp = Time().to_msg()  # zero stamp = latest available
        try:
            transformed = self._buffer.transform(
                point, self.target_frame, timeout=self.tf_timeout
            )
        except TransformException as exc:
            self.get_logger().warn(
                f"[{channel.sensor_name}] TF {channel.source_frame} -> "
                f"{self.target_frame} unavailable: {exc}",
                throttle_duration_sec=5.0,
            )
            return None
        return GridDetection(
            sensor_name=channel.sensor_name,
            frame_id=self.target_frame,
            stamp=measurement.stamp,
            xy=np.array([transformed.point.x, transformed.point.y], dtype=float),
        )

    # ------------------------------------------------------------------
    # Output
    # ------------------------------------------------------------------

    def _publish(self, channel: SensorChannel, detection: GridDetection) -> None:
        """Publish a grid-frame detection and run the extension hook.

        Args:
            channel: The originating sensor channel.
            detection: The converted detection.
        """
        msg = detection.to_point_stamped()
        channel.publisher.publish(msg)
        if self.fused_publisher is not None:
            self.fused_publisher.publish(msg)
        self.on_grid_detection(detection)

    def on_grid_detection(self, detection: GridDetection) -> None:
        """Hook called for every detection expressed in the grid frame.

        The default implementation only logs at debug level. Override it (or
        subscribe to ``fused_topic``) to plug in target association, a Kalman
        filter or any multi-sensor fusion stage without touching the transform
        logic.

        Args:
            detection: The converted detection.
        """
        self.get_logger().debug(
            f"[{detection.sensor_name}] grid ({detection.xy[0]:.3f}, {detection.xy[1]:.3f})"
        )


def main(args: Optional[List[str]] = None) -> None:
    """Entry point for ``ros2 run grid_sensor_calibration point_transform_node``.

    Args:
        args: Optional command line arguments forwarded to ``rclpy``.
    """
    rclpy.init(args=args)
    node: Optional[PointTransformNode] = None
    try:
        node = PointTransformNode()
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
