"""Optional RViz2 visualisation of the grid, the targets and the residuals.

Kept strictly separate from the calibration itself: this node only *reads* the
calibration input/result YAML files and republishes them as a
``visualization_msgs/MarkerArray``. Nothing here is required to calibrate or to
publish TFs.

Markers published on ``~/markers`` (frame ``grid``):

* the grid outline and its 3 m spacing lines,
* the known calibration targets,
* each sensor's position and heading, with a text label,
* each sensor's measurements re-projected into the grid frame,
* residual vectors between measurement and true target.
"""

from __future__ import annotations

from typing import Dict, List, Optional

import numpy as np
import rclpy
from builtin_interfaces.msg import Duration as DurationMsg
from geometry_msgs.msg import Point
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

from .calibration_io import (
    CalibrationInput,
    CalibrationIOError,
    SensorTransform,
    load_calibration_input,
    load_sensor_transforms,
)
from .calibration_math import sensor_to_grid, yaw_to_quaternion

__all__ = ["CalibrationVizNode", "main"]

_LATCHED_QOS = QoSProfile(
    depth=1,
    history=HistoryPolicy.KEEP_LAST,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)

#: Distinct colours cycled per sensor (RGBA).
_SENSOR_COLORS = (
    (0.10, 0.60, 1.00, 1.0),
    (1.00, 0.55, 0.10, 1.0),
    (0.30, 0.85, 0.35, 1.0),
    (0.85, 0.30, 0.85, 1.0),
)
_GRID_COLOR = (0.55, 0.55, 0.55, 0.6)
_TARGET_COLOR = (1.0, 0.95, 0.2, 1.0)
_RESIDUAL_COLOR = (1.0, 0.1, 0.1, 1.0)
_TARGET_DIAMETER = 0.35
_MEASUREMENT_DIAMETER = 0.22
_LINE_WIDTH = 0.04
_RESIDUAL_WIDTH = 0.06
_TEXT_HEIGHT = 0.6
_SENSOR_ARROW_LENGTH = 1.2


def _color(rgba) -> ColorRGBA:
    """Build a ``ColorRGBA`` message from a 4-tuple."""
    return ColorRGBA(r=float(rgba[0]), g=float(rgba[1]), b=float(rgba[2]), a=float(rgba[3]))


def _point(x: float, y: float, z: float = 0.0) -> Point:
    """Build a ``geometry_msgs/Point`` message."""
    return Point(x=float(x), y=float(y), z=float(z))


class CalibrationVizNode(Node):
    """Publishes a latched ``MarkerArray`` describing the calibration setup."""

    def __init__(self, **kwargs) -> None:
        """Declare parameters, load the YAML files and start the publish timer.

        Args:
            **kwargs: Forwarded to ``rclpy.node.Node`` (e.g.
                ``parameter_overrides`` in unit tests).
        """
        super().__init__("calibration_viz_node", **kwargs)

        self.declare_parameter("frame_id", "grid")
        self.declare_parameter("grid_width", 24.0)
        self.declare_parameter("grid_height", 15.0)
        self.declare_parameter("grid_spacing", 3.0)
        self.declare_parameter("calibration_file", "")
        self.declare_parameter("input_files", [""])
        self.declare_parameter("residual_scale", 1.0)
        self.declare_parameter("publish_period_sec", 2.0)

        self.frame_id: str = self.get_parameter("frame_id").value
        self.grid_width: float = float(self.get_parameter("grid_width").value)
        self.grid_height: float = float(self.get_parameter("grid_height").value)
        self.grid_spacing: float = float(self.get_parameter("grid_spacing").value)
        self.residual_scale: float = float(self.get_parameter("residual_scale").value)

        self._transforms: Dict[str, SensorTransform] = {}
        self._datasets: Dict[str, CalibrationInput] = {}
        self._load_files()

        self.publisher = self.create_publisher(MarkerArray, "~/markers", _LATCHED_QOS)
        period = max(float(self.get_parameter("publish_period_sec").value), 0.1)
        self.create_timer(period, self._publish_markers)
        self.get_logger().info(
            f"visualising {len(self._transforms)} sensor pose(s) and "
            f"{len(self._datasets)} calibration dataset(s) in frame '{self.frame_id}'"
        )

    # ------------------------------------------------------------------
    # Loading
    # ------------------------------------------------------------------

    def _load_files(self) -> None:
        """Load the calibration result and the per-sensor input datasets."""
        calibration_file: str = self.get_parameter("calibration_file").value
        if calibration_file:
            try:
                self._transforms = load_sensor_transforms(calibration_file)
            except CalibrationIOError as exc:
                self.get_logger().warn(f"no sensor pose to display: {exc}")

        for path in [item for item in self.get_parameter("input_files").value if item]:
            try:
                dataset = load_calibration_input(path)
            except CalibrationIOError as exc:
                self.get_logger().warn(f"skipping input file '{path}': {exc}")
                continue
            self._datasets[dataset.sensor_name] = dataset

    def _sensor_color(self, sensor_name: str):
        """Pick a stable colour for a sensor name."""
        names = sorted(set(self._transforms) | set(self._datasets))
        index = names.index(sensor_name) if sensor_name in names else 0
        return _SENSOR_COLORS[index % len(_SENSOR_COLORS)]

    # ------------------------------------------------------------------
    # Marker construction
    # ------------------------------------------------------------------

    def _base_marker(self, namespace: str, marker_id: int, marker_type: int) -> Marker:
        """Create a marker pre-filled with the common header fields.

        Args:
            namespace: Marker namespace.
            marker_id: Unique id inside the namespace.
            marker_type: One of the ``Marker`` type constants.

        Returns:
            The initialised marker.
        """
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = namespace
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.lifetime = DurationMsg(sec=0, nanosec=0)
        return marker

    def _grid_marker(self) -> Marker:
        """Build the grid outline and inner lines as a ``LINE_LIST``."""
        marker = self._base_marker("grid", 0, Marker.LINE_LIST)
        marker.scale.x = _LINE_WIDTH
        marker.color = _color(_GRID_COLOR)
        steps_x = int(round(self.grid_width / self.grid_spacing))
        steps_y = int(round(self.grid_height / self.grid_spacing))
        for index in range(steps_x + 1):
            x = min(index * self.grid_spacing, self.grid_width)
            marker.points += [_point(x, 0.0), _point(x, self.grid_height)]
        for index in range(steps_y + 1):
            y = min(index * self.grid_spacing, self.grid_height)
            marker.points += [_point(0.0, y), _point(self.grid_width, y)]
        return marker

    def _target_markers(self) -> List[Marker]:
        """Build the known calibration target markers (spheres + labels)."""
        markers: List[Marker] = []
        seen: List[tuple] = []
        for dataset in self._datasets.values():
            for row in dataset.grid_points:
                key = (round(float(row[0]), 3), round(float(row[1]), 3))
                if key not in seen:
                    seen.append(key)
        if not seen:
            return markers

        spheres = self._base_marker("targets", 0, Marker.SPHERE_LIST)
        spheres.scale.x = spheres.scale.y = spheres.scale.z = _TARGET_DIAMETER
        spheres.color = _color(_TARGET_COLOR)
        spheres.points = [_point(x, y) for x, y in seen]
        markers.append(spheres)

        for index, (x, y) in enumerate(seen):
            label = self._base_marker("target_labels", index, Marker.TEXT_VIEW_FACING)
            label.pose.position = _point(x, y, 0.6)
            label.scale.z = _TEXT_HEIGHT * 0.6
            label.color = _color(_TARGET_COLOR)
            label.text = f"G{index + 1} ({x:.1f}, {y:.1f})"
            markers.append(label)
        return markers

    def _sensor_markers(self) -> List[Marker]:
        """Build sensor position arrows and text labels."""
        markers: List[Marker] = []
        for index, (name, transform) in enumerate(sorted(self._transforms.items())):
            color = self._sensor_color(name)

            arrow = self._base_marker("sensors", index, Marker.ARROW)
            arrow.pose.position = _point(transform.x, transform.y, 0.0)
            qx, qy, qz, qw = yaw_to_quaternion(transform.yaw)
            arrow.pose.orientation.x = qx
            arrow.pose.orientation.y = qy
            arrow.pose.orientation.z = qz
            arrow.pose.orientation.w = qw
            arrow.scale.x = _SENSOR_ARROW_LENGTH
            arrow.scale.y = 0.18
            arrow.scale.z = 0.18
            arrow.color = _color(color)
            markers.append(arrow)

            rms = transform.calibration.get("rms_error")
            label = self._base_marker("sensor_labels", index, Marker.TEXT_VIEW_FACING)
            label.pose.position = _point(transform.x, transform.y, 0.9)
            label.scale.z = _TEXT_HEIGHT
            label.color = _color(color)
            label.text = (
                f"{name}\n({transform.x:.2f}, {transform.y:.2f}, "
                f"{transform.yaw_deg:.2f} deg)"
                + (f"\nRMS {rms:.3f} m" if isinstance(rms, (int, float)) else "")
            )
            markers.append(label)
        return markers

    def _measurement_markers(self) -> List[Marker]:
        """Build re-projected measurement markers and residual vectors."""
        markers: List[Marker] = []
        for index, (name, dataset) in enumerate(sorted(self._datasets.items())):
            transform = self._transforms.get(name)
            if transform is None:
                continue
            color = self._sensor_color(name)
            projected = sensor_to_grid(
                dataset.sensor_points, transform.x, transform.y, transform.yaw
            )

            spheres = self._base_marker("measurements", index, Marker.SPHERE_LIST)
            spheres.scale.x = spheres.scale.y = spheres.scale.z = _MEASUREMENT_DIAMETER
            spheres.color = _color(color)
            spheres.points = [_point(row[0], row[1], 0.05) for row in projected]
            markers.append(spheres)

            residuals = self._base_marker("residuals", index, Marker.LINE_LIST)
            residuals.scale.x = _RESIDUAL_WIDTH
            residuals.color = _color(_RESIDUAL_COLOR)
            for measured, truth in zip(projected, dataset.grid_points):
                # residual_scale > 1 exaggerates millimetre residuals so they
                # stay visible next to a 24 x 15 m grid.
                end = measured + (truth - measured) * self.residual_scale
                residuals.points += [
                    _point(measured[0], measured[1], 0.05),
                    _point(end[0], end[1], 0.05),
                ]
            if residuals.points:
                markers.append(residuals)

            errors = np.linalg.norm(projected - dataset.grid_points, axis=1)
            self.get_logger().debug(
                f"[{name}] max residual {float(np.max(errors)):.4f} m "
                f"over {errors.size} point(s)"
            )
        return markers

    def _publish_markers(self) -> None:
        """Assemble and publish the whole marker array."""
        array = MarkerArray()
        array.markers.append(self._grid_marker())
        array.markers.extend(self._target_markers())
        array.markers.extend(self._sensor_markers())
        array.markers.extend(self._measurement_markers())
        self.publisher.publish(array)


def main(args: Optional[List[str]] = None) -> None:
    """Entry point for ``ros2 run grid_sensor_calibration calibration_viz_node``.

    Args:
        args: Optional command line arguments forwarded to ``rclpy``.
    """
    rclpy.init(args=args)
    node: Optional[CalibrationVizNode] = None
    try:
        node = CalibrationVizNode()
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
