"""Adapters between sensor driver messages and the internal 2D measurement.

The real 2D LiDAR-like sensors have not been selected yet, so the calibration
and transform nodes never touch a driver message directly. They consume a
:class:`Measurement` produced by a :class:`MeasurementAdapter`. Swapping the
driver later means adding one adapter class and changing the
``input_message_type`` parameter -- no node logic changes.

Currently supported message types (standard ROS 2 messages only):

============================  ==========================================
``input_message_type``        ROS message
============================  ==========================================
``point_stamped``             ``geometry_msgs/msg/PointStamped``
``pose_stamped``              ``geometry_msgs/msg/PoseStamped``
``point``                     ``geometry_msgs/msg/Point`` (unstamped)
============================  ==========================================
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Any, Dict, Optional, Type

import numpy as np
from builtin_interfaces.msg import Time as TimeMsg
from geometry_msgs.msg import Point, PointStamped, PoseStamped

__all__ = [
    "Measurement",
    "MeasurementAdapter",
    "PointStampedAdapter",
    "PoseStampedAdapter",
    "PointAdapter",
    "ADAPTER_TYPES",
    "create_adapter",
]


@dataclass(frozen=True)
class Measurement:
    """A single planar target detection expressed in a sensor frame."""

    frame_id: str
    stamp: TimeMsg
    xy: np.ndarray  # (2,) [x, y] in the sensor frame [m]
    z: float = 0.0

    @property
    def x(self) -> float:
        """X coordinate in the sensor frame [m]."""
        return float(self.xy[0])

    @property
    def y(self) -> float:
        """Y coordinate in the sensor frame [m]."""
        return float(self.xy[1])

    def to_point_stamped(self) -> PointStamped:
        """Convert back into a ``geometry_msgs/PointStamped``.

        Returns:
            The equivalent stamped point (z preserved).
        """
        msg = PointStamped()
        msg.header.frame_id = self.frame_id
        msg.header.stamp = self.stamp
        msg.point.x = self.x
        msg.point.y = self.y
        msg.point.z = float(self.z)
        return msg


class MeasurementAdapter(ABC):
    """Converts a driver message into a frame-tagged planar measurement."""

    #: ROS message class this adapter subscribes to.
    message_type: Type[Any] = PointStamped

    def __init__(self, default_frame_id: str = "") -> None:
        """Create the adapter.

        Args:
            default_frame_id: Frame used when the message carries no header
                (or an empty ``frame_id``).
        """
        self.default_frame_id = str(default_frame_id)

    @abstractmethod
    def extract(self, msg: Any) -> Optional[Measurement]:
        """Extract a measurement from one message.

        Args:
            msg: Incoming ROS message of :attr:`message_type`.

        Returns:
            The measurement, or ``None`` when the message holds no valid
            detection (the caller simply skips it).
        """

    def _resolve_frame(self, frame_id: str) -> str:
        """Return ``frame_id`` or the configured fallback when it is empty."""
        return frame_id if frame_id else self.default_frame_id


class PointStampedAdapter(MeasurementAdapter):
    """Adapter for ``geometry_msgs/msg/PointStamped``."""

    message_type = PointStamped

    def extract(self, msg: PointStamped) -> Optional[Measurement]:
        """Read ``point.x`` / ``point.y`` and the header frame.

        Args:
            msg: Stamped point in the sensor frame.

        Returns:
            The measurement, or ``None`` if any coordinate is not finite.
        """
        xy = np.array([msg.point.x, msg.point.y], dtype=float)
        if not np.all(np.isfinite(xy)):
            return None
        return Measurement(
            frame_id=self._resolve_frame(msg.header.frame_id),
            stamp=msg.header.stamp,
            xy=xy,
            z=float(msg.point.z),
        )


class PoseStampedAdapter(MeasurementAdapter):
    """Adapter for ``geometry_msgs/msg/PoseStamped`` (position only)."""

    message_type = PoseStamped

    def extract(self, msg: PoseStamped) -> Optional[Measurement]:
        """Read ``pose.position`` and the header frame.

        Args:
            msg: Stamped pose in the sensor frame.

        Returns:
            The measurement, or ``None`` if any coordinate is not finite.
        """
        position = msg.pose.position
        xy = np.array([position.x, position.y], dtype=float)
        if not np.all(np.isfinite(xy)):
            return None
        return Measurement(
            frame_id=self._resolve_frame(msg.header.frame_id),
            stamp=msg.header.stamp,
            xy=xy,
            z=float(position.z),
        )


class PointAdapter(MeasurementAdapter):
    """Adapter for an unstamped ``geometry_msgs/msg/Point``.

    The frame is taken from ``default_frame_id`` and the stamp is left at zero,
    which makes the consumer fall back to "latest available transform".
    """

    message_type = Point

    def extract(self, msg: Point) -> Optional[Measurement]:
        """Read ``x`` / ``y`` from the unstamped point.

        Args:
            msg: Point in the sensor frame.

        Returns:
            The measurement, or ``None`` if any coordinate is not finite.
        """
        xy = np.array([msg.x, msg.y], dtype=float)
        if not np.all(np.isfinite(xy)):
            return None
        return Measurement(
            frame_id=self.default_frame_id,
            stamp=TimeMsg(),
            xy=xy,
            z=float(msg.z),
        )


#: Registry of the available adapters, keyed by the ROS parameter value.
ADAPTER_TYPES: Dict[str, Type[MeasurementAdapter]] = {
    "point_stamped": PointStampedAdapter,
    "pose_stamped": PoseStampedAdapter,
    "point": PointAdapter,
}


def create_adapter(input_message_type: str, default_frame_id: str = "") -> MeasurementAdapter:
    """Instantiate the adapter selected by a ROS parameter value.

    Args:
        input_message_type: Key from :data:`ADAPTER_TYPES`.
        default_frame_id: Frame used for unstamped/headerless messages.

    Returns:
        The adapter instance.

    Raises:
        ValueError: If ``input_message_type`` is unknown.
    """
    key = str(input_message_type).strip().lower()
    if key not in ADAPTER_TYPES:
        raise ValueError(
            f"unknown input_message_type '{input_message_type}', "
            f"expected one of {sorted(ADAPTER_TYPES)}"
        )
    return ADAPTER_TYPES[key](default_frame_id=default_frame_id)
