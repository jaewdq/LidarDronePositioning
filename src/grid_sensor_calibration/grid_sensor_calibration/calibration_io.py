"""YAML input/output for the grid sensor calibration package (no ROS deps).

Two file kinds are handled:

1. **Calibration input** (``config/sensor1.yaml``): the known grid target
   positions, the measured sensor points and the nominal installation pose.
2. **Calibration output** (``config/sensors_calibration.yaml``): the estimated
   ``grid -> sensorN`` transforms, reused by the static TF publisher.

The output loader accepts both the *combined* layout (recommended, several
sensors under a top-level ``sensors:`` key) and the *flat* layout (one mapping
per sensor at the top level), so hand-written files stay valid.
"""

from __future__ import annotations

import math
import os
from dataclasses import dataclass, field
from datetime import datetime, timezone
from typing import Any, Dict, List, Mapping, Optional, Sequence

import numpy as np
import yaml

from .calibration_math import (
    CalibrationError,
    InstallationError,
    InvalidShapeError,
    SensorPoseEstimate,
    normalize_angle,
)

__all__ = [
    "CalibrationIOError",
    "NominalPose",
    "CalibrationInput",
    "SensorTransform",
    "DEFAULT_PARENT_FRAME",
    "load_calibration_input",
    "load_sensor_transforms",
    "save_sensor_transforms",
    "update_calibration_file",
    "transform_from_estimate",
]

DEFAULT_PARENT_FRAME: str = "grid"
_SENSORS_KEY: str = "sensors"
_RESERVED_TOP_LEVEL_KEYS = {"grid", "metadata", "version"}


class CalibrationIOError(CalibrationError):
    """Raised when a calibration YAML file is missing or malformed."""


# --------------------------------------------------------------------------
# Data containers
# --------------------------------------------------------------------------


@dataclass(frozen=True)
class NominalPose:
    """Design (nominal) installation pose of a sensor in the grid frame."""

    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0

    @classmethod
    def from_mapping(cls, mapping: Optional[Mapping[str, Any]]) -> "NominalPose":
        """Build a nominal pose from a YAML mapping (missing keys default to 0).

        Args:
            mapping: Mapping with optional ``x``, ``y``, ``yaw`` keys; ``yaw``
                may instead be given as ``yaw_deg``.

        Returns:
            The parsed :class:`NominalPose`.
        """
        if not mapping:
            return cls()
        yaw = mapping.get("yaw")
        if yaw is None and "yaw_deg" in mapping:
            yaw = math.radians(float(mapping["yaw_deg"]))
        return cls(
            x=float(mapping.get("x", 0.0)),
            y=float(mapping.get("y", 0.0)),
            yaw=normalize_angle(float(yaw or 0.0)),
        )

    def as_dict(self) -> Dict[str, float]:
        """Plain-Python dict (YAML friendly)."""
        return {
            "x": float(self.x),
            "y": float(self.y),
            "yaw": float(self.yaw),
            "yaw_deg": float(math.degrees(self.yaw)),
        }


@dataclass(frozen=True)
class CalibrationInput:
    """One sensor's calibration dataset read from YAML."""

    sensor_name: str
    grid_points: np.ndarray          # (N, 2) known target positions [m]
    sensor_points: np.ndarray        # (N, 2) measurements in the sensor frame
    nominal_pose: NominalPose = field(default_factory=NominalPose)
    parent_frame: str = DEFAULT_PARENT_FRAME
    child_frame: Optional[str] = None
    source_path: Optional[str] = None

    def __post_init__(self) -> None:
        """Validate that both point arrays are matching ``(N, 2)`` arrays."""
        for name in ("grid_points", "sensor_points"):
            array = np.asarray(getattr(self, name), dtype=float)
            if array.ndim != 2 or array.shape[1] != 2:
                raise InvalidShapeError(
                    f"{name} in '{self.sensor_name}' must be (N, 2), got {array.shape}"
                )
            object.__setattr__(self, name, array)
        if self.grid_points.shape != self.sensor_points.shape:
            raise InvalidShapeError(
                f"'{self.sensor_name}': grid_points {self.grid_points.shape} and "
                f"sensor_points {self.sensor_points.shape} must match"
            )
        if self.child_frame is None:
            object.__setattr__(self, "child_frame", self.sensor_name)

    @property
    def number_of_points(self) -> int:
        """Number of correspondences in the dataset."""
        return int(self.grid_points.shape[0])


@dataclass(frozen=True)
class SensorTransform:
    """A calibrated ``parent -> child`` planar transform ready for TF."""

    child_frame: str
    x: float
    y: float
    yaw: float
    parent_frame: str = DEFAULT_PARENT_FRAME
    z: float = 0.0
    calibration: Dict[str, Any] = field(default_factory=dict)
    nominal: Optional[NominalPose] = None

    @property
    def yaw_deg(self) -> float:
        """Yaw in degrees."""
        return math.degrees(self.yaw)

    def as_dict(self) -> Dict[str, Any]:
        """Plain-Python dict used when serialising to YAML."""
        payload: Dict[str, Any] = {
            "parent_frame": self.parent_frame,
            "child_frame": self.child_frame,
            "x": float(self.x),
            "y": float(self.y),
            "z": float(self.z),
            "yaw": float(self.yaw),
            "yaw_deg": float(self.yaw_deg),
        }
        if self.nominal is not None:
            payload["nominal_pose"] = self.nominal.as_dict()
        if self.calibration:
            payload["calibration"] = dict(self.calibration)
        return payload


# --------------------------------------------------------------------------
# Helpers
# --------------------------------------------------------------------------


def _read_yaml(path: str) -> Dict[str, Any]:
    """Read a YAML file into a dict.

    Args:
        path: Path of the YAML file.

    Returns:
        The parsed mapping.

    Raises:
        CalibrationIOError: File missing, unreadable or not a mapping.
    """
    expanded = os.path.expanduser(str(path))
    if not os.path.isfile(expanded):
        raise CalibrationIOError(f"calibration file not found: {expanded}")
    try:
        with open(expanded, "r", encoding="utf-8") as handle:
            data = yaml.safe_load(handle)
    except yaml.YAMLError as exc:  # pragma: no cover - depends on file content
        raise CalibrationIOError(f"failed to parse YAML '{expanded}': {exc}") from exc
    if data is None:
        raise CalibrationIOError(f"calibration file is empty: {expanded}")
    if not isinstance(data, Mapping):
        raise CalibrationIOError(f"top level of '{expanded}' must be a mapping")
    return dict(data)


def _points_to_array(raw: Any, name: str, sensor: str) -> np.ndarray:
    """Convert a YAML list of ``[x, y]`` pairs into an ``(N, 2)`` array.

    Args:
        raw: YAML value (list of pairs, or list of ``{x:, y:}`` mappings).
        name: Field name, for error messages.
        sensor: Sensor name, for error messages.

    Returns:
        ``(N, 2)`` float array.

    Raises:
        CalibrationIOError: If the value cannot be interpreted as 2D points.
    """
    if raw is None:
        raise CalibrationIOError(f"'{sensor}': missing '{name}'")
    if isinstance(raw, Mapping):
        raise CalibrationIOError(f"'{sensor}': '{name}' must be a list of [x, y] pairs")
    rows: List[Sequence[float]] = []
    for index, item in enumerate(raw):
        if isinstance(item, Mapping):
            try:
                rows.append([float(item["x"]), float(item["y"])])
            except (KeyError, TypeError, ValueError) as exc:
                raise CalibrationIOError(
                    f"'{sensor}': {name}[{index}] must provide numeric x and y"
                ) from exc
            continue
        try:
            values = [float(value) for value in item]
        except (TypeError, ValueError) as exc:
            raise CalibrationIOError(
                f"'{sensor}': {name}[{index}] is not a numeric pair"
            ) from exc
        if len(values) != 2:
            raise CalibrationIOError(
                f"'{sensor}': {name}[{index}] must have exactly 2 values, got {len(values)}"
            )
        rows.append(values)
    if not rows:
        raise CalibrationIOError(f"'{sensor}': '{name}' is empty")
    return np.asarray(rows, dtype=float)


# --------------------------------------------------------------------------
# Calibration input (method A: YAML dataset)
# --------------------------------------------------------------------------


def load_calibration_input(path: str, sensor_name: Optional[str] = None) -> CalibrationInput:
    """Load one sensor's calibration dataset from YAML.

    Supported layouts::

        # single sensor file
        sensor_name: sensor1
        nominal_pose: {x: 12.0, y: -1.0, yaw: 0.0}
        grid_points: [[3.0, 3.0], ...]
        sensor_points: [[...], ...]

        # multi sensor file
        sensors:
          sensor1: {nominal_pose: {...}, grid_points: [...], sensor_points: [...]}

    Args:
        path: YAML file path.
        sensor_name: Required only for multi-sensor files with more than one
            entry; selects which sensor to load.

    Returns:
        The parsed :class:`CalibrationInput`.

    Raises:
        CalibrationIOError: Missing file/keys or ambiguous sensor selection.
    """
    data = _read_yaml(path)

    block: Mapping[str, Any]
    if _SENSORS_KEY in data and isinstance(data[_SENSORS_KEY], Mapping):
        sensors = data[_SENSORS_KEY]
        if sensor_name is None:
            if len(sensors) != 1:
                raise CalibrationIOError(
                    f"'{path}' contains {len(sensors)} sensors; specify sensor_name "
                    f"(available: {sorted(sensors)})"
                )
            sensor_name = next(iter(sensors))
        if sensor_name not in sensors:
            raise CalibrationIOError(
                f"sensor '{sensor_name}' not found in '{path}' "
                f"(available: {sorted(sensors)})"
            )
        block = sensors[sensor_name]
    else:
        block = data
        sensor_name = sensor_name or str(data.get("sensor_name", "sensor"))

    if not isinstance(block, Mapping):
        raise CalibrationIOError(f"sensor entry '{sensor_name}' in '{path}' is not a mapping")

    return CalibrationInput(
        sensor_name=str(block.get("sensor_name", sensor_name)),
        grid_points=_points_to_array(block.get("grid_points"), "grid_points", sensor_name),
        sensor_points=_points_to_array(block.get("sensor_points"), "sensor_points", sensor_name),
        nominal_pose=NominalPose.from_mapping(block.get("nominal_pose")),
        parent_frame=str(block.get("parent_frame", DEFAULT_PARENT_FRAME)),
        child_frame=block.get("child_frame"),
        source_path=os.path.expanduser(str(path)),
    )


# --------------------------------------------------------------------------
# Calibration output (transforms)
# --------------------------------------------------------------------------


def transform_from_estimate(
    sensor_name: str,
    estimate: SensorPoseEstimate,
    parent_frame: str = DEFAULT_PARENT_FRAME,
    child_frame: Optional[str] = None,
    nominal: Optional[NominalPose] = None,
    error: Optional[InstallationError] = None,
    extra: Optional[Mapping[str, Any]] = None,
) -> SensorTransform:
    """Wrap a pose estimate into a serialisable :class:`SensorTransform`.

    Args:
        sensor_name: Logical sensor name (``sensor1``...).
        estimate: The estimated pose and residual statistics.
        parent_frame: TF parent frame, normally ``grid``.
        child_frame: TF child frame; defaults to ``sensor_name``.
        nominal: Nominal installation pose, stored for reference.
        error: Installation error, stored for reference.
        extra: Additional key/values merged into the ``calibration`` block.

    Returns:
        The populated :class:`SensorTransform`.
    """
    calibration: Dict[str, Any] = {
        "rms_error": float(estimate.rms_error),
        "mean_error": float(estimate.mean_error),
        "max_error": float(estimate.max_error),
        "number_of_points": int(estimate.number_of_points),
        "method": estimate.method,
        "timestamp": datetime.now(timezone.utc).isoformat(timespec="seconds"),
    }
    if error is not None:
        calibration["installation_error"] = error.as_dict()
    if extra:
        calibration.update(dict(extra))

    return SensorTransform(
        child_frame=str(child_frame or sensor_name),
        x=float(estimate.tx),
        y=float(estimate.ty),
        yaw=float(estimate.yaw),
        parent_frame=str(parent_frame),
        calibration=calibration,
        nominal=nominal,
    )


def _transform_from_mapping(name: str, block: Mapping[str, Any]) -> SensorTransform:
    """Build a :class:`SensorTransform` from one YAML sensor block.

    Args:
        name: Sensor key in the YAML file.
        block: Mapping holding ``x``, ``y``, ``yaw`` and optional frame names.

    Returns:
        The parsed transform.

    Raises:
        CalibrationIOError: If required numeric fields are missing/invalid.
    """
    try:
        yaw = block.get("yaw")
        if yaw is None and "yaw_deg" in block:
            yaw = math.radians(float(block["yaw_deg"]))
        return SensorTransform(
            child_frame=str(block.get("child_frame", name)),
            x=float(block["x"]),
            y=float(block["y"]),
            yaw=normalize_angle(float(yaw)),
            parent_frame=str(block.get("parent_frame", DEFAULT_PARENT_FRAME)),
            z=float(block.get("z", 0.0)),
            calibration=dict(block.get("calibration", {}) or {}),
            nominal=(
                NominalPose.from_mapping(block.get("nominal_pose"))
                if block.get("nominal_pose")
                else None
            ),
        )
    except (KeyError, TypeError, ValueError) as exc:
        raise CalibrationIOError(
            f"sensor '{name}': x, y and yaw are required numeric fields ({exc})"
        ) from exc


def load_sensor_transforms(path: str) -> Dict[str, SensorTransform]:
    """Load all calibrated transforms from a results YAML file.

    Accepts the combined layout (``sensors:`` mapping) and the flat layout
    (one top-level mapping per sensor).

    Args:
        path: YAML file path.

    Returns:
        Mapping of sensor name to :class:`SensorTransform`, ordered as in the
        file.

    Raises:
        CalibrationIOError: If the file holds no usable sensor entry.
    """
    data = _read_yaml(path)
    blocks: Mapping[str, Any]
    if _SENSORS_KEY in data and isinstance(data[_SENSORS_KEY], Mapping):
        blocks = data[_SENSORS_KEY]
    else:
        blocks = {
            key: value
            for key, value in data.items()
            if isinstance(value, Mapping) and key not in _RESERVED_TOP_LEVEL_KEYS
        }
    transforms = {
        name: _transform_from_mapping(name, block)
        for name, block in blocks.items()
        if isinstance(block, Mapping) and {"x", "y"} <= set(block)
    }
    if not transforms:
        raise CalibrationIOError(f"no sensor transform found in '{path}'")
    return transforms


def save_sensor_transforms(
    path: str,
    transforms: Mapping[str, SensorTransform],
    grid_info: Optional[Mapping[str, Any]] = None,
    combined: bool = True,
) -> str:
    """Write calibrated transforms to YAML (atomically).

    Args:
        path: Destination file path (parent directories are created).
        transforms: Mapping of sensor name to transform.
        grid_info: Optional grid description stored under a ``grid`` key.
        combined: ``True`` writes the recommended ``sensors:`` layout, ``False``
            writes the flat one-mapping-per-sensor layout.

    Returns:
        The absolute path that was written.

    Raises:
        CalibrationIOError: If the file cannot be written.
    """
    payload: Dict[str, Any] = {}
    if grid_info:
        payload["grid"] = dict(grid_info)
    sensor_blocks = {name: tf.as_dict() for name, tf in transforms.items()}
    if combined:
        payload[_SENSORS_KEY] = sensor_blocks
    else:
        payload.update(sensor_blocks)

    expanded = os.path.abspath(os.path.expanduser(str(path)))
    directory = os.path.dirname(expanded)
    if directory:
        os.makedirs(directory, exist_ok=True)
    # Per-process temp name. Both sensors' calibration nodes write the same
    # combined file, and a shared "<path>.tmp" lets one node's replace() run
    # after the other has already renamed that exact temp away -- which fails
    # with ENOENT and loses the second sensor's result.
    temporary = f"{expanded}.{os.getpid()}.tmp"
    try:
        with open(temporary, "w", encoding="utf-8") as handle:
            yaml.safe_dump(payload, handle, default_flow_style=False, sort_keys=False)
        os.replace(temporary, expanded)
    except OSError as exc:
        raise CalibrationIOError(f"failed to write '{expanded}': {exc}") from exc
    return expanded


def update_calibration_file(
    path: str,
    sensor_name: str,
    transform: SensorTransform,
    grid_info: Optional[Mapping[str, Any]] = None,
) -> str:
    """Insert/replace one sensor inside a combined calibration file.

    Each sensor is calibrated independently, so the calibration node updates
    only its own entry and preserves the entries of the other sensors.

    Args:
        path: Combined calibration YAML path (created if missing).
        sensor_name: Key of the sensor to write.
        transform: The transform to store.
        grid_info: Optional grid description written when creating the file.

    Returns:
        The absolute path that was written.
    """
    existing: Dict[str, SensorTransform] = {}
    stored_grid: Optional[Dict[str, Any]] = None
    if os.path.isfile(os.path.expanduser(str(path))):
        try:
            existing = load_sensor_transforms(path)
            raw = _read_yaml(path)
            stored_grid = dict(raw.get("grid", {})) or None
        except CalibrationIOError:
            existing = {}
    existing[sensor_name] = transform
    return save_sensor_transforms(path, existing, grid_info or stored_grid, combined=True)
