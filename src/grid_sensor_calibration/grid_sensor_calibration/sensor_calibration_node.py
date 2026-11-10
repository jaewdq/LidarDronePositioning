"""ROS 2 node estimating one sensor's ``grid -> sensorN`` transform.

The node contains no calibration mathematics: it only handles parameters,
data acquisition, logging and persistence, and delegates every computation to
:mod:`grid_sensor_calibration.calibration_math`. One node instance calibrates
exactly one sensor, so any number of sensors can be calibrated by launching
more instances -- nothing about "two sensors" is hard coded.

Two data sources are supported:

* **Method A (offline YAML)** -- ``input_file`` holds the known grid targets and
  the recorded sensor measurements; calibration runs once at start-up.
* **Method B (live capture)** -- the node subscribes to the sensor detection
  topic through a :mod:`~grid_sensor_calibration.sensor_adapter` and exposes
  ``std_srvs/Trigger`` services to capture one correspondence at a time and to
  run the calibration when enough points have been collected.
"""

from __future__ import annotations

import datetime
import math
import os
from collections import deque
from typing import Any, Deque, List, Optional, Tuple

import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_srvs.srv import Trigger

from .calibration_io import (
    CalibrationInput,
    CalibrationIOError,
    NominalPose,
    load_calibration_input,
    transform_from_estimate,
    update_calibration_file,
)
from .calibration_math import (
    CalibrationError,
    DEFAULT_MIN_BASELINE,
    DEFAULT_MIN_POINTS,
    RECOMMENDED_MIN_POINTS,
    SensorPoseEstimate,
    estimate_sensor_pose,
    installation_error,
    yaw_to_quaternion,
)
from .report import format_calibration_report, format_residual_table
from .sensor_adapter import Measurement, create_adapter

__all__ = ["SensorCalibrationNode", "main"]

#: QoS used for the latched calibration result topic.
_LATCHED_QOS = QoSProfile(
    depth=1,
    history=HistoryPolicy.KEEP_LAST,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)
_SAMPLE_BUFFER_SIZE = 200
#: Delay before the start-up calibration, so publishers/services are up first.
_AUTO_CALIBRATION_DELAY_SEC = 0.1


class SensorCalibrationNode(Node):
    """Estimates and stores the pose of a single sensor in the grid frame."""

    def __init__(self, **kwargs) -> None:
        """Declare parameters, wire up interfaces and optionally auto-calibrate.

        Args:
            **kwargs: Forwarded to ``rclpy.node.Node`` (e.g.
                ``parameter_overrides`` in unit tests).
        """
        super().__init__("sensor_calibration_node", **kwargs)

        self._declare_parameters()
        self.sensor_name: str = self.get_parameter("sensor_name").value
        self.parent_frame: str = self.get_parameter("parent_frame").value
        self.child_frame: str = (
            self.get_parameter("child_frame").value or self.sensor_name
        )
        self.minimum_number_of_points: int = max(
            int(self.get_parameter("minimum_number_of_points").value), DEFAULT_MIN_POINTS
        )
        self.minimum_baseline: float = float(self.get_parameter("minimum_baseline").value)
        self.method: str = self.get_parameter("method").value
        self.calibration_file: str = self.get_parameter("calibration_file").value
        self.apply_tilt_correction: bool = bool(
            self.get_parameter("apply_tilt_correction").value
        )
        self.sensor_z: float = float(self.get_parameter("sensor_z").value)
        self.sensor_roll: float = float(self.get_parameter("sensor_roll").value)
        self.sensor_pitch: float = float(self.get_parameter("sensor_pitch").value)

        self.nominal_pose = NominalPose(
            x=float(self.get_parameter("nominal_x").value),
            y=float(self.get_parameter("nominal_y").value),
            yaw=float(self.get_parameter("nominal_yaw").value),
        )

        self._grid_points: np.ndarray = np.empty((0, 2))
        self._sensor_points: np.ndarray = np.empty((0, 2))
        self._captured: List[np.ndarray] = []
        self._samples: Deque[Measurement] = deque(maxlen=_SAMPLE_BUFFER_SIZE)
        self._last_estimate: Optional[SensorPoseEstimate] = None

        store = self.get_parameter("capture_store").value
        if not store:
            store = os.path.join(
                os.path.dirname(os.path.expanduser(self.calibration_file)) or ".",
                f"{self.sensor_name}_captures.yaml",
            )
        self._capture_store: str = os.path.expanduser(store)

        self._load_dataset()
        self._load_captures()
        self._setup_interfaces()

        self.get_logger().info(
            f"sensor_calibration_node ready for '{self.sensor_name}' "
            f"({self.parent_frame} -> {self.child_frame}), "
            f"{self._grid_points.shape[0]} known grid target(s)"
        )

        if self.get_parameter("auto_calibrate").value and self._sensor_points.size:
            # Run shortly after start-up, once publishers and services are up.
            self._auto_timer = self.create_timer(
                _AUTO_CALIBRATION_DELAY_SEC, self._run_auto_calibration
            )

    # ------------------------------------------------------------------
    # Setup
    # ------------------------------------------------------------------

    def _declare_parameters(self) -> None:
        """Declare every ROS parameter this node understands."""
        self.declare_parameter("sensor_name", "sensor1")
        self.declare_parameter("parent_frame", "grid")
        self.declare_parameter("child_frame", "")

        self.declare_parameter("nominal_x", 0.0)
        self.declare_parameter("nominal_y", 0.0)
        self.declare_parameter("nominal_yaw", 0.0)

        self.declare_parameter("minimum_number_of_points", DEFAULT_MIN_POINTS)
        self.declare_parameter("minimum_baseline", DEFAULT_MIN_BASELINE)
        self.declare_parameter("method", "closed_form")
        self.declare_parameter("apply_tilt_correction", False)
        self.declare_parameter("sensor_z", 0.0)
        self.declare_parameter("sensor_roll", 0.0)
        self.declare_parameter("sensor_pitch", 0.0)

        self.declare_parameter("input_file", "")
        self.declare_parameter("calibration_file", "calibration_result.yaml")
        self.declare_parameter("auto_calibrate", True)
        self.declare_parameter("save_result", True)
        self.declare_parameter("shutdown_after_calibration", False)
        self.declare_parameter("print_residual_table", True)

        # Inline dataset (flattened [x1, y1, x2, y2, ...]); overrides input_file.
        self.declare_parameter("grid_points", Parameter.Type.DOUBLE_ARRAY)
        self.declare_parameter("sensor_points", Parameter.Type.DOUBLE_ARRAY)

        # Live capture (method B).
        self.declare_parameter("input_topic", "")
        self.declare_parameter("input_message_type", "point_stamped")
        self.declare_parameter("capture_sample_count", 10)

        # Where captured correspondences are mirrored to disk. Captures are
        # expensive -- each one is a walk to a surveyed point on a 24 x 15 m
        # field -- and until this existed they lived only in node memory, so a
        # stray '~/reset' or a node restart threw the afternoon away. Written
        # after every mutation and reloaded at start-up. Empty disables.
        self.declare_parameter("capture_store", "")

        # Slot the next '~/capture_point' writes to. -1 appends, which is the
        # normal flow. Set it to redo ONE bad target without disturbing the
        # others; it returns to -1 automatically after a successful capture.
        #
        #   ros2 param set /sensor1_calibration capture_index 2
        #   ros2 service call /sensor1_calibration/capture_point ...
        self.declare_parameter("capture_index", -1)

    def _setup_interfaces(self) -> None:
        """Create the subscription, services and result publisher."""
        self.result_publisher = self.create_publisher(
            TransformStamped, "~/calibrated_transform", _LATCHED_QOS
        )

        adapter_name = self.get_parameter("input_message_type").value
        try:
            self._adapter = create_adapter(adapter_name, default_frame_id=self.child_frame)
        except ValueError as exc:
            self.get_logger().error(f"{exc}; falling back to 'point_stamped'")
            self._adapter = create_adapter("point_stamped", default_frame_id=self.child_frame)

        topic = self.get_parameter("input_topic").value or f"/{self.sensor_name}/target_point"
        self.create_subscription(self._adapter.message_type, topic, self._on_measurement, 10)
        self.get_logger().info(f"listening for calibration measurements on '{topic}'")

        self.create_service(Trigger, "~/capture_point", self._srv_capture_point)
        self.create_service(Trigger, "~/run_calibration", self._srv_run_calibration)
        self.create_service(Trigger, "~/reset", self._srv_reset)
        self.create_service(Trigger, "~/undo_point", self._srv_undo_point)
        self.create_service(Trigger, "~/restore", self._srv_restore)
        self.create_service(Trigger, "~/status", self._srv_status)

    # ------------------------------------------------------------------
    # Dataset handling
    # ------------------------------------------------------------------

    def _flat_parameter_points(self, name: str) -> Optional[np.ndarray]:
        """Read a flattened ``[x1, y1, ...]`` parameter into an ``(N, 2)`` array.

        Args:
            name: Parameter name (``grid_points`` or ``sensor_points``).

        Returns:
            The ``(N, 2)`` array, or ``None`` when the parameter is unset,
            empty or has an odd number of values.
        """
        try:
            raw = self.get_parameter(name).value
        except rclpy.exceptions.ParameterUninitializedException:
            return None
        if not raw:
            return None
        values = [float(value) for value in raw]
        if len(values) % 2 != 0:
            self.get_logger().error(
                f"parameter '{name}' must hold an even number of values "
                f"([x1, y1, x2, y2, ...]), got {len(values)}"
            )
            return None
        return np.asarray(values, dtype=float).reshape(-1, 2)

    def _load_dataset(self) -> None:
        """Populate the grid/sensor point arrays from parameters or YAML."""
        grid_from_param = self._flat_parameter_points("grid_points")
        sensor_from_param = self._flat_parameter_points("sensor_points")

        input_file = self.get_parameter("input_file").value
        dataset: Optional[CalibrationInput] = None
        if input_file:
            try:
                dataset = load_calibration_input(input_file, self.sensor_name)
            except CalibrationIOError as exc:
                self.get_logger().error(f"could not load input_file: {exc}")

        if dataset is not None:
            self._grid_points = dataset.grid_points
            self._sensor_points = dataset.sensor_points
            if not self.get_parameter("nominal_x").value and not self.get_parameter(
                "nominal_y"
            ).value:
                # Parameters left at their defaults: take the nominal pose from
                # the dataset instead.
                self.nominal_pose = dataset.nominal_pose
            self.get_logger().info(
                f"loaded {dataset.number_of_points} correspondence(s) from "
                f"'{dataset.source_path}'"
            )

        if grid_from_param is not None:
            self._grid_points = grid_from_param
        if sensor_from_param is not None:
            self._sensor_points = sensor_from_param

        if (
            self._grid_points.size
            and self._sensor_points.size
            and self._grid_points.shape != self._sensor_points.shape
        ):
            self.get_logger().error(
                f"grid_points {self._grid_points.shape} and sensor_points "
                f"{self._sensor_points.shape} disagree; live capture will be used instead"
            )
            self._sensor_points = np.empty((0, 2))

    # ------------------------------------------------------------------
    # Live capture (method B)
    # ------------------------------------------------------------------

    def _on_measurement(self, msg: Any) -> None:
        """Buffer an incoming detection for the next ``capture_point`` call.

        Args:
            msg: Driver message handled by the configured adapter.
        """
        measurement = self._adapter.extract(msg)
        if measurement is None:
            self.get_logger().warn("discarded a measurement with non-finite coordinates")
            return
        self._samples.append(measurement)

    def _srv_capture_point(
        self, request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        """Capture the averaged buffered measurement as the next correspondence.

        Args:
            request: Unused ``Trigger`` request.
            response: Response filled in place.

        Returns:
            The populated response; ``success`` is ``False`` when no samples
            arrived or when all known grid targets are already captured.
        """
        del request
        requested = int(self.get_parameter("capture_index").value)
        index = len(self._captured) if requested < 0 else requested

        if index >= self._grid_points.shape[0]:
            response.success = False
            response.message = (
                f"index {index} is outside the configured grid_points list, which "
                f"holds {self._grid_points.shape[0]} target(s)"
            )
            return response
        if index > len(self._captured):
            # Allowing this would leave an empty slot, and the correspondence
            # between capture order and grid target is positional.
            response.success = False
            response.message = (
                f"cannot write slot {index}: only {len(self._captured)} point(s) "
                f"captured, so that would leave a hole. Capture in order, or use "
                f"an index of at most {len(self._captured)}"
            )
            return response

        count = max(int(self.get_parameter("capture_sample_count").value), 1)
        samples = list(self._samples)[-count:]
        if not samples:
            response.success = False
            response.message = "no measurement received yet on the input topic"
            return response

        stacked = np.vstack([
            np.array([s.xy[0], s.xy[1], s.z], dtype=float) for s in samples
        ])
        averaged = np.mean(stacked, axis=0)
        spread = float(np.max(np.std(stacked, axis=0))) if len(samples) > 1 else 0.0

        replaced = index < len(self._captured)
        previous = self._captured[index].copy() if replaced else None
        if replaced:
            self._captured[index] = averaged
        else:
            self._captured.append(averaged)
        self._samples.clear()

        if requested >= 0:
            # One-shot: the next capture appends again, so a forgotten index
            # cannot silently overwrite a good point.
            self.set_parameters([Parameter("capture_index", Parameter.Type.INTEGER, -1)])

        self._save_captures()

        target = self._grid_points[index]
        verb = "re-captured" if replaced else "captured"
        response.success = True
        response.message = (
            f"{verb} point {index + 1}/{self._grid_points.shape[0]}: "
            f"sensor ({averaged[0]:.3f}, {averaged[1]:.3f}, {averaged[2]:.3f}) "
            f"[std {spread:.4f} m over {len(samples)} sample(s)] <-> "
            f"grid ({target[0]:.3f}, {target[1]:.3f})"
        )
        if previous is not None:
            moved = float(np.linalg.norm(averaged[:2] - previous[:2]))
            response.message += (
                f"; replaced ({previous[0]:.3f}, {previous[1]:.3f}), moved {moved:.3f} m"
            )
        self.get_logger().info(response.message)
        return response

    def _srv_reset(
        self, request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        """Drop all captured correspondences and buffered samples.

        Args:
            request: Unused ``Trigger`` request.
            response: Response filled in place.

        Returns:
            The populated response.
        """
        del request
        cleared = len(self._captured)
        backup = self._write_backup() if cleared else None
        self._captured.clear()
        self._samples.clear()
        self._save_captures()
        response.success = True
        response.message = f"cleared {cleared} captured point(s)"
        if backup:
            response.message += (
                f"; backed up to '{backup}' -- call '~/restore' to undo this"
            )
        self.get_logger().info(response.message)
        return response

    def _srv_undo_point(
        self, request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        """Drop the most recently captured correspondence.

        Args:
            request: Unused ``Trigger`` request.
            response: Response filled in place.

        Returns:
            The populated response; ``success`` is ``False`` when nothing has
            been captured yet.
        """
        del request
        if not self._captured:
            response.success = False
            response.message = "nothing to undo: no point captured yet"
            return response

        dropped = self._captured.pop()
        self._save_captures()
        index = len(self._captured)
        response.success = True
        response.message = (
            f"undid point {index + 1} "
            f"(sensor {dropped[0]:.3f}, {dropped[1]:.3f}); "
            f"{index} point(s) remain, next capture fills slot {index + 1} "
            f"= grid ({self._grid_points[index][0]:.3f}, "
            f"{self._grid_points[index][1]:.3f})"
        )
        self.get_logger().info(response.message)
        return response

    def _srv_restore(
        self, request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        """Reload the correspondences that the last ``~/reset`` cleared.

        Args:
            request: Unused ``Trigger`` request.
            response: Response filled in place.

        Returns:
            The populated response.
        """
        del request
        backup = self._backup_path()
        points = self._read_capture_file(backup)
        if points is None:
            response.success = False
            response.message = f"no usable backup at '{backup}'"
            self.get_logger().error(response.message)
            return response

        self._captured = points
        self._save_captures()
        response.success = True
        response.message = f"restored {len(points)} captured point(s) from '{backup}'"
        self.get_logger().info(response.message)
        return response

    def _srv_status(
        self, request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        """Report what has been captured, with a preview fit and residuals.

        Lets a bad target be spotted and re-captured BEFORE the result is
        written, instead of being discovered in the saved calibration.

        Args:
            request: Unused ``Trigger`` request.
            response: Response filled in place.

        Returns:
            The populated response; ``message`` holds the table.
        """
        del request
        total = self._grid_points.shape[0]
        lines = [
            "",
            f"{self.sensor_name}: {len(self._captured)}/{total} target(s) captured",
            f"  store: {self._capture_store}",
        ]
        if self.apply_tilt_correction:
            lines.append(
                f"  tilt correction: enabled "
                f"z={self.sensor_z:.3f} m, "
                f"roll={self.sensor_roll:.6f} rad ({math.degrees(self.sensor_roll):+.3f} deg), "
                f"pitch={self.sensor_pitch:.6f} rad ({math.degrees(self.sensor_pitch):+.3f} deg)"
            )

        estimate = None
        if len(self._captured) >= DEFAULT_MIN_POINTS:
            try:
                sensor_points, grid_points = self._current_correspondences()
                estimate = estimate_sensor_pose(
                    sensor_points,
                    grid_points,
                    minimum_number_of_points=DEFAULT_MIN_POINTS,
                    minimum_baseline=self.minimum_baseline,
                    method=self.method,
                )
            except (CalibrationError, CalibrationIOError) as exc:
                lines.append(f"  preview fit unavailable: {exc}")

        # Least squares spreads one bad target over every residual -- a 0.6 m
        # error on one of four points lands as 0.42 m on it and 0.13-0.19 m on
        # the innocent three, so "largest residual" barely separates them.
        # Refitting without each point in turn does: only dropping the real
        # outlier collapses the RMS.
        loo = self._leave_one_out_rms() if estimate is not None else []

        header = "  idx  grid (x, y)        sensor (x, y, z)              residual"
        if loo:
            header += "   rms if dropped"
        lines += ["", header, "  " + "-" * (78 if loo else 62)]
        for i in range(total):
            grid = self._grid_points[i]
            if i >= len(self._captured):
                lines.append(
                    f"  {i:3d}  ({grid[0]:6.2f},{grid[1]:6.2f})   "
                    f"{'-- not captured --':30s}"
                )
                continue
            point = self._captured[i]
            residual = (
                f"{estimate.errors[i]:8.3f} m"
                if estimate is not None and i < estimate.errors.size
                else "       --"
            )
            row = (
                f"  {i:3d}  ({grid[0]:6.2f},{grid[1]:6.2f})   "
                f"({point[0]:7.3f},{point[1]:7.3f},{point[2]:7.3f})       {residual}"
            )
            if loo and i < len(loo):
                row += "  " + (f"{loo[i]:9.3f} m" if loo[i] is not None else "        --")
            lines.append(row)

        if estimate is not None:
            lines += [
                "",
                f"  preview: x={estimate.tx:.3f} m  y={estimate.ty:.3f} m  "
                f"yaw={estimate.yaw:.6f} rad "
                f"({estimate.yaw_deg:.3f} deg)  rms={estimate.rms_error:.4f} m",
            ]
            lines += ["", self._format_tracker_yaml_snippet(estimate)]
            candidates = [(v, i) for i, v in enumerate(loo) if v is not None]
            if candidates:
                best, worst_point = min(candidates)
                # Half the RMS is a large improvement for removing one of four
                # points; noise alone does not do that.
                if best < 0.5 * estimate.rms_error:
                    lines += [
                        "",
                        f"  dropping point {worst_point} takes the RMS from "
                        f"{estimate.rms_error:.3f} m to {best:.3f} m -- that target is",
                        "  the problem, not the fit. Re-capture just it:",
                        "",
                        f"    ros2 param set /{self.get_name()} capture_index {worst_point}",
                        f"    ros2 service call /{self.get_name()}/capture_point "
                        "std_srvs/srv/Trigger",
                    ]
        lines.append("")

        response.success = True
        response.message = "\n".join(lines)
        self.get_logger().info(response.message)
        return response

    def _leave_one_out_rms(self) -> List[Optional[float]]:
        """RMS of the fit with each captured point removed in turn.

        Needs at least four captures to say anything: dropping one of three
        leaves two, which any rigid transform matches exactly, so every entry
        would read 0.000 and point at nothing.

        Returns:
            One entry per capture -- the RMS without it, or ``None`` where the
            reduced set cannot be fitted.
        """
        count = len(self._captured)
        if count < RECOMMENDED_MIN_POINTS:
            return []

        sensor_points = self._captured_sensor_xy()
        grid_points = self._grid_points[:count]

        results: List[Optional[float]] = []
        for index in range(count):
            keep = [i for i in range(count) if i != index]
            try:
                estimate = estimate_sensor_pose(
                    sensor_points[keep],
                    grid_points[keep],
                    minimum_number_of_points=DEFAULT_MIN_POINTS,
                    minimum_baseline=self.minimum_baseline,
                    method=self.method,
                )
                results.append(float(estimate.rms_error))
            except CalibrationError:
                results.append(None)
        return results

    # ------------------------------------------------------------------
    # Capture persistence
    # ------------------------------------------------------------------

    def _backup_path(self) -> str:
        """Path of the pre-reset backup file."""
        return self._capture_store + ".bak"

    def _capture_document(self) -> dict:
        """Build the YAML document mirroring the current captures.

        The grid targets are stored alongside the measurements: the pairing is
        positional, so a store reloaded against a different target list would
        silently mis-associate every point.
        """
        return {
            "sensor_name": self.sensor_name,
            "timestamp": datetime.datetime.now(datetime.timezone.utc).isoformat(),
            "grid_points": [[float(v) for v in row] for row in self._grid_points],
            "captured": [[float(v) for v in point] for point in self._captured],
        }

    def _save_captures(self) -> None:
        """Mirror the captures to disk; never fatal to the run in progress."""
        if not self._capture_store:
            return
        try:
            directory = os.path.dirname(self._capture_store)
            if directory:
                os.makedirs(directory, exist_ok=True)
            with open(self._capture_store, "w", encoding="utf-8") as handle:
                yaml.safe_dump(self._capture_document(), handle, sort_keys=False)
        except OSError as exc:
            self.get_logger().error(
                f"could not write capture store '{self._capture_store}': {exc}"
            )

    def _write_backup(self) -> Optional[str]:
        """Copy the current captures aside so ``~/reset`` can be undone.

        Returns:
            The backup path, or ``None`` if it could not be written.
        """
        path = self._backup_path()
        try:
            directory = os.path.dirname(path)
            if directory:
                os.makedirs(directory, exist_ok=True)
            with open(path, "w", encoding="utf-8") as handle:
                yaml.safe_dump(self._capture_document(), handle, sort_keys=False)
            return path
        except OSError as exc:
            self.get_logger().error(f"could not write backup '{path}': {exc}")
            return None

    def _read_capture_file(self, path: str) -> Optional[List[np.ndarray]]:
        """Read a capture store, checking it belongs to this configuration.

        Args:
            path: File to read.

        Returns:
            The captured points, or ``None`` when the file is missing,
            unreadable, or was recorded against different grid targets.
        """
        if not path or not os.path.isfile(path):
            return None
        try:
            with open(path, "r", encoding="utf-8") as handle:
                document = yaml.safe_load(handle) or {}
        except (OSError, yaml.YAMLError) as exc:
            self.get_logger().error(f"could not read '{path}': {exc}")
            return None

        raw = document.get("captured") or []
        if not raw:
            return None

        stored_grid = document.get("grid_points") or []
        if stored_grid:
            stored = np.asarray(stored_grid, dtype=float)
            configured = self._grid_points[: stored.shape[0]]
            if stored.shape != configured.shape or not np.allclose(
                stored, configured, atol=1e-6
            ):
                self.get_logger().error(
                    f"'{path}' was recorded against different grid targets; "
                    "ignoring it rather than pairing measurements with the wrong "
                    "positions. Delete the file to start fresh."
                )
                return None

        points: List[np.ndarray] = []
        for entry in raw:
            values = [float(v) for v in entry]
            if len(values) == 2:      # written before z was recorded
                values.append(0.0)
            if len(values) != 3:
                self.get_logger().error(f"'{path}' holds a malformed entry {entry!r}")
                return None
            points.append(np.asarray(values, dtype=float))
        return points

    def _load_captures(self) -> None:
        """Restore captures from the store at start-up, if any survive."""
        points = self._read_capture_file(self._capture_store)
        if not points:
            return
        self._captured = points
        self.get_logger().info(
            f"restored {len(points)} captured point(s) from "
            f"'{self._capture_store}'; call '~/reset' to start over"
        )

    def _srv_run_calibration(
        self, request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        """Run the calibration on the currently available correspondences.

        Args:
            request: Unused ``Trigger`` request.
            response: Response filled in place.

        Returns:
            The populated response; ``message`` holds the one-line summary or
            the error reason.
        """
        del request
        try:
            estimate = self.calibrate()
        except (CalibrationError, CalibrationIOError) as exc:
            response.success = False
            response.message = str(exc)
            self.get_logger().error(f"calibration failed: {exc}")
            return response
        response.success = True
        response.message = (
            f"{self.sensor_name}: x={estimate.tx:.3f} m, y={estimate.ty:.3f} m, "
            f"yaw={estimate.yaw:.6f} rad ({estimate.yaw_deg:.3f} deg), "
            f"rms={estimate.rms_error:.4f} m"
        )
        if self.apply_tilt_correction:
            response.message += (
                f" [tilt z={self.sensor_z:.3f} m, "
                f"roll={self.sensor_roll:.6f}, pitch={self.sensor_pitch:.6f}]"
            )
        response.message += "\n\n" + self._format_tracker_yaml_snippet(estimate)
        return response

    # ------------------------------------------------------------------
    # Calibration
    # ------------------------------------------------------------------

    def _current_correspondences(self) -> Tuple[np.ndarray, np.ndarray]:
        """Return the correspondence set to calibrate with.

        Live captured points take precedence over the configured dataset.

        Returns:
            Tuple ``(sensor_points, grid_points)`` as ``(N, 2)`` arrays.

        Raises:
            CalibrationError: If no usable correspondence set exists.
        """
        if self._captured:
            count = len(self._captured)
            sensor_points = self._captured_sensor_xy()
            grid_points = self._grid_points[:count]
            return sensor_points, grid_points
        if self._sensor_points.size and self._grid_points.size:
            return self._sensor_points, self._grid_points
        raise CalibrationError(
            "no calibration data available: provide input_file / grid_points+sensor_points "
            "parameters, or capture points with the '~/capture_point' service"
        )

    def _format_tracker_yaml_snippet(self, estimate: SensorPoseEstimate) -> str:
        """Format values so they can be pasted into lidar_localization YAML."""
        prefix = self.child_frame or self.sensor_name
        yaw_pi = estimate.yaw / math.pi
        lines = [
            "  paste into lidar_localization yaml:",
            "```yaml",
            f"{prefix}_x:     {estimate.tx: .3f}",
            f"{prefix}_y:     {estimate.ty: .3f}",
        ]
        if self.apply_tilt_correction:
            lines += [
                f"{prefix}_z:     {self.sensor_z: .3f}",
                f"{prefix}_roll:  {self.sensor_roll: .6f}",
                f"{prefix}_pitch: {self.sensor_pitch: .6f}",
            ]
        lines += [
            f"{prefix}_yaw:   {estimate.yaw: .6f}  # {yaw_pi:.6f} * pi",
            "```",
        ]
        return "\n".join(lines)

    def _captured_sensor_xy(self) -> np.ndarray:
        """Return captured points as the planar coordinates used by the fit."""
        points = np.vstack(self._captured)
        if not self.apply_tilt_correction:
            return points[:, :2]
        return self._tilt_correct_points(points, self.sensor_roll, self.sensor_pitch)[:, :2]

    @staticmethod
    def _tilt_correct_points(points: np.ndarray, roll: float, pitch: float) -> np.ndarray:
        """Apply the known Rx/Ry part of the sensor->world rotation.

        The planar solver estimates only yaw and x/y. The sensor z translation
        is kept as metadata for the calibration procedure; the xy correction
        itself needs roll/pitch because measured point z leaks into xy through
        the tilt rotation. Pre-applying the known roll/pitch makes the points
        match the tracker transform convention:
        P_world = Rz(yaw) * Ry(pitch) * Rx(roll) * P_sensor + t.
        """
        pts = np.asarray(points, dtype=float).reshape(-1, 3)
        cr, sr = math.cos(float(roll)), math.sin(float(roll))
        cp, sp = math.cos(float(pitch)), math.sin(float(pitch))
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
        return (ry @ rx @ pts.T).T

    def calibrate(self) -> SensorPoseEstimate:
        """Estimate the sensor pose, log the report and persist the result.

        Returns:
            The :class:`SensorPoseEstimate` for this sensor.

        Raises:
            CalibrationError: If the data are missing, too few or degenerate.
            CalibrationIOError: If the result file cannot be written.
        """
        sensor_points, grid_points = self._current_correspondences()

        if sensor_points.shape[0] < RECOMMENDED_MIN_POINTS:
            self.get_logger().warn(
                f"calibrating with only {sensor_points.shape[0]} point(s); "
                f"{RECOMMENDED_MIN_POINTS} or more well-spread targets are recommended"
            )

        estimate = estimate_sensor_pose(
            sensor_points,
            grid_points,
            minimum_number_of_points=self.minimum_number_of_points,
            minimum_baseline=self.minimum_baseline,
            method=self.method,
        )
        self._last_estimate = estimate

        error = installation_error(
            estimate.tx,
            estimate.ty,
            estimate.yaw,
            self.nominal_pose.x,
            self.nominal_pose.y,
            self.nominal_pose.yaw,
        )

        saved_path: Optional[str] = None
        if self.get_parameter("save_result").value:
            # Store the ground-plane values next to x/y/yaw so apply_calibration
            # can write all of lidarN_* into the tracker YAML from this file.
            tilt = None
            if self.apply_tilt_correction:
                tilt = {
                    "sensor_z": float(self.sensor_z),
                    "sensor_roll": float(self.sensor_roll),
                    "sensor_pitch": float(self.sensor_pitch),
                }
            transform = transform_from_estimate(
                self.sensor_name,
                estimate,
                parent_frame=self.parent_frame,
                child_frame=self.child_frame,
                nominal=self.nominal_pose,
                error=error,
                extra=tilt,
            )
            saved_path = update_calibration_file(
                self.calibration_file, self.sensor_name, transform
            )

        report = format_calibration_report(
            self.sensor_name,
            estimate,
            self.nominal_pose,
            error,
            parent_frame=self.parent_frame,
            child_frame=self.child_frame,
            output_path=saved_path,
        )
        if self.get_parameter("print_residual_table").value:
            report += "\n" + format_residual_table(estimate)
        self.get_logger().info(report)

        self.result_publisher.publish(self._to_transform_msg(estimate))
        return estimate

    def _to_transform_msg(self, estimate: SensorPoseEstimate) -> TransformStamped:
        """Convert an estimate into a ``grid -> sensor`` ``TransformStamped``.

        Args:
            estimate: The estimated planar pose.

        Returns:
            The stamped transform (z = 0, roll = pitch = 0).
        """
        msg = TransformStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.parent_frame
        msg.child_frame_id = self.child_frame
        msg.transform.translation.x = float(estimate.tx)
        msg.transform.translation.y = float(estimate.ty)
        msg.transform.translation.z = 0.0
        qx, qy, qz, qw = yaw_to_quaternion(estimate.yaw)
        msg.transform.rotation.x = qx
        msg.transform.rotation.y = qy
        msg.transform.rotation.z = qz
        msg.transform.rotation.w = qw
        return msg

    def _run_auto_calibration(self) -> None:
        """One-shot timer callback running the start-up calibration."""
        self._auto_timer.cancel()
        try:
            self.calibrate()
        except (CalibrationError, CalibrationIOError) as exc:
            self.get_logger().error(f"automatic calibration failed: {exc}")
            return
        if self.get_parameter("shutdown_after_calibration").value:
            self.get_logger().info("shutdown_after_calibration is set; exiting")
            raise SystemExit(0)


def main(args: Optional[List[str]] = None) -> None:
    """Entry point for ``ros2 run grid_sensor_calibration sensor_calibration_node``.

    Args:
        args: Optional command line arguments forwarded to ``rclpy``.
    """
    rclpy.init(args=args)
    node: Optional[SensorCalibrationNode] = None
    try:
        node = SensorCalibrationNode()
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
