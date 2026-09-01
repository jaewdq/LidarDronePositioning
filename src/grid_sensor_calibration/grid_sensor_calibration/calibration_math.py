"""Pure-Python 2D rigid-body calibration math (no ROS dependency).

Convention (single source of truth for the whole package)
--------------------------------------------------------
A point measured by a sensor, expressed in that sensor's own frame::

    Ps = [xs, ys]^T

is mapped into the ``grid`` frame by::

    Pg = R(yaw) * Ps + t,      t = [tx, ty]^T

    R(yaw) = [[cos(yaw), -sin(yaw)],
              [sin(yaw),  cos(yaw)]]

Therefore:

* ``(tx, ty)`` is the position of the **sensor origin expressed in the grid frame**.
* ``yaw`` is the rotation of the **sensor x-axis relative to the grid x-axis**
  (positive = counter-clockwise).

This is exactly the semantics of a ROS TF whose ``header.frame_id`` (parent) is
``grid`` and whose ``child_frame_id`` is ``sensorN``: tf2 transforming a
``PointStamped`` stamped in ``sensorN`` into ``grid`` computes ``R * Ps + t``.

Units: metre, radian. Angles are normalised to ``[-pi, pi)``.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Iterable, Literal, Sequence, Tuple

import numpy as np

__all__ = [
    "CalibrationError",
    "InvalidShapeError",
    "InsufficientPointsError",
    "DegenerateGeometryError",
    "ResidualReport",
    "SensorPoseEstimate",
    "InstallationError",
    "DEFAULT_MIN_POINTS",
    "DEFAULT_MIN_BASELINE",
    "RECOMMENDED_MIN_POINTS",
    "normalize_angle",
    "rotation_matrix",
    "get_transform_matrix",
    "invert_transform_matrix",
    "sensor_to_grid",
    "grid_to_sensor",
    "calculate_residuals",
    "estimate_sensor_pose",
    "estimate_sensor_pose_two_points",
    "installation_error",
    "yaw_to_quaternion",
    "quaternion_to_yaw",
    "max_pairwise_distance",
]

# --------------------------------------------------------------------------
# Constants (avoid magic numbers scattered through the code)
# --------------------------------------------------------------------------

#: Mathematical minimum number of correspondences to solve for (tx, ty, yaw).
DEFAULT_MIN_POINTS: int = 2
#: Recommended number of correspondences for a field calibration.
RECOMMENDED_MIN_POINTS: int = 4
#: Default minimum spatial spread [m] required from the calibration points.
DEFAULT_MIN_BASELINE: float = 1.0
#: Numerical guard used when checking for a degenerate point set.
_EPS: float = 1e-12
_DIM: int = 2


# --------------------------------------------------------------------------
# Exceptions
# --------------------------------------------------------------------------


class CalibrationError(Exception):
    """Base class for every calibration input/geometry error."""


class InvalidShapeError(CalibrationError):
    """Raised when the point arrays are not two matching ``(N, 2)`` arrays."""


class InsufficientPointsError(CalibrationError):
    """Raised when fewer correspondences than required are supplied."""


class DegenerateGeometryError(CalibrationError):
    """Raised when the calibration geometry cannot constrain the yaw angle."""


# --------------------------------------------------------------------------
# Result containers
# --------------------------------------------------------------------------


@dataclass(frozen=True)
class ResidualReport:
    """Per-point and aggregated residuals of a calibration fit."""

    residuals: np.ndarray = field(repr=False)  # (N, 2) Gi - Gi_est
    errors: np.ndarray = field(repr=False)     # (N,)   ||Gi - Gi_est||
    rms_error: float
    mean_error: float
    max_error: float
    number_of_points: int

    @property
    def max_error_index(self) -> int:
        """Index of the calibration point with the largest residual."""
        return int(np.argmax(self.errors)) if self.errors.size else -1


@dataclass(frozen=True)
class SensorPoseEstimate:
    """Estimated ``grid -> sensor`` pose plus the quality of the fit."""

    tx: float
    ty: float
    yaw: float
    rms_error: float
    mean_error: float
    max_error: float
    number_of_points: int
    method: str = "closed_form"
    residuals: np.ndarray = field(default_factory=lambda: np.empty((0, 2)), repr=False)
    errors: np.ndarray = field(default_factory=lambda: np.empty((0,)), repr=False)

    @property
    def yaw_deg(self) -> float:
        """Yaw in degrees."""
        return math.degrees(self.yaw)

    @property
    def translation(self) -> np.ndarray:
        """Translation ``t = [tx, ty]`` as a numpy array."""
        return np.array([self.tx, self.ty], dtype=float)

    def transform_matrix(self) -> np.ndarray:
        """Homogeneous ``3x3`` ``T_grid_sensor`` of this estimate."""
        return get_transform_matrix(self.tx, self.ty, self.yaw)

    def as_dict(self) -> dict:
        """Plain-Python dict (YAML/JSON friendly)."""
        return {
            "tx": float(self.tx),
            "ty": float(self.ty),
            "yaw": float(self.yaw),
            "yaw_deg": float(self.yaw_deg),
            "rms_error": float(self.rms_error),
            "mean_error": float(self.mean_error),
            "max_error": float(self.max_error),
            "number_of_points": int(self.number_of_points),
            "method": self.method,
        }


@dataclass(frozen=True)
class InstallationError:
    """Deviation of the estimated pose from the nominal installation pose."""

    dx: float
    dy: float
    dyaw: float

    @property
    def dyaw_deg(self) -> float:
        """Yaw deviation in degrees."""
        return math.degrees(self.dyaw)

    @property
    def distance(self) -> float:
        """Planar distance between estimated and nominal sensor origin."""
        return float(math.hypot(self.dx, self.dy))

    def as_dict(self) -> dict:
        """Plain-Python dict (YAML/JSON friendly)."""
        return {
            "dx": float(self.dx),
            "dy": float(self.dy),
            "dyaw": float(self.dyaw),
            "dyaw_deg": float(self.dyaw_deg),
            "distance": float(self.distance),
        }


# --------------------------------------------------------------------------
# Basic angle / matrix helpers
# --------------------------------------------------------------------------


def normalize_angle(angle: float) -> float:
    """Normalise ``angle`` [rad] into ``[-pi, pi)``.

    Args:
        angle: Angle in radians.

    Returns:
        The equivalent angle in ``[-pi, pi)``.
    """
    wrapped = (float(angle) + math.pi) % (2.0 * math.pi) - math.pi
    # ``math.fmod`` style wrapping can return exactly +pi for some inputs.
    if wrapped >= math.pi:
        wrapped -= 2.0 * math.pi
    return wrapped


def rotation_matrix(yaw: float) -> np.ndarray:
    """Return the ``2x2`` CCW rotation matrix ``R(yaw)``.

    Args:
        yaw: Rotation of the sensor x-axis w.r.t. the grid x-axis [rad].

    Returns:
        ``[[cos, -sin], [sin, cos]]`` as a ``(2, 2)`` float array.
    """
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s], [s, c]], dtype=float)


def get_transform_matrix(tx: float, ty: float, yaw: float) -> np.ndarray:
    """Return the homogeneous ``3x3`` transform ``T_grid_sensor``.

    ``[Xg, Yg, 1]^T = T_grid_sensor @ [xs, ys, 1]^T``

    Args:
        tx: Sensor origin x in the grid frame [m].
        ty: Sensor origin y in the grid frame [m].
        yaw: Sensor yaw in the grid frame [rad].

    Returns:
        ``(3, 3)`` homogeneous transformation matrix.
    """
    matrix = np.eye(3, dtype=float)
    matrix[:2, :2] = rotation_matrix(yaw)
    matrix[0, 2] = float(tx)
    matrix[1, 2] = float(ty)
    return matrix


def invert_transform_matrix(matrix: np.ndarray) -> np.ndarray:
    """Invert a homogeneous 2D transform analytically (``R^T``, ``-R^T t``).

    Args:
        matrix: ``(3, 3)`` homogeneous rigid transform.

    Returns:
        ``(3, 3)`` inverse transform (``T_sensor_grid`` when given
        ``T_grid_sensor``).

    Raises:
        InvalidShapeError: If ``matrix`` is not ``(3, 3)``.
    """
    matrix = np.asarray(matrix, dtype=float)
    if matrix.shape != (3, 3):
        raise InvalidShapeError(f"expected a (3, 3) matrix, got {matrix.shape}")
    rot = matrix[:2, :2]
    trans = matrix[:2, 2]
    inverse = np.eye(3, dtype=float)
    inverse[:2, :2] = rot.T
    inverse[:2, 2] = -rot.T @ trans
    return inverse


def yaw_to_quaternion(yaw: float) -> Tuple[float, float, float, float]:
    """Convert a planar yaw into a ``(x, y, z, w)`` quaternion.

    Roll and pitch are assumed to be zero, which is the case for a 2D sensor
    mounted level with the grid plane.

    Args:
        yaw: Rotation about the z-axis [rad].

    Returns:
        Quaternion in ROS order ``(x, y, z, w)``.
    """
    half = 0.5 * float(yaw)
    return (0.0, 0.0, math.sin(half), math.cos(half))


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    """Extract the yaw angle from a ``(x, y, z, w)`` quaternion.

    Args:
        x: Quaternion x component.
        y: Quaternion y component.
        z: Quaternion z component.
        w: Quaternion w component.

    Returns:
        Yaw [rad] normalised to ``[-pi, pi)``.
    """
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return normalize_angle(math.atan2(siny_cosp, cosy_cosp))


# --------------------------------------------------------------------------
# Point transforms
# --------------------------------------------------------------------------


def _as_point_array(points: Sequence | np.ndarray, name: str) -> Tuple[np.ndarray, bool]:
    """Coerce input into an ``(N, 2)`` float array.

    Args:
        points: Either a single ``(2,)`` point or an ``(N, 2)`` array-like.
        name: Argument name used in error messages.

    Returns:
        Tuple of the ``(N, 2)`` array and a flag telling whether the input was
        a single point (so the caller can squeeze the result back).

    Raises:
        InvalidShapeError: If the input is not 2D-point shaped or not finite.
    """
    array = np.asarray(points, dtype=float)
    single = array.ndim == 1
    if single:
        array = array.reshape(1, -1)
    if array.ndim != 2 or array.shape[1] != _DIM:
        raise InvalidShapeError(
            f"{name} must have shape (N, 2) or (2,), got {np.shape(points)}"
        )
    if not np.all(np.isfinite(array)):
        raise InvalidShapeError(f"{name} contains NaN or Inf values")
    return array, single


def sensor_to_grid(
    sensor_point: Sequence[float] | np.ndarray,
    tx: float,
    ty: float,
    yaw: float,
) -> np.ndarray:
    """Transform sensor-frame point(s) into the grid frame.

    Implements ``Pg = R(yaw) * Ps + t``.

    Args:
        sensor_point: ``(2,)`` point or ``(N, 2)`` points in the sensor frame.
        tx: Sensor origin x in the grid frame [m].
        ty: Sensor origin y in the grid frame [m].
        yaw: Sensor yaw in the grid frame [rad].

    Returns:
        ``(2,)`` or ``(N, 2)`` array of grid-frame points, matching the input
        shape.
    """
    points, single = _as_point_array(sensor_point, "sensor_point")
    rot = rotation_matrix(yaw)
    grid = points @ rot.T + np.array([tx, ty], dtype=float)
    return grid[0] if single else grid


def grid_to_sensor(
    grid_point: Sequence[float] | np.ndarray,
    tx: float,
    ty: float,
    yaw: float,
) -> np.ndarray:
    """Transform grid-frame point(s) into the sensor frame.

    Implements the inverse relation ``Ps = R(yaw)^T * (Pg - t)``.

    Args:
        grid_point: ``(2,)`` point or ``(N, 2)`` points in the grid frame.
        tx: Sensor origin x in the grid frame [m].
        ty: Sensor origin y in the grid frame [m].
        yaw: Sensor yaw in the grid frame [rad].

    Returns:
        ``(2,)`` or ``(N, 2)`` array of sensor-frame points, matching the input
        shape.
    """
    points, single = _as_point_array(grid_point, "grid_point")
    rot = rotation_matrix(yaw)
    sensor = (points - np.array([tx, ty], dtype=float)) @ rot
    # ``p @ R`` equals ``(R^T @ p^T)^T`` -- i.e. R^T applied to each row.
    return sensor[0] if single else sensor


# --------------------------------------------------------------------------
# Validation helpers
# --------------------------------------------------------------------------


def max_pairwise_distance(points: np.ndarray) -> float:
    """Largest distance between any two points of an ``(N, 2)`` set.

    Args:
        points: ``(N, 2)`` array of planar points.

    Returns:
        Maximum pairwise Euclidean distance [m]; ``0.0`` for fewer than two
        points.
    """
    points = np.asarray(points, dtype=float)
    if points.shape[0] < 2:
        return 0.0
    diff = points[:, None, :] - points[None, :, :]
    return float(np.sqrt(np.einsum("ijk,ijk->ij", diff, diff)).max())


def _validate_correspondences(
    sensor_points: Sequence | np.ndarray,
    grid_points: Sequence | np.ndarray,
    minimum_number_of_points: int,
    minimum_baseline: float,
) -> Tuple[np.ndarray, np.ndarray]:
    """Validate shapes, count and geometry of a correspondence set.

    Args:
        sensor_points: ``(N, 2)`` measurements in the sensor frame.
        grid_points: ``(N, 2)`` known target positions in the grid frame.
        minimum_number_of_points: Minimum accepted ``N`` (never below 2).
        minimum_baseline: Minimum accepted spatial spread [m] of the point set.

    Returns:
        The validated ``(sensor_points, grid_points)`` float arrays.

    Raises:
        InvalidShapeError: Shape mismatch, wrong dimensionality or non-finite.
        InsufficientPointsError: Not enough correspondences.
        DegenerateGeometryError: Coincident points or spread below baseline.
    """
    sensor_array, _ = _as_point_array(sensor_points, "sensor_points")
    grid_array, _ = _as_point_array(grid_points, "grid_points")

    if sensor_array.shape != grid_array.shape:
        raise InvalidShapeError(
            "sensor_points and grid_points must have the same shape, got "
            f"{sensor_array.shape} and {grid_array.shape}"
        )

    required = max(int(minimum_number_of_points), DEFAULT_MIN_POINTS)
    n_points = sensor_array.shape[0]
    if n_points < required:
        raise InsufficientPointsError(
            f"need at least {required} correspondences, got {n_points}"
        )

    sensor_spread = max_pairwise_distance(sensor_array)
    grid_spread = max_pairwise_distance(grid_array)

    if sensor_spread <= _EPS or grid_spread <= _EPS:
        raise DegenerateGeometryError(
            "all calibration points are coincident; yaw is unobservable "
            f"(sensor spread {sensor_spread:.6f} m, grid spread {grid_spread:.6f} m)"
        )

    baseline = float(minimum_baseline)
    if baseline > 0.0 and (sensor_spread < baseline or grid_spread < baseline):
        raise DegenerateGeometryError(
            f"calibration geometry too small: sensor spread {sensor_spread:.3f} m, "
            f"grid spread {grid_spread:.3f} m, required baseline {baseline:.3f} m"
        )

    return sensor_array, grid_array


# --------------------------------------------------------------------------
# Residuals
# --------------------------------------------------------------------------


def calculate_residuals(
    sensor_points: Sequence | np.ndarray,
    grid_points: Sequence | np.ndarray,
    tx: float,
    ty: float,
    yaw: float,
) -> ResidualReport:
    """Evaluate the fit quality of a ``(tx, ty, yaw)`` estimate.

    For every correspondence the sensor measurement is re-projected with
    ``Gi_est = R * Si + t`` and compared against the known grid position.

    Args:
        sensor_points: ``(N, 2)`` measurements in the sensor frame.
        grid_points: ``(N, 2)`` known target positions in the grid frame.
        tx: Sensor origin x in the grid frame [m].
        ty: Sensor origin y in the grid frame [m].
        yaw: Sensor yaw in the grid frame [rad].

    Returns:
        A :class:`ResidualReport` with per-point residual vectors, per-point
        Euclidean errors and the RMS / mean / max aggregates.

    Raises:
        InvalidShapeError: If the two arrays do not have a matching ``(N, 2)``
            shape.
    """
    sensor_array, _ = _as_point_array(sensor_points, "sensor_points")
    grid_array, _ = _as_point_array(grid_points, "grid_points")
    if sensor_array.shape != grid_array.shape:
        raise InvalidShapeError(
            "sensor_points and grid_points must have the same shape, got "
            f"{sensor_array.shape} and {grid_array.shape}"
        )

    estimated = sensor_to_grid(sensor_array, tx, ty, yaw)
    residuals = grid_array - estimated
    errors = np.linalg.norm(residuals, axis=1)
    return ResidualReport(
        residuals=residuals,
        errors=errors,
        rms_error=float(np.sqrt(np.mean(errors**2))),
        mean_error=float(np.mean(errors)),
        max_error=float(np.max(errors)),
        number_of_points=int(errors.size),
    )


# --------------------------------------------------------------------------
# Pose estimation
# --------------------------------------------------------------------------


def _yaw_closed_form(centered_sensor: np.ndarray, centered_grid: np.ndarray) -> float:
    """Closed-form 2D least-squares yaw (see module docs / README section 7).

    ``C = sum(ui . vi)``, ``D = sum(ui x vi)``, ``yaw = atan2(D, C)``.

    Args:
        centered_sensor: ``(N, 2)`` sensor points minus their centroid.
        centered_grid: ``(N, 2)`` grid points minus their centroid.

    Returns:
        Optimal yaw [rad], not yet normalised.
    """
    c_term = float(np.sum(centered_sensor * centered_grid))
    d_term = float(
        np.sum(
            centered_sensor[:, 0] * centered_grid[:, 1]
            - centered_sensor[:, 1] * centered_grid[:, 0]
        )
    )
    if abs(c_term) <= _EPS and abs(d_term) <= _EPS:
        raise DegenerateGeometryError(
            "cross-covariance vanished; yaw cannot be determined from this "
            "point configuration"
        )
    return math.atan2(d_term, c_term)


def _yaw_svd(centered_sensor: np.ndarray, centered_grid: np.ndarray) -> float:
    """General Kabsch/Procrustes yaw via SVD of the cross-covariance matrix.

    ``H = sum(ui vi^T)``, ``H = U S V^T``, ``R = V * diag(1, det(V U^T)) * U^T``.
    The ``det`` correction removes the reflection solution, which the 2D
    closed-form variant excludes implicitly. Both estimators minimise the same
    cost ``sum ||Gi - (R Si + t)||^2`` and agree to numerical precision; the
    closed form is the package default because it is branch-free and cheaper,
    while this SVD path is kept as a cross-check and as the natural entry point
    for a future 3D extension.

    Args:
        centered_sensor: ``(N, 2)`` sensor points minus their centroid.
        centered_grid: ``(N, 2)`` grid points minus their centroid.

    Returns:
        Optimal yaw [rad], not yet normalised.
    """
    cross_covariance = centered_sensor.T @ centered_grid  # (2, 2) = sum ui vi^T
    u_mat, singular_values, vt_mat = np.linalg.svd(cross_covariance)
    if float(np.max(singular_values)) <= _EPS:
        raise DegenerateGeometryError(
            "cross-covariance vanished; yaw cannot be determined from this "
            "point configuration"
        )
    v_mat = vt_mat.T
    correction = np.diag([1.0, float(np.sign(np.linalg.det(v_mat @ u_mat.T)) or 1.0)])
    rot = v_mat @ correction @ u_mat.T
    return math.atan2(rot[1, 0], rot[0, 0])


def estimate_sensor_pose(
    sensor_points: Sequence | np.ndarray,
    grid_points: Sequence | np.ndarray,
    minimum_number_of_points: int = DEFAULT_MIN_POINTS,
    minimum_baseline: float = DEFAULT_MIN_BASELINE,
    method: Literal["closed_form", "svd"] = "closed_form",
) -> SensorPoseEstimate:
    """Estimate the ``grid -> sensor`` pose from known correspondences.

    Solves the 2D rigid-body (Procrustes/Kabsch) problem

    ``min_{R, t} sum_i || Gi - (R Si + t) ||^2``

    in four steps: centroids -> centred coordinates -> rotation -> translation
    ``t = G_bar - R * S_bar``. Works for any ``N >= 2``; for ``N == 2`` the
    least-squares solution coincides with the exact two-point construction
    (see :func:`estimate_sensor_pose_two_points`).

    Args:
        sensor_points: ``(N, 2)`` measurements in the sensor frame, ordered so
            that row ``i`` corresponds to row ``i`` of ``grid_points``.
        grid_points: ``(N, 2)`` known target positions in the grid frame.
        minimum_number_of_points: Minimum accepted ``N`` (clamped to >= 2).
        minimum_baseline: Minimum accepted spatial spread [m]; ``0`` disables
            the check.
        method: ``"closed_form"`` (2D atan2 solution) or ``"svd"`` (Kabsch).

    Returns:
        A :class:`SensorPoseEstimate` with the pose and residual statistics.

    Raises:
        InvalidShapeError: Bad array shapes or non-finite values.
        InsufficientPointsError: Fewer correspondences than required.
        DegenerateGeometryError: Geometry that cannot constrain the yaw.
        ValueError: Unknown ``method``.
    """
    sensor_array, grid_array = _validate_correspondences(
        sensor_points, grid_points, minimum_number_of_points, minimum_baseline
    )

    sensor_centroid = sensor_array.mean(axis=0)   # S_bar
    grid_centroid = grid_array.mean(axis=0)       # G_bar
    centered_sensor = sensor_array - sensor_centroid  # ui
    centered_grid = grid_array - grid_centroid        # vi

    if method == "closed_form":
        yaw = _yaw_closed_form(centered_sensor, centered_grid)
    elif method == "svd":
        yaw = _yaw_svd(centered_sensor, centered_grid)
    else:
        raise ValueError(f"unknown method {method!r}, expected 'closed_form' or 'svd'")

    yaw = normalize_angle(yaw)
    translation = grid_centroid - rotation_matrix(yaw) @ sensor_centroid

    report = calculate_residuals(
        sensor_array, grid_array, translation[0], translation[1], yaw
    )
    return SensorPoseEstimate(
        tx=float(translation[0]),
        ty=float(translation[1]),
        yaw=float(yaw),
        rms_error=report.rms_error,
        mean_error=report.mean_error,
        max_error=report.max_error,
        number_of_points=report.number_of_points,
        method=method,
        residuals=report.residuals,
        errors=report.errors,
    )


def estimate_sensor_pose_two_points(
    sensor_points: Sequence | np.ndarray,
    grid_points: Sequence | np.ndarray,
    minimum_baseline: float = DEFAULT_MIN_BASELINE,
) -> SensorPoseEstimate:
    """Exact two-point construction, kept for verification and teaching.

    ``yaw = atan2(dxs*dyg - dys*dxg, dxs*dxg + dys*dyg)`` followed by the
    averaged translation ``t = mean(Gi - R Si)``. Mathematically identical to
    :func:`estimate_sensor_pose` with ``N = 2``; the general least-squares
    routine is what the ROS nodes use.

    Args:
        sensor_points: ``(2, 2)`` sensor measurements.
        grid_points: ``(2, 2)`` known grid positions.
        minimum_baseline: Minimum accepted distance [m] between the points.

    Returns:
        A :class:`SensorPoseEstimate` (residuals are zero by construction).

    Raises:
        InvalidShapeError: If exactly two correspondences are not supplied.
        DegenerateGeometryError: If the two points are (nearly) coincident.
    """
    sensor_array, grid_array = _validate_correspondences(
        sensor_points, grid_points, DEFAULT_MIN_POINTS, minimum_baseline
    )
    if sensor_array.shape[0] != 2:
        raise InvalidShapeError(
            f"exactly two correspondences required, got {sensor_array.shape[0]}"
        )

    delta_sensor = sensor_array[1] - sensor_array[0]
    delta_grid = grid_array[1] - grid_array[0]
    yaw = normalize_angle(
        math.atan2(
            delta_sensor[0] * delta_grid[1] - delta_sensor[1] * delta_grid[0],
            delta_sensor[0] * delta_grid[0] + delta_sensor[1] * delta_grid[1],
        )
    )
    rot = rotation_matrix(yaw)
    translation = np.mean(grid_array - sensor_array @ rot.T, axis=0)

    report = calculate_residuals(
        sensor_array, grid_array, translation[0], translation[1], yaw
    )
    return SensorPoseEstimate(
        tx=float(translation[0]),
        ty=float(translation[1]),
        yaw=float(yaw),
        rms_error=report.rms_error,
        mean_error=report.mean_error,
        max_error=report.max_error,
        number_of_points=report.number_of_points,
        method="two_point",
        residuals=report.residuals,
        errors=report.errors,
    )


def installation_error(
    tx: float,
    ty: float,
    yaw: float,
    nominal_x: float,
    nominal_y: float,
    nominal_yaw: float = 0.0,
) -> InstallationError:
    """Deviation between an estimated pose and the nominal design pose.

    ``dx = tx - nominal_x``, ``dy = ty - nominal_y``,
    ``dyaw = normalize_angle(yaw - nominal_yaw)``.

    Args:
        tx: Estimated sensor x in the grid frame [m].
        ty: Estimated sensor y in the grid frame [m].
        yaw: Estimated sensor yaw [rad].
        nominal_x: Design x of the sensor [m] (e.g. 12.0).
        nominal_y: Design y of the sensor [m] (e.g. -1.0 or 16.0).
        nominal_yaw: Design yaw [rad], usually 0.

    Returns:
        An :class:`InstallationError` with ``dx``, ``dy`` and ``dyaw``.
    """
    return InstallationError(
        dx=float(tx) - float(nominal_x),
        dy=float(ty) - float(nominal_y),
        dyaw=normalize_angle(float(yaw) - float(nominal_yaw)),
    )
