"""Unit tests for the ROS-free calibration mathematics."""

from __future__ import annotations

import math

import numpy as np
import pytest

from grid_sensor_calibration.calibration_math import (
    DegenerateGeometryError,
    InsufficientPointsError,
    InvalidShapeError,
    calculate_residuals,
    estimate_sensor_pose,
    estimate_sensor_pose_two_points,
    get_transform_matrix,
    grid_to_sensor,
    installation_error,
    invert_transform_matrix,
    normalize_angle,
    quaternion_to_yaw,
    rotation_matrix,
    sensor_to_grid,
    yaw_to_quaternion,
)

RECOMMENDED_TARGETS = np.array(
    [[3.0, 3.0], [21.0, 3.0], [21.0, 12.0], [3.0, 12.0]], dtype=float
)
SENSOR1_TRUE_POSE = (12.20, -0.85, math.radians(2.0))
SENSOR2_TRUE_POSE = (11.93, 16.08, math.radians(-1.5))


# --------------------------------------------------------------------------
# Angles and matrices
# --------------------------------------------------------------------------


@pytest.mark.parametrize(
    "angle, expected",
    [
        (0.0, 0.0),
        (math.pi, -math.pi),
        (-math.pi, -math.pi),
        (3.0 * math.pi, -math.pi),
        (math.radians(370.0), math.radians(10.0)),
        (math.radians(-190.0), math.radians(170.0)),
    ],
)
def test_normalize_angle_wraps_into_half_open_interval(angle, expected):
    result = normalize_angle(angle)
    assert -math.pi <= result < math.pi
    assert result == pytest.approx(expected, abs=1e-12)


def test_rotation_matrix_is_ccw_and_orthonormal():
    rot = rotation_matrix(math.pi / 2.0)
    assert np.allclose(rot @ np.array([1.0, 0.0]), [0.0, 1.0])  # +x -> +y is CCW
    assert np.allclose(rot @ rot.T, np.eye(2))
    assert np.linalg.det(rot) == pytest.approx(1.0)


def test_transform_matrix_matches_sensor_to_grid():
    tx, ty, yaw = 4.0, -2.0, math.radians(37.0)
    matrix = get_transform_matrix(tx, ty, yaw)
    point = np.array([1.5, -0.75])
    homogeneous = matrix @ np.array([point[0], point[1], 1.0])
    assert np.allclose(homogeneous[:2], sensor_to_grid(point, tx, ty, yaw))
    assert np.allclose(invert_transform_matrix(matrix) @ matrix, np.eye(3))


def test_yaw_quaternion_round_trip():
    for yaw_deg in (-179.0, -90.0, -2.0, 0.0, 2.0, 90.0, 179.0):
        yaw = math.radians(yaw_deg)
        assert quaternion_to_yaw(*yaw_to_quaternion(yaw)) == pytest.approx(yaw, abs=1e-12)


# --------------------------------------------------------------------------
# Point transforms -- the sign/direction sanity checks of the specification
# --------------------------------------------------------------------------


def test_specification_example_sensor_at_10_5_rotated_90_deg():
    """Section 29: pose (10, 5, 90 deg) maps sensor point (1, 0) to (10, 6)."""
    grid_point = sensor_to_grid([1.0, 0.0], 10.0, 5.0, math.radians(90.0))
    assert grid_point == pytest.approx([10.0, 6.0], abs=1e-12)


def test_translation_is_the_sensor_origin_in_the_grid_frame():
    tx, ty, yaw = 12.2, -0.85, math.radians(2.0)
    assert sensor_to_grid([0.0, 0.0], tx, ty, yaw) == pytest.approx([tx, ty])


def test_grid_to_sensor_is_the_inverse_of_sensor_to_grid():
    tx, ty, yaw = SENSOR2_TRUE_POSE
    points = np.array([[0.0, 0.0], [1.0, -2.0], [24.0, 15.0], [-5.5, 7.25]])
    round_trip = sensor_to_grid(grid_to_sensor(points, tx, ty, yaw), tx, ty, yaw)
    assert np.allclose(round_trip, points, atol=1e-12)


def test_transforms_accept_single_points_and_arrays():
    tx, ty, yaw = 1.0, 2.0, 0.3
    single = sensor_to_grid([1.0, 1.0], tx, ty, yaw)
    batch = sensor_to_grid([[1.0, 1.0], [2.0, 2.0]], tx, ty, yaw)
    assert single.shape == (2,)
    assert batch.shape == (2, 2)
    assert np.allclose(batch[0], single)


# --------------------------------------------------------------------------
# Pose estimation
# --------------------------------------------------------------------------


@pytest.mark.parametrize("true_pose", [SENSOR1_TRUE_POSE, SENSOR2_TRUE_POSE])
@pytest.mark.parametrize("method", ["closed_form", "svd"])
def test_noise_free_pose_is_recovered_exactly(true_pose, method):
    tx, ty, yaw = true_pose
    sensor_points = grid_to_sensor(RECOMMENDED_TARGETS, tx, ty, yaw)

    estimate = estimate_sensor_pose(sensor_points, RECOMMENDED_TARGETS, method=method)

    assert estimate.tx == pytest.approx(tx, abs=1e-9)
    assert estimate.ty == pytest.approx(ty, abs=1e-9)
    assert estimate.yaw == pytest.approx(yaw, abs=1e-9)
    assert estimate.yaw_deg == pytest.approx(math.degrees(yaw), abs=1e-7)
    assert estimate.rms_error < 1e-9
    assert estimate.mean_error < 1e-9
    assert estimate.max_error < 1e-9
    assert estimate.number_of_points == RECOMMENDED_TARGETS.shape[0]


def test_closed_form_and_svd_agree_on_noisy_data():
    rng = np.random.default_rng(1234)
    tx, ty, yaw = SENSOR1_TRUE_POSE
    noisy = grid_to_sensor(RECOMMENDED_TARGETS, tx, ty, yaw) + rng.normal(
        0.0, 0.02, RECOMMENDED_TARGETS.shape
    )

    closed = estimate_sensor_pose(noisy, RECOMMENDED_TARGETS, method="closed_form")
    svd = estimate_sensor_pose(noisy, RECOMMENDED_TARGETS, method="svd")

    assert closed.tx == pytest.approx(svd.tx, abs=1e-12)
    assert closed.ty == pytest.approx(svd.ty, abs=1e-12)
    assert closed.yaw == pytest.approx(svd.yaw, abs=1e-12)


def test_minimum_two_points_are_enough():
    tx, ty, yaw = SENSOR1_TRUE_POSE
    targets = RECOMMENDED_TARGETS[[0, 2]]
    sensor_points = grid_to_sensor(targets, tx, ty, yaw)

    estimate = estimate_sensor_pose(sensor_points, targets)

    assert estimate.number_of_points == 2
    assert estimate.tx == pytest.approx(tx, abs=1e-9)
    assert estimate.ty == pytest.approx(ty, abs=1e-9)
    assert estimate.yaw == pytest.approx(yaw, abs=1e-9)
    assert estimate.rms_error < 1e-9  # two points always fit exactly


def test_two_point_closed_form_matches_least_squares():
    tx, ty, yaw = SENSOR2_TRUE_POSE
    targets = RECOMMENDED_TARGETS[[1, 3]]
    sensor_points = grid_to_sensor(targets, tx, ty, yaw)

    least_squares = estimate_sensor_pose(sensor_points, targets)
    two_point = estimate_sensor_pose_two_points(sensor_points, targets)

    assert two_point.tx == pytest.approx(least_squares.tx, abs=1e-12)
    assert two_point.ty == pytest.approx(least_squares.ty, abs=1e-12)
    assert two_point.yaw == pytest.approx(least_squares.yaw, abs=1e-12)
    assert two_point.method == "two_point"


def test_least_squares_beats_a_single_pair_under_noise():
    """With N > 2 the fit averages the noise instead of interpolating it."""
    rng = np.random.default_rng(99)
    tx, ty, yaw = SENSOR1_TRUE_POSE
    sigma = 0.05
    many_errors, two_errors = [], []
    for _ in range(200):
        noise = rng.normal(0.0, sigma, RECOMMENDED_TARGETS.shape)
        noisy = grid_to_sensor(RECOMMENDED_TARGETS, tx, ty, yaw) + noise
        many = estimate_sensor_pose(noisy, RECOMMENDED_TARGETS)
        two = estimate_sensor_pose(noisy[[0, 2]], RECOMMENDED_TARGETS[[0, 2]])
        many_errors.append(abs(normalize_angle(many.yaw - yaw)))
        two_errors.append(abs(normalize_angle(two.yaw - yaw)))
    assert np.mean(many_errors) < np.mean(two_errors)


def test_residual_statistics_are_consistent():
    tx, ty, yaw = 0.0, 0.0, 0.0
    grid_points = np.array([[0.0, 0.0], [10.0, 0.0], [10.0, 10.0]])
    # Displace each measurement by a known amount along +x.
    sensor_points = grid_points - np.array([[0.1, 0.0], [0.2, 0.0], [0.3, 0.0]])

    report = calculate_residuals(sensor_points, grid_points, tx, ty, yaw)

    assert report.errors == pytest.approx([0.1, 0.2, 0.3])
    assert report.mean_error == pytest.approx(0.2)
    assert report.max_error == pytest.approx(0.3)
    assert report.rms_error == pytest.approx(math.sqrt((0.01 + 0.04 + 0.09) / 3.0))
    assert report.max_error_index == 2
    assert report.number_of_points == 3


def test_reported_residuals_match_a_fresh_evaluation():
    rng = np.random.default_rng(7)
    tx, ty, yaw = SENSOR1_TRUE_POSE
    noisy = grid_to_sensor(RECOMMENDED_TARGETS, tx, ty, yaw) + rng.normal(
        0.0, 0.01, RECOMMENDED_TARGETS.shape
    )
    estimate = estimate_sensor_pose(noisy, RECOMMENDED_TARGETS)
    report = calculate_residuals(
        noisy, RECOMMENDED_TARGETS, estimate.tx, estimate.ty, estimate.yaw
    )
    assert report.rms_error == pytest.approx(estimate.rms_error)
    assert report.max_error == pytest.approx(estimate.max_error)


# --------------------------------------------------------------------------
# Installation error
# --------------------------------------------------------------------------


def test_installation_error_for_sensor1_nominal_pose():
    error = installation_error(12.034, -0.982, math.radians(0.367), 12.0, -1.0, 0.0)
    assert error.dx == pytest.approx(0.034)
    assert error.dy == pytest.approx(0.018)
    assert error.dyaw_deg == pytest.approx(0.367)


def test_installation_error_for_sensor2_nominal_pose():
    error = installation_error(11.93, 16.08, math.radians(-1.5), 12.0, 16.0, 0.0)
    assert error.dx == pytest.approx(-0.07)
    assert error.dy == pytest.approx(0.08)
    assert error.dyaw_deg == pytest.approx(-1.5)


def test_installation_error_wraps_the_yaw_difference():
    error = installation_error(0.0, 0.0, math.radians(179.0), 0.0, 0.0, math.radians(-179.0))
    assert error.dyaw_deg == pytest.approx(-2.0, abs=1e-9)


# --------------------------------------------------------------------------
# Validation
# --------------------------------------------------------------------------


def test_single_point_is_rejected():
    with pytest.raises(InsufficientPointsError):
        estimate_sensor_pose([[0.0, 0.0]], [[3.0, 3.0]])


def test_shape_mismatch_is_rejected():
    with pytest.raises(InvalidShapeError):
        estimate_sensor_pose(RECOMMENDED_TARGETS, RECOMMENDED_TARGETS[:3])


def test_three_dimensional_points_are_rejected():
    with pytest.raises(InvalidShapeError):
        estimate_sensor_pose(np.zeros((4, 3)), np.zeros((4, 3)))


def test_non_finite_values_are_rejected():
    corrupted = RECOMMENDED_TARGETS.copy()
    corrupted[1, 0] = float("nan")
    with pytest.raises(InvalidShapeError):
        estimate_sensor_pose(corrupted, RECOMMENDED_TARGETS)


def test_coincident_points_are_rejected():
    identical = np.tile([5.0, 5.0], (4, 1))
    with pytest.raises(DegenerateGeometryError):
        estimate_sensor_pose(identical, RECOMMENDED_TARGETS, minimum_baseline=0.0)


def test_baseline_below_the_minimum_is_rejected():
    targets = np.array([[3.0, 3.0], [3.4, 3.0]])
    sensor_points = grid_to_sensor(targets, *SENSOR1_TRUE_POSE)
    with pytest.raises(DegenerateGeometryError):
        estimate_sensor_pose(sensor_points, targets, minimum_baseline=1.0)
    # ... but the same data pass once the requirement is relaxed.
    assert estimate_sensor_pose(sensor_points, targets, minimum_baseline=0.3)


def test_unknown_method_is_rejected():
    sensor_points = grid_to_sensor(RECOMMENDED_TARGETS, *SENSOR1_TRUE_POSE)
    with pytest.raises(ValueError):
        estimate_sensor_pose(sensor_points, RECOMMENDED_TARGETS, method="kalman")
