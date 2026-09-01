"""Simulation tests: synthetic measurements, noise levels and baselines.

These cover sections 22-24 of the specification:

* a known sensor pose is recovered exactly from noise-free measurements,
* the estimation error scales with the measurement noise and shrinks with the
  number of targets,
* the yaw uncertainty follows ``sigma_yaw ~ sqrt(2) * sigma / L``.
"""

from __future__ import annotations

import math

import numpy as np
import pytest

from grid_sensor_calibration.simulation import (
    DEFAULT_TRUE_POSE,
    generate_sensor_measurements,
    grid_target_layout,
    run_monte_carlo,
    run_trial,
    study_baseline,
    study_noise_and_point_count,
)

SENSOR1_TRUE_POSE = (12.20, -0.85, math.radians(2.0))
SENSOR2_TRUE_POSE = (11.93, 16.08, math.radians(-1.5))


# --------------------------------------------------------------------------
# Noise-free recovery (section 22)
# --------------------------------------------------------------------------


@pytest.mark.parametrize("true_pose", [SENSOR1_TRUE_POSE, SENSOR2_TRUE_POSE])
@pytest.mark.parametrize("count", [2, 3, 4, 8])
def test_known_pose_is_recovered_from_synthetic_measurements(true_pose, count):
    result = run_trial(grid_target_layout(count), true_pose, sigma=0.0)

    assert result.estimate.tx == pytest.approx(true_pose[0], abs=1e-9)
    assert result.estimate.ty == pytest.approx(true_pose[1], abs=1e-9)
    assert result.estimate.yaw == pytest.approx(true_pose[2], abs=1e-9)
    assert result.position_error < 1e-9
    assert abs(result.yaw_error_deg) < 1e-7
    assert result.rms_residual < 1e-9


def test_generated_measurements_reproject_onto_the_targets():
    targets = grid_target_layout(4)
    tx, ty, yaw = SENSOR1_TRUE_POSE
    sensor_points = generate_sensor_measurements(targets, tx, ty, yaw, sigma=0.0)

    from grid_sensor_calibration.calibration_math import sensor_to_grid

    assert np.allclose(sensor_to_grid(sensor_points, tx, ty, yaw), targets, atol=1e-9)


def test_noise_is_reproducible_for_a_given_seed():
    targets = grid_target_layout(4)
    first = generate_sensor_measurements(
        targets, *SENSOR1_TRUE_POSE, sigma=0.03, rng=np.random.default_rng(5)
    )
    second = generate_sensor_measurements(
        targets, *SENSOR1_TRUE_POSE, sigma=0.03, rng=np.random.default_rng(5)
    )
    assert np.allclose(first, second)


# --------------------------------------------------------------------------
# Noise sensitivity (section 23)
# --------------------------------------------------------------------------


@pytest.mark.parametrize("sigma", [0.01, 0.03, 0.05])
def test_position_error_stays_within_the_noise_order(sigma):
    result = run_monte_carlo(grid_target_layout(4), sigma=sigma, trials=200, seed=11)

    # With 4 well-spread targets the mean position error is of the order of the
    # per-point noise, not a multiple of it.
    assert result.position_error_mean < 1.5 * sigma
    assert result.rms_residual_mean < 2.0 * sigma
    assert result.number_of_points == 4


def test_error_grows_with_the_noise_level():
    errors = [
        run_monte_carlo(grid_target_layout(4), sigma=sigma, trials=200, seed=3).position_error_mean
        for sigma in (0.01, 0.03, 0.05)
    ]
    assert errors[0] < errors[1] < errors[2]


def test_more_targets_reduce_the_error():
    results = study_noise_and_point_count(
        sigmas=(0.03,), point_counts=(2, 4, 8), trials=300, seed=21
    )
    position_errors = [result.position_error_mean for result in results]
    yaw_errors = [result.yaw_error_rms_deg for result in results]

    assert position_errors[0] > position_errors[1] > position_errors[2]
    assert yaw_errors[0] > yaw_errors[2]


def test_two_point_residual_understates_the_real_error():
    """A small residual is not proof of accuracy when only two targets are used.

    With N = 2 the 3-DOF transform absorbs everything except the change in the
    inter-target distance, so the reported residual is systematically smaller
    than the actual pose error.
    """
    two = run_monte_carlo(grid_target_layout(2), sigma=0.05, trials=300, seed=1)
    many = run_monte_carlo(grid_target_layout(8), sigma=0.05, trials=300, seed=1)

    assert two.rms_residual_mean < two.position_error_mean
    # With more targets the residual becomes a meaningful noise estimate.
    assert many.rms_residual_mean == pytest.approx(0.05, rel=0.4)
    assert many.rms_residual_mean > many.position_error_mean


# --------------------------------------------------------------------------
# Baseline versus yaw accuracy (section 24)
# --------------------------------------------------------------------------


def test_yaw_error_follows_the_sqrt2_sigma_over_baseline_law():
    results = study_baseline(baselines=(3.0, 6.0, 12.0, 24.0), sigma=0.02, trials=600)

    for result, predicted in results:
        ratio = result.yaw_error_rms_deg / predicted
        assert 0.7 < ratio < 1.4, f"L={result.baseline}: ratio {ratio:.2f}"

    measured = [result.yaw_error_rms_deg for result, _ in results]
    assert measured[0] > measured[1] > measured[2] > measured[3]
    # Doubling the baseline should roughly halve the yaw error.
    assert measured[0] / measured[1] == pytest.approx(2.0, rel=0.35)
    assert measured[1] / measured[2] == pytest.approx(2.0, rel=0.35)


def test_default_true_pose_matches_the_specification_example():
    assert DEFAULT_TRUE_POSE[0] == pytest.approx(12.20)
    assert DEFAULT_TRUE_POSE[1] == pytest.approx(-0.85)
    assert math.degrees(DEFAULT_TRUE_POSE[2]) == pytest.approx(2.0)
