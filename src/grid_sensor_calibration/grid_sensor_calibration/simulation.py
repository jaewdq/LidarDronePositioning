"""Synthetic calibration studies (no ROS, no hardware required).

Generates sensor measurements from a *known* sensor pose with

``Ps = R(yaw)^T (Pg - t)``

optionally corrupted by Gaussian noise, feeds them back into
:func:`~grid_sensor_calibration.calibration_math.estimate_sensor_pose` and
reports how well the true pose is recovered.

Three studies are provided and printed by ``main()``:

1. **Noise sweep** -- sigma in {0.01, 0.03, 0.05} m for N in {2, 3, 4, 8}.
2. **Point-count sweep** -- error versus number of targets.
3. **Baseline sweep** -- yaw error versus target separation L, compared with
   the analytic order of magnitude ``sigma_yaw ~ sqrt(2) * sigma / L``.

Run it with::

    ros2 run grid_sensor_calibration calibration_simulation
    python3 -m grid_sensor_calibration.simulation --trials 500
"""

from __future__ import annotations

import argparse
import math
from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

from .calibration_math import (
    CalibrationError,
    SensorPoseEstimate,
    estimate_sensor_pose,
    grid_to_sensor,
    normalize_angle,
)

__all__ = [
    "GRID_WIDTH",
    "GRID_HEIGHT",
    "TrialResult",
    "StudyResult",
    "grid_target_layout",
    "generate_sensor_measurements",
    "run_trial",
    "run_monte_carlo",
    "study_noise_and_point_count",
    "study_baseline",
    "main",
]

#: Field geometry used by the studies [m].
GRID_WIDTH: float = 24.0
GRID_HEIGHT: float = 15.0
GRID_SPACING: float = 3.0

#: Default "true" sensor 1 pose used by the studies (tx [m], ty [m], yaw [rad]).
DEFAULT_TRUE_POSE: Tuple[float, float, float] = (12.20, -0.85, math.radians(2.0))

#: Target layouts per point count, spread as widely as the grid allows.
_TARGET_LAYOUTS: Dict[int, List[Tuple[float, float]]] = {
    2: [(3.0, 3.0), (21.0, 12.0)],
    3: [(3.0, 3.0), (21.0, 3.0), (12.0, 12.0)],
    4: [(3.0, 3.0), (21.0, 3.0), (21.0, 12.0), (3.0, 12.0)],
    6: [(3.0, 3.0), (12.0, 3.0), (21.0, 3.0), (21.0, 12.0), (12.0, 12.0), (3.0, 12.0)],
    8: [
        (3.0, 3.0), (12.0, 3.0), (21.0, 3.0), (21.0, 7.5),
        (21.0, 12.0), (12.0, 12.0), (3.0, 12.0), (3.0, 7.5),
    ],
}


@dataclass(frozen=True)
class TrialResult:
    """Errors of a single synthetic calibration trial."""

    position_error: float   # ||(tx, ty)_est - (tx, ty)_true|| [m]
    yaw_error: float        # signed yaw error [rad]
    rms_residual: float     # RMS calibration residual reported by the fit [m]
    estimate: SensorPoseEstimate

    @property
    def yaw_error_deg(self) -> float:
        """Signed yaw error in degrees."""
        return math.degrees(self.yaw_error)


@dataclass(frozen=True)
class StudyResult:
    """Monte-Carlo aggregate for one (N, sigma, layout) configuration."""

    number_of_points: int
    sigma: float
    trials: int
    position_error_mean: float
    position_error_max: float
    yaw_error_rms_deg: float
    rms_residual_mean: float
    baseline: float

    def as_row(self) -> str:
        """Format the aggregate as one fixed-width table row."""
        return (
            f"{self.number_of_points:>3} {self.sigma:>8.3f} {self.baseline:>9.2f} "
            f"{self.position_error_mean:>12.4f} {self.position_error_max:>11.4f} "
            f"{self.yaw_error_rms_deg:>12.4f} {self.rms_residual_mean:>13.4f}"
        )


def grid_target_layout(number_of_points: int) -> np.ndarray:
    """Return a well-spread target layout for ``number_of_points`` targets.

    Args:
        number_of_points: Requested number of calibration targets (>= 2).

    Returns:
        ``(N, 2)`` array of grid-frame target positions [m].

    Raises:
        ValueError: If fewer than two points are requested.
    """
    if number_of_points < 2:
        raise ValueError("at least two calibration targets are required")
    if number_of_points in _TARGET_LAYOUTS:
        return np.asarray(_TARGET_LAYOUTS[number_of_points], dtype=float)
    # Fall back to points evenly spaced on the perimeter of the inner grid.
    margin = GRID_SPACING
    angles = np.linspace(0.0, 2.0 * math.pi, number_of_points, endpoint=False)
    center = np.array([GRID_WIDTH / 2.0, GRID_HEIGHT / 2.0])
    radii = np.array([GRID_WIDTH / 2.0 - margin, GRID_HEIGHT / 2.0 - margin])
    return center + radii * np.stack([np.cos(angles), np.sin(angles)], axis=1)


def generate_sensor_measurements(
    grid_points: np.ndarray,
    tx: float,
    ty: float,
    yaw: float,
    sigma: float = 0.0,
    rng: Optional[np.random.Generator] = None,
) -> np.ndarray:
    """Create synthetic sensor-frame measurements of known grid targets.

    Args:
        grid_points: ``(N, 2)`` true target positions in the grid frame.
        tx: True sensor x in the grid frame [m].
        ty: True sensor y in the grid frame [m].
        yaw: True sensor yaw [rad].
        sigma: Standard deviation of the isotropic Gaussian measurement noise
            added in the sensor frame [m].
        rng: Random generator; a fresh default one is used when omitted.

    Returns:
        ``(N, 2)`` array of noisy sensor-frame measurements.
    """
    sensor_points = grid_to_sensor(np.asarray(grid_points, dtype=float), tx, ty, yaw)
    if sigma > 0.0:
        generator = rng if rng is not None else np.random.default_rng()
        sensor_points = sensor_points + generator.normal(0.0, sigma, sensor_points.shape)
    return sensor_points


def run_trial(
    grid_points: np.ndarray,
    true_pose: Tuple[float, float, float] = DEFAULT_TRUE_POSE,
    sigma: float = 0.0,
    rng: Optional[np.random.Generator] = None,
    minimum_baseline: float = 0.0,
) -> TrialResult:
    """Run one generate -> estimate -> compare cycle.

    Args:
        grid_points: ``(N, 2)`` true target positions.
        true_pose: ``(tx, ty, yaw)`` of the simulated sensor.
        sigma: Measurement noise standard deviation [m].
        rng: Random generator.
        minimum_baseline: Baseline check passed to the estimator (``0``
            disables it, which the baseline study needs for short baselines).

    Returns:
        The :class:`TrialResult` of this trial.
    """
    tx, ty, yaw = true_pose
    sensor_points = generate_sensor_measurements(grid_points, tx, ty, yaw, sigma, rng)
    estimate = estimate_sensor_pose(
        sensor_points, grid_points, minimum_baseline=minimum_baseline
    )
    return TrialResult(
        position_error=float(math.hypot(estimate.tx - tx, estimate.ty - ty)),
        yaw_error=normalize_angle(estimate.yaw - yaw),
        rms_residual=estimate.rms_error,
        estimate=estimate,
    )


def run_monte_carlo(
    grid_points: np.ndarray,
    true_pose: Tuple[float, float, float] = DEFAULT_TRUE_POSE,
    sigma: float = 0.01,
    trials: int = 200,
    seed: int = 0,
    minimum_baseline: float = 0.0,
) -> StudyResult:
    """Repeat :func:`run_trial` and aggregate the errors.

    Args:
        grid_points: ``(N, 2)`` true target positions.
        true_pose: ``(tx, ty, yaw)`` of the simulated sensor.
        sigma: Measurement noise standard deviation [m].
        trials: Number of Monte-Carlo repetitions.
        seed: Seed of the random generator (reproducible results).
        minimum_baseline: Baseline check passed to the estimator.

    Returns:
        The aggregated :class:`StudyResult`.
    """
    rng = np.random.default_rng(seed)
    points = np.asarray(grid_points, dtype=float)
    position_errors: List[float] = []
    yaw_errors: List[float] = []
    residuals: List[float] = []
    for _ in range(max(int(trials), 1)):
        result = run_trial(points, true_pose, sigma, rng, minimum_baseline)
        position_errors.append(result.position_error)
        yaw_errors.append(result.yaw_error)
        residuals.append(result.rms_residual)

    diff = points[:, None, :] - points[None, :, :]
    baseline = float(np.sqrt(np.einsum("ijk,ijk->ij", diff, diff)).max())
    yaw_array = np.asarray(yaw_errors)
    return StudyResult(
        number_of_points=int(points.shape[0]),
        sigma=float(sigma),
        trials=int(trials),
        position_error_mean=float(np.mean(position_errors)),
        position_error_max=float(np.max(position_errors)),
        yaw_error_rms_deg=float(math.degrees(math.sqrt(float(np.mean(yaw_array**2))))),
        rms_residual_mean=float(np.mean(residuals)),
        baseline=baseline,
    )


def study_noise_and_point_count(
    sigmas: Sequence[float] = (0.01, 0.03, 0.05),
    point_counts: Sequence[int] = (2, 3, 4, 8),
    true_pose: Tuple[float, float, float] = DEFAULT_TRUE_POSE,
    trials: int = 200,
    seed: int = 0,
) -> List[StudyResult]:
    """Sweep measurement noise and target count.

    Args:
        sigmas: Measurement noise levels [m].
        point_counts: Numbers of calibration targets to test.
        true_pose: ``(tx, ty, yaw)`` of the simulated sensor.
        trials: Monte-Carlo repetitions per configuration.
        seed: Base seed; each configuration gets a derived seed.

    Returns:
        One :class:`StudyResult` per (sigma, N) combination.
    """
    results: List[StudyResult] = []
    for sigma_index, sigma in enumerate(sigmas):
        for count_index, count in enumerate(point_counts):
            results.append(
                run_monte_carlo(
                    grid_target_layout(count),
                    true_pose=true_pose,
                    sigma=sigma,
                    trials=trials,
                    seed=seed + 100 * sigma_index + count_index,
                )
            )
    return results


def study_baseline(
    baselines: Sequence[float] = (3.0, 6.0, 12.0, 24.0),
    sigma: float = 0.02,
    true_pose: Tuple[float, float, float] = DEFAULT_TRUE_POSE,
    trials: int = 400,
    seed: int = 7,
) -> List[Tuple[StudyResult, float]]:
    """Sweep the separation between two calibration targets.

    Two targets are placed symmetrically about the grid centre along x, so the
    baseline is exactly ``L``.

    Args:
        baselines: Target separations ``L`` [m].
        sigma: Measurement noise standard deviation [m].
        true_pose: ``(tx, ty, yaw)`` of the simulated sensor.
        trials: Monte-Carlo repetitions per baseline.
        seed: Base seed.

    Returns:
        List of ``(StudyResult, predicted_yaw_sigma_deg)`` pairs, where the
        prediction is ``sqrt(2) * sigma / L`` converted to degrees.
    """
    results: List[Tuple[StudyResult, float]] = []
    center_x, center_y = GRID_WIDTH / 2.0, GRID_HEIGHT / 2.0
    for index, baseline in enumerate(baselines):
        targets = np.array(
            [
                [center_x - baseline / 2.0, center_y],
                [center_x + baseline / 2.0, center_y],
            ],
            dtype=float,
        )
        result = run_monte_carlo(
            targets,
            true_pose=true_pose,
            sigma=sigma,
            trials=trials,
            seed=seed + index,
        )
        predicted = math.degrees(math.sqrt(2.0) * sigma / baseline)
        results.append((result, predicted))
    return results


# --------------------------------------------------------------------------
# Command line report
# --------------------------------------------------------------------------

_HEADER = (
    f"{'N':>3} {'sigma[m]':>8} {'base[m]':>9} {'pos err[m]':>12} "
    f"{'max err[m]':>11} {'yaw rms[d]':>12} {'residual[m]':>13}"
)


def _print_exact_recovery(true_pose: Tuple[float, float, float]) -> None:
    """Print the noise-free round-trip check of section 22 of the spec."""
    tx, ty, yaw = true_pose
    targets = grid_target_layout(4)
    result = run_trial(targets, true_pose, sigma=0.0)
    print("Noise-free recovery of a known sensor pose")
    print(f"  true      : tx={tx:.3f} m, ty={ty:.3f} m, yaw={math.degrees(yaw):.3f} deg")
    print(
        f"  estimated : tx={result.estimate.tx:.6f} m, ty={result.estimate.ty:.6f} m, "
        f"yaw={result.estimate.yaw_deg:.6f} deg"
    )
    print(
        f"  errors    : position {result.position_error:.3e} m, "
        f"yaw {result.yaw_error_deg:.3e} deg, residual {result.rms_residual:.3e} m\n"
    )


def main(argv: Optional[Sequence[str]] = None) -> int:
    """Run the three studies and print their tables.

    Args:
        argv: Optional argument vector (defaults to ``sys.argv[1:]``).

    Returns:
        Process exit code (``0`` on success, ``1`` on a calibration error).
    """
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--trials", type=int, default=200, help="Monte-Carlo repetitions")
    parser.add_argument("--seed", type=int, default=0, help="base random seed")
    parser.add_argument("--tx", type=float, default=DEFAULT_TRUE_POSE[0], help="true tx [m]")
    parser.add_argument("--ty", type=float, default=DEFAULT_TRUE_POSE[1], help="true ty [m]")
    parser.add_argument(
        "--yaw-deg", type=float, default=math.degrees(DEFAULT_TRUE_POSE[2]), help="true yaw [deg]"
    )
    parser.add_argument(
        "--baseline-sigma", type=float, default=0.02, help="noise used in the baseline study [m]"
    )
    # ROS 2 passes extra arguments (e.g. --ros-args) to ``ros2 run`` targets.
    args, _unknown = parser.parse_known_args(argv)

    true_pose = (args.tx, args.ty, math.radians(args.yaw_deg))

    try:
        _print_exact_recovery(true_pose)

        print("Noise / target-count sweep (Monte-Carlo)")
        print(_HEADER)
        for result in study_noise_and_point_count(
            true_pose=true_pose, trials=args.trials, seed=args.seed
        ):
            print(result.as_row())

        print("\nBaseline sweep, two targets, sigma = " f"{args.baseline_sigma:.3f} m")
        print(
            f"{'L[m]':>6} {'yaw rms[deg]':>14} {'predicted[deg]':>16} "
            f"{'ratio':>8} {'pos err[m]':>12}"
        )
        for result, predicted in study_baseline(
            sigma=args.baseline_sigma, true_pose=true_pose, trials=max(args.trials, 200)
        ):
            ratio = result.yaw_error_rms_deg / predicted if predicted > 0 else float("nan")
            print(
                f"{result.baseline:>6.1f} {result.yaw_error_rms_deg:>14.4f} "
                f"{predicted:>16.4f} {ratio:>8.2f} {result.position_error_mean:>12.4f}"
            )
        print(
            "\nyaw uncertainty follows sigma_yaw ~ sqrt(2) * sigma / L: "
            "doubling the baseline halves the yaw error."
        )
    except CalibrationError as exc:
        print(f"simulation failed: {exc}")
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
