"""Human-readable calibration report formatting (no ROS dependency)."""

from __future__ import annotations

import math
from typing import List, Optional

from .calibration_io import NominalPose
from .calibration_math import InstallationError, SensorPoseEstimate

__all__ = ["format_calibration_report", "format_residual_table"]

_SEPARATOR = "=" * 40
_LABEL_WIDTH = 12


def _row(label: str, value: str) -> str:
    """Format one ``label : value`` line of the report."""
    return f"{label:<{_LABEL_WIDTH}}: {value}"


def format_calibration_report(
    sensor_name: str,
    estimate: SensorPoseEstimate,
    nominal: NominalPose,
    error: InstallationError,
    parent_frame: str = "grid",
    child_frame: Optional[str] = None,
    output_path: Optional[str] = None,
) -> str:
    """Build the multi-line calibration summary printed after a calibration.

    Args:
        sensor_name: Logical sensor name.
        estimate: Estimated pose and residual statistics.
        nominal: Nominal (design) installation pose.
        error: Installation error w.r.t. the nominal pose.
        parent_frame: TF parent frame.
        child_frame: TF child frame; defaults to ``sensor_name``.
        output_path: Optional path where the result was saved.

    Returns:
        The formatted report as a single string (no trailing newline).
    """
    lines: List[str] = [
        "",
        _SEPARATOR,
        "Sensor Calibration Result",
        _SEPARATOR,
        "",
        _row("Sensor", sensor_name),
        _row("TF", f"{parent_frame} -> {child_frame or sensor_name}"),
        "",
        "Estimated pose in GRID frame",
        _row("X", f"{estimate.tx:.3f} m"),
        _row("Y", f"{estimate.ty:.3f} m"),
        _row("Yaw", f"{estimate.yaw_deg:.3f} deg ({estimate.yaw:.6f} rad)"),
        "",
        "Nominal pose",
        _row("X", f"{nominal.x:.3f} m"),
        _row("Y", f"{nominal.y:.3f} m"),
        _row("Yaw", f"{math.degrees(nominal.yaw):.3f} deg"),
        "",
        "Installation error",
        _row("dX", f"{error.dx:+.3f} m"),
        _row("dY", f"{error.dy:+.3f} m"),
        _row("dYaw", f"{error.dyaw_deg:+.3f} deg"),
        _row("dist", f"{error.distance:.3f} m"),
        "",
        "Calibration quality",
        _row("Method", estimate.method),
        _row("Points", f"{estimate.number_of_points}"),
        _row("RMS error", f"{estimate.rms_error:.4f} m"),
        _row("Mean error", f"{estimate.mean_error:.4f} m"),
        _row("Max error", f"{estimate.max_error:.4f} m"),
    ]
    if output_path:
        lines += ["", _row("Saved to", output_path)]
    lines += [_SEPARATOR]
    return "\n".join(lines)


def format_residual_table(estimate: SensorPoseEstimate) -> str:
    """Build a per-point residual table for the calibration log.

    Args:
        estimate: Estimate carrying ``residuals`` and ``errors`` arrays.

    Returns:
        The formatted table, or an empty string when no residuals are stored.
    """
    if estimate.errors.size == 0:
        return ""
    lines = ["", "Per-point residuals", f"{'idx':>4} {'ex [m]':>10} {'ey [m]':>10} {'|e| [m]':>10}"]
    for index, (residual, magnitude) in enumerate(zip(estimate.residuals, estimate.errors)):
        lines.append(
            f"{index:>4} {residual[0]:>10.4f} {residual[1]:>10.4f} {magnitude:>10.4f}"
        )
    return "\n".join(lines)
