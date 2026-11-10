"""Write a calibration result into the tracker parameter YAML.

``sensor_calibration_node`` solves x / y / yaw and saves them, together with the
ground-plane z / roll / pitch it was given and the fit RMS, in a combined
results file (``sensors_calibration.yaml``). Copying those numbers into the
tracker YAML by hand was the last manual step of the procedure; this node does
it and prints what changed::

    ros2 run grid_sensor_calibration apply_calibration --ros-args \\
      -p calibration_file:=$HOME/grid_calibration/<session>/sensors_calibration.yaml \\
      -p tracker_params_file:=<.../lidar_localization_os0_indoor.yaml>

Each sensor's ``child_frame`` (``lidar1``...) is the key prefix in the tracker
YAML, so ``lidar1_x``, ``lidar1_y``, ``lidar1_z``, ``lidar1_roll``,
``lidar1_pitch``, ``lidar1_yaw`` and ``lidar1_extrinsic_rmse`` are replaced in
place -- comments and every other line are left untouched -- after the old file
is copied to ``<file>.<timestamp>.bak``.

Any of those keys can also be given as a parameter (``-p lidar1_z:=1.267``) to
override or fill in a value; results saved before the file carried z / roll /
pitch need this. ``dry_run:=true`` prints the changes without writing.
"""

from __future__ import annotations

import math
import re
import shutil
import sys
from datetime import datetime
from typing import Dict, List, Mapping, Optional, Tuple

import rclpy
from rclpy.node import Node

from .calibration_io import CalibrationIOError, SensorTransform, load_sensor_transforms

TRACKER_KEYS = ("x", "y", "z", "roll", "pitch", "yaw", "extrinsic_rmse")
_OVERRIDE_RE = re.compile(r"^lidar\d+_(%s)$" % "|".join(TRACKER_KEYS))

# Above this fit RMS the result is still written but flagged.
RMS_WARN_M = 0.02


def tracker_values(transforms: Mapping[str, SensorTransform]) -> Dict[str, float]:
    """Map calibrated transforms to tracker YAML keys.

    Args:
        transforms: Sensor name -> transform, as loaded from the results file.

    Returns:
        ``{"lidar1_x": ..., ...}``. z / roll / pitch are present only when the
        result was solved with ``apply_tilt_correction`` and stored them.
    """
    values: Dict[str, float] = {}
    for transform in transforms.values():
        prefix = transform.child_frame
        cal = transform.calibration or {}
        values[f"{prefix}_x"] = float(transform.x)
        values[f"{prefix}_y"] = float(transform.y)
        values[f"{prefix}_yaw"] = float(transform.yaw)
        if "rms_error" in cal:
            values[f"{prefix}_extrinsic_rmse"] = float(cal["rms_error"])
        for key in ("z", "roll", "pitch"):
            if f"sensor_{key}" in cal:
                values[f"{prefix}_{key}"] = float(cal[f"sensor_{key}"])
    return values


def apply_to_yaml_text(
    text: str, values: Mapping[str, float]
) -> Tuple[str, List[Tuple[str, str, str]], List[str]]:
    """Replace ``key: value`` lines in a YAML text, keeping everything else.

    Args:
        text: Original YAML text.
        values: Keys to set and their new values.

    Returns:
        ``(new_text, changes, missing)`` where ``changes`` lists
        ``(key, old, new)`` and ``missing`` the keys not found in the text.
    """
    changes: List[Tuple[str, str, str]] = []
    missing: List[str] = []
    for key, value in values.items():
        new = f"{value:.3f}" if key.endswith(("_x", "_y", "_z")) else f"{value:.6f}"
        pattern = re.compile(rf"^(\s*{re.escape(key)}:\s*)([^\s#]+)", re.MULTILINE)
        match = pattern.search(text)
        if match is None:
            missing.append(key)
            continue
        changes.append((key, match.group(2), new))
        text = text[:match.start(2)] + new + text[match.end(2):]
    return text, changes, missing


class ApplyCalibrationNode(Node):
    """One-shot node: results file -> tracker YAML."""

    def __init__(self) -> None:
        super().__init__(
            "apply_calibration",
            automatically_declare_parameters_from_overrides=True,
        )
        self.calibration_file = self._param("calibration_file", "")
        self.tracker_params_file = self._param("tracker_params_file", "")
        self.dry_run = bool(self._param("dry_run", False))

    def _param(self, name: str, default):
        if not self.has_parameter(name):
            self.declare_parameter(name, default)
        return self.get_parameter(name).value

    def overrides(self) -> Dict[str, float]:
        """``lidarN_<key>`` parameters given on the command line."""
        return {
            name: float(param.value)
            for name, param in self._parameters.items()
            if _OVERRIDE_RE.match(name)
        }

    def run(self) -> bool:
        log = self.get_logger()
        if not self.calibration_file or not self.tracker_params_file:
            log.error("calibration_file and tracker_params_file are both required")
            return False
        try:
            transforms = load_sensor_transforms(self.calibration_file)
        except CalibrationIOError as exc:
            log.error(str(exc))
            return False
        try:
            with open(self.tracker_params_file, "r", encoding="utf-8") as handle:
                original = handle.read()
        except OSError as exc:
            log.error(f"cannot read tracker YAML: {exc}")
            return False

        values = tracker_values(transforms)
        values.update(self.overrides())
        new_text, changes, missing = apply_to_yaml_text(original, values)

        lines = [f"calibration file: {self.calibration_file}",
                 f"tracker YAML    : {self.tracker_params_file}", ""]
        lines += [f"  {key:24s} {old:>12s} -> {new}" for key, old, new in changes]
        lines += self._warnings(transforms, values, missing)
        log.info("\n" + "\n".join(lines))

        if self.dry_run:
            log.info("dry_run: nothing written")
            return True
        backup = f"{self.tracker_params_file}.{datetime.now():%Y%m%d_%H%M%S}.bak"
        shutil.copy2(self.tracker_params_file, backup)
        with open(self.tracker_params_file, "w", encoding="utf-8") as handle:
            handle.write(new_text)
        log.info(f"written ({len(changes)} values); previous file kept as {backup}")
        return True

    @staticmethod
    def _warnings(
        transforms: Mapping[str, SensorTransform],
        values: Mapping[str, float],
        missing: List[str],
    ) -> List[str]:
        out: List[str] = []
        for key in missing:
            out.append(f"  WARNING: '{key}' is not in the tracker YAML; not written")
        for transform in transforms.values():
            prefix = transform.child_frame
            rms = (transform.calibration or {}).get("rms_error")
            if rms is not None and rms > RMS_WARN_M:
                out.append(f"  WARNING: {prefix} fit RMS {rms * 1000:.1f} mm > "
                           f"{RMS_WARN_M * 1000:.0f} mm -- recapture the worst point")
            absent = [k for k in ("z", "roll", "pitch") if f"{prefix}_{k}" not in values]
            if absent:
                out.append(f"  WARNING: {prefix} {'/'.join(absent)} not in the results file; "
                           f"pass -p {prefix}_z:=... etc. from ground_plane_node")
        yaws = [transform.yaw for transform in transforms.values()]
        if len(yaws) == 2:
            gap = abs((math.degrees(yaws[0] - yaws[1]) + 180.0) % 360.0 - 180.0)
            out.append(f"  yaw gap between sensors: {gap:.1f} deg "
                       f"({'OK' if gap > 170.0 else 'expected about 180 for facing sensors'})")
        return out


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = ApplyCalibrationNode()
    ok = node.run()
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
