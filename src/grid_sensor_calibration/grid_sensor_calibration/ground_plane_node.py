"""Measure a LiDAR's height and tilt by fitting the ground plane.

``sensor_calibration_node`` solves a 2D rigid transform -- (tx, ty, yaw) only --
and writes ``z = 0``. Height therefore has to come from somewhere else, and roll
and pitch are simply assumed to be zero. On a 24 x 15 m field that assumption is
expensive: a 1 deg tilt is 17 cm of altitude error at 10 m and 33 cm at 19 m,
against a scoring budget of 0.20 m RMSE.

The ground itself is the best available reference. A LiDAR 2 m up sees tens of
thousands of ground returns per scan, so fitting a plane to them yields all
three missing numbers at once::

    plane  n . p + d = 0,  |n| = 1,  n oriented upward (nz > 0)

        lidar_z = d                      distance from sensor origin to ground
        pitch   = asin(-nx)              from n = (-sin0, cos0 sin1, cos0 cos1)
        roll    = atan2(ny, nz)          matching SciPyRotation 'xyz' (extrinsic)

which is the same convention ``lidar_drone_tracker`` uses to build R_s2w, so the
printed values paste straight into the tracker YAML.

Robustness to bystanders
------------------------
A 360 deg sensor sees whoever is standing around, but they barely matter here,
for three reasons stacked in front of RANSAC:

* the z band. Ground sits near ``z = -lidar_z`` in the sensor frame, so cropping
  a narrow band around it drops every standing person's torso and head before
  the fit runs. Only shins survive -- and those rest ON the plane.
* the range ring. Below ``h / tan(22.5 deg)`` (4.8 m at h = 2 m) the OS1 cannot
  see the ground at all, and past ~15 m the incidence angle turns grazing and
  inflates range noise into z noise. Both ends are excluded.
* the count. That ring holds ~40k ground returns; a person contributes ~100
  points to the band. Ten bystanders are 3% of the fit.

RANSAC then handles the remainder, and the reported inlier fraction says whether
it had to. Fits are repeated over several frames and reduced by median, so a
frame spoiled by someone walking through is outvoted rather than averaged in.

Typical use, with the driver already running::

    ros2 run grid_sensor_calibration ground_plane_node --ros-args \\
        -p input_topic:=/lidar1/points \\
        -p sensor_label:=lidar1 \\
        -p expected_height:=2.0
"""

from __future__ import annotations

import math
from typing import List, Optional, Tuple

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2

try:
    from sensor_msgs_py import point_cloud2 as pc2_util
except ImportError:  # pragma: no cover - older distributions
    from sensor_msgs import point_cloud2 as pc2_util

__all__ = ["GroundPlaneNode", "fit_plane_ransac", "plane_to_pose", "main"]

#: OS1-128 vertical half-FOV. Sets the inner radius of the usable ground ring.
_HALF_FOV_DEG = 22.5


# --------------------------------------------------------------------------
# Pure geometry (no ROS)
# --------------------------------------------------------------------------


def fit_plane_ransac(
    points: np.ndarray,
    threshold: float,
    iterations: int,
    rng: np.random.Generator,
) -> Optional[Tuple[np.ndarray, float, np.ndarray]]:
    """Fit a plane to ``points`` with RANSAC, then refine on the inliers.

    Args:
        points: ``(N, 3)`` candidate ground points in the sensor frame.
        threshold: Inlier distance to the plane [m].
        iterations: Number of random 3-point hypotheses.
        rng: Random generator (passed in so runs are reproducible).

    Returns:
        ``(normal, offset, inlier_mask)`` where ``normal`` is unit length with
        ``nz > 0`` and the plane is ``normal . p + offset = 0``; ``None`` if no
        hypothesis found enough support.
    """
    n_points = points.shape[0]
    if n_points < 3:
        return None

    best_count = 0
    best_normal: Optional[np.ndarray] = None
    best_offset = 0.0

    for _ in range(int(iterations)):
        idx = rng.choice(n_points, size=3, replace=False)
        p0, p1, p2 = points[idx]
        normal = np.cross(p1 - p0, p2 - p0)
        norm = float(np.linalg.norm(normal))
        if norm < 1e-9:            # collinear sample
            continue
        normal = normal / norm
        offset = -float(normal @ p0)
        distances = np.abs(points @ normal + offset)
        count = int(np.count_nonzero(distances <= threshold))
        if count > best_count:
            best_count = count
            best_normal = normal
            best_offset = offset

    if best_normal is None or best_count < 3:
        return None

    # Least-squares refinement on the consensus set: the smallest right
    # singular vector of the centred inliers is the plane normal.
    mask = np.abs(points @ best_normal + best_offset) <= threshold
    inliers = points[mask]
    centroid = inliers.mean(axis=0)
    _, _, vt = np.linalg.svd(inliers - centroid, full_matrices=False)
    normal = vt[-1]
    normal = normal / float(np.linalg.norm(normal))

    # Orient upward so that the offset is the sensor's height above the plane.
    if normal[2] < 0.0:
        normal = -normal
    offset = -float(normal @ centroid)

    mask = np.abs(points @ normal + offset) <= threshold
    return normal, offset, mask


def plane_to_pose(normal: np.ndarray, offset: float) -> Tuple[float, float, float]:
    """Convert an upward plane normal into height, roll and pitch.

    Inverts ``n = (-sin(pitch), cos(pitch) sin(roll), cos(pitch) cos(roll))``,
    the world-up vector expressed in the sensor frame for the extrinsic
    ``R = Rz(yaw) Ry(pitch) Rx(roll)`` that the tracker builds.

    Args:
        normal: Unit plane normal with ``nz > 0``.
        offset: Plane offset ``d`` in ``n . p + d = 0``.

    Returns:
        ``(height, roll, pitch)`` in metres and radians.
    """
    n_x, n_y, n_z = (float(v) for v in normal)
    pitch = math.asin(max(-1.0, min(1.0, -n_x)))
    roll = math.atan2(n_y, n_z)
    return float(offset), roll, pitch


# --------------------------------------------------------------------------
# ROS node
# --------------------------------------------------------------------------


class GroundPlaneNode(Node):
    """Collect several scans, fit the ground in each, report the median pose."""

    def __init__(self, **kwargs) -> None:
        super().__init__("ground_plane_node", **kwargs)

        self.declare_parameter("input_topic", "/lidar1/points")
        # Only used to label the printed YAML snippet.
        self.declare_parameter("sensor_label", "lidar1")

        # Rough height, used to place the crop band and the range ring. It does
        # not need to be accurate -- that is what this node measures -- but it
        # should be within z_band of the truth.
        self.declare_parameter("expected_height", 2.0)
        self.declare_parameter("z_band", 0.5)

        # Range ring in the sensor's xy plane. 0 for range_min auto-selects the
        # geometric limit h / tan(22.5 deg), inside which no beam reaches the
        # ground at all.
        self.declare_parameter("range_min", 0.0)
        self.declare_parameter("range_max", 15.0)

        # Sensor-frame azimuth window [deg], atan2(y, x). Use it to exclude a
        # fence, a building or a parked car behind the sensor -- those are the
        # things that actually threaten the fit, not bystanders. The window may
        # wrap through +/-180.
        self.declare_parameter("azimuth_min", -180.0)
        self.declare_parameter("azimuth_max", 180.0)

        self.declare_parameter("ransac_threshold", 0.03)
        self.declare_parameter("ransac_iterations", 200)
        # Below this the fit is flagged. Not an error on its own: a rough
        # surface legitimately pushes ground points past the threshold, so the
        # fraction is only alarming when the residual sigma stays SMALL -- that
        # combination means the rejected points belong to a competing surface
        # rather than to a bumpy one. The report says which case it is.
        self.declare_parameter("min_inlier_fraction", 0.70)

        self.declare_parameter("num_frames", 20)
        # Thins the cloud before fitting; 0 disables. The ring holds tens of
        # thousands of points and the plane does not need all of them.
        self.declare_parameter("voxel_size", 0.10)

        self.declare_parameter("shutdown_when_done", True)
        self.declare_parameter("timeout_sec", 30.0)

        gp = lambda name: self.get_parameter(name).value  # noqa: E731
        self._topic: str = gp("input_topic")
        self._label: str = gp("sensor_label")
        self._expected_h: float = float(gp("expected_height"))
        self._z_band: float = float(gp("z_band"))
        self._range_max: float = float(gp("range_max"))
        self._az_min: float = float(gp("azimuth_min"))
        self._az_max: float = float(gp("azimuth_max"))
        self._threshold: float = float(gp("ransac_threshold"))
        self._iterations: int = int(gp("ransac_iterations"))
        self._min_inlier: float = float(gp("min_inlier_fraction"))
        self._num_frames: int = int(gp("num_frames"))
        self._voxel: float = float(gp("voxel_size"))
        self._shutdown: bool = bool(gp("shutdown_when_done"))

        range_min = float(gp("range_min"))
        if range_min <= 0.0:
            range_min = self._expected_h / math.tan(math.radians(_HALF_FOV_DEG))
        self._range_min = range_min

        self._results: List[Tuple[float, float, float, float, float]] = []
        self._n_scans = 0
        self._rng = np.random.default_rng(0)
        self._done = False

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self.create_subscription(PointCloud2, self._topic, self._on_cloud, sensor_qos)

        timeout = float(gp("timeout_sec"))
        self._timeout_timer = self.create_timer(timeout, self._on_timeout) if timeout > 0 else None

        self.get_logger().info(
            f"fitting ground plane from '{self._topic}' over {self._num_frames} frame(s)\n"
            f"  crop z    : [{-(self._expected_h + self._z_band):+.2f}, "
            f"{-(self._expected_h - self._z_band):+.2f}] m (sensor frame)\n"
            f"  crop range: [{self._range_min:.2f}, {self._range_max:.2f}] m"
            f"   (inner limit is where a {_HALF_FOV_DEG:.1f} deg beam reaches the ground)\n"
            f"  azimuth   : [{self._az_min:.1f}, {self._az_max:.1f}] deg\n"
            f"  RANSAC    : {self._threshold * 100:.0f} mm over {self._iterations} iterations"
        )

    # ------------------------------------------------------------------

    def _on_cloud(self, msg: PointCloud2) -> None:
        """Fit one scan and, once enough have succeeded, print the report."""
        if self._done:
            return
        self._n_scans += 1

        pts = self._to_numpy(msg)
        if pts is None:
            return

        pts = self._crop(pts)
        n_cropped = pts.shape[0]
        if n_cropped < 50:
            self.get_logger().warn(
                f"only {n_cropped} point(s) survived the crop; check "
                "expected_height / range window",
                throttle_duration_sec=3.0,
            )
            return

        if self._voxel > 0.0:
            pts = self._voxel_downsample(pts, self._voxel)

        fit = fit_plane_ransac(pts, self._threshold, self._iterations, self._rng)
        if fit is None:
            self.get_logger().warn("RANSAC found no plane", throttle_duration_sec=3.0)
            return

        normal, offset, mask = fit
        height, roll, pitch = plane_to_pose(normal, offset)
        inlier_fraction = float(np.count_nonzero(mask)) / float(pts.shape[0])
        residual_std = float(np.std(pts[mask] @ normal + offset))

        self._results.append((height, roll, pitch, inlier_fraction, residual_std))
        self.get_logger().info(
            f"frame {len(self._results)}/{self._num_frames}: "
            f"z={height:.3f} m  roll={math.degrees(roll):+.3f} deg  "
            f"pitch={math.degrees(pitch):+.3f} deg  "
            f"inliers={inlier_fraction * 100:.1f}%  sigma={residual_std * 1000:.0f} mm"
        )

        if len(self._results) >= self._num_frames:
            self._finish()

    # ------------------------------------------------------------------

    def _crop(self, pts: np.ndarray) -> np.ndarray:
        """Keep the ground band: a z slice, a range ring and an azimuth window.

        Args:
            pts: ``(N, 3)`` sensor-frame points.

        Returns:
            The surviving ``(M, 3)`` subset.
        """
        z_low = -(self._expected_h + self._z_band)
        z_high = -(self._expected_h - self._z_band)
        radius = np.hypot(pts[:, 0], pts[:, 1])

        keep = (
            (pts[:, 2] >= z_low)
            & (pts[:, 2] <= z_high)
            & (radius >= self._range_min)
            & (radius <= self._range_max)
        )

        if not (self._az_min <= -180.0 and self._az_max >= 180.0):
            azimuth = np.degrees(np.arctan2(pts[:, 1], pts[:, 0]))
            if self._az_min <= self._az_max:
                keep &= (azimuth >= self._az_min) & (azimuth <= self._az_max)
            else:  # window wraps through +/-180
                keep &= (azimuth >= self._az_min) | (azimuth <= self._az_max)

        return pts[keep]

    def _finish(self) -> None:
        """Reduce the per-frame fits by median and print the final report."""
        self._done = True
        data = np.asarray(self._results, dtype=float)
        heights, rolls, pitches, fractions, sigmas = (data[:, i] for i in range(5))

        height = float(np.median(heights))
        roll = float(np.median(rolls))
        pitch = float(np.median(pitches))
        tilt = math.degrees(math.hypot(roll, pitch))

        # Azimuth of the downhill direction in the SENSOR frame: the plane
        # normal projected onto xy and reversed. Reported because roll and
        # pitch on their own cannot be compared between two sensors that are
        # installed pointing different ways, while a direction can.
        #
        # n = (-sin(pitch), cos(pitch) sin(roll), ...), so the downhill
        # direction is (sin(pitch), -cos(pitch) sin(roll)).
        downhill = math.degrees(
            math.atan2(-math.cos(pitch) * math.sin(roll), math.sin(pitch))
        )

        lines = [
            "",
            "=" * 66,
            f" GROUND PLANE FIT -- {self._label}   ({len(self._results)} frames, "
            f"{self._n_scans} scans seen)",
            "=" * 66,
            "",
            f"  height   z     = {height:8.3f} m      "
            f"(spread {heights.max() - heights.min():.3f} m over frames)",
            f"  roll           = {math.degrees(roll):+8.3f} deg    "
            f"(spread {math.degrees(rolls.max() - rolls.min()):.3f} deg)",
            f"  pitch          = {math.degrees(pitch):+8.3f} deg    "
            f"(spread {math.degrees(pitches.max() - pitches.min()):.3f} deg)",
            "",
            f"  total tilt     = {tilt:8.3f} deg   downhill toward "
            f"azimuth {downhill:+.0f} deg (sensor frame)",
            f"  inlier frac    = {np.median(fractions) * 100:7.1f} %",
            f"  residual sigma = {np.median(sigmas) * 1000:7.0f} mm     "
            "(ground roughness + range noise)",
            "",
        ]

        # The decision-relevant number: what the tilt costs at the far corner.
        # The scorer awards full altitude marks below 0.20 m RMSE.
        error_at_19m = 19.0 * math.tan(math.radians(tilt))
        lines.append(
            f"  leaving roll/pitch at 0 would cost {error_at_19m * 100:.0f} cm "
            "of z error at 19 m"
        )
        if error_at_19m > 0.20:
            lines.append("  -> that alone exceeds the 0.20 m altitude budget. Apply them.")
        elif error_at_19m > 0.05:
            lines.append("  -> worth applying; it is a visible slice of the budget.")
        else:
            lines.append("  -> negligible; the sensor is level enough to leave at 0.")
        lines.append("")

        median_fraction = float(np.median(fractions))
        median_sigma = float(np.median(sigmas))
        if median_fraction < self._min_inlier:
            lines.append(
                f"  WARNING: inlier fraction {median_fraction * 100:.1f} % is below "
                f"{self._min_inlier * 100:.0f} %."
            )
            # Rough ground and a competing surface both drop the fraction, but
            # they leave different residuals: roughness spreads the inliers too,
            # a second plane does not.
            if median_sigma > 0.6 * self._threshold:
                lines += [
                    f"  Residual sigma is {median_sigma * 1000:.0f} mm against a "
                    f"{self._threshold * 1000:.0f} mm threshold, so the surface is",
                    "  simply rough -- grass or gravel. The plane itself is fine; raise",
                    "  'ransac_threshold' to about 3x sigma to recover the fraction.",
                    "",
                ]
            else:
                lines += [
                    f"  Residual sigma is only {median_sigma * 1000:.0f} mm, so the "
                    "ground it did find is flat and the",
                    "  rejected points are a SECOND surface -- a fence, a wall or a",
                    "  vehicle behind the sensor. Narrow 'azimuth_min' / 'azimuth_max'",
                    "  to the field side and refit before trusting these numbers.",
                    "",
                ]

        # Both sensors stand on the same ground, so their two reports separate
        # a tilted sensor from a sloping field -- which roll and pitch alone
        # cannot do, since the two sensors' frames point different ways.
        if tilt > 0.2:
            lines += [
                "  run the other sensor too and compare the downhill azimuths:",
                "    ~180 deg apart (for sensors installed facing each other)",
                "        -> the FIELD slopes and both sensors are level. Nothing to",
                "           adjust; the world z axis is simply normal to the field.",
                "    unrelated",
                "        -> the SENSORS are tilted independently. Level the tripods,",
                "           or put these roll/pitch values in the YAML.",
                "",
            ]

        lines += [
            "  paste into lidar_localization_os0_indoor.yaml:",
            "",
            f"    {self._label}_z:     {height:.3f}",
            f"    {self._label}_roll:  {roll:.6f}",
            f"    {self._label}_pitch: {pitch:.6f}",
            "",
            "=" * 66,
        ]
        self.get_logger().info("\n".join(lines))

        if self._shutdown:
            raise SystemExit(0)

    def _on_timeout(self) -> None:
        """Report whatever was collected if the frame quota is never reached."""
        if self._done:
            return
        if self._results:
            self.get_logger().warn(
                f"timed out with {len(self._results)}/{self._num_frames} frames; "
                "reporting what was collected"
            )
            self._finish()
            return
        self.get_logger().error(
            f"no usable frame after {self._n_scans} scan(s) on '{self._topic}'. "
            "Is the driver publishing, and is expected_height roughly right?"
        )
        if self._shutdown:
            raise SystemExit(1)

    # ------------------------------------------------------------------

    @staticmethod
    def _voxel_downsample(points: np.ndarray, size: float) -> np.ndarray:
        """Average the points falling in each ``size`` cube."""
        idx = np.floor(points / size).astype(np.int64)
        _, inverse = np.unique(idx, axis=0, return_inverse=True)
        n = int(inverse.max()) + 1
        sums = np.zeros((n, 3), dtype=np.float64)
        np.add.at(sums, inverse, points)
        counts = np.bincount(inverse, minlength=n)
        return (sums / counts[:, None]).astype(np.float32)

    def _to_numpy(self, msg: PointCloud2) -> Optional[np.ndarray]:
        """Nx3 float32 in the sensor frame, non-finite points stripped.

        read_points returns a structured array; non-returning rays come back as
        +/-inf rather than NaN, so skip_nans does not remove them.
        """
        try:
            arr = pc2_util.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
            if arr is None or arr.size == 0:
                return None
            pts = np.stack([arr["x"], arr["y"], arr["z"]], axis=-1).astype(np.float32)
            pts = pts.reshape(-1, 3)
            pts = pts[np.isfinite(pts).all(axis=1)]
            return pts if len(pts) else None
        except Exception as exc:  # pragma: no cover - driver dependent
            self.get_logger().error(
                f"PointCloud2 read failed: {exc}", throttle_duration_sec=5.0)
            return None


def main(args: Optional[List[str]] = None) -> None:
    """Entry point for ``ros2 run grid_sensor_calibration ground_plane_node``."""
    rclpy.init(args=args)
    node = None
    try:
        node = GroundPlaneNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
