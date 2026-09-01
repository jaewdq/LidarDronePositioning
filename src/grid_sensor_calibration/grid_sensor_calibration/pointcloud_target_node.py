"""PointCloud2 -> PointStamped target extractor for live calibration.

``sensor_calibration_node`` consumes a single 2D measurement per capture, via
one of the adapters in :mod:`~grid_sensor_calibration.sensor_adapter`
(``PointStamped`` / ``PoseStamped`` / ``Point``). A 3D LiDAR publishes
``sensor_msgs/msg/PointCloud2`` instead, so nothing in this package can read it
directly -- the adapter docstring notes the real sensors were not chosen yet.

This node closes that gap. It crops the cloud to a box in the SENSOR frame,
clusters what is left, and republishes one cluster's centre as a
``PointStamped`` that ``sensor_calibration_node`` can capture:

    /lidarN/points  (PointCloud2, sensor frame)
        -> box crop  (+ optional range ring and azimuth window)
        -> DBSCAN
        -> pick cluster (largest, or nearest to box centre)
        -> /sensorN/target_point  (PointStamped, same sensor frame)

Why a box AND clustering, when the older extrinsic_calibrator used a box alone:
that tool took the median of every point inside the box, so the box had to
bound the target tightly and nothing else could be in it. Getting such a box
right needs the target's sensor-frame coordinates, which is what calibration is
supposed to find -- and in practice most captures failed because the box missed.
Clustering removes that circularity: the box only has to be roughly right, and
whatever stray wall or floor points come with it end up in other clusters.

The centre is a median, not a mean: a mean is dragged by a single stray point
in the cluster, a median barely moves.

Typical use, one node per sensor::

    ros2 run grid_sensor_calibration pointcloud_target_node --ros-args \\
        -p input_topic:=/lidar1/points \\
        -p output_topic:=/sensor1/target_point \\
        -p frame_id:=sensor1 \\
        -p box:='[0.5, 4.0, -2.0, 2.0, 0.0, 0.8]'

An x/y box has to be re-aimed for every sensor and every target position, since
the same target lands at a different (x, y) in each sensor's frame. Prefer the
rotation-invariant form: leave x/y wide, keep only the z band in the box, and
cut with the ring and the window instead -- the same numbers then work for both
sensors and for every grid point::

    ros2 run grid_sensor_calibration pointcloud_target_node --ros-args \\
        -p input_topic:=/lidar1/points \\
        -p output_topic:=/sensor1/target_point \\
        -p frame_id:=lidar1 \\
        -p box:='[-30.0, 30.0, -30.0, 30.0, -0.75, 0.0]' \\
        -p range_min:=1.0 -p range_max:=6.0 \\
        -p azimuth_min:=90.0 -p azimuth_max:=-90.0

Then point ``sensor_calibration_node`` at ``/sensor1/target_point``.
"""

from __future__ import annotations

from typing import List, Optional

import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2

try:
    from sensor_msgs_py import point_cloud2 as pc2_util
except ImportError:  # pragma: no cover - older distributions
    from sensor_msgs import point_cloud2 as pc2_util

try:
    from sklearn.cluster import DBSCAN
except ImportError as exc:  # pragma: no cover
    raise ImportError(
        "scikit-learn is required: pip3 install scikit-learn"
    ) from exc


def fit_circle_fixed_radius(
    xy: np.ndarray,
    radius: float,
    iterations: int = 30,
    tolerance: float = 1e-6,
) -> tuple:
    """Locate the axis of a cylinder of KNOWN radius from its visible arc.

    A LiDAR only ever sees the half of a cylinder that faces it, so the
    centroid of the returns sits about ``0.7 * radius`` in front of the true
    axis -- 12 cm for a 30 cm tube, biased consistently toward the sensor at
    every range. Because the bias points at the sensor it does not cancel
    across targets at different bearings: it shows up as a radial shrink of the
    whole target set, which a rigid-body fit cannot absorb, so it corrupts yaw
    and inflates the residuals rather than shifting the origin harmlessly.

    Fixing the radius leaves only the two centre coordinates to solve, which a
    half arc constrains perfectly well. Solved by Gauss-Newton on
    ``sum (||p_i - c|| - R)^2``; the Jacobian row for point ``i`` is the unit
    vector ``n_i`` from the centre toward the point.

    Args:
        xy: ``(N, 2)`` points on the visible surface, sensor frame.
        radius: Known cylinder radius [m].
        iterations: Maximum Gauss-Newton iterations.
        tolerance: Stop once the step is shorter than this [m].

    Returns:
        ``(centre, residual_rms)``; ``centre`` is the ``(2,)`` axis position
        and ``residual_rms`` the RMS of ``||p_i - c|| - R`` [m].
    """
    xy = np.asarray(xy, dtype=float)
    centre = xy.mean(axis=0)

    # Seed the search behind the visible surface: push the centroid directly
    # away from the sensor (the origin) by one radius.
    norm = float(np.linalg.norm(centre))
    if norm > 1e-9:
        centre = centre + radius * centre / norm

    for _ in range(int(iterations)):
        delta = xy - centre
        distance = np.linalg.norm(delta, axis=1)
        valid = distance > 1e-9
        if not np.any(valid):
            break
        normals = delta[valid] / distance[valid, None]
        error = distance[valid] - radius
        # Normal equations of the 2x2 Gauss-Newton system, with a small ridge
        # term so a very short arc cannot make it singular.
        hessian = normals.T @ normals + 1e-9 * np.eye(2)
        step = np.linalg.solve(hessian, normals.T @ error)
        centre = centre + step
        if float(np.linalg.norm(step)) < tolerance:
            break

    residual = np.linalg.norm(xy - centre, axis=1) - radius
    return centre, float(np.sqrt(np.mean(residual**2)))


class PointCloudTargetNode(Node):
    """Extract one target centre per scan and publish it as a PointStamped."""

    def __init__(self) -> None:
        super().__init__("pointcloud_target_node")

        self.declare_parameter("input_topic", "/lidar1/points")
        self.declare_parameter("output_topic", "/sensor1/target_point")
        # Frame stamped on the output. Leave empty to pass the cloud's own
        # frame through; set it to the calibration node's child_frame when the
        # driver's frame name differs from the sensor name used there.
        self.declare_parameter("frame_id", "")

        # Crop box in the SENSOR frame: [xmin, xmax, ymin, ymax, zmin, zmax].
        # Generous is fine -- clustering sorts out what the box lets in. Keep
        # the z band tight enough to drop floor and ceiling.
        # Defaults suit a 1.5 m tube standing on the ground, seen by a sensor
        # about 2 m up. In the SENSOR frame that puts the ground at z = -2.0
        # and the tube at z = -2.0 .. -0.5, so:
        #
        #   [0.0, 0.8]   sees nothing -- correct only for a ground-level sensor
        #   [-2.2, -0.4] contains the GROUND as well as the tube
        #
        # The second is the expensive mistake. On a clear field the ground is
        # the only other thing present, and it is ~35k points in the 5-15 m
        # ring: DBSCAN takes 188 ms on that against a 100 ms budget at 10 Hz,
        # where the tube alone takes 1.3 ms. The extent bound does reject the
        # ground cluster, but only after paying for it.
        #
        # So the band starts 0.3 m ABOVE the ground -- high enough to clear it
        # even if the sensor height is off by 0.2 m -- and stops below the top
        # of the tube.
        #
        # x and y are then symmetric and field-sized rather than aimed. The box
        # is in the SENSOR frame, and a sensor's x-axis can point anywhere: at
        # a yaw of -89.5 deg, as measured on this installation, every target
        # lands at negative x and a positive-x box sees nothing. Aiming the box
        # would need the heading that the calibration exists to find.
        self.declare_parameter(
            "box", [-30.0, 30.0, -30.0, 30.0, -1.7, -0.6])

        # Range ring and azimuth window in the sensor's xy plane, applied on
        # top of the box. Both are rotation-invariant about z, which is what
        # the box is not: an axis-aligned x/y box has to be re-aimed for every
        # sensor and every target position, because the same target sits at a
        # different (x, y) in each sensor's frame. A ring plus a window does
        # not -- "1 to 6 m out, in the half I face" is the same sentence for
        # both sensors and for all nine grid points.
        #
        # So the intended split is:
        #   box       -- z band only (floor and ceiling), x/y left wide open
        #   range_*   -- how far the targets can be
        #   azimuth_* -- which side of the sensor they are on
        #
        # 0 disables a range bound. The azimuth window is the same convention
        # as ground_plane_node -- degrees(atan2(y, x)), -180..180 -- and it may
        # wrap through +/-180 by giving azimuth_min > azimuth_max.
        self.declare_parameter("range_min", 0.0)
        self.declare_parameter("range_max", 0.0)
        self.declare_parameter("azimuth_min", -180.0)
        self.declare_parameter("azimuth_max", 180.0)

        self.declare_parameter("cluster_tolerance", 0.20)
        self.declare_parameter("cluster_min_points", 8)
        self.declare_parameter("cluster_max_points", 100000)
        # Largest side of a cluster's bounding box. A calibration target is
        # small; anything bigger is a wall fragment the box failed to exclude.
        # 2.0 clears a 1.5 m tube. Leaving this at 1.0 rejects the tube outright
        # -- its height, not its width, sets the extent.
        self.declare_parameter("cluster_max_extent", 2.0)

        # "largest"      -- most points. Right when the target is the only
        #                   thing in the box.
        # "nearest_box"  -- closest to the box centre. Right when the box is
        #                   aimed at the target but other things share it.
        self.declare_parameter("selection", "largest")

        # How the cluster is reduced to one point.
        #   centroid -- median of the points. Correct only for a target small
        #               enough that its own thickness does not matter.
        #   cylinder -- fit a circle of radius 'target_radius' to the xy
        #               projection and take its centre. Removes the ~0.7 R bias
        #               of a one-sided view; use this for the tube.
        self.declare_parameter("target_shape", "centroid")
        self.declare_parameter("target_radius", 0.15)
        # Reject a cylinder fit whose points do not lie on a circle of that
        # radius. This is the one filter that separates the tube from a PERSON:
        # extent lets a standing person through (they are shorter than the 2 m
        # bound) and 'largest' actively prefers them, since a bystander at 8 m
        # returns more points than the tube at 16 m. Simulated at 8-17 m, a
        # 45 x 28 cm torso fits a 15 cm circle to 30-35 mm while the tube fits
        # to 5 mm -- a 6x separation.
        #
        # 40 mm rather than something tighter because that 5 mm assumed 5 mm of
        # range noise, and the OS1's real precision on cardboard at 17 m is not
        # known here. Read the 'fit residual' the node logs for the actual tube
        # on the actual field and set this to about 3x it; if that lands above
        # ~50 mm the fit is no longer separating tube from person and the field
        # has to be kept clear instead.
        #
        # Note it does NOT catch a leaning tube: 5 deg of lean costs 7.5 cm of
        # centre error but only lifts the residual to 11 mm. Stand it plumb.
        self.declare_parameter("max_fit_residual", 0.04)

        # 0 disables downsampling. At close range a target can be thousands of
        # points and DBSCAN slows down; 0.02 keeps shape while thinning it.
        self.declare_parameter("voxel_size", 0.0)

        self.declare_parameter("log_period", 1.0)

        gp = lambda name: self.get_parameter(name).value  # noqa: E731
        self._input_topic: str = gp("input_topic")
        self._output_topic: str = gp("output_topic")
        self._frame_id: str = gp("frame_id")
        box = [float(v) for v in gp("box")]
        if len(box) != 6:
            raise ValueError(
                f"box needs 6 values [xmin,xmax,ymin,ymax,zmin,zmax], got {len(box)}")
        self._box = box
        self._range_min: float = float(gp("range_min"))
        self._range_max: float = float(gp("range_max"))
        self._az_min: float = float(gp("azimuth_min"))
        self._az_max: float = float(gp("azimuth_max"))
        self._eps: float = float(gp("cluster_tolerance"))
        self._min_pts: int = int(gp("cluster_min_points"))
        self._max_pts: int = int(gp("cluster_max_points"))
        self._max_extent: float = float(gp("cluster_max_extent"))
        self._selection: str = gp("selection")
        self._shape: str = gp("target_shape")
        self._radius: float = float(gp("target_radius"))
        self._max_fit_residual: float = float(gp("max_fit_residual"))
        if self._shape not in ("centroid", "cylinder"):
            self.get_logger().error(
                f"target_shape '{self._shape}' unknown; falling back to 'centroid'")
            self._shape = "centroid"
        self._voxel: float = float(gp("voxel_size"))
        self._log_period: float = float(gp("log_period"))

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self._pub = self.create_publisher(
            PointStamped, self._output_topic, 10)
        self.create_subscription(
            PointCloud2, self._input_topic, self._on_cloud, sensor_qos)

        self._last_log = 0.0
        self._n_scans = 0
        self._n_hits = 0
        self._last_residual: Optional[float] = None

        ring = ""
        if self._range_min > 0.0 or self._range_max > 0.0:
            hi = f"{self._range_max:.1f}" if self._range_max > 0.0 else "inf"
            ring += f" range[{self._range_min:.1f},{hi}]"
        if not (self._az_min <= -180.0 and self._az_max >= 180.0):
            ring += f" azimuth[{self._az_min:.1f},{self._az_max:.1f}]"

        self.get_logger().info(
            f"{self._input_topic} -> {self._output_topic}  "
            f"box x[{box[0]},{box[1]}] y[{box[2]},{box[3]}] z[{box[4]},{box[5]}]"
            f"{ring}  "
            f"eps={self._eps} min_pts={self._min_pts} "
            f"max_extent={self._max_extent} selection={self._selection}\n"
            + (f"  cylinder fit, radius {self._radius:.3f} m "
               f"(removes the ~{0.7 * self._radius * 100:.0f} cm one-sided-view bias)"
               if self._shape == "cylinder" else
               "  centroid mode: median of the cluster, no shape correction"))

    # ------------------------------------------------------------------
    def _on_cloud(self, msg: PointCloud2) -> None:
        self._n_scans += 1
        pts = self._to_numpy(msg)
        if pts is None:
            self._maybe_log()
            return

        x0, x1, y0, y1, z0, z1 = self._box
        keep = (
            (pts[:, 0] >= x0) & (pts[:, 0] <= x1)
            & (pts[:, 1] >= y0) & (pts[:, 1] <= y1)
            & (pts[:, 2] >= z0) & (pts[:, 2] <= z1)
        )

        if self._range_min > 0.0 or self._range_max > 0.0:
            radius = np.hypot(pts[:, 0], pts[:, 1])
            if self._range_min > 0.0:
                keep &= radius >= self._range_min
            if self._range_max > 0.0:
                keep &= radius <= self._range_max

        if not (self._az_min <= -180.0 and self._az_max >= 180.0):
            azimuth = np.degrees(np.arctan2(pts[:, 1], pts[:, 0]))
            if self._az_min <= self._az_max:
                keep &= (azimuth >= self._az_min) & (azimuth <= self._az_max)
            else:  # window wraps through +/-180
                keep &= (azimuth >= self._az_min) | (azimuth <= self._az_max)

        pts = pts[keep]
        if len(pts) < self._min_pts:
            self._maybe_log()
            return

        if self._voxel > 0.0:
            pts = self._voxel_downsample(pts, self._voxel)
            if len(pts) < self._min_pts:
                self._maybe_log()
                return

        centre = self._pick_cluster(pts)
        if centre is None:
            self._maybe_log()
            return

        out = PointStamped()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self._frame_id or msg.header.frame_id
        out.point.x = float(centre[0])
        out.point.y = float(centre[1])
        out.point.z = float(centre[2])
        self._pub.publish(out)
        self._n_hits += 1
        self._maybe_log(centre)

    # ------------------------------------------------------------------
    def _pick_cluster(self, pts: np.ndarray) -> Optional[np.ndarray]:
        labels = DBSCAN(
            eps=self._eps, min_samples=self._min_pts).fit(pts).labels_

        best = None
        box_centre = np.array([
            0.5 * (self._box[0] + self._box[1]),
            0.5 * (self._box[2] + self._box[3]),
            0.5 * (self._box[4] + self._box[5]),
        ])
        for label in np.unique(labels):
            if label < 0:
                continue
            cluster = pts[labels == label]
            n = len(cluster)
            if n < self._min_pts or n > self._max_pts:
                continue
            extent = float((cluster.max(0) - cluster.min(0)).max())
            if extent > self._max_extent:
                continue
            centre = self._cluster_centre(cluster)
            if centre is None:
                continue
            if self._selection == "nearest_box":
                score = -float(np.linalg.norm(centre - box_centre))
            else:
                score = float(n)
            if best is None or score > best[0]:
                best = (score, centre)
        return None if best is None else best[1]

    def _cluster_centre(self, cluster: np.ndarray) -> Optional[np.ndarray]:
        """Reduce one cluster to the target's centre.

        Args:
            cluster: ``(N, 3)`` points of a single cluster, sensor frame.

        Returns:
            The ``(3,)`` centre, or ``None`` when a cylinder fit was requested
            and the points do not lie on a circle of the configured radius.
        """
        if self._shape != "cylinder":
            return np.median(cluster, axis=0)

        centre_xy, residual = fit_circle_fixed_radius(cluster[:, :2], self._radius)
        self._last_residual = residual
        if residual > self._max_fit_residual:
            self.get_logger().warn(
                f"cylinder fit residual {residual * 100:.1f} cm exceeds "
                f"{self._max_fit_residual * 100:.0f} cm; cluster rejected "
                "(wrong object, or target_radius is wrong)",
                throttle_duration_sec=3.0,
            )
            return None
        # z is not constrained by the circle: the tube is vertical, so every
        # height is equally valid. Median of the visible surface is the
        # sensible summary and is only used as a diagnostic.
        return np.array(
            [centre_xy[0], centre_xy[1], float(np.median(cluster[:, 2]))])

    @staticmethod
    def _voxel_downsample(points: np.ndarray, size: float) -> np.ndarray:
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
            arr = pc2_util.read_points(
                msg, field_names=("x", "y", "z"), skip_nans=True)
            if arr is None or arr.size == 0:
                return None
            pts = np.stack(
                [arr["x"], arr["y"], arr["z"]], axis=-1).astype(np.float32)
            pts = pts.reshape(-1, 3)
            pts = pts[np.isfinite(pts).all(axis=1)]
            return pts if len(pts) else None
        except Exception as exc:  # pragma: no cover - driver dependent
            self.get_logger().error(
                f"PointCloud2 read failed: {exc}", throttle_duration_sec=5.0)
            return None

    def _maybe_log(self, centre: Optional[np.ndarray] = None) -> None:
        if self._log_period <= 0.0:
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self._last_log < self._log_period:
            return
        self._last_log = now
        if centre is None:
            self.get_logger().warn(
                f"no target in box ({self._n_hits}/{self._n_scans} scans hit); "
                "widen 'box' or lower 'cluster_min_points'")
        else:
            extra = ""
            if self._shape == "cylinder" and self._last_residual is not None:
                # Ranges from experience: a plumb tube of the configured radius
                # lands near 5 mm. Anything much larger means the points do not
                # share one circle -- almost always a leaning tube, whose axis
                # then drifts with height (a 2 deg lean over 1.5 m spreads the
                # per-height centres by ~5 cm and the reported centre is the
                # one at mid-height, not at the ground mark).
                extra = f"  fit residual {self._last_residual * 1000:.0f} mm"
                if self._last_residual > 0.020:
                    extra += "  <- high: leaning tube, or not the tube at all?"
            self.get_logger().info(
                f"target ({centre[0]:+.3f}, {centre[1]:+.3f}, {centre[2]:+.3f})  "
                f"{self._n_hits}/{self._n_scans} scans{extra}")


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = PointCloudTargetNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
