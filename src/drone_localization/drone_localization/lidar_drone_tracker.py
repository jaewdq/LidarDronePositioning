#!/usr/bin/env python3
"""
Dual-Ouster drone tracker (ground-fixed LiDARs, outside-in).

두 저장소가 같은 파일을 쓴다 — 고칠 때는 양쪽을 함께 맞출 것.
  - jaewdq/LidarDronePositioning   src/drone_localization/.../lidar_drone_tracker.py   (실제 운용)
  - jaewdq/DronePositioningSystem  ros2_ws/src/drone_localization/.../lidar_drone_tracker_cov.py (시뮬)

융합 구조 (fusion_mode) — 논문 표기
  cloud       PF 점군 융합   두 라이다 점을 합쳐 한 번에 클러스터링 → 관측 하나
  position    MF 관측 융합   라이다별 클러스터 위치를 3x3 공분산으로 가중 평균 → 관측 하나
  sequential  SU 순차 갱신   라이다별 관측을 촬영 시각 순서대로 KF에 차례로 넣음 (짝짓기 없음)
  PF/MF는 process_on_arrival=true 로 짝 스캔이 도착하는 즉시 처리하는 것이 논문 조건이다.

대회(2026-09, LidarDronePositioning 391f40a + 대회 당일 진단 기록)에서 바뀐 점
  - SU 추가, process_on_arrival / max_sensor_range 옵션 추가
  - 프로세스잡음: accel_noise_std(구간별 일정 가속도, dt^4/4) → process_noise_q
    (연속 백색 가속도). 갱신 간격과 무관하게 초당 같은 불확실성이 쌓여 PF/MF/SU가
    같은 운동 가정을 쓴다.
  - 측정 공분산의 외재 캘리브 RMSE 항 → 스캔 나이 항(age_noise_velocity x 수신 후
    경과시간). lidarN_extrinsic_rmse 는 이제 배경모델 서명에만 쓰인다.
  - 짝짓기 정체 수정: 후보 쌍의 dt가 같으면 가장 최신 쌍을 고른다
    (_select_unprocessed_pair, 시뮬에서 참값 대비 지연 0.18 s → 0.07 s).
  - 첫 관측 전에는 홈 시드 위치를 발행하지 않는다.
  - 대회 당일 추가한 라이다별 클러스터 점 수 기록(raw_detections.csv의
    cluster_points_<라이다> 칸, 2초 상태 로그)은 그대로 두고 SU 행에도 채운다.
  검증: 시뮬 bag 재생에서 PF/MF/SU 상태가 병합 전 시뮬 트래커와 비트 단위로 같다.

Architecture
------------
Each LiDAR is processed independently:

  /lidarN/points
      -> calibrated sensor-to-world transform
      -> ROI crop
      -> optional static-background subtraction
      -> optional voxel + statistical outlier filtering
      -> DBSCAN
      -> cluster validation
      -> per-LiDAR position + covariance

The two POSITION measurements are then time-checked, consistency-checked,
and fused using covariance/information weighting. The fused measurement is
fed to one 6-state constant-velocity Kalman filter.

Important design properties
---------------------------
* Point clouds are NEVER fused before drone detection.
* Each input frame is processed at most once.
* Small per-sensor timestamp queues are used to select the closest pair.
* Per-sensor world/filtered clouds and per-sensor pose estimates are published.
* Background files include an extrinsic-calibration signature and are rejected
  automatically if either sensor pose or the ROI changes.
* PTP is not configured by this node, but inter-sensor timestamp differences
  are continuously measured and reported.
* Fusion uses 3x3 measurement covariance, not point-count-only weighting.

Nominal physical layout
-----------------------
World frame:
  x: 0 ... 24 m
  y: 0 ... 15 m
  z: up

Nominal LiDAR mounting:
  lidar1 = (12, -1, 2.0), facing +Y
  lidar2 = (12, 16, 2.0), facing -Y

These nominal extrinsics are ONLY defaults. For final operation, replace them
through the YAML file with Kabsch/SVD-calibrated x,y,z,roll,pitch,yaw values.

Inputs
------
/lidar1/points                       sensor_msgs/msg/PointCloud2
/lidar2/points                       sensor_msgs/msg/PointCloud2
/competition/run_config              std_msgs/msg/String (JSON)
/competition/mission_event           std_msgs/msg/String

Outputs
-------
/lidar1/points_world                 sensor_msgs/msg/PointCloud2
/lidar2/points_world                 sensor_msgs/msg/PointCloud2
/lidar1/filtered_points              sensor_msgs/msg/PointCloud2
/lidar2/filtered_points              sensor_msgs/msg/PointCloud2
/drone/lidar1_pose                   geometry_msgs/msg/PoseWithCovarianceStamped
/drone/lidar2_pose                   geometry_msgs/msg/PoseWithCovarianceStamped
/drone/estimated_pose                geometry_msgs/msg/PoseWithCovarianceStamped
/lidar/cluster_markers               visualization_msgs/msg/MarkerArray
/drone/waypoint_estimates            visualization_msgs/msg/MarkerArray

Background learning
-------------------
Run once AFTER PTP setup and AFTER extrinsic calibration, with an empty field:

  ros2 run <pkg> <exe> --ros-args \
      -p learn_background:=true \
      -p use_background:=true

The node stores a calibration signature in the .npz background file. If the
extrinsics or ROI later change, the old background is ignored and must be
regenerated.
"""

import csv
import hashlib
import json
import math
import time
from collections import deque
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import rclpy
from builtin_interfaces.msg import Time as TimeMsg
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation as SciPyRotation
from sensor_msgs.msg import PointCloud2
from sklearn.cluster import DBSCAN
from std_msgs.msg import Header, String
from visualization_msgs.msg import Marker, MarkerArray

try:
    from sensor_msgs_py import point_cloud2 as pc2_util
except ImportError:
    from sensor_msgs import point_cloud2 as pc2_util


@dataclass
class Detection:
    sensor_index: int
    sensor_name: str
    position: np.ndarray
    covariance: np.ndarray
    cluster_points: int
    spread: float
    quality: float
    sensor_range: float
    stamp: object
    candidate_count: int


class KF6D:
    """Constant-velocity KF with state [x,y,z,vx,vy,vz]."""

    def __init__(self, process_noise_q: float, default_meas_noise_std: float):
        self.x = np.zeros(6, dtype=np.float64)
        self.P = np.eye(6, dtype=np.float64) * 9.0
        self.process_noise_q = float(process_noise_q)
        self.default_R = np.eye(3, dtype=np.float64) * float(default_meas_noise_std) ** 2
        self.H = np.zeros((3, 6), dtype=np.float64)
        self.H[0, 0] = self.H[1, 1] = self.H[2, 2] = 1.0
        self.initialized = False

    def init(self, pos: np.ndarray, pos_cov: np.ndarray | None = None):
        self.x[:3] = np.asarray(pos, dtype=np.float64)
        self.x[3:] = 0.0
        self.P = np.eye(6, dtype=np.float64) * 0.25
        if pos_cov is not None:
            self.P[:3, :3] = self._regularize_cov(pos_cov)
        self.initialized = True

    @staticmethod
    def _regularize_cov(cov: np.ndarray) -> np.ndarray:
        cov = np.asarray(cov, dtype=np.float64).reshape(3, 3)
        cov = 0.5 * (cov + cov.T)
        vals, vecs = np.linalg.eigh(cov)
        vals = np.clip(vals, 1e-6, 100.0)
        return vecs @ np.diag(vals) @ vecs.T

    def predict(self, dt: float):
        if not self.initialized:
            return
        dt = max(1e-3, min(1.0, float(dt)))
        F = np.eye(6, dtype=np.float64)
        F[0, 3] = dt
        F[1, 4] = dt
        F[2, 5] = dt

        self.x = F @ self.x
        self.P = F @ self.P @ F.T + self.process_noise(dt)

    def process_noise(self, dt: float) -> np.ndarray:
        # Continuous white-noise acceleration: the same uncertainty accumulates
        # per second regardless of update interval, so PF/MF/SU share one
        # motion assumption. The earlier dt⁴/4 form (piecewise-constant
        # acceleration) accumulated less per second at higher update rates.
        block = np.array([[dt**3 / 3.0, dt**2 / 2.0],
                          [dt**2 / 2.0, dt]], dtype=np.float64)
        return self.process_noise_q * np.kron(block, np.eye(3))

    def update(self, z: np.ndarray, R_meas: np.ndarray | None = None):
        if not self.initialized:
            self.init(z, R_meas)
            return
        Rm = self.default_R if R_meas is None else self._regularize_cov(R_meas)
        z = np.asarray(z, dtype=np.float64).reshape(3)
        innov = z - self.H @ self.x
        S = self.H @ self.P @ self.H.T + Rm
        K = self.P @ self.H.T @ np.linalg.solve(S, np.eye(3))
        self.x = self.x + K @ innov
        IKH = np.eye(6) - K @ self.H
        # Joseph form for numerical robustness.
        self.P = IKH @ self.P @ IKH.T + K @ Rm @ K.T

    @property
    def position(self):
        return self.x[:3].copy()

    @property
    def velocity(self):
        return self.x[3:].copy()

    @property
    def cov_pos(self):
        return self.P[:3, :3].copy()


class SensorCfg:
    """Fixed LiDAR pose and sensor-to-world rigid transform."""

    def __init__(self, name, topic, x, y, z, roll, pitch, yaw, extrinsic_rmse):
        self.name = str(name)
        self.topic = str(topic)
        self.x, self.y, self.z = float(x), float(y), float(z)
        self.roll, self.pitch, self.yaw = float(roll), float(pitch), float(yaw)
        self.extrinsic_rmse = max(0.0, float(extrinsic_rmse))
        self.R_s2w = SciPyRotation.from_euler(
            'xyz', [self.roll, self.pitch, self.yaw]
        ).as_matrix()
        self.t_s2w = np.array([self.x, self.y, self.z], dtype=np.float64)

    def to_world(self, pts_sensor: np.ndarray) -> np.ndarray:
        pts = np.asarray(pts_sensor, dtype=np.float64).reshape(-1, 3)
        return (self.R_s2w @ pts.T).T + self.t_s2w

    def signature_values(self):
        return [
            self.x, self.y, self.z,
            self.roll, self.pitch, self.yaw,
            self.extrinsic_rmse,
        ]


class LidarDroneTracker(Node):

    def __init__(self):
        super().__init__('lidar_drone_tracker')

        self._declare_parameters()
        self._read_parameters()
        self._build_sensors()

        self._kf = KF6D(
            process_noise_q=self.process_noise_q,
            default_meas_noise_std=self.meas_noise_pos,
        )
        if self.seed_at_home:
            self._kf.init(
                self.home_xyz.copy(),
                np.eye(3) * self.home_seed_std**2,
            )

        self._coast_count = 0
        self._last_process_monotonic = time.monotonic()

        # Last accepted measurement's sensor timestamp, and the monotonic
        # reading taken alongside it. Coasting output is stamped by advancing
        # the former with elapsed monotonic time, so every pose this node
        # publishes stays on the sensors' clock -- see _coasting_stamp().
        self._last_meas_stamp_s = None
        self._last_meas_monotonic = None

        # fusion_mode 'cloud' only: filtered per-sensor clouds of the current
        # frame, waiting to be concatenated and clustered together.
        self._merge_buffer = []

        # 이번 사이클에 라이다별 클러스터가 몇 점이었나. 못 잡았으면 None.
        # _fuse_detections 가 합산하며 버리는 값이라 여기 따로 붙잡아 둔다.
        self._sensor_cluster_pts = [None, None]
        self._last_cluster_label = None

        # Timestamped queues: each item = (stamp_sec, points_world, recv_time, stamp_msg)
        self._cloud_queues = [
            deque(maxlen=self.queue_depth),
            deque(maxlen=self.queue_depth),
        ]
        self._last_processed_stamp_ns = [None, None]

        # Background, separately per LiDAR.
        self._bg_points = [None, None]
        self._bg_trees = [None, None]
        self._bg_counts = [dict(), dict()]
        self._bg_frames_seen = [0, 0]
        self._background_saved = False
        self._background_signature = self._make_calibration_signature()
        self._load_background_if_available()

        # Timestamp diagnostics.
        self._sync_samples_ms = deque(maxlen=self.sync_stats_window)
        self._sync_last_log = time.monotonic()

        # Waypoint hover state.
        self._wp_active_id = None
        self._wp_samples = []
        self._wp_break_count = 0
        self._wp_visit_counts = {}
        self._wp_markers = MarkerArray()

        # Competition run / CSV state.
        self._run_active = False
        self._run_output_dir = None
        self._team_name = ''
        self._team_id = ''
        self._run_id = 0
        self._raw_csv_f = self._raw_csv_w = None
        self._traj_csv_f = self._traj_csv_w = None
        self._wp_csv_f = self._wp_csv_w = None
        self._raw_csv_path = self._traj_csv_path = self._wp_csv_path = None

        # Stats.
        self._stat_period = 2.0
        self._stat_t0 = time.monotonic()
        self._reset_stats()
        self._timing_t0 = time.monotonic()
        self._reset_timing_stats()

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self._subs = [
            self.create_subscription(
                PointCloud2, self._sensors[0].topic,
                lambda msg: self._store_cloud(0, msg), sensor_qos),
            self.create_subscription(
                PointCloud2, self._sensors[1].topic,
                lambda msg: self._store_cloud(1, msg), sensor_qos),
        ]

        self._run_config_sub = self.create_subscription(
            String, '/competition/run_config', self._run_config_callback, 10)
        self._event_sub = self.create_subscription(
            String, '/competition/mission_event', self._mission_event_callback, 20)

        # Final pose and per-sensor diagnostic outputs.
        self._pub_pose = self.create_publisher(
            PoseWithCovarianceStamped, '/drone/estimated_pose', reliable_qos)
        # Position and KF velocity in one message (same stamp as /drone/estimated_pose),
        # so a logger never has to pair two topics.
        self._pub_odom = self.create_publisher(
            Odometry, '/drone/estimated_odometry', reliable_qos)
        self._pub_sensor_pose = [
            self.create_publisher(
                PoseWithCovarianceStamped, '/drone/lidar1_pose', reliable_qos),
            self.create_publisher(
                PoseWithCovarianceStamped, '/drone/lidar2_pose', reliable_qos),
        ]
        self._pub_world = [
            self.create_publisher(PointCloud2, '/lidar1/points_world', sensor_qos),
            self.create_publisher(PointCloud2, '/lidar2/points_world', sensor_qos),
        ]
        self._pub_filtered = [
            self.create_publisher(PointCloud2, '/lidar1/filtered_points', sensor_qos),
            self.create_publisher(PointCloud2, '/lidar2/filtered_points', sensor_qos),
        ]
        self._pub_markers = self.create_publisher(
            MarkerArray, '/lidar/cluster_markers', reliable_qos)
        self._pub_waypoints = self.create_publisher(
            MarkerArray, '/drone/waypoint_estimates', reliable_qos)

        self._proc_timer = self.create_timer(self._dt, self._process)

        names = ', '.join(
            f'{c.name}({c.topic} @ {c.x:.3f},{c.y:.3f},{c.z:.3f}; '
            f'rpy={c.roll:.5f},{c.pitch:.5f},{c.yaw:.5f}; '
            f'extrinsic_rmse={c.extrinsic_rmse:.3f}m)'
            for c in self._sensors)
        self.get_logger().info(
            f'Final dual-LiDAR tracker ready: {names}; '
            f'update={self.update_rate:.1f} Hz; fusion_mode={self.fusion_mode}')
        self.get_logger().info(
            f'Field ROI: x=[{self.roi_x_min:.2f},{self.roi_x_max:.2f}], '
            f'y=[{self.roi_y_min:.2f},{self.roi_y_max:.2f}], '
            f'z=[{self.roi_z_min:.2f},{self.roi_z_max:.2f}] frame={self.world_frame}')
        self.get_logger().info(
            f'PTP/scan matching tolerance={self.sync_tolerance*1000.0:.1f} ms; '
            f'agreement gate={self.sensor_agreement_dist:.2f} m')
        self.get_logger().info(
            f'Background signature={self._background_signature[:12]}...; '
            f'use_background={self.use_background}; learn_background={self.learn_background}')
        if self.learn_background:
            self.get_logger().warn(
                'BACKGROUND LEARNING MODE: field must be static and empty; '
                f'collecting {self.background_frames} frames PER LiDAR.')
        if self.warn_nominal_extrinsics:
            self._warn_if_nominal_extrinsics()

    # ------------------------------------------------------------------
    # Parameters
    # ------------------------------------------------------------------
    def _declare_parameters(self):
        # Nominal competition-field extrinsics. Replace via YAML after Kabsch calibration.
        self.declare_parameter('lidar1_topic', '/lidar1/points')
        self.declare_parameter('lidar1_x', 12.0)
        self.declare_parameter('lidar1_y', -1.0)
        self.declare_parameter('lidar1_z', 2.0)
        self.declare_parameter('lidar1_roll', 0.0)
        self.declare_parameter('lidar1_pitch', 0.0)
        self.declare_parameter('lidar1_yaw', math.pi / 2.0)
        self.declare_parameter('lidar1_extrinsic_rmse', 0.05)

        self.declare_parameter('lidar2_topic', '/lidar2/points')
        self.declare_parameter('lidar2_x', 12.0)
        self.declare_parameter('lidar2_y', 16.0)
        self.declare_parameter('lidar2_z', 2.0)
        self.declare_parameter('lidar2_roll', 0.0)
        self.declare_parameter('lidar2_pitch', 0.0)
        self.declare_parameter('lidar2_yaw', -math.pi / 2.0)
        self.declare_parameter('lidar2_extrinsic_rmse', 0.05)
        self.declare_parameter('warn_nominal_extrinsics', True)

        # Timing and timestamp pairing.
        self.declare_parameter('lidar_update_rate', 10.0)
        self.declare_parameter('buffer_stale_timeout', 0.30)
        self.declare_parameter('queue_depth', 8)
        self.declare_parameter('sync_tolerance_sec', 0.010)
        # >0: drop points farther than this from their sensor (emulates a dark
        # target's shorter detection range on a recorded bag). 0 = off.
        self.declare_parameter('max_sensor_range', 0.0)
        # True (논문 조건): process a scan pair as soon as it arrives instead of
        # waiting for the next timer tick; the timer then only flushes an
        # unpaired scan. False waits up to one tick (0.1 s) -- PF/MF only.
        self.declare_parameter('process_on_arrival', True)

        # 'sequential' (SU, default): filter each sensor's detection on its own,
        #   in scan-time order, never paired. Lowest error in the paper runs.
        # 'position' (MF): cluster each sensor separately, then fuse
        #   the two POSITIONS by covariance weight. Point clouds are never
        #   concatenated, so each detection keeps its own range and R.
        # 'cloud' (PF): concatenate both filtered clouds and cluster once. Use
        #   when the target is too sparse for either sensor alone -- see
        #   _detect_from_merged_cloud for what it gives up.
        self.declare_parameter('fusion_mode', 'sequential')
        self.declare_parameter('sync_stats_window', 200)
        self.declare_parameter('sensor_agreement_dist', 0.35)

        # 24 x 15 m competition field and 1-3 m flight envelope.
        self.declare_parameter('roi_x_min', 0.0)
        self.declare_parameter('roi_x_max', 24.0)
        self.declare_parameter('roi_y_min', 0.0)
        self.declare_parameter('roi_y_max', 15.0)
        self.declare_parameter('roi_z_min', 0.8)
        self.declare_parameter('roi_z_max', 3.2)
        self.declare_parameter('world_frame', 'world')

        # A competition field is generally not a closed rectangular room.
        self.declare_parameter('remove_boundary_surfaces', False)
        self.declare_parameter('boundary_surface_margin', 0.10)

        # Static background model. Regenerate after any extrinsic/ROI change.
        self.declare_parameter('use_background', True)
        self.declare_parameter('learn_background', False)
        self.declare_parameter(
            'background_file',
            str(Path('~/competition_results/calibration/dual_ouster_background.npz').expanduser()))
        self.declare_parameter('background_frames', 200)
        self.declare_parameter('background_voxel_size', 0.05)
        self.declare_parameter('background_distance', 0.08)
        self.declare_parameter('background_min_observation_ratio', 0.70)

        # Point filtering.
        self.declare_parameter('voxel_leaf_size', 0.04)
        self.declare_parameter('sor_mean_k', 10)
        self.declare_parameter('sor_std_ratio', 2.0)
        self.declare_parameter('publish_world_clouds', True)
        self.declare_parameter('publish_filtered_clouds', True)
        self.declare_parameter('debug_timing', False)
        self.declare_parameter('debug_timing_period', 2.0)

        # DBSCAN / cluster shape.
        self.declare_parameter('cluster_tolerance', 0.22)
        self.declare_parameter('cluster_min_points', 5)
        self.declare_parameter('cluster_max_points', 2000)
        self.declare_parameter('cluster_max_extent', 1.20)
        self.declare_parameter('cluster_min_extent', 0.02)

        # Per-detection covariance model.
        self.declare_parameter('meas_noise_pos', 0.10)
        self.declare_parameter('centroid_noise_floor', 0.025)
        self.declare_parameter('range_noise_per_meter', 0.002)
        # Age term: a detection describes where the drone was when the scan was
        # taken, not where it is when the filter consumes it. The uncertainty
        # that adds is not the elapsed time itself but the distance the target
        # could have covered in it, so the age is scaled by a reference speed
        # (default = the mission's horizontal cruise, 1.5 m/s). Age is measured
        # on the monotonic clock (receive -> use), never by differencing header
        # stamps against it -- see _coasting_stamp() for what mixing clock bases
        # costs. This replaces the extrinsic-RMSE term: on an outdoor field a
        # Kabsch/SVD calibration lands well inside the other terms, while
        # staleness is the error source that actually moves.
        self.declare_parameter('age_noise_velocity', 1.5)
        self.declare_parameter('min_detection_std', 0.03)
        self.declare_parameter('max_detection_std', 0.50)

        # KF / association.
        self.declare_parameter('process_noise_q', 0.4)
        self.declare_parameter('seed_at_home', True)
        self.declare_parameter('home_x', 0.0)
        self.declare_parameter('home_y', 0.0)
        self.declare_parameter('home_z', 1.0)
        self.declare_parameter('home_seed_std', 0.50)
        self.declare_parameter('max_assoc_dist', 1.75)
        self.declare_parameter('gate_sigma_k', 3.0)
        self.declare_parameter('max_assoc_dist_cap', 4.0)
        self.declare_parameter('coast_max_frames', 10)
        self.declare_parameter('reset_after_frames', 50)

        # Waypoint estimation (optional; override with actual competition markers).
        self.declare_parameter('enable_waypoint_tracking', True)
        self.declare_parameter('waypoint_gate_radius', 1.0)
        self.declare_parameter('waypoint_speed_threshold', 0.25)
        self.declare_parameter('waypoint_min_frames', 4)
        self.declare_parameter('waypoint_grace_frames', 5)
        # Default marker coordinates are only valid placeholders inside 24x15 field.
        self.declare_parameter('marker1_x', 5.0)
        self.declare_parameter('marker1_y', 4.0)
        self.declare_parameter('marker2_x', 5.0)
        self.declare_parameter('marker2_y', 11.0)
        self.declare_parameter('marker3_x', 19.0)
        self.declare_parameter('marker3_y', 4.0)
        self.declare_parameter('marker4_x', 12.0)
        self.declare_parameter('marker4_y', 11.0)

    def _read_parameters(self):
        gp = lambda n: self.get_parameter(n).value
        self.update_rate = float(gp('lidar_update_rate'))
        self._dt = 1.0 / max(1e-3, self.update_rate)
        self.buffer_stale_timeout = float(gp('buffer_stale_timeout'))
        self.queue_depth = max(2, int(gp('queue_depth')))
        self.sync_tolerance = max(0.0, float(gp('sync_tolerance_sec')))
        self._process_on_arrival = bool(gp('process_on_arrival'))
        self._max_sensor_range = float(gp('max_sensor_range'))
        self.fusion_mode = str(gp('fusion_mode')).strip().lower()
        if self.fusion_mode not in ('position', 'cloud', 'sequential'):
            self.get_logger().warn(
                f"unknown fusion_mode '{self.fusion_mode}'; using 'sequential'")
            self.fusion_mode = 'sequential'
        # SU: every scan is filtered as soon as it arrives, never paired.
        if self.fusion_mode == 'sequential':
            self._process_on_arrival = True
        self.sync_stats_window = max(20, int(gp('sync_stats_window')))
        self.sensor_agreement_dist = float(gp('sensor_agreement_dist'))
        self.warn_nominal_extrinsics = bool(gp('warn_nominal_extrinsics'))

        self.roi_x_min = float(gp('roi_x_min'))
        self.roi_x_max = float(gp('roi_x_max'))
        self.roi_y_min = float(gp('roi_y_min'))
        self.roi_y_max = float(gp('roi_y_max'))
        self.roi_z_min = float(gp('roi_z_min'))
        self.roi_z_max = float(gp('roi_z_max'))
        self.world_frame = str(gp('world_frame'))
        self.remove_boundary_surfaces = bool(gp('remove_boundary_surfaces'))
        self.boundary_surface_margin = float(gp('boundary_surface_margin'))

        self.use_background = bool(gp('use_background'))
        self.learn_background = bool(gp('learn_background'))
        self.background_file = str(gp('background_file'))
        self.background_frames = max(1, int(gp('background_frames')))
        self.background_voxel_size = float(gp('background_voxel_size'))
        self.background_distance = float(gp('background_distance'))
        self.background_min_obs_ratio = float(gp('background_min_observation_ratio'))

        self.voxel_leaf_size = float(gp('voxel_leaf_size'))
        self.sor_k = int(gp('sor_mean_k'))
        self.sor_std = float(gp('sor_std_ratio'))
        self.publish_world_clouds = bool(gp('publish_world_clouds'))
        self.publish_filtered_clouds = bool(gp('publish_filtered_clouds'))
        self.debug_timing = bool(gp('debug_timing'))
        self.debug_timing_period = max(0.5, float(gp('debug_timing_period')))

        self.eps = float(gp('cluster_tolerance'))
        self.min_pts = int(gp('cluster_min_points'))
        self.max_pts = int(gp('cluster_max_points'))
        self.cluster_max_extent = float(gp('cluster_max_extent'))
        self.cluster_min_extent = float(gp('cluster_min_extent'))

        self.meas_noise_pos = float(gp('meas_noise_pos'))
        self.centroid_noise_floor = float(gp('centroid_noise_floor'))
        self.range_noise_per_meter = float(gp('range_noise_per_meter'))
        self.age_noise_velocity = max(0.0, float(gp('age_noise_velocity')))
        self.min_detection_std = float(gp('min_detection_std'))
        self.max_detection_std = float(gp('max_detection_std'))

        self.process_noise_q = float(gp('process_noise_q'))
        self.seed_at_home = bool(gp('seed_at_home'))
        self.home_xyz = np.array(
            [gp('home_x'), gp('home_y'), gp('home_z')], dtype=np.float64)
        self.home_seed_std = float(gp('home_seed_std'))
        self.max_assoc_dist = float(gp('max_assoc_dist'))
        self.gate_sigma_k = float(gp('gate_sigma_k'))
        self.max_assoc_dist_cap = float(gp('max_assoc_dist_cap'))
        self.coast_max = int(gp('coast_max_frames'))
        self.reset_after = int(gp('reset_after_frames'))

        self.enable_waypoint_tracking = bool(gp('enable_waypoint_tracking'))
        self.waypoint_gate_radius = float(gp('waypoint_gate_radius'))
        self.waypoint_speed_threshold = float(gp('waypoint_speed_threshold'))
        self.waypoint_min_frames = int(gp('waypoint_min_frames'))
        self.waypoint_grace_frames = int(gp('waypoint_grace_frames'))
        self._marker_xy = {
            1: (float(gp('marker1_x')), float(gp('marker1_y'))),
            2: (float(gp('marker2_x')), float(gp('marker2_y'))),
            3: (float(gp('marker3_x')), float(gp('marker3_y'))),
            4: (float(gp('marker4_x')), float(gp('marker4_y'))),
        }

    def _build_sensors(self):
        gp = lambda n: self.get_parameter(n).value
        self._sensors = [
            SensorCfg(
                'lidar1', gp('lidar1_topic'),
                gp('lidar1_x'), gp('lidar1_y'), gp('lidar1_z'),
                gp('lidar1_roll'), gp('lidar1_pitch'), gp('lidar1_yaw'),
                gp('lidar1_extrinsic_rmse')),
            SensorCfg(
                'lidar2', gp('lidar2_topic'),
                gp('lidar2_x'), gp('lidar2_y'), gp('lidar2_z'),
                gp('lidar2_roll'), gp('lidar2_pitch'), gp('lidar2_yaw'),
                gp('lidar2_extrinsic_rmse')),
        ]

    def _warn_if_nominal_extrinsics(self):
        nominal = [
            np.array([12.0, -1.0, 2.0, 0.0, 0.0, math.pi / 2.0]),
            np.array([12.0, 16.0, 2.0, 0.0, 0.0, -math.pi / 2.0]),
        ]
        for i, s in enumerate(self._sensors):
            cur = np.array([s.x, s.y, s.z, s.roll, s.pitch, s.yaw])
            if np.allclose(cur, nominal[i], atol=1e-7, rtol=0.0):
                self.get_logger().warn(
                    f'{s.name}: using NOMINAL extrinsics. Replace YAML values with '
                    'Kabsch-calibrated x,y,z,roll,pitch,yaw before final scoring.')

    # ------------------------------------------------------------------
    # Background model with calibration signature
    # ------------------------------------------------------------------
    def _make_calibration_signature(self) -> str:
        payload = {
            'world_frame': self.world_frame,
            'roi': [
                self.roi_x_min, self.roi_x_max,
                self.roi_y_min, self.roi_y_max,
                self.roi_z_min, self.roi_z_max,
            ],
            'sensors': [s.signature_values() for s in self._sensors],
            'background_voxel_size': self.background_voxel_size,
        }
        raw = json.dumps(payload, sort_keys=True, separators=(',', ':')).encode('utf-8')
        return hashlib.sha256(raw).hexdigest()

    def _load_background_if_available(self):
        if not self.use_background or self.learn_background:
            return
        p = Path(self.background_file).expanduser()
        if not p.exists():
            self.get_logger().warn(
                f'Background file not found: {p}. Continuing without learned background.')
            return
        try:
            data = np.load(str(p), allow_pickle=False)
            stored_sig = ''
            if 'calibration_signature' in data:
                stored_sig = str(np.asarray(data['calibration_signature']).item())
            if stored_sig != self._background_signature:
                self.get_logger().error(
                    'Background model calibration signature does not match current '
                    'extrinsics/ROI. Ignoring old background; regenerate it.')
                return
            for i, cfg in enumerate(self._sensors):
                key = cfg.name
                if key not in data:
                    self.get_logger().warn(f'No {key} background in {p}')
                    continue
                pts = np.asarray(data[key], dtype=np.float32).reshape(-1, 3)
                if len(pts):
                    self._bg_points[i] = pts
                    self._bg_trees[i] = cKDTree(pts)
                    self.get_logger().info(
                        f'Loaded {len(pts)} background voxels for {cfg.name}')
        except Exception as exc:
            self.get_logger().error(f'Failed to load background model {p}: {exc}')

    def _accumulate_background(self, sensor_index, pts_world):
        if len(pts_world) == 0:
            return
        voxel = max(1e-3, self.background_voxel_size)
        keys = np.floor(pts_world / voxel).astype(np.int32)
        keys = np.unique(keys, axis=0)
        counts = self._bg_counts[sensor_index]
        for k in map(tuple, keys.tolist()):
            counts[k] = counts.get(k, 0) + 1
        self._bg_frames_seen[sensor_index] += 1

    def _background_learning_complete(self):
        return all(n >= self.background_frames for n in self._bg_frames_seen)

    def _finalize_background_learning(self):
        if self._background_saved or not self._background_learning_complete():
            return
        voxel = max(1e-3, self.background_voxel_size)
        payload = {
            'calibration_signature': np.array(self._background_signature),
        }
        for i, cfg in enumerate(self._sensors):
            min_count = max(
                1,
                int(math.ceil(
                    self.background_min_obs_ratio * self._bg_frames_seen[i])))
            kept = [k for k, c in self._bg_counts[i].items() if c >= min_count]
            if kept:
                arr = np.asarray(kept, dtype=np.float32)
                pts = (arr + 0.5) * voxel
            else:
                pts = np.empty((0, 3), dtype=np.float32)
            payload[cfg.name] = pts.astype(np.float32)
            self._bg_points[i] = payload[cfg.name]
            if len(pts):
                self._bg_trees[i] = cKDTree(pts)
            self.get_logger().info(
                f'Background {cfg.name}: {len(pts)} persistent voxels retained.')
        p = Path(self.background_file).expanduser()
        p.parent.mkdir(parents=True, exist_ok=True)
        np.savez_compressed(str(p), **payload)
        self._background_saved = True
        self.learn_background = False
        self.get_logger().info(f'Background calibration COMPLETE: {p}')

    def _subtract_background(self, sensor_index, points):
        tree = self._bg_trees[sensor_index]
        if not self.use_background or tree is None or len(points) == 0:
            return points
        dists, _ = tree.query(points, k=1)
        return points[dists > self.background_distance]

    # ------------------------------------------------------------------
    # Competition run management
    # ------------------------------------------------------------------
    def _run_config_callback(self, msg):
        try:
            cfg = json.loads(msg.data)
            team_name = str(cfg['team_name']).strip()
            team_id = str(cfg['team_id']).strip()
            run_id = int(cfg['run_id'])
            output_dir = Path(cfg['output_dir']).expanduser()
            if not team_name or not team_id:
                raise ValueError('team_name and team_id must not be empty')
        except Exception as exc:
            self.get_logger().error(f'Invalid /competition/run_config: {exc}')
            return

        if self._run_active:
            self._finalize_waypoint_episode()
        self._close_run_logs()

        output_dir.mkdir(parents=True, exist_ok=True)
        self._run_output_dir = output_dir
        self._team_name = team_name
        self._team_id = team_id
        self._run_id = run_id

        try:
            self._raw_csv_path = output_dir / 'raw_detections.csv'
            self._raw_csv_f = open(self._raw_csv_path, 'w', newline='')
            self._raw_csv_w = csv.writer(self._raw_csv_f)
            raw_header = [
                'time_s', 'x_m', 'y_m', 'z_m',
                'cov_x', 'cov_y', 'cov_z',
                'cluster_points', 'candidate_clusters',
                'sensor_count', 'sync_dt_ms']
            # 라이다별 클러스터 점 개수. 그 라이다가 이 프레임에서 드론을
            # 못 잡았으면 빈칸이다. 두 칸이 다 차면 합이 cluster_points 와
            # 같고, 한쪽만 채택된 프레임에서는 채택된 쪽과만 같다.
            raw_header += [f'cluster_points_{cfg.name}' for cfg in self._sensors]
            self._raw_csv_w.writerow(raw_header)

            self._traj_csv_path = output_dir / 'drone_trajectory.csv'
            self._traj_csv_f = open(self._traj_csv_path, 'w', newline='')
            self._traj_csv_w = csv.writer(self._traj_csv_f)
            self._traj_csv_w.writerow([
                'time_s', 'x_m', 'y_m', 'z_m',
                'vx_mps', 'vy_mps', 'vz_mps',
                'cov_x', 'cov_y', 'cov_z'])

            self._wp_csv_path = output_dir / 'waypoint_estimates.csv'
            self._wp_csv_f = open(self._wp_csv_path, 'w', newline='')
            self._wp_csv_w = csv.writer(self._wp_csv_f)
            self._wp_csv_w.writerow([
                'marker_id', 'visit', 'x', 'y', 'z', 'n_samples', 'duration_s'])

            self._raw_csv_f.flush()
            self._traj_csv_f.flush()
            self._wp_csv_f.flush()
        except Exception as exc:
            self.get_logger().error(f'Cannot open run CSV files: {exc}')
            self._close_run_logs()
            return

        self._wp_active_id = None
        self._wp_samples = []
        self._wp_break_count = 0
        self._wp_visit_counts = {}
        self._wp_markers = MarkerArray()
        self._coast_count = 0
        if self.seed_at_home:
            self._kf.init(
                self.home_xyz.copy(),
                np.eye(3) * self.home_seed_std**2)
        else:
            self._kf.initialized = False
        self._run_active = False
        self.get_logger().info(
            f'Run configured: team={team_name}, id={team_id}, run={run_id}; '
            f'output={output_dir}')

    def _mission_event_callback(self, msg):
        event = msg.data.strip().upper()
        if event == 'START':
            if self._run_output_dir is None:
                self.get_logger().warn(
                    'START received before /competition/run_config; CSV logging inactive.')
            else:
                self._run_active = True
                self.get_logger().info(
                    f'Logging STARTED for {self._team_name} ({self._team_id}), '
                    f'run {self._run_id}')
        elif event in ('FINISH', 'ABORT'):
            if self._run_active:
                self._finalize_waypoint_episode()
            self._run_active = False
            self._close_run_logs()
            self.get_logger().info(
                f'Run logging closed on {event}: {self._team_name} '
                f'({self._team_id}), run {self._run_id}')

    def _close_run_logs(self):
        for attr in ('_raw_csv_f', '_traj_csv_f', '_wp_csv_f'):
            f = getattr(self, attr, None)
            if f is not None:
                try:
                    f.flush()
                    f.close()
                except Exception:
                    pass
                setattr(self, attr, None)
        self._raw_csv_w = self._traj_csv_w = self._wp_csv_w = None

    # ------------------------------------------------------------------
    # Input buffering and timestamp pairing
    # ------------------------------------------------------------------
    def _store_cloud(self, sensor_index, msg):
        if self.debug_timing:
            t0 = time.perf_counter()
            pts = self._pc2_to_numpy(msg)
            t1 = time.perf_counter()
            if pts is None or len(pts) == 0:
                return
            pts_w = self._sensors[sensor_index].to_world(pts)
            t2 = time.perf_counter()
            self._record_store_timing(
                sensor_index, len(pts),
                pc2_ms=(t1 - t0) * 1000.0,
                tf_ms=(t2 - t1) * 1000.0,
                total_ms=(t2 - t0) * 1000.0,
            )
        else:
            pts = self._pc2_to_numpy(msg)
            if pts is None or len(pts) == 0:
                return
            pts_w = self._sensors[sensor_index].to_world(pts)
        if self._max_sensor_range > 0.0:
            d = np.linalg.norm(pts_w - self._sensors[sensor_index].t_s2w, axis=1)
            pts_w = pts_w[d <= self._max_sensor_range]
        stamp_s = self._stamp_to_seconds(msg.header.stamp)
        recv_t = time.monotonic()
        q = self._cloud_queues[sensor_index]
        # Ignore duplicate incoming timestamp.
        if q and abs(q[-1][0] - stamp_s) < 1e-12:
            return
        q.append((stamp_s, pts_w, recv_t, msg.header.stamp))
        if self._process_on_arrival:
            self._process()

    def _select_unprocessed_pair(self, now):
        """Return [(sensor_index, pts, stamp_msg, recv_t), ...], <=1 cloud/sensor.

        recv_t is the monotonic reading taken when the cloud arrived; the caller
        turns it into the detection's age for the covariance's age term.

        If both sensors have fresh unprocessed frames, choose the pair with minimum
        |t1-t2|. If only one sensor has an unprocessed fresh frame, return it after
        it has waited approximately sync_tolerance so a matching frame can arrive.
        """
        fresh = []
        for i, q in enumerate(self._cloud_queues):
            candidates = []
            for stamp_s, pts, recv_t, stamp_msg in q:
                stamp_ns = int(round(stamp_s * 1e9))
                if (self._last_processed_stamp_ns[i] is not None and
                        stamp_ns <= self._last_processed_stamp_ns[i]):
                    continue
                if now - recv_t > self.buffer_stale_timeout:
                    continue
                candidates.append((stamp_s, pts, recv_t, stamp_msg))
            fresh.append(candidates)

        if self.fusion_mode == 'sequential':
            frames = sorted(((c[0], i, c) for i in (0, 1) for c in fresh[i]),
                            key=lambda f: f[0])
            for stamp_s, i, _ in frames:
                self._last_processed_stamp_ns[i] = int(round(stamp_s * 1e9))
            return [(i, c[1], c[3], c[2]) for _, i, c in frames]

        if fresh[0] and fresh[1]:
            best = None
            for a in fresh[0]:
                for b in fresh[1]:
                    d = abs(a[0] - b[0])
                    # <= (not <): 스캔이 완벽히 동기화된 소스(sim)에서는 모든
                    # 후보 쌍의 dt가 0으로 동점이 난다. 엄격한 부등호면 동점을
                    # 절대 안 바꾸므로 fresh[i]가 오래된 것부터 최신 순으로
                    # 쌓이는 큐 순회 특성상 '가장 오래된' 쌍이 계속 이겨,
                    # 매 틱 새로 도착한 쌍을 두고도 영구적인 FIFO 정체가
                    # 생긴다(실측 참값 대비 지연 0.18s -> 0.07s로 개선,
                    # A와 동일해짐). <=로 동점이면 나중에(=더 최신) 찾은
                    # 쌍이 이기도록 해 최신 쌍을 우선한다 — 바로 아래
                    # '한쪽만 있을 때' 분기가 이미 최신 우선인 것과도 맞춘다.
                    if best is None or d <= best[0]:
                        best = (d, a, b)
            dt, a, b = best
            # Mark selected frames processed even if the dt is larger than the
            # fusion tolerance; they can still be processed independently.
            self._last_processed_stamp_ns[0] = int(round(a[0] * 1e9))
            self._last_processed_stamp_ns[1] = int(round(b[0] * 1e9))
            self._record_sync_sample(dt)
            return [(0, a[1], a[3], a[2]), (1, b[1], b[3], b[2])]

        # One-sensor fallback. Prefer newest fresh unprocessed frame.
        for i in (0, 1):
            if fresh[i]:
                a = fresh[i][-1]
                # Briefly wait for its partner unless it is old enough already.
                if now - a[2] < max(self.sync_tolerance, 0.002):
                    return []
                self._last_processed_stamp_ns[i] = int(round(a[0] * 1e9))
                return [(i, a[1], a[3], a[2])]
        return []

    def _record_sync_sample(self, dt_sec):
        ms = float(dt_sec) * 1000.0
        self._sync_samples_ms.append(ms)
        now = time.monotonic()
        if now - self._sync_last_log >= 5.0 and self._sync_samples_ms:
            arr = np.asarray(self._sync_samples_ms, dtype=np.float64)
            self.get_logger().info(
                f'PTP/scan timestamp delta: median={np.median(arr):.3f} ms, '
                f'p95={np.percentile(arr,95):.3f} ms, max={np.max(arr):.3f} ms '
                f'(limit={self.sync_tolerance*1000.0:.1f} ms)')
            self._sync_last_log = now

    # ------------------------------------------------------------------
    # Main processing
    # ------------------------------------------------------------------
    def _process(self):
        now = time.monotonic()
        process_t0 = time.perf_counter() if self.debug_timing else None
        selected = self._select_unprocessed_pair(now)
        if not selected:
            if self.debug_timing:
                self._timing_no_frame += 1
            self._maybe_log_stats(now)
            self._maybe_log_timing(now)
            return

        dt = max(1e-3, min(0.5, now - self._last_process_monotonic))
        self._last_process_monotonic = now

        if self._kf.initialized:
            self._kf.predict(dt)
            predicted = self._kf.position
        else:
            predicted = None

        detections = []
        self._merge_buffer = []
        self._sensor_cluster_pts = [None] * len(self._sensors)
        for sensor_index, pts_w, stamp, recv_t in selected:
            self._stat_sensor_hits[sensor_index] += 1
            self._stat_sensor_pts[sensor_index] += len(pts_w)
            # Age on the monotonic clock only: recv_t was taken with the same
            # clock as `now`, so this holds whatever base the sensors stamp in.
            det = self._process_sensor_cloud(
                sensor_index, pts_w, stamp, predicted,
                age=max(0.0, now - recv_t),
                detect=(self.fusion_mode != 'cloud'))
            if det is not None:
                detections.append(det)
                self._sensor_cluster_pts[sensor_index] = int(det.cluster_points)
                self._stat_sensor_cluster_hits[sensor_index] += 1
                self._stat_sensor_cluster_pts[sensor_index] += int(det.cluster_points)

        if self.learn_background:
            self._finalize_background_learning()
            self._maybe_log_stats(now)
            return

        self._stat_frames += 1
        if self.fusion_mode == 'sequential':
            self._sequential_updates(detections, now)
            self._maybe_log_stats(now)
            return
        if self.fusion_mode == 'cloud':
            measurement, cov, out_stamp, fused_points, fused_cands, sensor_count, sync_dt = \
                self._detect_from_merged_cloud(selected, predicted)
        else:
            measurement, cov, out_stamp, fused_points, fused_cands, sensor_count, sync_dt = \
                self._fuse_detections(detections, predicted)

        header = Header()
        header.frame_id = self.world_frame
        if out_stamp is not None:
            header.stamp = out_stamp
            self._last_meas_stamp_s = self._stamp_to_seconds(out_stamp)
            self._last_meas_monotonic = now
        else:
            header.stamp = self._coasting_stamp(now)

        if measurement is not None:
            if not self._kf.initialized:
                self._kf.init(measurement, cov)
            else:
                self._kf.update(measurement, cov)
            self._coast_count = 0
            self._stat_detect += 1
            self._stat_cluster_pts += fused_points
            self._stat_cand += fused_cands

            if self._run_active and self._raw_csv_w is not None:
                raw_row = [
                    f'{self._stamp_seconds(header):.9f}',
                    f'{measurement[0]:.6f}', f'{measurement[1]:.6f}', f'{measurement[2]:.6f}',
                    f'{cov[0,0]:.9f}', f'{cov[1,1]:.9f}', f'{cov[2,2]:.9f}',
                    int(fused_points), int(fused_cands), int(sensor_count),
                    '' if sync_dt is None else f'{sync_dt*1000.0:.3f}',
                ]
                raw_row += ['' if n is None else int(n)
                            for n in self._sensor_cluster_pts]
                self._raw_csv_w.writerow(raw_row)
                self._raw_csv_f.flush()

            if self._run_active and self.enable_waypoint_tracking:
                self._update_waypoint_tracking(measurement)
            self._publish_pose(header)
        else:
            self._coast_count += 1
            # _last_meas_stamp_s 가 없으면 아직 관측이 하나도 없다는 뜻이다.
            # seed_at_home 이 켜져 있으면 KF 는 시작 시점에 이미 initialized 가
            # 되므로 이 조건이 없으면 관측 없이 홈 좌표가 그대로 발행되고,
            # _coasting_stamp() 는 전진시킬 기준이 없어 ROS 클럭으로 떨어진다.
            # 그러면 한 토픽에 센서 시계와 ROS 시계가 섞인다 — _coasting_stamp()
            # 독스트링이 "첫 관측 전에는 발행되지 않는다"고 전제한 그 경로다.
            if self._kf.initialized and self._last_meas_stamp_s is not None:
                if self._coast_count <= self.coast_max:
                    self._publish_pose(header)
                elif self._coast_count > self.reset_after:
                    self.get_logger().warn(
                        f'Tracking lost for {self._coast_count} frames -> KF reset')
                    self._kf.initialized = False
                    self._coast_count = 0
                    if self._run_active:
                        self._finalize_waypoint_episode()

        self._maybe_log_stats(now)
        if self.debug_timing and process_t0 is not None:
            self._record_process_timing(
                (time.perf_counter() - process_t0) * 1000.0)
            self._maybe_log_timing(time.monotonic())

    # ------------------------------------------------------------------
    # Per-sensor processing
    # ------------------------------------------------------------------
    def _process_sensor_cloud(self, sensor_index, pts_w, stamp, predicted,
                              age=0.0, detect=True):
        """Per-sensor pipeline. Returns a Detection, or None.

        With detect=False it runs everything up to and including the filtered
        cloud publish, then stops and returns None. fusion_mode 'cloud' uses
        that: it still wants the per-sensor world/filtered clouds for RViz and
        the background accumulator, but clusters the two sensors together
        afterwards rather than each on its own.
        """
        cfg = self._sensors[sensor_index]
        timings = None
        if self.debug_timing:
            timings = {
                't0': time.perf_counter(),
                'raw_pts': 0,
                'roi_pts': 0,
                'filtered_pts': 0,
                'world_pub_ms': 0.0,
                'roi_ms': 0.0,
                'boundary_ms': 0.0,
                'background_ms': 0.0,
                'voxel_ms': 0.0,
                'sor_ms': 0.0,
                'filtered_pub_ms': 0.0,
                'dbscan_ms': 0.0,
                'select_ms': 0.0,
                'detected': 0,
            }
        pts = np.asarray(pts_w, dtype=np.float64)
        pts = pts[np.isfinite(pts).all(axis=1)]
        if timings is not None:
            timings['raw_pts'] = len(pts)
        if len(pts) < self.min_pts:
            self._finish_sensor_timing(sensor_index, timings)
            return None

        header = Header()
        header.stamp = stamp
        header.frame_id = self.world_frame

        # Full transformed clouds are useful in RViz, but expensive with
        # dual OS1-128 scans. Disable them for timing-sensitive GUI runs.
        if self.publish_world_clouds:
            t = time.perf_counter() if timings is not None else None
            try:
                self._pub_world[sensor_index].publish(
                    self._numpy_to_pc2(pts.astype(np.float32), header))
            except Exception as exc:
                self.get_logger().warn(
                    f'{cfg.name} world-cloud publish failed: {exc}',
                    throttle_duration_sec=5.0)
            if timings is not None:
                timings['world_pub_ms'] = (time.perf_counter() - t) * 1000.0

        # ROI.
        t = time.perf_counter() if timings is not None else None
        mask = (
            (pts[:, 0] >= self.roi_x_min) & (pts[:, 0] <= self.roi_x_max) &
            (pts[:, 1] >= self.roi_y_min) & (pts[:, 1] <= self.roi_y_max) &
            (pts[:, 2] >= self.roi_z_min) & (pts[:, 2] <= self.roi_z_max)
        )
        pts = pts[mask]
        if timings is not None:
            timings['roi_ms'] = (time.perf_counter() - t) * 1000.0
            timings['roi_pts'] = len(pts)
        if len(pts) < self.min_pts:
            self._finish_sensor_timing(sensor_index, timings)
            return None

        # Optional rejection of points close to the ROI boundary planes.
        if self.remove_boundary_surfaces:
            t = time.perf_counter() if timings is not None else None
            pts = self._remove_boundary_surfaces(pts)
            if timings is not None:
                timings['boundary_ms'] = (time.perf_counter() - t) * 1000.0
                timings['roi_pts'] = len(pts)
            if len(pts) < self.min_pts:
                self._finish_sensor_timing(sensor_index, timings)
                return None

        # Background learning occurs after transform/ROI, separately per sensor.
        if self.learn_background:
            t = time.perf_counter() if timings is not None else None
            self._accumulate_background(sensor_index, pts)
            if timings is not None:
                timings['background_ms'] = (time.perf_counter() - t) * 1000.0
                timings['filtered_pts'] = len(pts)
            if self.publish_filtered_clouds:
                t = time.perf_counter() if timings is not None else None
                try:
                    self._pub_filtered[sensor_index].publish(
                        self._numpy_to_pc2(pts.astype(np.float32), header))
                except Exception:
                    pass
                if timings is not None:
                    timings['filtered_pub_ms'] = (time.perf_counter() - t) * 1000.0
            self._finish_sensor_timing(sensor_index, timings)
            return None

        t = time.perf_counter() if timings is not None else None
        pts = self._subtract_background(sensor_index, pts)
        if timings is not None:
            timings['background_ms'] = (time.perf_counter() - t) * 1000.0
        if len(pts) < self.min_pts:
            self._finish_sensor_timing(sensor_index, timings)
            return None

        if self.voxel_leaf_size > 0.0:
            t = time.perf_counter() if timings is not None else None
            pts = self._voxel_downsample(pts, self.voxel_leaf_size)
            if timings is not None:
                timings['voxel_ms'] = (time.perf_counter() - t) * 1000.0
        if timings is not None:
            timings['filtered_pts'] = len(pts)
        if len(pts) < self.min_pts:
            self._finish_sensor_timing(sensor_index, timings)
            return None

        if self.sor_k > 1 and len(pts) > max(self.sor_k * 3, 30):
            t = time.perf_counter() if timings is not None else None
            pts = self._remove_statistical_outliers(pts)
            if timings is not None:
                timings['sor_ms'] = (time.perf_counter() - t) * 1000.0
                timings['filtered_pts'] = len(pts)
        if len(pts) < self.min_pts:
            self._finish_sensor_timing(sensor_index, timings)
            return None

        if self.publish_filtered_clouds:
            t = time.perf_counter() if timings is not None else None
            try:
                self._pub_filtered[sensor_index].publish(
                    self._numpy_to_pc2(pts.astype(np.float32), header))
            except Exception as exc:
                self.get_logger().warn(
                    f'{cfg.name} filtered-cloud publish failed: {exc}',
                    throttle_duration_sec=5.0)
            if timings is not None:
                timings['filtered_pub_ms'] = (time.perf_counter() - t) * 1000.0

        if not detect:
            self._merge_buffer.append((sensor_index, pts, stamp, age))
            self._finish_sensor_timing(sensor_index, timings)
            return None

        try:
            t = time.perf_counter() if timings is not None else None
            labels = DBSCAN(
                eps=self.eps,
                min_samples=self.min_pts,
                metric='euclidean',
                n_jobs=-1,
            ).fit_predict(pts)
            if timings is not None:
                timings['dbscan_ms'] = (time.perf_counter() - t) * 1000.0
        except Exception as exc:
            self.get_logger().error(
                f'{cfg.name} DBSCAN error: {exc}', throttle_duration_sec=5.0)
            self._finish_sensor_timing(sensor_index, timings)
            return None

        t = time.perf_counter() if timings is not None else None
        det = self._select_drone_cluster(sensor_index, pts, labels, stamp,
                                         predicted, age)
        if timings is not None:
            timings['select_ms'] = (time.perf_counter() - t) * 1000.0
            timings['detected'] = 1 if det is not None else 0
            self._finish_sensor_timing(sensor_index, timings)
        if det is not None:
            self._publish_sensor_pose(det)
        return det

    def _remove_boundary_surfaces(self, points):
        m = max(0.0, self.boundary_surface_margin)
        keep = (
            (points[:, 0] > self.roi_x_min + m) &
            (points[:, 0] < self.roi_x_max - m) &
            (points[:, 1] > self.roi_y_min + m) &
            (points[:, 1] < self.roi_y_max - m) &
            (points[:, 2] > self.roi_z_min + m) &
            (points[:, 2] < self.roi_z_max - m)
        )
        return points[keep]

    @staticmethod
    def _voxel_downsample(points, voxel_size):
        if len(points) == 0 or voxel_size <= 0.0:
            return points
        idx = np.floor(points / voxel_size).astype(np.int64)
        _, inverse = np.unique(idx, axis=0, return_inverse=True)
        n = int(inverse.max()) + 1
        sums = np.zeros((n, 3), dtype=np.float64)
        np.add.at(sums, inverse, points)
        counts = np.bincount(inverse, minlength=n)
        return (sums / counts[:, None]).astype(np.float32)

    def _remove_statistical_outliers(self, points):
        if self.sor_k <= 1 or len(points) <= self.sor_k:
            return points
        k = min(self.sor_k + 1, len(points))
        tree = cKDTree(points)
        distances, _ = tree.query(points, k=k)
        mean_dist = np.mean(distances[:, 1:], axis=1)
        threshold = mean_dist.mean() + self.sor_std * mean_dist.std()
        return points[mean_dist <= threshold]

    # ------------------------------------------------------------------
    # Cluster selection and covariance estimation
    # ------------------------------------------------------------------
    def _select_drone_cluster(self, sensor_index, pts, labels, stamp,
                              predicted, age=0.0, nearest_sensor_range=False):
        cfg = self._sensors[sensor_index]
        unique = [label for label in np.unique(labels) if label >= 0]
        if not unique:
            return None

        candidates = []
        marker_msgs = MarkerArray()
        marker_id = sensor_index * 1000

        for label in unique:
            cluster = pts[labels == label]
            n = len(cluster)
            if n < self.min_pts or n > self.max_pts:
                continue
            mins = np.min(cluster, axis=0)
            maxs = np.max(cluster, axis=0)
            extent = maxs - mins
            if np.max(extent) > self.cluster_max_extent:
                continue
            if np.max(extent) < self.cluster_min_extent:
                continue

            centroid = np.median(cluster, axis=0)
            radial = np.linalg.norm(cluster - centroid, axis=1)
            spread = float(np.median(radial)) if len(radial) else 1.0
            if nearest_sensor_range:
                # A merged cluster has no single sensor; size R from the nearer one.
                sensor_range = min(float(np.linalg.norm(centroid - c.t_s2w))
                                   for c in self._sensors)
            else:
                sensor_range = float(np.linalg.norm(centroid - cfg.t_s2w))
            cov = self._estimate_detection_covariance(
                cfg, cluster, centroid, spread, sensor_range, age)

            # A diagnostic quality score only; fusion itself uses covariance.
            quality = 1.0 / max(1e-9, float(np.trace(cov)))
            assoc_dist = 0.0 if predicted is None else float(
                np.linalg.norm(centroid - predicted))
            candidates.append(
                (centroid, cov, n, spread, quality, sensor_range, assoc_dist, label))

            m = Marker()
            m.header.frame_id = self.world_frame
            m.header.stamp = stamp
            m.ns = f'{cfg.name}_clusters'
            m.id = marker_id
            marker_id += 1
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = float(centroid[0])
            m.pose.position.y = float(centroid[1])
            m.pose.position.z = float(centroid[2])
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = 0.20
            if sensor_index == 0:
                m.color.r, m.color.g, m.color.b = 0.1, 1.0, 0.2
            else:
                m.color.r, m.color.g, m.color.b = 0.2, 0.4, 1.0
            m.color.a = 0.8
            m.lifetime.sec = 1
            marker_msgs.markers.append(m)

        if marker_msgs.markers:
            self._pub_markers.publish(marker_msgs)
        if not candidates:
            return None

        if predicted is not None:
            candidates.sort(key=lambda c: c[6])
            best = candidates[0]
            sigma = float(np.sqrt(max(1e-9, np.trace(self._kf.cov_pos) / 3.0)))
            gate = min(
                self.max_assoc_dist + self.gate_sigma_k * sigma,
                self.max_assoc_dist_cap)
            if best[6] > gate:
                return None
        else:
            candidates.sort(key=lambda c: c[4], reverse=True)
            best = candidates[0]

        self._last_cluster_label = int(best[7])
        return Detection(
            sensor_index=sensor_index,
            sensor_name=cfg.name,
            position=np.asarray(best[0], dtype=np.float64),
            covariance=np.asarray(best[1], dtype=np.float64),
            cluster_points=int(best[2]),
            spread=float(best[3]),
            quality=float(best[4]),
            sensor_range=float(best[5]),
            stamp=stamp,
            candidate_count=len(candidates),
        )

    def _estimate_detection_covariance(self, cfg, cluster, centroid, spread,
                                       sensor_range, age=0.0):
        """Estimate a conservative 3x3 world-frame centroid covariance.

        Components:
          * empirical cluster covariance / N  -> centroid uncertainty
          * centroid noise floor
          * range-dependent term
          * age term: reference speed x seconds since this scan was received

        This is not a complete OS1 beam model, but it is statistically meaningful
        and much better than treating point count as a fusion weight.

        The extrinsic-RMSE term that used to sit here is gone. Outdoors a
        Kabsch/SVD extrinsic lands at a few centimetres, which is below the
        floor term already in the sum, so it only shifted every detection's
        covariance by the same constant and never changed the relative
        weighting of the two sensors. Age does change it: a stale detection is
        genuinely worse than a fresh one, and until now nothing in R said so --
        the pairing backlog this node used to run with (see
        _select_unprocessed_pair) inflated every measurement's real error by
        ~0.1 s of travel and the covariance reported it as perfectly good.
        """
        n = max(1, len(cluster))
        if n >= 3:
            empirical = np.cov(np.asarray(cluster, dtype=np.float64).T)
            if empirical.shape != (3, 3) or not np.isfinite(empirical).all():
                empirical = np.eye(3) * max(spread, self.centroid_noise_floor) ** 2
        else:
            empirical = np.eye(3) * max(spread, self.centroid_noise_floor) ** 2

        centroid_cov = empirical / float(n)
        base_std = self.centroid_noise_floor
        range_std = self.range_noise_per_meter * max(0.0, sensor_range)
        age_std = self.age_noise_velocity * max(0.0, age)
        isotropic_var = base_std**2 + range_std**2 + age_std**2
        cov = centroid_cov + np.eye(3) * isotropic_var

        # Clamp eigenvalues to avoid extreme confidence or instability.
        cov = 0.5 * (cov + cov.T)
        vals, vecs = np.linalg.eigh(cov)
        vals = np.clip(
            vals,
            self.min_detection_std**2,
            self.max_detection_std**2)
        return vecs @ np.diag(vals) @ vecs.T

    # ------------------------------------------------------------------
    # Covariance-based position-level fusion
    # ------------------------------------------------------------------
    def _detect_from_merged_cloud(self, selected, predicted):
        """fusion_mode 'cloud': concatenate both sensors, then cluster ONCE.

        Returns the same tuple shape as _fuse_detections so _process does not
        care which path produced the measurement.

        The point of merging before detection is reach. Each sensor sees only
        a slice of a small airframe, and the count falls with range: an X500 at
        19 m works out to roughly five points per sensor, under any sensible
        cluster_min_points, so BOTH sensors drop it and the frame is lost.
        Concatenated they clear the threshold together.

        What it costs, and why 'position' stays the default:

        * Provenance is gone. _estimate_detection_covariance sizes R from the
          cluster's range to ITS sensor; a merged cluster has two ranges and
          only one can be used. A near 20-point view and a far 2-point view end
          up in one median, weighted the same.
        * Extrinsic error inflates the cluster instead of averaging out. The
          two clouds sit about 5 cm apart here, so the merged blob is that much
          wider and its median lands between the two views rather than being a
          covariance-weighted compromise between two independent estimates.
        * The two scans are up to ~51 ms apart (the heads are not phase
          locked), and merging bakes both instants into one measurement with
          no sync_tolerance check to catch it. Harmless at 0.2 m/s, 15 cm of
          smear at 3 m/s.

        So this is the mode for far, sparse targets; 'position' is better when
        both sensors can see the target on their own.
        """
        merged = self._merge_buffer
        self._merge_buffer = []
        if not merged:
            return None, None, None, 0, 0, 0, None

        pts = np.vstack([m[1] for m in merged])
        stamps = [m[2] for m in merged]
        # The merged cloud is only as fresh as its stalest half, so the age
        # term takes the worst of the two rather than an average.
        age = max((m[3] for m in merged), default=0.0)
        sensor_count = len(merged)
        sync_dt = None
        if sensor_count == 2:
            t0, t1 = (self._stamp_to_seconds(s) for s in stamps)
            sync_dt = abs(t0 - t1)
        # Newest stamp: the merged cloud is no older than its freshest half.
        stamp = max(stamps, key=self._stamp_to_seconds)

        if len(pts) < self.min_pts:
            return None, None, None, 0, 0, sensor_count, sync_dt

        try:
            labels = DBSCAN(
                eps=self.eps,
                min_samples=self.min_pts,
                metric='euclidean',
                n_jobs=-1,
            ).fit_predict(pts)
        except Exception as exc:
            self.get_logger().error(
                f'merged DBSCAN error: {exc}', throttle_duration_sec=5.0)
            return None, None, None, 0, 0, sensor_count, sync_dt

        # Attributed to sensor 0 purely so covariance and markers have a
        # sensor to reference; the cluster itself spans both.
        det = self._select_drone_cluster(0, pts, labels, stamp, predicted, age,
                                         nearest_sensor_range=True)
        if det is None:
            return None, None, None, 0, 0, sensor_count, sync_dt

        # 병합 클러스터에는 두 라이다의 점이 섞여 있다. 위 vstack 의 행 순서가
        # 그대로 남아 있으니 구간을 잘라 각자 몇 점을 냈는지 되돌린다.
        chosen = labels == self._last_cluster_label
        offset = 0
        for s_idx, s_pts, *_ in merged:     # (sensor, points, stamp, age)
            n_s = len(s_pts)
            c = int(np.count_nonzero(chosen[offset:offset + n_s]))
            self._sensor_cluster_pts[s_idx] = c
            self._stat_sensor_cluster_hits[s_idx] += 1
            self._stat_sensor_cluster_pts[s_idx] += c
            offset += n_s

        return (det.position.copy(), det.covariance.copy(), det.stamp,
                det.cluster_points, det.candidate_count, sensor_count, sync_dt)

    def _sequential_updates(self, detections, now):
        """SU: one Kalman update per sensor detection, in timestamp order.

        Detection, gating and covariance are exactly the MF ones; only the
        fusion step differs -- no pairing, no agreement check, and each
        detection is published as soon as it has been applied.
        """
        if not detections:
            self._coast_count += 1
            if self._coast_count > self.reset_after and self._kf.initialized:
                self.get_logger().warn(
                    f'Tracking lost for {self._coast_count} frames -> KF reset')
                self._kf.initialized = False
                self._coast_count = 0
                if self._run_active:
                    self._finalize_waypoint_episode()
            return
        detections = sorted(detections, key=lambda d: self._stamp_to_seconds(d.stamp))
        prev_t = None
        for det in detections:
            t = self._stamp_to_seconds(det.stamp)
            if prev_t is not None and self._kf.initialized and t > prev_t:
                self._kf.predict(t - prev_t)
            prev_t = t
            if not self._kf.initialized:
                self._kf.init(det.position, det.covariance)
            else:
                self._kf.update(det.position, det.covariance)
            self._coast_count = 0
            self._stat_detect += 1
            self._stat_cluster_pts += det.cluster_points
            self._stat_cand += det.candidate_count
            self._last_meas_stamp_s = t
            self._last_meas_monotonic = now
            if self._run_active and self._raw_csv_w is not None:
                # SU는 행 하나가 라이다 하나의 관측이다 — 그 라이다 칸만 채우고 나머지는 빈칸.
                per_sensor = ['' if i != det.sensor_index else int(det.cluster_points)
                              for i in range(len(self._sensors))]
                self._raw_csv_w.writerow([
                    f'{t:.9f}', f'{det.position[0]:.6f}', f'{det.position[1]:.6f}',
                    f'{det.position[2]:.6f}', f'{det.covariance[0,0]:.9f}',
                    f'{det.covariance[1,1]:.9f}', f'{det.covariance[2,2]:.9f}',
                    int(det.cluster_points), int(det.candidate_count), 1, '',
                    *per_sensor])
                self._raw_csv_f.flush()
            if self._run_active and self.enable_waypoint_tracking:
                self._update_waypoint_tracking(det.position)
            header = Header()
            header.frame_id = self.world_frame
            header.stamp = det.stamp
            self._publish_pose(header)

    def _fuse_detections(self, detections, predicted):
        if not detections:
            return None, None, None, 0, 0, 0, None

        if len(detections) == 1:
            d = detections[0]
            return (
                d.position.copy(), d.covariance.copy(), d.stamp,
                d.cluster_points, d.candidate_count, 1, None)

        # Exactly one detection per sensor is expected.
        d0, d1 = detections[:2]
        t0 = self._stamp_to_seconds(d0.stamp)
        t1 = self._stamp_to_seconds(d1.stamp)
        dt = abs(t0 - t1)
        disagreement = float(np.linalg.norm(d0.position - d1.position))

        if dt <= self.sync_tolerance and disagreement <= self.sensor_agreement_dist:
            C0 = self._regularize_cov3(d0.covariance)
            C1 = self._regularize_cov3(d1.covariance)
            I0 = np.linalg.inv(C0)
            I1 = np.linalg.inv(C1)
            Cf = np.linalg.inv(I0 + I1)
            zf = Cf @ (I0 @ d0.position + I1 @ d1.position)
            stamp = d0.stamp if t0 >= t1 else d1.stamp
            return (
                zf, Cf, stamp,
                d0.cluster_points + d1.cluster_points,
                d0.candidate_count + d1.candidate_count,
                2, dt)

        # Poor timing -> do not combine measurements from different physical times.
        if dt > self.sync_tolerance:
            chosen = d0 if t0 > t1 else d1
            self.get_logger().warn(
                f'LiDAR timestamp mismatch {dt*1000.0:.2f} ms > '
                f'{self.sync_tolerance*1000.0:.2f} ms; using {chosen.sensor_name}',
                throttle_duration_sec=2.0)
            return (
                chosen.position.copy(), chosen.covariance.copy(), chosen.stamp,
                chosen.cluster_points, chosen.candidate_count, 1, dt)

        # Synchronized but spatially inconsistent. Prefer prediction-consistent
        # measurement; at cold start prefer lower covariance trace.
        if predicted is not None:
            e0 = float(np.linalg.norm(d0.position - predicted))
            e1 = float(np.linalg.norm(d1.position - predicted))
            chosen = d0 if e0 <= e1 else d1
        else:
            chosen = d0 if np.trace(d0.covariance) <= np.trace(d1.covariance) else d1
        self.get_logger().warn(
            f'LiDAR position disagreement {disagreement:.3f} m > '
            f'{self.sensor_agreement_dist:.3f} m; using {chosen.sensor_name}',
            throttle_duration_sec=2.0)
        return (
            chosen.position.copy(), chosen.covariance.copy(), chosen.stamp,
            chosen.cluster_points, chosen.candidate_count, 1, dt)

    @staticmethod
    def _regularize_cov3(cov):
        cov = np.asarray(cov, dtype=np.float64).reshape(3, 3)
        cov = 0.5 * (cov + cov.T)
        vals, vecs = np.linalg.eigh(cov)
        vals = np.clip(vals, 1e-6, 100.0)
        return vecs @ np.diag(vals) @ vecs.T

    # ------------------------------------------------------------------
    # Pose publishers
    # ------------------------------------------------------------------
    def _publish_sensor_pose(self, det: Detection):
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = det.stamp
        msg.header.frame_id = self.world_frame
        msg.pose.pose.position.x = float(det.position[0])
        msg.pose.pose.position.y = float(det.position[1])
        msg.pose.pose.position.z = float(det.position[2])
        msg.pose.pose.orientation.w = 1.0
        cov = [0.0] * 36
        for r in range(3):
            for c in range(3):
                cov[r * 6 + c] = float(det.covariance[r, c])
        msg.pose.covariance = cov
        self._pub_sensor_pose[det.sensor_index].publish(msg)

    def _publish_pose(self, header):
        pos = self._kf.position
        vel = self._kf.velocity
        cov_pos = self._kf.cov_pos
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = header.stamp
        msg.header.frame_id = self.world_frame
        msg.pose.pose.position.x = float(pos[0])
        msg.pose.pose.position.y = float(pos[1])
        msg.pose.pose.position.z = float(pos[2])
        msg.pose.pose.orientation.w = 1.0
        cov = [0.0] * 36
        for r in range(3):
            for c in range(3):
                cov[r * 6 + c] = float(cov_pos[r, c])
        msg.pose.covariance = cov
        self._pub_pose.publish(msg)

        odom = Odometry()
        odom.header = msg.header
        odom.child_frame_id = 'drone'
        odom.pose = msg.pose
        odom.twist.twist.linear.x = float(vel[0])
        odom.twist.twist.linear.y = float(vel[1])
        odom.twist.twist.linear.z = float(vel[2])
        cov_vel = self._kf.P[3:6, 3:6]
        vcov = [0.0] * 36
        for r in range(3):
            for c in range(3):
                vcov[r * 6 + c] = float(cov_vel[r, c])
        odom.twist.covariance = vcov
        self._pub_odom.publish(odom)

        if self._run_active and self._traj_csv_w is not None:
            self._traj_csv_w.writerow([
                f'{self._stamp_seconds(header):.9f}',
                f'{pos[0]:.6f}', f'{pos[1]:.6f}', f'{pos[2]:.6f}',
                f'{vel[0]:.6f}', f'{vel[1]:.6f}', f'{vel[2]:.6f}',
                f'{cov_pos[0,0]:.9f}', f'{cov_pos[1,1]:.9f}', f'{cov_pos[2,2]:.9f}',
            ])
            self._traj_csv_f.flush()

    # ------------------------------------------------------------------
    # Waypoint hover estimation
    # ------------------------------------------------------------------
    def _update_waypoint_tracking(self, measurement):
        speed = float(np.linalg.norm(self._kf.velocity[:2]))
        slow = speed <= self.waypoint_speed_threshold

        if self._wp_active_id is not None:
            mx, my = self._marker_xy[self._wp_active_id]
            d = math.hypot(measurement[0] - mx, measurement[1] - my)
            if d <= self.waypoint_gate_radius and slow:
                self._wp_samples.append(measurement.copy())
                self._wp_break_count = 0
                return
            self._wp_break_count += 1
            if self._wp_break_count <= self.waypoint_grace_frames:
                return
            self._finalize_waypoint_episode()

        best_id = None
        best_d = None
        for marker_id, (mx, my) in self._marker_xy.items():
            d = math.hypot(measurement[0] - mx, measurement[1] - my)
            if best_d is None or d < best_d:
                best_id, best_d = marker_id, d
        if best_d is not None and best_d <= self.waypoint_gate_radius and slow:
            self._wp_active_id = best_id
            self._wp_samples = [measurement.copy()]
            self._wp_break_count = 0

    def _finalize_waypoint_episode(self):
        marker_id = self._wp_active_id
        samples = self._wp_samples
        self._wp_active_id = None
        self._wp_samples = []
        self._wp_break_count = 0
        if marker_id is None or len(samples) < self.waypoint_min_frames:
            return
        arr = np.asarray(samples)
        med = np.median(arr, axis=0)
        n = len(samples)
        duration = n / self.update_rate
        self._wp_visit_counts[marker_id] = self._wp_visit_counts.get(marker_id, 0) + 1
        visit = self._wp_visit_counts[marker_id]
        self.get_logger().info(
            f'Waypoint {marker_id} visit {visit}: '
            f'({med[0]:.3f},{med[1]:.3f},{med[2]:.3f}), n={n}, '
            f'duration={duration:.1f}s')
        if self._run_active and self._wp_csv_w is not None:
            self._wp_csv_w.writerow([
                marker_id, visit,
                f'{med[0]:.4f}', f'{med[1]:.4f}', f'{med[2]:.4f}',
                n, f'{duration:.2f}'])
            self._wp_csv_f.flush()
        self._publish_waypoint_marker(marker_id, visit, med)

    def _publish_waypoint_marker(self, marker_id, visit, pos):
        m = Marker()
        m.header.frame_id = self.world_frame
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'waypoint_estimates'
        m.id = marker_id * 100 + visit
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.position.x = float(pos[0])
        m.pose.position.y = float(pos[1])
        m.pose.position.z = float(pos[2])
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = 0.30
        m.color.r, m.color.g, m.color.b, m.color.a = 1.0, 0.0, 1.0, 0.9
        self._wp_markers.markers.append(m)
        # close() finalizes a last episode from inside the shutdown path, where
        # the context is already torn down. Publishing there raises and would
        # skip _close_run_logs(), losing whatever the raw/trajectory CSV
        # buffers still hold.
        if not rclpy.ok():
            return
        self._pub_waypoints.publish(self._wp_markers)

    # ------------------------------------------------------------------
    # Statistics
    # ------------------------------------------------------------------
    def _blank_timing_bucket(self):
        return {
            'count': 0,
            'raw_pts': 0,
            'roi_pts': 0,
            'filtered_pts': 0,
            'detected': 0,
            'pc2_ms': 0.0,
            'tf_ms': 0.0,
            'store_ms': 0.0,
            'world_pub_ms': 0.0,
            'roi_ms': 0.0,
            'boundary_ms': 0.0,
            'background_ms': 0.0,
            'voxel_ms': 0.0,
            'sor_ms': 0.0,
            'filtered_pub_ms': 0.0,
            'dbscan_ms': 0.0,
            'select_ms': 0.0,
            'process_sensor_ms': 0.0,
        }

    def _reset_timing_stats(self):
        self._timing_store = [
            self._blank_timing_bucket(),
            self._blank_timing_bucket(),
        ]
        self._timing_sensor = [
            self._blank_timing_bucket(),
            self._blank_timing_bucket(),
        ]
        self._timing_process_count = 0
        self._timing_process_ms = 0.0
        self._timing_no_frame = 0

    def _record_store_timing(self, sensor_index, raw_pts, pc2_ms, tf_ms, total_ms):
        b = self._timing_store[sensor_index]
        b['count'] += 1
        b['raw_pts'] += int(raw_pts)
        b['pc2_ms'] += float(pc2_ms)
        b['tf_ms'] += float(tf_ms)
        b['store_ms'] += float(total_ms)

    def _finish_sensor_timing(self, sensor_index, timings):
        if timings is None:
            return
        b = self._timing_sensor[sensor_index]
        b['count'] += 1
        b['raw_pts'] += int(timings['raw_pts'])
        b['roi_pts'] += int(timings['roi_pts'])
        b['filtered_pts'] += int(timings['filtered_pts'])
        b['detected'] += int(timings['detected'])
        for key in (
            'world_pub_ms', 'roi_ms', 'boundary_ms', 'background_ms',
            'voxel_ms', 'sor_ms', 'filtered_pub_ms', 'dbscan_ms',
            'select_ms',
        ):
            b[key] += float(timings[key])
        b['process_sensor_ms'] += (
            time.perf_counter() - timings['t0']) * 1000.0

    def _record_process_timing(self, total_ms):
        self._timing_process_count += 1
        self._timing_process_ms += float(total_ms)

    @staticmethod
    def _avg(bucket, key):
        return bucket[key] / bucket['count'] if bucket['count'] else 0.0

    def _maybe_log_timing(self, now):
        if not self.debug_timing:
            return
        if now - self._timing_t0 < self.debug_timing_period:
            return

        process_avg = (
            self._timing_process_ms / self._timing_process_count
            if self._timing_process_count else 0.0
        )
        self.get_logger().info(
            f'TIMING process: avg={process_avg:.1f}ms/'
            f'{self._timing_process_count} processed, '
            f'no_frame_ticks={self._timing_no_frame}'
        )
        for i, cfg in enumerate(self._sensors):
            store = self._timing_store[i]
            sensor = self._timing_sensor[i]
            self.get_logger().info(
                f'TIMING {cfg.name} store: frames={store["count"]}, '
                f'raw={self._avg(store, "raw_pts"):.0f}pts, '
                f'pc2={self._avg(store, "pc2_ms"):.1f}ms, '
                f'tf={self._avg(store, "tf_ms"):.1f}ms, '
                f'total={self._avg(store, "store_ms"):.1f}ms'
            )
            self.get_logger().info(
                f'TIMING {cfg.name} detect: frames={sensor["count"]}, '
                f'detected={sensor["detected"]}, '
                f'roi={self._avg(sensor, "roi_pts"):.0f}pts, '
                f'filtered={self._avg(sensor, "filtered_pts"):.0f}pts, '
                f'world_pub={self._avg(sensor, "world_pub_ms"):.1f}ms, '
                f'roi_crop={self._avg(sensor, "roi_ms"):.1f}ms, '
                f'bg={self._avg(sensor, "background_ms"):.1f}ms, '
                f'voxel={self._avg(sensor, "voxel_ms"):.1f}ms, '
                f'sor={self._avg(sensor, "sor_ms"):.1f}ms, '
                f'filtered_pub={self._avg(sensor, "filtered_pub_ms"):.1f}ms, '
                f'dbscan={self._avg(sensor, "dbscan_ms"):.1f}ms, '
                f'select={self._avg(sensor, "select_ms"):.1f}ms, '
                f'total={self._avg(sensor, "process_sensor_ms"):.1f}ms'
            )
        self._timing_t0 = now
        self._reset_timing_stats()

    def _reset_stats(self):
        self._stat_frames = 0
        self._stat_detect = 0
        self._stat_cluster_pts = 0
        self._stat_cand = 0
        self._stat_sensor_pts = [0, 0]
        self._stat_sensor_hits = [0, 0]
        self._stat_sensor_cluster_pts = [0, 0]
        self._stat_sensor_cluster_hits = [0, 0]

    def _maybe_log_stats(self, now):
        if now - self._stat_t0 < self._stat_period:
            return
        if self.learn_background:
            status = ', '.join(
                f'{cfg.name}:{min(self._bg_frames_seen[i], self.background_frames)}/'
                f'{self.background_frames}'
                for i, cfg in enumerate(self._sensors))
            self.get_logger().info(f'Background learning [{status}]')
            self._stat_t0 = now
            return

        f = self._stat_frames
        if f == 0:
            self.get_logger().warn(
                'No NEW LiDAR frame pairs/frames processed in the last '
                f'{self._stat_period:.0f}s; check topics and timestamps.')
        else:
            det = self._stat_detect
            rate = 100.0 * det / f
            avg_pts = self._stat_cluster_pts / det if det else 0.0
            avg_cand = self._stat_cand / f
            sensors = []
            for i, cfg in enumerate(self._sensors):
                hits = self._stat_sensor_hits[i]
                avg = self._stat_sensor_pts[i] / hits if hits else 0.0
                c_hits = self._stat_sensor_cluster_hits[i]
                c_avg = (self._stat_sensor_cluster_pts[i] / c_hits
                         if c_hits else 0.0)
                sensors.append(
                    f'{cfg.name}: raw {avg:.0f}pts/{hits}frames, '
                    f'cluster {c_avg:.1f}pts/{c_hits}det')
            self.get_logger().info(
                f'Detection {det}/{f} ({rate:.0f}%), '
                f'cluster={avg_pts:.1f}pts, candidates={avg_cand:.1f}, '
                f'sensors=[{" + ".join(sensors)}]')
        self._stat_t0 = now
        self._reset_stats()

    # ------------------------------------------------------------------
    # PointCloud2 / timestamps
    # ------------------------------------------------------------------
    def _pc2_to_numpy(self, msg):
        try:
            arr = pc2_util.read_points(
                msg, field_names=('x', 'y', 'z'), skip_nans=True)
            if arr is None:
                return None
            if isinstance(arr, np.ndarray):
                if arr.size == 0:
                    return None
                if arr.dtype.names is not None:
                    pts = np.stack([arr['x'], arr['y'], arr['z']], axis=-1)
                else:
                    pts = np.asarray(arr)[..., :3]
            else:
                pts = np.array([[p[0], p[1], p[2]] for p in arr], dtype=np.float32)
            pts = np.asarray(pts, dtype=np.float32).reshape(-1, 3)
            pts = pts[np.isfinite(pts).all(axis=1)]
            return pts if len(pts) else None
        except Exception as exc:
            self.get_logger().error(
                f'PointCloud2 read error: {exc}', throttle_duration_sec=5.0)
            return None

    def _numpy_to_pc2(self, pts, ref_header):
        h = Header()
        h.stamp = ref_header.stamp
        h.frame_id = self.world_frame
        return pc2_util.create_cloud_xyz32(h, pts.tolist())

    @staticmethod
    def _stamp_to_seconds(stamp):
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9

    @staticmethod
    def _stamp_seconds(header):
        return float(header.stamp.sec) + float(header.stamp.nanosec) * 1e-9

    def _coasting_stamp(self, now_monotonic):
        """Stamp for a pose published without a measurement behind it.

        Must NOT be self.get_clock().now(): with timestamp_mode
        TIME_FROM_PTP_1588 the sensors stamp in PTP time, which is TAI-based,
        while the ROS clock is UTC. Mixing the two put a 37 s step into every
        entry and exit from coasting -- measured on run tello_11/run_01, where
        the two clock bases matched coasting and measurement perfectly (153
        coasting samples on one base, 208 measured on the other, 100%
        agreement) and produced 39 jumps of +38.2 s and 38 of -37.5 s in a
        150 s run. The KF itself was unharmed, since predict() uses a
        monotonic dt, but the published poses and both CSVs were unusable in
        time order.

        Advancing the last measurement's stamp by elapsed monotonic time keeps
        the whole output stream on one clock, whichever base the sensors use.
        Before any measurement has arrived there is nothing to advance from, so
        fall back to the ROS clock -- the KF is not initialized then either, so
        no pose is published down that path in practice.
        """
        if self._last_meas_stamp_s is None or self._last_meas_monotonic is None:
            return self.get_clock().now().to_msg()
        t = self._last_meas_stamp_s + max(0.0, now_monotonic - self._last_meas_monotonic)
        msg = TimeMsg()
        msg.sec = int(t)
        msg.nanosec = int(round((t - msg.sec) * 1e9))
        if msg.nanosec >= 1_000_000_000:      # rounding can carry
            msg.sec += 1
            msg.nanosec -= 1_000_000_000
        return msg

    # ------------------------------------------------------------------
    # Shutdown
    # ------------------------------------------------------------------
    def close(self):
        if self.learn_background and self._background_learning_complete():
            self._finalize_background_learning()
        if self._run_active:
            self._finalize_waypoint_episode()
        self._run_active = False
        self._close_run_logs()
        self.get_logger().info('LiDAR tracker closed; run CSV files flushed.')


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = LidarDroneTracker()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
