#!/usr/bin/env python3

"""ROI 안의 점을 DBSCAN 으로 군집화하고 가장 큰 군집의 median 좌표를 낸다.

벽/바닥은 roi_filter 가 이미 잘라냈으므로 여기 들어오는 점은 관심 물체다.
중심은 평균이 아니라 median 으로 잡는다 - 평균은 군집에 섞인 이상점 하나에
끌려가지만 median 은 거의 움직이지 않는다.

  입력  /lidar1/roi_points   (map 프레임, roi_filter 출력)
  출력  /lidar1/cluster_markers  visualization_msgs/MarkerArray
        /lidar1/cluster_poses    geometry_msgs/PoseArray
        로그에 median 좌표

점이 많으면 DBSCAN 이 느려진다(실측: 12,000점에서 약 2~3 Hz). ROI 를 좁게
잡아 물체만 남기면 문제 없고, 그래도 느리면 voxel_size 를 0.05 로 켠다.
"""

import time

import numpy as np
import rclpy
from geometry_msgs.msg import Pose, PoseArray
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2_util
from sklearn.cluster import DBSCAN
from visualization_msgs.msg import Marker, MarkerArray


class ClusterMedian(Node):

    def __init__(self):
        super().__init__('cluster_median')

        self.declare_parameter('input_topic', '/lidar1/roi_points')

        # 복셀 한 변(m). 0 이면 다운샘플 안 함(기본).
        # 위치만 뽑는 용도라 꺼 둔다. 점이 많아 DBSCAN 이 느려지면 0.05 정도로 켠다.
        self.declare_parameter('voxel_size', 0.0)

        # 이 거리 안이면 같은 군집으로 본다.
        self.declare_parameter('eps', 0.20)
        # 군집으로 인정할 최소 (복셀)점 개수.
        self.declare_parameter('min_samples', 8)

        # 군집 크기 필터. 너무 작으면 잡음, 너무 크면 벽 잔재.
        self.declare_parameter('min_points', 8)
        self.declare_parameter('max_points', 100000)
        self.declare_parameter('max_span', 2.0)

        # RViz 에서 라이다별로 구분하려고. 두 대를 겹쳐 볼 때 같은 색이면
        # 어느 쪽이 잡은 건지 알 수 없다.
        self.declare_parameter('marker_color', [0.1, 1.0, 0.2])
        self.declare_parameter('log_period', 0.5)
        # 주기/지연 통계를 몇 초마다 찍을지. 0 이면 끔.
        self.declare_parameter('stats_period', 2.0)

        gp = lambda n: self.get_parameter(n).value
        self._in_topic = str(gp('input_topic'))

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(PointCloud2, self._in_topic, self._cb, qos)

        # 발행 토픽은 입력에서 이름공간을 떼어 쓴다. lidar1 을 박아두면
        # lidar_id:=2 로 띄웠을 때 하류가 엉뚱한 토픽을 본다.
        ns = self._in_topic.rsplit('/', 1)[0] or '/lidar1'
        self._pub_markers = self.create_publisher(
            MarkerArray, f'{ns}/cluster_markers', 10)
        self._pub_poses = self.create_publisher(
            PoseArray, f'{ns}/cluster_poses', 10)

        self._latest = None
        self._frame = 'map'
        self._prev_marker_count = 0
        self._had_objects = False
        self._src_stamp = None
        self._last_stamp = 0.0
        self._last_latency = 0.0
        self.create_timer(float(gp('log_period')), self._log)

        # 주기 계측용. 콜백마다 쌓고 통계 출력 때 비운다.
        self._proc_ms = []
        self._latency_ms = []
        self._recv_wall = []
        self._prev_recv = None
        stats_period = float(gp('stats_period'))
        if stats_period > 0.0:
            self.create_timer(stats_period, self._stats)

        self.get_logger().info(
            f'cluster_median: {self._in_topic} -> DBSCAN '
            f'(eps={gp("eps")}, min_samples={gp("min_samples")}, '
            f'voxel={gp("voxel_size")})')

    # ------------------------------------------------------------------
    @staticmethod
    def _voxel(points, size):
        """복셀 격자마다 점 하나로 줄인다. 격자 안 점들의 평균을 쓴다."""
        if size <= 0.0 or len(points) == 0:
            return points
        idx = np.floor(points / size).astype(np.int64)
        _, inverse = np.unique(idx, axis=0, return_inverse=True)
        n = int(inverse.max()) + 1
        sums = np.zeros((n, 3), dtype=np.float64)
        np.add.at(sums, inverse, points)
        counts = np.bincount(inverse, minlength=n)
        return sums / counts[:, None]

    def _cb(self, msg):
        t_start = time.perf_counter()

        # 이 메시지가 실제로 몇 초 전에 찍힌 것인지. roi_filter 가 원본
        # 라이다 스탬프를 그대로 물려주므로 센서 촬영 시각과 같다.
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        now = self.get_clock().now().nanoseconds * 1e-9
        self._last_stamp = stamp
        self._last_latency = now - stamp
        self._latency_ms.append(self._last_latency * 1e3)

        wall = time.perf_counter()
        if self._prev_recv is not None:
            self._recv_wall.append((wall - self._prev_recv) * 1e3)
        self._prev_recv = wall

        try:
            self._process(msg)
        finally:
            self._proc_ms.append((time.perf_counter() - t_start) * 1e3)

    def _process(self, msg):
        gp = lambda n: self.get_parameter(n).value
        self._frame = msg.header.frame_id or 'map'
        # 출력에 찍을 시각. 처리한 시각이 아니라 '촬영된 시각'이어야 한다.
        # 현재 시각으로 덮으면 처리 지연(약 215 ms)이 그대로 오차가 된다.
        self._src_stamp = msg.header.stamp

        arr = pc2_util.read_points(
            msg, field_names=('x', 'y', 'z'), skip_nans=True)
        if arr is None or arr.size == 0:
            self._empty()
            return
        pts = np.stack([arr['x'], arr['y'], arr['z']], axis=-1)
        pts = np.asarray(pts, dtype=np.float64).reshape(-1, 3)
        pts = pts[np.isfinite(pts).all(axis=1)]
        if len(pts) == 0:
            self._empty()
            return

        v = self._voxel(pts, float(gp('voxel_size')))
        if len(v) < int(gp('min_samples')):
            self._empty()
            return

        labels = DBSCAN(
            eps=float(gp('eps')),
            min_samples=int(gp('min_samples')),
            n_jobs=-1,
        ).fit_predict(v)

        min_pts = int(gp('min_points'))
        max_pts = int(gp('max_points'))
        max_span = float(gp('max_span'))

        out = []
        for label in np.unique(labels):
            if label < 0:          # -1 은 잡음
                continue
            c = v[labels == label]
            if not (min_pts <= len(c) <= max_pts):
                continue
            ext = c.max(axis=0) - c.min(axis=0)
            if max(ext[0], ext[1]) > max_span:
                continue
            out.append({
                'median': np.median(c, axis=0),
                'n': len(c),
                'ext': ext,
            })

        # 가장 큰 군집 하나만 남긴다.
        if out:
            out = [max(out, key=lambda d: d['n'])]
        self._latest = out
        self._publish(out)

    def _empty(self):
        """군집이 없는 프레임. 빈 PoseArray 를 그대로 내보낸다.

        발행을 건너뛰면 하류(flight_recorder)에서 '못 잡은 프레임'과
        '노드가 죽은 것'을 구분할 수 없다. 놓친 구간을 세려면 매 프레임이
        와야 한다.
        """
        self._latest = []
        self._publish([])

    # ------------------------------------------------------------------
    def _publish(self, clusters):
        stamp = self._src_stamp

        poses = PoseArray()
        poses.header.stamp = stamp
        poses.header.frame_id = self._frame

        markers = MarkerArray()
        for i, c in enumerate(clusters):
            m = c['median']

            p = Pose()
            p.position.x = float(m[0])
            p.position.y = float(m[1])
            p.position.z = float(m[2])
            p.orientation.w = 1.0
            poses.poses.append(p)

            sphere = Marker()
            sphere.header.stamp = stamp
            sphere.header.frame_id = self._frame
            sphere.ns = 'cluster'
            sphere.id = i
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD
            sphere.pose = p
            sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.2
            col = self.get_parameter('marker_color').value
            sphere.color.r = float(col[0])
            sphere.color.g = float(col[1])
            sphere.color.b = float(col[2])
            sphere.color.a = 0.9
            markers.markers.append(sphere)

            text = Marker()
            text.header.stamp = stamp
            text.header.frame_id = self._frame
            text.ns = 'cluster_text'
            text.id = i
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position.x = float(m[0])
            text.pose.position.y = float(m[1])
            text.pose.position.z = float(m[2]) + 0.25
            text.pose.orientation.w = 1.0
            text.scale.z = 0.15
            # 글자는 구슬보다 밝게 해서 어두운 배경에서도 읽히게 한다.
            text.color.r = min(1.0, float(col[0]) + 0.4)
            text.color.g = min(1.0, float(col[1]) + 0.4)
            text.color.b = min(1.0, float(col[2]) + 0.4)
            text.color.a = 1.0
            text.text = f'#{i} ({m[0]:.2f}, {m[1]:.2f}, {m[2]:.2f})'
            markers.markers.append(text)

        # 지난 프레임보다 군집이 줄면 남은 마커를 지운다.
        for i in range(len(clusters), self._prev_marker_count):
            for ns in ('cluster', 'cluster_text'):
                d = Marker()
                d.header.stamp = stamp
                d.header.frame_id = self._frame
                d.ns = ns
                d.id = i
                d.action = Marker.DELETE
                markers.markers.append(d)
        self._prev_marker_count = len(clusters)

        self._pub_poses.publish(poses)
        self._pub_markers.publish(markers)

    # ------------------------------------------------------------------
    def _log(self):
        if self._latest is None:
            self.get_logger().warn(f'입력 없음 - {self._in_topic} 확인')
            return

        if not self._latest:
            # 비어 있을 때 매번 찍으면 로그가 묻힌다. 사라진 순간만 알린다.
            if self._had_objects:
                self.get_logger().info('물체 없음')
                self._had_objects = False
            return

        self._had_objects = True
        m = self._latest[0]['median']
        self.get_logger().info(
            f'median   x={m[0]:6.2f}   y={m[1]:6.2f}   z={m[2]:6.2f}   '
            f'[t={self._last_stamp:.3f}  지연 {self._last_latency * 1e3:.0f}ms]')

    def _stats(self):
        if not self._proc_ms:
            self.get_logger().warn('입력 없음 - 주기 측정 불가')
            return

        proc = np.asarray(self._proc_ms)
        lat = np.asarray(self._latency_ms)
        gap = np.asarray(self._recv_wall) if self._recv_wall else None

        parts = [f'처리 {proc.mean():.1f}/{proc.max():.1f}ms(평균/최대)']
        if gap is not None and len(gap):
            hz = 1000.0 / gap.mean()
            parts.append(f'입력 {hz:.1f}Hz({gap.mean():.0f}ms 간격)')
        parts.append(f'지연 {lat.mean():.0f}/{lat.max():.0f}ms')
        msg = '[주기] ' + '  |  '.join(parts)

        # 처리가 입력 간격을 넘으면 큐가 밀린다.
        if gap is not None and len(gap) and proc.mean() > gap.mean():
            msg += '  <== 처리가 입력보다 느림!'
        self.get_logger().info(msg)

        self._proc_ms.clear()
        self._latency_ms.clear()
        self._recv_wall.clear()


def main(args=None):
    rclpy.init(args=args)
    node = ClusterMedian()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 정리 중에 또 Ctrl+C 가 들어와도 트레이스백을 뱉지 않게 한다.
        # KeyboardInterrupt 는 BaseException 이라 except Exception 으로는
        # 못 잡는다.
        for fn in (node.destroy_node, rclpy.shutdown):
            try:
                fn()
            except BaseException:
                pass


if __name__ == '__main__':
    main()
