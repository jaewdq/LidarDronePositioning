#!/usr/bin/env python3

"""라이다 포인트클라우드에서 관심영역(ROI)만 잘라 다시 발행한다.

들어온 점을 z축으로 yaw_deg 만큼 돌리고 lidar_z 만큼 올려 월드(map)
좌표로 바꾼 뒤, 박스 안에 든 점만 남긴다.

  입력  /lidar1/points      (lidar1 프레임)
  출력  /lidar1/roi_points  (map 프레임)

실측(사람이 정면에 서서 확인) 결과 이 라이다는 정면이 프레임의 -x 였다.
그래서 yaw_deg 기본값을 180 으로 두어 정면이 +x 가 되게 한다.

기본 박스: x 0~4.5(정면), y -2.15~2.1(좌우), z 0.25~2(높이). z=0 이 바닥.
"""

import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2_util
from std_msgs.msg import Header


class RoiFilter(Node):

    def __init__(self):
        super().__init__('roi_filter')

        self.declare_parameter('input_topic', '/lidar1/points')
        self.declare_parameter('output_topic', '/lidar1/roi_points')
        self.declare_parameter('output_frame', 'map')

        # 라이다 장착 높이. 입력 점의 z 에 그대로 더해 월드 z 를 만든다.
        self.declare_parameter('lidar_z', 0.84)

        # z축 회전(도). 180 이면 라이다의 -x 가 월드 +x 가 된다.
        self.declare_parameter('yaw_deg', 180.0)

        # 원점 이동. 회전을 적용한 뒤 이 값을 뺀다.
        # 마커 보정(pose_solve)으로 구한 값을 넣는다.
        self.declare_parameter('origin_x', 0.0)
        self.declare_parameter('origin_y', 0.0)

        # 실측으로 벽/바닥을 걸러낸 값. 정면 벽 x~4.7, 옆 벽 y~+2.3/-2.4,
        # 바닥 z~0.2 아래. 라이다를 옮기면 다시 맞춰야 한다.
        self.declare_parameter('roi_x_min', 0.0)
        self.declare_parameter('roi_x_max', 4.5)
        self.declare_parameter('roi_y_min', -2.15)
        self.declare_parameter('roi_y_max', 2.1)
        self.declare_parameter('roi_z_min', 0.25)
        self.declare_parameter('roi_z_max', 2.0)

        # 원점 근처의 자기 반사/무효점 제거용 최소 거리.
        self.declare_parameter('min_range', 0.1)
        # 이 자리 주변은 통째로 뺀다. map 좌표 [x1,y1, x2,y2, ...].
        # 라이다를 마주 보게 놓으면 서로를 물체로 잡는다. 실제로 라이다2 가
        # 라이다1 을 55점짜리 군집으로 계속 잡았다. 삼각대와 케이블도 같이
        # 걸리므로 반경으로 빼는 편이 ROI 를 줄이는 것보다 낫다.
        self.declare_parameter('exclude_xy', [0.0])
        self.declare_parameter('exclude_radius', 0.35)

        gp = lambda n: self.get_parameter(n).value
        self.out_frame = str(gp('output_frame'))
        self._yaw_cached = None
        self._rot = None
        self._refresh_params()

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self._pub = self.create_publisher(
            PointCloud2, str(gp('output_topic')), sensor_qos)
        self._sub = self.create_subscription(
            PointCloud2, str(gp('input_topic')), self._cb, sensor_qos)

        self._n_in = 0
        self._n_out = 0
        self._frames = 0
        self._proc_ms = []
        self._latency_ms = []
        self._recv_gap_ms = []
        self._prev_recv = None
        self.create_timer(2.0, self._report)

        self.get_logger().info(
            f'ROI filter: {gp("input_topic")} -> {gp("output_topic")} '
            f'[{self.out_frame}], lidar_z={self.lidar_z:.2f}, '
            f'yaw={self.yaw_deg:.2f}deg, origin=('
            f'{self.origin[0]:.3f}, {self.origin[1]:.3f})')
        self.get_logger().info(
            f'box x[{self.lo[0]:.1f},{self.hi[0]:.1f}] '
            f'y[{self.lo[1]:.1f},{self.hi[1]:.1f}] '
            f'z[{self.lo[2]:.1f},{self.hi[2]:.1f}]')

        # 보정값이 안 실렸는지 검사한다.
        #
        # params 파일은 노드 이름으로 매칭된다. YAML 키가 'roi_filter:' 인데
        # 노드를 roi_filter_1 로 띄우면 아무 경고 없이 통째로 무시되고
        # 기본값으로 돈다. 실제로 이것 때문에 두 라이다 점군이 2 m 어긋나
        # 보였고, 원인을 찾는 데 한참 걸렸다. 조용한 실패는 다시 겪지 않는다.
        if abs(self.origin[0]) < 1e-9 and abs(self.origin[1]) < 1e-9:
            self.get_logger().warn(
                'origin 이 정확히 (0,0) 입니다 — 보정값이 안 실렸을 수 있습니다.')
            self.get_logger().warn(
                '  params 파일의 최상위 키가 "/**:" 인지, --params-file 경로가 '
                '맞는지 확인하세요.')
            self.get_logger().warn(
                f'  (지금 이 노드 이름: {self.get_name()})')

    def _refresh_params(self):
        """파라미터를 매 프레임 다시 읽는다. 덕분에 노드를 재시작하지 않고
        ros2 param set 으로 박스를 바로 조절할 수 있다."""
        gp = lambda n: self.get_parameter(n).value
        self.lidar_z = float(gp('lidar_z'))
        self.min_range = float(gp('min_range'))
        ex = [float(v) for v in (gp('exclude_xy') or [])]
        # 짝이 안 맞으면 마지막 하나는 버린다
        self._excl = (np.asarray(ex[:len(ex) // 2 * 2], dtype=np.float32).reshape(-1, 2)
                      if len(ex) >= 2 else None)
        self._excl_r = float(gp('exclude_radius'))
        self.origin = np.array(
            [gp('origin_x'), gp('origin_y')], dtype=np.float32)
        self.lo = np.array(
            [gp('roi_x_min'), gp('roi_y_min'), gp('roi_z_min')], dtype=np.float32)
        self.hi = np.array(
            [gp('roi_x_max'), gp('roi_y_max'), gp('roi_z_max')], dtype=np.float32)

        yaw = float(gp('yaw_deg'))
        if yaw != self._yaw_cached:
            c = np.cos(np.radians(yaw))
            s = np.sin(np.radians(yaw))
            # z축 회전을 xy 에만 적용. (x', y') = (cx - sy, sx + cy)
            self._rot = np.array([[c, -s], [s, c]], dtype=np.float32)
            self._yaw_cached = yaw
            self.yaw_deg = yaw

    def _cb(self, msg):
        t_start = time.perf_counter()

        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        now = self.get_clock().now().nanoseconds * 1e-9
        self._latency_ms.append((now - stamp) * 1e3)
        if self._prev_recv is not None:
            self._recv_gap_ms.append((t_start - self._prev_recv) * 1e3)
        self._prev_recv = t_start

        try:
            self._process(msg)
        finally:
            self._proc_ms.append((time.perf_counter() - t_start) * 1e3)

    def _process(self, msg):
        self._refresh_params()
        arr = pc2_util.read_points(
            msg, field_names=('x', 'y', 'z'), skip_nans=True)
        if arr is None or arr.size == 0:
            return

        pts = np.stack([arr['x'], arr['y'], arr['z']], axis=-1)
        pts = np.asarray(pts, dtype=np.float32).reshape(-1, 3)
        pts = pts[np.isfinite(pts).all(axis=1)]
        if len(pts) == 0:
            return

        # 원점 근처 무효점 제거는 라이다 좌표에서 해야 한다.
        pts = pts[np.linalg.norm(pts, axis=1) >= self.min_range]
        if len(pts) == 0:
            return

        self._n_in += len(pts)

        pts[:, :2] = pts[:, :2] @ self._rot.T - self.origin
        pts[:, 2] += self.lidar_z
        keep = np.all((pts >= self.lo) & (pts <= self.hi), axis=1)
        if self._excl is not None and self._excl_r > 0:
            for cx, cy in self._excl:
                keep &= (np.hypot(pts[:, 0] - cx, pts[:, 1] - cy) > self._excl_r)
        roi = pts[keep]

        self._n_out += len(roi)
        self._frames += 1

        header = Header()
        header.stamp = msg.header.stamp
        header.frame_id = self.out_frame
        self._pub.publish(pc2_util.create_cloud_xyz32(header, roi))

    def _report(self):
        if self._frames == 0:
            self.get_logger().warn('입력 없음: /lidar1/points 확인 필요')
            self._proc_ms.clear()
            self._latency_ms.clear()
            self._recv_gap_ms.clear()
            return
        avg_in = self._n_in / self._frames
        avg_out = self._n_out / self._frames
        pct = 100.0 * avg_out / avg_in if avg_in else 0.0

        line = f'{self._frames}프레임: {avg_in:.0f} -> {avg_out:.0f}점 ({pct:.1f}%)'
        if self._proc_ms:
            proc = np.asarray(self._proc_ms)
            lat = np.asarray(self._latency_ms)
            line += f'  |  처리 {proc.mean():.1f}/{proc.max():.1f}ms'
            if self._recv_gap_ms:
                gap = np.asarray(self._recv_gap_ms)
                line += f'  |  입력 {1000.0 / gap.mean():.1f}Hz'
            line += f'  |  지연 {lat.mean():.0f}/{lat.max():.0f}ms'
        self.get_logger().info(line)

        self._n_in = self._n_out = self._frames = 0
        self._proc_ms.clear()
        self._latency_ms.clear()
        self._recv_gap_ms.clear()


def main(args=None):
    rclpy.init(args=args)
    node = RoiFilter()
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
