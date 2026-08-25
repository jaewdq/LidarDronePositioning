#!/usr/bin/env python3

"""라이다 XY 좌표계 방향을 실측으로 확인하는 도구.

라이다 주위를 한 바퀴 8방향으로 나눠, 각 방향에서 가장 가까운 물체까지의
거리를 실시간으로 보여준다. 상자나 손을 특정 방향에 두고 어느 축이
반응하는지 보면 +x 가 실제로 어느 쪽인지 알 수 있다.

방위각(azimuth)은 +x 축에서 반시계 방향으로 잰다:
    0deg = +x,  90deg = +y,  180deg = -x,  270deg = -y

사용:
    ros2 run dual_ouster_driver axis_probe

높이 밴드(기본 라이다 높이 +-0.3 m)만 보므로 바닥/천장에 영향받지 않는다.
"""

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2_util

# 8방위. (라벨, 중심 방위각도)
SECTORS = [
    ('+x  (  0deg)', 0),
    ('    ( 45deg)', 45),
    ('+y  ( 90deg)', 90),
    ('    (135deg)', 135),
    ('-x  (180deg)', 180),
    ('    (225deg)', 225),
    ('-y  (270deg)', 270),
    ('    (315deg)', 315),
]


class AxisProbe(Node):

    def __init__(self):
        super().__init__('axis_probe')

        self.declare_parameter('input_topic', '/lidar1/points')
        # z축 회전(도). roi_filter 와 같은 값을 써야 결과가 일치한다.
        self.declare_parameter('yaw_deg', 180.0)
        # 라이다 자체 높이 기준 위아래로 이만큼만 본다.
        self.declare_parameter('z_band', 0.3)
        self.declare_parameter('min_range', 0.15)
        self.declare_parameter('max_range', 5.0)
        self.declare_parameter('report_period', 1.0)

        gp = lambda n: self.get_parameter(n).value
        self.z_band = float(gp('z_band'))
        self.min_range = float(gp('min_range'))
        self.max_range = float(gp('max_range'))
        self.yaw_deg = float(gp('yaw_deg'))
        c = np.cos(np.radians(self.yaw_deg))
        s = np.sin(np.radians(self.yaw_deg))
        self._rot = np.array([[c, -s], [s, c]], dtype=np.float32)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(
            PointCloud2, str(gp('input_topic')), self._cb, qos)

        self._latest = None
        self.create_timer(float(gp('report_period')), self._report)

        self.get_logger().info(
            '라이다 주위에 물체(상자/손)를 두고 어느 방향이 가까워지는지 보세요.')
        self.get_logger().info(
            f'높이 밴드 +-{self.z_band:.2f}m, 거리 {self.min_range:.2f}~{self.max_range:.1f}m, '
            f'yaw={self.yaw_deg:.0f}deg')

    def _cb(self, msg):
        arr = pc2_util.read_points(
            msg, field_names=('x', 'y', 'z'), skip_nans=True)
        if arr is None or arr.size == 0:
            return
        p = np.stack([arr['x'], arr['y'], arr['z']], axis=-1)
        p = np.asarray(p, dtype=np.float32).reshape(-1, 3)
        p = p[np.isfinite(p).all(axis=1)]

        # 라이다 자체 높이대만 남긴다 (입력은 라이다 좌표계이므로 z=0 이 라이다).
        p = p[np.abs(p[:, 2]) <= self.z_band]
        if len(p) == 0:
            return

        p[:, :2] = p[:, :2] @ self._rot.T

        r = np.hypot(p[:, 0], p[:, 1])
        keep = (r >= self.min_range) & (r <= self.max_range)
        self._latest = (p[keep], r[keep])

    def _report(self):
        if self._latest is None or len(self._latest[0]) == 0:
            self.get_logger().warn('점 없음 - /lidar1/points 확인')
            return
        p, r = self._latest
        ang = np.degrees(np.arctan2(p[:, 1], p[:, 0])) % 360.0

        lines = ['', '  방향            최근접거리   점개수']
        best_label, best_d = None, np.inf
        for label, center in SECTORS:
            d = np.abs((ang - center + 180.0) % 360.0 - 180.0)
            sel = r[d <= 22.5]
            if len(sel) == 0:
                lines.append(f'  {label}      -         0')
                continue
            near = float(sel.min())
            bar = '#' * max(0, int(40 * (1.0 - near / self.max_range)))
            lines.append(f'  {label}   {near:5.2f} m  {len(sel):6d}  {bar}')
            if near < best_d:
                best_label, best_d = label, near
        lines.append(f'  => 가장 가까운 방향: {best_label}  ({best_d:.2f} m)')
        self.get_logger().info('\n'.join(lines))


def main(args=None):
    rclpy.init(args=args)
    node = AxisProbe()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
