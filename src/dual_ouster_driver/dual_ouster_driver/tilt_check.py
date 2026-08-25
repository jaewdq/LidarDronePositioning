#!/usr/bin/env python3

"""라이다가 수평인지 실시간으로 보여준다. 마운트를 돌리면서 보면 된다.

바닥에 평면을 맞춰 그 법선이 수직에서 얼마나 벗어났는지 잰다. 평면 잔차가
작으면(1 cm 미만) 바닥은 평평한 것이고, 기울기는 라이다 자세 탓이다.

기울기는 z 를 거리에 비례해 틀어놓는다. 보정은 '바닥이 수평'이라는 가정
위에 서 있어서, 기울면 그 가정이 깨진다. x, y 는 거의 영향받지 않는다.

  1도 기울면   10 m 에서 17 cm,  20 m 에서 35 cm

사용:
    ros2 run dual_ouster_driver tilt_check
    ros2 run dual_ouster_driver tilt_check --ros-args -p lidar_ids:="[1,2]"

Ctrl+C 로 끝낸다.
"""

import math
import sys

import numpy as np
import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2_util

TAN_DOWN = math.tan(math.radians(21.46))


def fit_plane(pts, rng, iters=300, tol=0.02):
    """RANSAC 으로 큰 평면 하나. 반환 (법선, 내부점, 잔차)."""
    if len(pts) < 100:
        return None
    if len(pts) > 30000:
        pts = pts[rng.choice(len(pts), 30000, replace=False)]
    best = None
    for _ in range(iters):
        s = pts[rng.choice(len(pts), 3, replace=False)]
        nv = np.cross(s[1] - s[0], s[2] - s[0])
        nn = np.linalg.norm(nv)
        if nn < 1e-9:
            continue
        nv = nv / nn
        k = int((np.abs((pts - s[0]) @ nv) < tol).sum())
        if best is None or k > best[0]:
            best = (k, nv, s[0])
    if best is None:
        return None
    _, nv, p0 = best
    g = pts[np.abs((pts - p0) @ nv) < tol]
    if len(g) < 50:
        return None
    cen = g.mean(axis=0)
    # 공분산 최소 고유벡터가 평면 법선
    w, V = np.linalg.eigh(np.cov((g - cen).T))
    nv = V[:, 0]
    if nv[2] < 0:
        nv = -nv
    rms = float(np.sqrt(np.mean(((g - cen) @ nv) ** 2)))
    return nv, len(g), rms


class TiltCheck(Node):

    def __init__(self):
        super().__init__('tilt_check')
        num = ParameterDescriptor(dynamic_typing=True)
        self.declare_parameter('lidar_ids', [1, 2])
        self.declare_parameter('period', 2.0, num)
        # 바닥을 찾을 수평거리 범위. 사각지대 밖이어야 한다.
        self.declare_parameter('r_min', 2.4, num)
        self.declare_parameter('r_max', 7.0, num)

        gp = lambda k: self.get_parameter(k).value
        self.lids = [int(v) for v in gp('lidar_ids')]
        self.rng = np.random.default_rng(0)
        self._latest = {i: None for i in self.lids}

        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=1)
        for i in self.lids:
            self.create_subscription(
                PointCloud2, f'/lidar{i}/points',
                (lambda lid: (lambda m: self._cb(m, lid)))(i), qos)
        self.create_timer(float(gp('period')), self._tick)

        print('=' * 62)
        print('  라이다 수평 확인 — 마운트를 돌리면서 보세요')
        print('=' * 62)
        print('  기울기가 0.2도 아래면 충분합니다.')
        print('  평면 잔차가 크면(>15mm) 바닥이 평평하지 않은 것이니')
        print('  다른 자리에서 다시 재세요.\n')

    def _cb(self, msg, lid):
        arr = pc2_util.read_points(
            msg, field_names=('x', 'y', 'z'), skip_nans=True)
        if arr is None or arr.size == 0:
            return
        p = np.stack([arr['x'], arr['y'], arr['z']], axis=-1)
        p = np.asarray(p, dtype=np.float64).reshape(-1, 3)
        self._latest[lid] = p[np.isfinite(p).all(axis=1)]

    def _tick(self):
        gp = lambda k: self.get_parameter(k).value
        r0, r1 = float(gp('r_min')), float(gp('r_max'))
        line = []
        for lid in self.lids:
            p = self._latest.get(lid)
            if p is None:
                line.append(f'라이다{lid} 입력없음')
                continue
            r = np.hypot(p[:, 0], p[:, 1])
            # 바닥 후보: 사각지대 밖의 아래쪽 점
            c = p[(r > r0) & (r < r1) & (p[:, 2] < -0.4) & (p[:, 2] > -1.6)]
            res = fit_plane(c, self.rng)
            if res is None:
                line.append(f'라이다{lid} 바닥 못찾음')
                continue
            nv, n, rms = res
            tilt = math.degrees(math.acos(min(1.0, abs(nv[2]))))
            azi = math.degrees(math.atan2(nv[1], nv[0]))
            mark = 'OK ' if tilt < 0.2 else ('..  ' if tilt < 0.5 else '기움')
            line.append(f'라이다{lid} {tilt:5.2f}도 {mark} '
                        f'(방위{azi:+4.0f} 잔차{rms*1000:4.1f}mm {n}점)')
        print('  ' + '   |   '.join(line))


def main(args=None):
    rclpy.init(args=args)
    node = TiltCheck()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('\n끝')
    finally:
        for fn in (node.destroy_node, rclpy.shutdown):
            try:
                fn()
            except BaseException:
                pass
    sys.exit(0)


if __name__ == '__main__':
    main()
