#!/usr/bin/env python3

"""라이다 주변 형상을 한 번 측정해 ROI 설정에 필요한 값을 뽑는다.

라이다를 새 위치에 놓을 때마다 실행한다. 출력은 세 가지다.

  1) 바닥 높이  -> lidar_z 파라미터에 넣을 값
  2) 위에서 본 점유 지도 -> 벽이 어디 있는지
  3) 추천 ROI 박스 -> 벽 안쪽으로 여유를 두고 자른 값

높이 밴드를 라이다 높이 근처로 좁혀서 본다. 그래야 바닥/천장이 빠지고
벽만 남아 지도가 읽힌다.

사용:
    ros2 run dual_ouster_driver room_survey
    ros2 run dual_ouster_driver room_survey --ros-args -p yaw_deg:=180.0
"""

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2_util

DIRS = [
    ('+x 정면', 0), ('   45도', 45), ('+y 왼쪽', 90), ('  135도', 135),
    ('-x 뒤  ', 180), ('  225도', 225), ('-y 오른쪽', 270), ('  315도', 315),
]


class RoomSurvey(Node):

    def __init__(self):
        super().__init__('room_survey')

        self.declare_parameter('input_topic', '/lidar1/points')
        # roi_filter 와 같은 값을 써야 결과가 일치한다.
        self.declare_parameter('yaw_deg', 180.0)
        # 벽을 볼 높이 밴드. 라이다 높이 기준 +-이 값.
        self.declare_parameter('wall_band', 0.4)
        # 지도 범위와 격자 수.
        self.declare_parameter('map_range', 5.0)
        self.declare_parameter('map_bins', 16)
        # 벽으로 인정할 격자당 최소 점 개수.
        self.declare_parameter('wall_min_points', 100)
        # 추천 ROI 를 벽에서 이만큼 안쪽으로 잡는다.
        self.declare_parameter('wall_margin', 0.25)
        # 바닥 위 이만큼은 잘라낸다(추천값 계산용).
        self.declare_parameter('floor_margin', 0.25)
        self.declare_parameter('repeat', False)

        gp = lambda n: self.get_parameter(n).value
        self.yaw = float(gp('yaw_deg'))
        self.band = float(gp('wall_band'))
        self.rng = float(gp('map_range'))
        self.bins = int(gp('map_bins'))
        self.wall_min = int(gp('wall_min_points'))
        self.margin = float(gp('wall_margin'))
        self.floor_margin = float(gp('floor_margin'))
        self.repeat = bool(gp('repeat'))
        self.done = False

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(PointCloud2, str(gp('input_topic')), self._cb, qos)
        self.get_logger().info(f'측정 대기 중... (yaw={self.yaw:.0f}도)')

    def _cb(self, msg):
        if self.done and not self.repeat:
            return

        arr = pc2_util.read_points(
            msg, field_names=('x', 'y', 'z'), skip_nans=True)
        if arr is None or arr.size == 0:
            return
        p = np.stack([arr['x'], arr['y'], arr['z']], axis=-1)
        p = np.asarray(p, dtype=np.float64).reshape(-1, 3)
        p = p[np.isfinite(p).all(axis=1)]
        p = p[np.linalg.norm(p, axis=1) > 0.05]
        if len(p) < 1000:
            return

        c, s = np.cos(np.radians(self.yaw)), np.sin(np.radians(self.yaw))
        p[:, :2] = p[:, :2] @ np.array([[c, -s], [s, c]]).T

        out = ['', '=' * 62, f'  라이다 주변 측정  (유효점 {len(p)})', '=' * 62]
        lidar_z = self._floor(p, out)
        self._directions(p, out)
        cells = self._map(p, lidar_z, out)
        self._suggest(lidar_z, cells, out)
        self.get_logger().info('\n'.join(out))
        self.done = True

    # ------------------------------------------------------------------
    def _floor(self, p, out):
        """바닥은 수직 사각지대 바깥에서만 보인다. 그 고리에서 z 최빈값을 찾는다."""
        r = np.hypot(p[:, 0], p[:, 1])
        ring = p[(r > 2.5) & (r < 5.0)]
        lo = ring[ring[:, 2] < 0] if len(ring) else np.empty((0, 3))
        if len(lo) < 50:
            out.append('\n[바닥] 표본 부족 - 사각지대 밖 바닥이 안 보인다')
            out.append('       lidar_z 는 직접 재서 넣어야 한다')
            return None
        h, e = np.histogram(lo[:, 2], bins=60)
        fz = (e[h.argmax()] + e[h.argmax() + 1]) / 2
        out.append(f'\n[바닥]  z = {fz:.2f} m (라이다 기준)')
        out.append(f'        -> lidar_z:={-fz:.2f}')
        blind = -fz / np.tan(np.radians(21.2))
        out.append(f'        수직 사각지대 반경 약 {blind:.1f} m (이 안쪽 바닥은 안 보임)')
        return -fz

    def _directions(self, p, out):
        band = p[np.abs(p[:, 2]) <= 0.3]
        if len(band) == 0:
            return
        ang = np.degrees(np.arctan2(band[:, 1], band[:, 0])) % 360.0
        rr = np.hypot(band[:, 0], band[:, 1])
        out.append('\n[방향별 최근접 거리]  라이다 높이 +-0.3 m')
        for lab, ctr in DIRS:
            d = np.abs((ang - ctr + 180.0) % 360.0 - 180.0)
            sel = rr[d <= 22.5]
            if len(sel) == 0:
                out.append(f'  {lab}      없음')
                continue
            bar = '#' * int(30 * (1.0 - min(sel.min(), 5.0) / 5.0))
            out.append(f'  {lab}  {sel.min():6.2f} m  (점 {len(sel):6d})  {bar}')

    def _map(self, p, lidar_z, out):
        """라이다 높이 밴드의 점을 위에서 본 격자로 만든다."""
        if lidar_z is None:
            band = p[np.abs(p[:, 2]) <= self.band]
        else:
            band = p[np.abs(p[:, 2]) <= self.band]
        R = self.rng
        H, xe, ye = np.histogram2d(
            band[:, 0], band[:, 1], bins=[self.bins, self.bins],
            range=[[-R, R], [-R, R]])

        out.append(f'\n[위에서 본 지도]  세로=x(정면), 가로=y(왼쪽+)')
        out.append(f'  숫자=점개수/100,  +=1~{self.wall_min - 1},  .=없음')
        hdr = '        ' + ''.join(
            f'{ye[i]:5.1f}' for i in range(0, self.bins, 2))
        out.append(hdr)
        for i in range(self.bins - 1, -1, -1):
            row = f'  x{xe[i]:5.1f} '
            for j in range(self.bins):
                v = H[i, j]
                row += '    .' if v == 0 else (
                    f'{int(v / 100):5d}' if v >= self.wall_min else '    +')
            out.append(row)

        # 벽 후보 격자의 중심 좌표를 모은다.
        occ = H >= self.wall_min
        cells = []
        if occ.any():
            xi, yi = np.where(occ)
            cells = list(zip((xe[xi] + xe[xi + 1]) / 2,
                             (ye[yi] + ye[yi + 1]) / 2))
        return cells

    def _suggest(self, lidar_z, cells, out):
        """벽에 막히지 않는 앞쪽 빈 상자를 찾는다.

        가장 먼 점유 격자가 아니라 가장 가까운 것을 기준으로 삼아야 한다.
        먼저 좌우 한계를 정하고, 그 폭 안에서 정면 한계를 찾는다."""
        out.append('')
        out.append('[추천 ROI]  벽에서 %.2f m 안쪽' % self.margin)
        if not cells:
            out.append('  벽을 못 찾음 - wall_min_points 를 낮춰볼 것')
            return

        R = self.rng
        fwd = [(x, y) for x, y in cells if x > 0.3]
        if not fwd:
            out.append('  정면에 점유 격자가 없음')
            return

        # 좌우: 중심선에서 바깥으로 나가다 처음 만나는 벽
        ys_pos = [y for _, y in fwd if y > 0.5]
        ys_neg = [y for _, y in fwd if y < -0.5]
        y_hi = min(ys_pos) if ys_pos else R
        y_lo = max(ys_neg) if ys_neg else -R

        # 정면: 위에서 정한 폭 안에서 처음 만나는 벽
        xs = [x for x, y in fwd if y_lo < y < y_hi]
        x_hi = min(xs) if xs else R

        roi = {
            'roi_x_min': 0.0,
            'roi_x_max': max(0.5, x_hi - self.margin),
            'roi_y_min': min(-0.5, y_lo + self.margin),
            'roi_y_max': max(0.5, y_hi - self.margin),
            'roi_z_min': self.floor_margin,
            'roi_z_max': 2.0,
        }
        out.append(f'  감지된 벽:  정면 x={x_hi:.2f}   왼쪽 y={y_hi:.2f}   오른쪽 y={y_lo:.2f}')
        out.append('')
        if lidar_z is not None:
            out.append(f'  -p lidar_z:={lidar_z:.2f}')
        for k, v in roi.items():
            out.append(f'  -p {k}:={v:.2f}')
        out.append('')
        out.append('  실행 중 적용:')
        args = '  '.join(f'{k} {v:.2f}' for k, v in roi.items())
        out.append(f'    ros2 param set /roi_filter roi_x_max {roi["roi_x_max"]:.2f}')
        out.append('=' * 62)


def main(args=None):
    rclpy.init(args=args)
    node = RoomSurvey()
    try:
        while rclpy.ok() and not (node.done and not node.repeat):
            rclpy.spin_once(node, timeout_sec=1.0)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
