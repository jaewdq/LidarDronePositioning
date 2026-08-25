#!/usr/bin/env python3

"""보정된 좌표가 맞는지 실물로 확인한다.

줄자로 잰 자리에 물체를 놓고 라이다가 뭐라고 하는지 본다. 보정 자체의
검증(calibrate 가 출력하는 잔차)은 "계산이 맞나"만 보지만, 이건 "실제
위치와 맞나"를 본다. 둘은 다른 질문이다.

  입력  /lidarN/points   (원본. roi_filter 의 ROI 설정에 방해받지 않게
                          보정값을 여기서 직접 적용한다)
  출력  터미널에 좌표와 오차

두 가지 중심을 같이 낸다:

  median   비행 파이프라인(cluster_median)이 쓰는 값 그대로.
           라이다는 물체 앞면만 보므로 반지름만큼 라이다 쪽으로 치우친다.
  원피팅   marker_radius 를 주면 계산한다. 가려진 뒷면까지 포함한 참 중심.
           원기둥일 때만 의미가 있다.

둘의 차이가 곧 그 물체의 surface bias 다.

사용:
    # 물체를 놓고 대략 어디인지만 알려주면 계속 읽어준다
    ros2 run dual_ouster_driver verify_pose --ros-args -p near:="[0.0, 2.0]"

    # 줄자로 잰 값을 주면 오차까지 계산한다
    ros2 run dual_ouster_driver verify_pose --ros-args \
        -p near:="[0.0, 2.0]" -p expect:="[0.0, 2.06, 0.0]" \
        -p marker_radius:=0.045

    # 바닥만 확인 (기울기 점검)
    ros2 run dual_ouster_driver verify_pose --ros-args -p floor_only:=true

Ctrl+C 로 끝내면 전체 통계가 나온다.
"""

import math
import os
import sys

import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from scipy.optimize import least_squares
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2_util
from sklearn.cluster import DBSCAN

TAN_DOWN = math.tan(math.radians(21.46))


def rot2(deg):
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    return np.array([[c, -s], [s, c]])


def fit_circle(xy, r_fixed):
    """calibrate 와 같은 방식. Kåsa 대수해로 초기값, 기하 잔차로 다듬기."""
    x, y = xy[:, 0], xy[:, 1]
    A = np.column_stack([x, y, np.ones(len(x))])
    b = x ** 2 + y ** 2
    try:
        sol, *_ = np.linalg.lstsq(A, b, rcond=None)
        cx0, cy0 = sol[0] / 2.0, sol[1] / 2.0
    except np.linalg.LinAlgError:
        cx0, cy0 = x.mean(), y.mean()
    out = least_squares(
        lambda p: np.hypot(x - p[0], y - p[1]) - r_fixed,
        [cx0, cy0], method='lm')
    return float(out.x[0]), float(out.x[1]), float(np.sqrt(np.mean(out.fun ** 2)))


class VerifyPose(Node):

    def __init__(self):
        super().__init__('verify_pose')
        num = ParameterDescriptor(dynamic_typing=True)

        self.declare_parameter('lidar_id', 1, num)
        self.declare_parameter('input_topic', '')
        self.declare_parameter('config', '')
        # 물체가 대략 어디인지 (map 좌표 x, y). 이 반경만 본다.
        self.declare_parameter('near', [0.0, 0.0])
        self.declare_parameter('radius', 0.6, num)
        # 줄자로 잰 참값 (x, y, z). 주면 오차를 계산한다. z 는 바닥 기준 물체 밑면.
        self.declare_parameter('expect', [999.0, 999.0, 999.0])
        # 원기둥이면 반지름(m). 0 이면 원 피팅을 건너뛴다.
        self.declare_parameter('marker_radius', 0.0, num)
        # 물체로 볼 높이 범위 (바닥 기준)
        self.declare_parameter('z_min', 0.06, num)
        self.declare_parameter('z_max', 1.50, num)
        self.declare_parameter('eps', 0.15, num)
        self.declare_parameter('min_samples', 4, num)
        self.declare_parameter('min_points', 10, num)
        self.declare_parameter('period', 1.0, num)
        self.declare_parameter('floor_only', False)

        gp = lambda k: self.get_parameter(k).value
        self.lid = int(gp('lidar_id'))
        topic = str(gp('input_topic')) or f'/lidar{self.lid}/points'
        self.floor_only = bool(gp('floor_only'))

        cfg = str(gp('config'))
        if not cfg:
            cfg = os.path.join(
                get_package_share_directory('dual_ouster_driver'),
                'config', f'calibration_lidar{self.lid}.yaml')
        with open(cfg) as f:
            p = yaml.safe_load(f)['roi_filter']['ros__parameters']
        self.lidar_z = float(p['lidar_z'])
        self.yaw = float(p['yaw_deg'])
        self.origin = np.array([p['origin_x'], p['origin_y']])
        self._rot = rot2(self.yaw)
        # 라이다는 원점 반대편에 있다 (map 좌표)
        self.lidar_xy = -self.origin

        self.near = np.array(gp('near'), dtype=float)[:2]
        ex = np.array(gp('expect'), dtype=float)
        self.expect = None if (ex >= 900).any() else ex[:3]

        self._acc = []          # (median xyz, fit xy, n)
        self._floor = []        # (거리고리 중앙, 그 고리 바닥 z)

        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=1)
        self._latest = None
        self.create_subscription(PointCloud2, topic, self._cb, qos)
        self.create_timer(float(gp('period')), self._tick)

        print('=' * 66)
        print('  좌표 검증')
        print('=' * 66)
        print(f'  보정 파일   {cfg}')
        print(f'  lidar_z {self.lidar_z:.4f}   yaw {self.yaw:.4f}   '
              f'origin ({self.origin[0]:+.4f}, {self.origin[1]:+.4f})')
        print(f'  라이다 위치 map ({self.lidar_xy[0]:+.3f}, {self.lidar_xy[1]:+.3f})')
        if self.floor_only:
            print('  모드        바닥만 검증 (기울기 점검)')
        else:
            print(f'  찾을 자리   ({self.near[0]:+.2f}, {self.near[1]:+.2f}) '
                  f'반경 {gp("radius"):.2f} m')
            if self.expect is not None:
                print(f'  참값(줄자)  ({self.expect[0]:+.3f}, '
                      f'{self.expect[1]:+.3f}, {self.expect[2]:+.3f})')
            r = float(gp('marker_radius'))
            print(f'  원 피팅     {"반지름 %.4f m 고정" % r if r > 0 else "안 함"}')
        print('=' * 66)
        print('  Ctrl+C 로 끝내면 통계가 나옵니다.\n')

    # ------------------------------------------------------------------
    def _cb(self, msg):
        arr = pc2_util.read_points(
            msg, field_names=('x', 'y', 'z'), skip_nans=True)
        if arr is None or arr.size == 0:
            return
        p = np.stack([arr['x'], arr['y'], arr['z']], axis=-1)
        p = np.asarray(p, dtype=np.float64).reshape(-1, 3)
        p = p[np.isfinite(p).all(axis=1)]
        if len(p) == 0:
            return
        # roi_filter 와 똑같은 변환. 돌리고 나서 빼야 한다.
        p[:, :2] = p[:, :2] @ self._rot.T - self.origin
        p[:, 2] += self.lidar_z
        self._latest = p

    # ------------------------------------------------------------------
    def _tick(self):
        p = self._latest
        if p is None:
            self.get_logger().warn('입력 없음 — 드라이버 확인')
            return
        if self.floor_only:
            self._check_floor(p)
        else:
            self._check_object(p)

    def _check_floor(self, p):
        """거리 고리별 바닥 높이. 평평하고 수평이면 전부 0 이어야 한다."""
        d = np.hypot(p[:, 0] - self.lidar_xy[0], p[:, 1] - self.lidar_xy[1])
        blind = self.lidar_z / TAN_DOWN
        rows = []
        for lo, hi in ((blind * 1.15, blind * 1.5), (blind * 1.5, blind * 2.0),
                       (blind * 2.0, blind * 2.7), (blind * 2.7, blind * 3.6)):
            ring = p[(d > lo) & (d < hi) & (np.abs(p[:, 2]) < 0.35)]
            if len(ring) < 100:
                continue
            h, e = np.histogram(ring[:, 2], bins=60)
            z = (e[h.argmax()] + e[h.argmax() + 1]) / 2
            rows.append(((lo + hi) / 2, z, len(ring)))
            self._floor.append(((lo + hi) / 2, z))
        if not rows:
            self.get_logger().warn('바닥 점 부족')
            return
        s = '  '.join(f'{r:.1f}m:{z * 1000:+5.0f}mm' for r, z, _ in rows)
        drift = max(z for _, z, _ in rows) - min(z for _, z, _ in rows)
        print(f'  바닥 z   {s}    최대편차 {drift * 1000:.0f}mm')

    def _check_object(self, p):
        gp = lambda k: self.get_parameter(k).value
        rad = float(gp('radius'))
        q = p[(np.hypot(p[:, 0] - self.near[0], p[:, 1] - self.near[1]) <= rad)
              & (p[:, 2] >= float(gp('z_min')))
              & (p[:, 2] <= float(gp('z_max')))]
        if len(q) < int(gp('min_points')):
            print(f'  물체 없음 (점 {len(q)}개) — near 위치나 z 범위 확인')
            return

        lab = DBSCAN(eps=float(gp('eps')),
                     min_samples=int(gp('min_samples')), n_jobs=-1).fit_predict(q)
        best = None
        for k in np.unique(lab):
            if k < 0:
                continue
            c = q[lab == k]
            if len(c) < int(gp('min_points')):
                continue
            if best is None or len(c) > len(best):
                best = c
        if best is None:
            print(f'  군집 없음 (점 {len(q)}개)')
            return

        med = np.median(best, axis=0)
        r = float(gp('marker_radius'))
        fit = rms = None
        if r > 0:
            try:
                cx, cy, rms = fit_circle(best[:, :2], r)
                fit = np.array([cx, cy])
            except Exception:
                fit = None
        self._acc.append((med, fit, len(best), best[:, 2].min(), best[:, 2].max()))

        line = (f'  median ({med[0]:+.3f}, {med[1]:+.3f})  '
                f'z {best[:, 2].min():.3f}~{best[:, 2].max():.3f}  {len(best)}점')
        if fit is not None:
            line += f'   원피팅 ({fit[0]:+.3f}, {fit[1]:+.3f}) 잔차 {rms * 1000:.1f}mm'
        print(line)

    # ------------------------------------------------------------------
    def close(self):
        print('\n' + '=' * 66)
        if self.floor_only:
            if not self._floor:
                print('  바닥 측정 없음')
            else:
                a = np.array(self._floor)
                print('  바닥 검증 결과')
                print('=' * 66)
                for d0 in sorted(set(np.round(a[:, 0], 1))):
                    m = np.abs(a[:, 0] - d0) < 0.05
                    print(f'    거리 {d0:4.1f} m :  z {a[m, 1].mean() * 1000:+6.1f} mm '
                          f'(표준편차 {a[m, 1].std() * 1000:.1f})')
                # 거리에 따라 흘러가면 기울기
                if len(set(np.round(a[:, 0], 1))) >= 2:
                    k = np.polyfit(a[:, 0], a[:, 1], 1)[0]
                    tilt = math.degrees(math.atan(abs(k)))
                    print(f'\n    거리에 따른 기울기  {k * 1000:+.1f} mm/m  '
                          f'=> 약 {tilt:.2f}도')
                    if tilt > 0.2:
                        print(f'    라이다가 기울어져 있습니다. '
                              f'20 m 에서 {20 * abs(k) * 100:.0f} cm 오차가 됩니다.')
                    else:
                        print('    수평 양호')
            print('=' * 66 + '\n')
            return

        if not self._acc:
            print('  측정된 것이 없습니다.')
            print('=' * 66 + '\n')
            return

        med = np.array([a[0] for a in self._acc])
        n = len(med)
        print(f'  검증 결과   {n} 회 측정')
        print('=' * 66)
        print(f'\n  [median]  비행 파이프라인이 내는 값')
        print(f'    x {med[:, 0].mean():+.4f} m   흔들림 {med[:, 0].std() * 1000:.1f} mm')
        print(f'    y {med[:, 1].mean():+.4f} m   흔들림 {med[:, 1].std() * 1000:.1f} mm')
        print(f'    z {med[:, 2].mean():+.4f} m   흔들림 {med[:, 2].std() * 1000:.1f} mm')

        fits = np.array([a[1] for a in self._acc if a[1] is not None])
        if len(fits):
            print(f'\n  [원 피팅]  가려진 뒷면까지 포함한 참 중심')
            print(f'    x {fits[:, 0].mean():+.4f} m   흔들림 {fits[:, 0].std() * 1000:.1f} mm')
            print(f'    y {fits[:, 1].mean():+.4f} m   흔들림 {fits[:, 1].std() * 1000:.1f} mm')
            b = np.hypot(fits[:, 0].mean() - med[:, 0].mean(),
                         fits[:, 1].mean() - med[:, 1].mean())
            print(f'\n    median 과의 차이 {b * 1000:.1f} mm  = 이 물체의 surface bias')
            print(f'    (라이다가 앞면만 보므로 median 은 라이다 쪽으로 치우칩니다)')

        if self.expect is not None:
            ref = fits.mean(axis=0) if len(fits) else med[:, :2].mean(axis=0)
            which = '원 피팅' if len(fits) else 'median'
            ex, ey, ez = self.expect
            dx, dy = ref[0] - ex, ref[1] - ey
            dz = med[:, 2].min() - ez     # 물체 밑면 기준
            print(f'\n  [줄자 대비 오차]   기준: {which}')
            print(f'    x  {dx * 1000:+7.1f} mm')
            print(f'    y  {dy * 1000:+7.1f} mm')
            print(f'    수평 거리 {math.hypot(dx, dy) * 1000:.1f} mm')
            print(f'    z(밑면) {dz * 1000:+7.1f} mm')
            d_l = math.hypot(ref[0] - self.lidar_xy[0], ref[1] - self.lidar_xy[1])
            print(f'\n    이 물체는 라이다에서 {d_l:.2f} m 거리입니다.')
            print(f'    yaw 오차 0.1도면 여기서 {d_l * math.radians(0.1) * 1000:.0f} mm 가 됩니다.')

        print('\n  [주의]  이 값은 물체를 놓은 자리와 줄자 측정의 정확도까지')
        print('          같이 반영합니다. 라이다만의 오차가 아닙니다.')
        print('=' * 66 + '\n')


def main(args=None):
    rclpy.init(args=args)
    node = VerifyPose()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        for fn in (node.destroy_node, rclpy.shutdown):
            try:
                fn()
            except BaseException:
                pass
    sys.exit(0)


if __name__ == '__main__':
    main()
