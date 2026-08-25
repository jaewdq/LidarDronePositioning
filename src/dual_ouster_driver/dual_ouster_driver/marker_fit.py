#!/usr/bin/env python3

"""원기둥 마커의 중심을 원 피팅으로 추정한다.

라이다는 물체의 앞면만 본다. 그래서 점들의 median 을 중심으로 쓰면 실제
중심보다 라이다 쪽으로 반지름만큼 치우친다. 원기둥은 이 편향을 없앨 수
있다 - 보이는 호에 원을 맞추면 가려진 뒤쪽까지 포함한 진짜 중심이 나온다.

세로 방향은 어느 높이에서 잘라도 같은 원이므로, 모든 높이의 점을 XY 로
투영해 한꺼번에 맞춘다. 점이 10~20배로 늘어 훨씬 안정적이다.

  입력  /lidar1/roi_points  (map 프레임, roi_filter 출력)
  출력  로그에 중심/반지름/잔차, /lidar1/marker_markers 에 시각화

사용:
    # 1단계 - 반지름을 모를 때 (반지름까지 추정)
    ros2 run dual_ouster_driver marker_fit

    # 2단계 - 반지름을 아는 뒤 (중심만 추정, 더 안정적)
    ros2 run dual_ouster_driver marker_fit --ros-args -p radius:=0.15
"""

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from scipy.optimize import least_squares
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2_util
from sklearn.cluster import DBSCAN
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray


def fit_circle(xy, r_fixed=None):
    """XY 점들에 원을 맞춘다.

    r_fixed 가 None 이면 반지름까지 추정하고, 값이 있으면 중심만 찾는다.
    반지름을 고정하면 미지수가 2개로 줄어 짧은 호에서도 안정적이다.
    반환: (cx, cy, r, rms잔차)
    """
    x, y = xy[:, 0], xy[:, 1]

    # 초기값: 대수적 원 피팅(Kasa). 선형 최소제곱이라 닫힌 해가 나온다.
    A = np.column_stack([x, y, np.ones(len(x))])
    b = x ** 2 + y ** 2
    try:
        sol, *_ = np.linalg.lstsq(A, b, rcond=None)
        cx0, cy0 = sol[0] / 2.0, sol[1] / 2.0
        r0 = np.sqrt(max(sol[2] + cx0 ** 2 + cy0 ** 2, 1e-6))
    except np.linalg.LinAlgError:
        cx0, cy0 = x.mean(), y.mean()
        r0 = np.std(np.hypot(x - cx0, y - cy0)) + 0.1

    if r_fixed is None:
        def resid(p):
            return np.hypot(x - p[0], y - p[1]) - p[2]
        out = least_squares(resid, [cx0, cy0, r0], method='lm')
        cx, cy, r = out.x
    else:
        r = float(r_fixed)
        def resid(p):
            return np.hypot(x - p[0], y - p[1]) - r
        out = least_squares(resid, [cx0, cy0], method='lm')
        cx, cy = out.x

    rms = float(np.sqrt(np.mean(out.fun ** 2)))
    return float(cx), float(cy), float(r), rms


class MarkerFit(Node):

    def __init__(self):
        super().__init__('marker_fit')

        self.declare_parameter('input_topic', '/lidar1/roi_points')
        # 0 이면 반지름도 추정, >0 이면 그 값으로 고정.
        self.declare_parameter('radius', 0.0)
        # 군집화
        self.declare_parameter('eps', 0.15)
        self.declare_parameter('min_samples', 5)
        self.declare_parameter('min_points', 30)
        # 마커보다 큰 덩어리(벽/바닥)를 거른다. 지름의 2배쯤이 적당.
        self.declare_parameter('max_span', 0.50)
        # 이보다 잔차가 크면 원기둥이 아니라고 보고 버린다.
        self.declare_parameter('max_rms', 0.05)
        # 이 프레임 수만큼 모아 평균과 표준편차를 낸다.
        self.declare_parameter('n_average', 20)
        # 원 피팅에 쓸 높이 범위(map z). 바닥/천장 잡음을 뺀다.
        self.declare_parameter('z_min', 0.25)
        self.declare_parameter('z_max', 1.60)
        # record 서비스로 저장할 파일. pose_solve 가 이걸 읽는다.
        self.declare_parameter('record_file', '/tmp/marker_records.txt')
        # 기록을 허용할 최대 흔들림(m). 이보다 흔들리면 거부한다.
        self.declare_parameter('record_max_std', 0.03)

        gp = lambda n: self.get_parameter(n).value
        self._in = str(gp('input_topic'))
        self.n_avg = int(gp('n_average'))

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST, depth=1)
        self.create_subscription(PointCloud2, self._in, self._cb, qos)
        self._pub = self.create_publisher(
            MarkerArray, '/lidar1/marker_markers', 10)

        self._acc = []
        self._last = None          # 마지막 평균 결과 (record 용)
        self._n_drop = 0
        self._frame = 'map'
        self.create_service(Trigger, '~/record', self._srv_record)
        self.get_logger().info(
            f'marker_fit: {self._in}, radius='
            + ('추정' if float(gp('radius')) <= 0 else f'{gp("radius"):.3f}m 고정')
            + f', {self.n_avg}프레임 평균')
        self.get_logger().info('마커를 ROI 안에 세워두세요.')

    def _cb(self, msg):
        gp = lambda n: self.get_parameter(n).value
        self._frame = msg.header.frame_id or 'map'

        arr = pc2_util.read_points(
            msg, field_names=('x', 'y', 'z'), skip_nans=True)
        if arr is None or arr.size == 0:
            return
        p = np.stack([arr['x'], arr['y'], arr['z']], axis=-1)
        p = np.asarray(p, dtype=np.float64).reshape(-1, 3)
        p = p[np.isfinite(p).all(axis=1)]
        p = p[(p[:, 2] >= float(gp('z_min'))) & (p[:, 2] <= float(gp('z_max')))]
        if len(p) < int(gp('min_points')):
            return

        labels = DBSCAN(eps=float(gp('eps')),
                        min_samples=int(gp('min_samples')),
                        n_jobs=-1).fit_predict(p)

        r_par = float(gp('radius'))
        r_fixed = r_par if r_par > 0 else None
        min_pts = int(gp('min_points'))
        max_span = float(gp('max_span'))
        max_rms = float(gp('max_rms'))

        # 가장 큰 덩어리가 아니라 '원기둥처럼 생긴' 덩어리를 고른다.
        # 벽이나 바닥은 크기로 먼저 거르고, 남은 것 중 잔차가 가장 작은 것을 쓴다.
        best, best_fit, best_rms = None, None, np.inf
        for l in np.unique(labels):
            if l < 0:
                continue
            c = p[labels == l]
            if len(c) < min_pts:
                continue
            ext = c.max(axis=0) - c.min(axis=0)
            if max(ext[0], ext[1]) > max_span:
                continue
            try:
                fit = fit_circle(c[:, :2], r_fixed)
            except Exception:
                continue
            if fit[3] < best_rms:
                best, best_fit, best_rms = c, fit, fit[3]

        if best is None:
            return
        if best_rms > max_rms:
            self.get_logger().warn(
                f'원기둥 같은 덩어리를 못 찾음 (최선 잔차 {best_rms*1000:.0f}mm)',
                throttle_duration_sec=5.0)
            return
        cx, cy, r, rms = best_fit
        best_n = len(best)

        med = np.median(best, axis=0)
        self._acc.append((cx, cy, r, rms, med[0], med[1], best_n, med[2]))
        self._publish(cx, cy, r, med)

        if len(self._acc) >= self.n_avg:
            self._report()
            self._acc.clear()

    def _report(self):
        a = np.asarray(self._acc)

        # 가끔 다른 물체를 골라 튀는 프레임이 섞인다. 중앙값에서 크게
        # 벗어난 프레임을 빼고 나서 평균을 낸다.
        mx_, my_ = np.median(a[:, 0]), np.median(a[:, 1])
        d = np.hypot(a[:, 0] - mx_, a[:, 1] - my_)
        keep = d <= max(0.05, 3.0 * np.median(d) + 1e-6)
        n_drop = int((~keep).sum())
        if keep.sum() >= 3:
            a = a[keep]
        else:
            n_drop = 0

        cx, cy, r, rms = a[:, 0], a[:, 1], a[:, 2], a[:, 3]
        mx, my = a[:, 4], a[:, 5]
        n = a[:, 6]
        self._n_drop = n_drop

        # 원 피팅 중심과 단순 median 의 차이 = 표면 편향
        bias = np.hypot(cx.mean() - mx.mean(), cy.mean() - my.mean())
        dist = np.hypot(cx.mean(), cy.mean())

        self._last = (cx.mean(), cy.mean(), r.mean(), cx.std(), cy.std(), len(a))

        lines = [
            '',
            '─' * 58,
            f'  마커 측정 결과   ({len(a)}프레임 평균, 점 {n.mean():.0f}개)'
            + (f'   [이상 {self._n_drop}프레임 제외]' if self._n_drop else ''),
            '─' * 58,
            f'  원 피팅 중심   x = {cx.mean():8.4f}  ± {cx.std():.4f} m',
            f'                 y = {cy.mean():8.4f}  ± {cy.std():.4f} m',
            f'  반지름         r = {r.mean():8.4f}  ± {r.std():.4f} m'
            f'   (지름 {2 * r.mean():.3f} m)',
            f'  피팅 잔차 RMS      {rms.mean() * 1000:6.1f} mm'
            + ('   원기둥으로 양호' if rms.mean() < 0.02 else '   ⚠ 큼 - 테이퍼/기울어짐 의심'),
            '',
            f'  단순 median    x = {mx.mean():8.4f}   y = {my.mean():8.4f}',
            f'  표면 편향          {bias * 1000:6.1f} mm  ← 원 피팅이 보정한 양',
            '',
            f'  라이다로부터 거리  {dist:.3f} m',
            '─' * 58,
        ]
        self.get_logger().info('\n'.join(lines))

    def _srv_record(self, req, resp):
        if self._last is None:
            resp.success = False
            resp.message = '아직 측정 결과가 없습니다. 마커가 보이는지 확인하세요.'
            return resp
        cx, cy, r, sx, sy, n = self._last
        limit = float(self.get_parameter('record_max_std').value)
        if max(sx, sy) > limit:
            resp.success = False
            resp.message = (
                f'흔들림이 큽니다 ({sx*1000:.0f}/{sy*1000:.0f}mm > {limit*1000:.0f}mm). '
                '마커가 안정적으로 잡히는지 확인하고 다시 시도하세요.')
            self.get_logger().warn(resp.message)
            return resp
        path = str(self.get_parameter('record_file').value)
        with open(path, 'a') as fp:
            fp.write(f'{cx:.6f} {cy:.6f} {r:.6f} {sx:.6f} {sy:.6f} {n}\n')
        cnt = sum(1 for _ in open(path))
        resp.success = True
        resp.message = (f'#{cnt} 저장: x={cx:.4f} y={cy:.4f} '
                        f'(흔들림 {sx*1000:.1f}/{sy*1000:.1f}mm)')
        self.get_logger().info(resp.message)
        return resp

    def _publish(self, cx, cy, r, med):
        stamp = self.get_clock().now().to_msg()
        ma = MarkerArray()

        cyl = Marker()
        cyl.header.stamp = stamp
        cyl.header.frame_id = self._frame
        cyl.ns = 'marker_fit'
        cyl.id = 0
        cyl.type = Marker.CYLINDER
        cyl.action = Marker.ADD
        cyl.pose.position.x = cx
        cyl.pose.position.y = cy
        cyl.pose.position.z = float(med[2])
        cyl.pose.orientation.w = 1.0
        cyl.scale.x = cyl.scale.y = 2.0 * r
        cyl.scale.z = 0.05
        cyl.color.r, cyl.color.g, cyl.color.b, cyl.color.a = 0.1, 0.6, 1.0, 0.6
        ma.markers.append(cyl)

        txt = Marker()
        txt.header.stamp = stamp
        txt.header.frame_id = self._frame
        txt.ns = 'marker_fit_text'
        txt.id = 1
        txt.type = Marker.TEXT_VIEW_FACING
        txt.action = Marker.ADD
        txt.pose.position.x = cx
        txt.pose.position.y = cy
        txt.pose.position.z = float(med[2]) + 0.4
        txt.pose.orientation.w = 1.0
        txt.scale.z = 0.12
        txt.color.r = txt.color.g = txt.color.b = txt.color.a = 1.0
        txt.text = f'({cx:.3f}, {cy:.3f})  r={r:.3f}'
        ma.markers.append(txt)

        self._pub.publish(ma)


def main(args=None):
    rclpy.init(args=args)
    node = MarkerFit()
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
