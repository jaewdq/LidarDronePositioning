#!/usr/bin/env python3

"""마커를 놓아가며 라이다 좌표계를 잡는 대화형 보정 도구.

원기둥 마커 하나를 두 자리에 차례로 놓으면 원점과 축이 정해진다.
맞춰야 할 정답이 없다 - 놓은 자리가 곧 정답이므로 배치 오차라는 개념이 없다.

    M2 ●
        \\
          \\   이 방향이 +x (또는 front_axis 로 지정한 축)
            \\
              ● M1     여기가 (0, 0)

절차:
    1) 바닥 높이       -> lidar_z
    2) 정면 방향       -> yaw 기준 (사람이 서서 확인, 건너뛸 수 있음)
    3) 마커 반지름     -> 원기둥인지도 같이 검증
    4) 원점 지점 측정
    5) +x 방향 지점 측정
    6) 계산 + 벽 탐색 -> ROI
    7) YAML 저장

사용:
    ros2 run dual_ouster_driver calibrate
    ros2 run dual_ouster_driver calibrate --ros-args -p front_axis:=+y -p lidar_id:=1
    ros2 run dual_ouster_driver calibrate --ros-args -p skip_front_check:=true -p yaw_base:=180.0

라이다는 아무 방향으로 놓아도 된다. 어디를 보고 있는지 측정해서 보정하는 것이
이 도구의 목적이다.
"""

import math
import os
import shutil
import sys
import threading
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from scipy.optimize import least_squares
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2_util
from sklearn.cluster import DBSCAN

# 8방위. (라벨, 중심 방위각)
SECTORS = [(f'{a}deg', a) for a in range(0, 360, 45)]

FRONT_AXIS_ROT = {'+x': 0.0, '+y': 90.0, '-x': 180.0, '-y': -90.0}


def rot2(deg):
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    return np.array([[c, -s], [s, c]])


def fit_circle(xy, r_fixed=None):
    """XY 점들에 원을 맞춘다. 반환 (cx, cy, r, rms).

    라이다는 물체 앞면만 보므로 median 을 쓰면 반지름만큼 치우친다.
    원을 맞추면 가려진 뒤쪽까지 포함한 진짜 중심이 나온다.
    반지름을 고정하면 미지수가 2개로 줄어 훨씬 안정적이다.
    """
    x, y = xy[:, 0], xy[:, 1]
    A = np.column_stack([x, y, np.ones(len(x))])
    b = x ** 2 + y ** 2
    try:
        sol, *_ = np.linalg.lstsq(A, b, rcond=None)
        cx0, cy0 = sol[0] / 2.0, sol[1] / 2.0
        r0 = math.sqrt(max(sol[2] + cx0 ** 2 + cy0 ** 2, 1e-6))
    except np.linalg.LinAlgError:
        cx0, cy0, r0 = x.mean(), y.mean(), 0.1

    if r_fixed is None:
        out = least_squares(
            lambda p: np.hypot(x - p[0], y - p[1]) - p[2],
            [cx0, cy0, r0], method='lm')
        cx, cy, r = out.x
    else:
        r = float(r_fixed)
        out = least_squares(
            lambda p: np.hypot(x - p[0], y - p[1]) - r,
            [cx0, cy0], method='lm')
        cx, cy = out.x
    return float(cx), float(cy), float(r), float(np.sqrt(np.mean(out.fun ** 2)))


class Calibrator(Node):

    def __init__(self):
        super().__init__('calibrate')

        # 숫자 파라미터는 10 과 10.0 을 모두 받게 한다. 그러지 않으면
        # -p countdown:=10 처럼 정수로 쓸 때 타입 오류로 죽는다.
        self._num = ParameterDescriptor(dynamic_typing=True)

        self.declare_parameter('lidar_id', 1, self._num)
        # 여러 대를 동시에 보정할 때. 비우면 lidar_id 한 대만 한다.
        # 같은 마커를 동시에 재므로 마커를 두 번만 놓으면 되고,
        # 따로 보정할 때 생기는 '두 번째에 마커가 어긋남' 오차가 없다.
        self.declare_parameter('lidar_ids', [0])
        self.declare_parameter('input_topic', '')
        self.declare_parameter('output_file', '')
        # M1 -> M2 방향을 최종 좌표계의 어느 축으로 삼을지.
        # 이름이 front_axis 지만 '라이다가 보는 방향'이 아니다. 좌표계는
        # 마커가 정의하므로, 라이다 두 대가 같은 마커와 같은 값을 쓰면
        # 서로 마주보고 있어도 같은 좌표계가 나온다.
        self.declare_parameter('front_axis', '+y')
        # 정면 확인 단계를 건너뛸 때 쓸 값. 건너뛰지 않으면 무시된다.
        self.declare_parameter('skip_front_check', False)
        self.declare_parameter('yaw_base', 180.0, self._num)
        # 0 보다 크면 자동 측정을 건너뛰고 이 값을 쓴다.
        # 경사진 바닥이거나 자동 측정이 실패할 때 줄자로 재서 넣는다.
        self.declare_parameter('lidar_z', 0.0, self._num)
        # 각 측정에 쓸 프레임 수. 많을수록 안정적이지만 오래 걸린다.
        self.declare_parameter('n_frames', 25, self._num)
        # 마커 검출
        self.declare_parameter('marker_z_min', 0.045, self._num)
        self.declare_parameter('marker_z_max', 0.60, self._num)
        self.declare_parameter('marker_max_span', 0.60, self._num)
        self.declare_parameter('eps', 0.15, self._num)
        self.declare_parameter('min_samples', 4, self._num)
        self.declare_parameter('min_points', 12, self._num)
        # 이보다 잔차가 크면 원기둥이 아니라고 보고 버린다.
        self.declare_parameter('max_rms', 0.05, self._num)
        # 마커 주변 이 반경 안에서만 찾는다. 어제 ROI 로 좁혀서 잘 됐던 것을
        # 자동화한 것이다. 범위를 좁히는 게 배경 차분보다 훨씬 강하다.
        self.declare_parameter('search_radius', 0.8, self._num)
        # 마커 반지름(m). 0 이면 라이다로 추정한다.
        # 줄자로 잰 값을 넣으면 추정 단계를 건너뛰고 더 정확해진다.
        # (추정은 점이 적으면 크게 흔들린다 — 실측 ±84mm 사례 있음)
        self.declare_parameter('marker_radius', 0.0, self._num)
        # 탐색 지점의 실제 바닥에서 이만큼 위부터 마커로 본다.
        # 고정 z 창을 쓰면 거리에 따라 바닥이 들어왔다 나갔다 한다.
        self.declare_parameter('ground_clear', 0.06, self._num)
        # 마커를 찾을 영역. 라이다 정면 쪽 직사각형으로 한정한다.
        #   정면 0 ~ search_fwd_max,  좌우 +-search_side_max
        # 뒤쪽/옆쪽 벽과 지나가는 사람을 애초에 배제한다.
        self.declare_parameter('search_fwd_max', 4.0, self._num)
        self.declare_parameter('search_side_max', 2.0, self._num)
        # Enter 를 누르고 이 초만큼 기다린 뒤 측정한다. 혼자 할 때 자리로
        # 걸어갈 시간이 필요하다(측정 구역 안에서는 Enter 를 못 누른다).
        self.declare_parameter('countdown', 0.0, self._num)
        # ROI 계산
        self.declare_parameter('wall_band', 0.40, self._num)
        self.declare_parameter('wall_min_points', 60, self._num)
        self.declare_parameter('wall_margin', 0.20, self._num)
        self.declare_parameter('grid', 0.10, self._num)
        self.declare_parameter('roi_z_min', 0.30, self._num)
        self.declare_parameter('roi_z_max', 2.50, self._num)

        gp = lambda n: self.get_parameter(n).value
        self.lid = int(gp('lidar_id'))

        ids = [int(v) for v in (gp('lidar_ids') or []) if int(v) > 0]
        self.lids = ids or [self.lid]
        self.lid = self.lids[0]

        self._lock = threading.Lock()
        self._latest = {i: None for i in self.lids}
        self._topics = {}
        # grab() 이 어느 라이다를 읽을지. 측정 직전에 바꿔 쓴다.
        self._active = self.lids[0]
        self.last_diag = None

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST, depth=1)
        override = str(gp('input_topic'))
        for i in self.lids:
            t = override if (override and len(self.lids) == 1) \
                else f'/lidar{i}/points'
            self._topics[i] = t
            self.create_subscription(
                PointCloud2, t,
                (lambda lid: (lambda m: self._cb(m, lid)))(i), qos)
        self._topic = self._topics[self.lids[0]]

    # ------------------------------------------------------------------
    def _cb(self, msg, lid=None):
        arr = pc2_util.read_points(
            msg, field_names=('x', 'y', 'z'), skip_nans=True)
        if arr is None or arr.size == 0:
            return
        p = np.stack([arr['x'], arr['y'], arr['z']], axis=-1)
        p = np.asarray(p, dtype=np.float64).reshape(-1, 3)
        p = p[np.isfinite(p).all(axis=1)]
        with self._lock:
            self._latest[lid if lid is not None else self._active] = p

    def grab(self, n=1, timeout=15.0):
        """최근 프레임을 n 개 모은다. 같은 프레임을 두 번 세지 않는다."""
        out, t0, last = [], time.time(), None
        while len(out) < n and time.time() - t0 < timeout:
            with self._lock:
                cur = self._latest.get(self._active)
            if cur is not None and cur is not last:
                out.append(cur)
                last = cur
            time.sleep(0.02)
        return out

    # ------------------------------------------------------------------
    TAN_DOWN = math.tan(math.radians(21.46))    # 아래쪽 빔 한계

    @staticmethod
    def _mode_z(pts, lo_r, hi_r):
        """주어진 수평거리 고리에서 아래쪽 z 최빈값을 찾는다."""
        r = np.hypot(pts[:, 0], pts[:, 1])
        ring = pts[(r > lo_r) & (r < hi_r)]
        lo = ring[ring[:, 2] < -0.05]
        if len(lo) < 50:
            return None
        h, e = np.histogram(lo[:, 2], bins=80)
        return float((e[h.argmax()] + e[h.argmax() + 1]) / 2)

    def measure_floor(self, yaw):
        """바닥 z 최빈값으로 설치 높이를 구한다.

        높이 h 로 달면 발밑 반경 h/tan(21.46도) 안쪽 바닥은 원리적으로 안 보인다.
        (2 m 설치 -> 5.1 m) 그래서 고정 고리로 뒤지면 높이 올렸을 때 통째로
        사각지대에 들어가 실패한다. 두 번 돌려서 고리를 높이에 맞춘다.
        """
        frames = self.grab(5)
        if not frames:
            return None
        p = np.vstack(frames)

        # 1차: 아래쪽 점 전체로 대략 추정
        z0 = self._mode_z(p, 0.5, 60.0)
        if z0 is None:
            return None
        h0 = -z0

        # 2차: 그 높이로 사각지대를 계산해 고리를 바깥으로 옮기고 다시
        blind = h0 / self.TAN_DOWN
        z1 = self._mode_z(p, blind * 1.15, blind * 4.0)
        if z1 is None:
            # 사각지대 밖에 바닥이 안 보인다(벽이 가깝거나 공간이 좁음).
            # 1차 추정으로 만족한다.
            return h0
        return -z1

    def sector_ranges(self, band=0.3):
        """8방위 최근접 거리. 정면 방향 확인용."""
        frames = self.grab(5)
        if not frames:
            return None
        p = np.vstack(frames)
        b = p[np.abs(p[:, 2]) <= band]
        if len(b) == 0:
            return None
        ang = np.degrees(np.arctan2(b[:, 1], b[:, 0])) % 360.0
        rr = np.hypot(b[:, 0], b[:, 1])
        out = {}
        for _, ctr in SECTORS:
            d = np.abs((ang - ctr + 180.0) % 360.0 - 180.0)
            sel = rr[d <= 22.5]
            out[ctr] = (float(sel.min()) if len(sel) else np.inf, int(len(sel)))
        return out

    def measure_marker(self, yaw, lidar_z, r_fixed, n, near=None, front_deg=0.0,
                       use_box=True):
        """마커 중심을 n 프레임 평균으로 구한다.

        가끔 다른 물체를 골라 튀는 프레임이 섞이므로 중앙값에서 벗어난 것은 뺀다.
        """
        gp = lambda k: self.get_parameter(k).value
        R = rot2(yaw)
        acc = []
        rad = float(gp('search_radius'))
        clear = float(gp('ground_clear'))
        fdir = rot2(front_deg) @ np.array([1.0, 0.0])
        ldir = rot2(front_deg + 90.0) @ np.array([1.0, 0.0])
        self.last_diag = None
        for raw in self.grab(n, timeout=n * 0.3 + 10):
            q = raw.copy()
            if len(q) == 0:
                continue
            q[:, :2] = q[:, :2] @ R.T
            q[:, 2] += lidar_z
            # 정면 쪽 직사각형으로 먼저 자른다. 이 밖은 마커일 수 없다.
            # (fwd>0 이 정면 반쪽을 만들고, 거기에 사각 한계를 씌운다)
            if use_box:
                fwd = q[:, 0] * fdir[0] + q[:, 1] * fdir[1]
                side = q[:, 0] * ldir[0] + q[:, 1] * ldir[1]
                q = q[(fwd > 0.0) & (fwd <= float(gp('search_fwd_max')))
                      & (np.abs(side) <= float(gp('search_side_max')))]
            else:
                # 상자 없이 360도. 대신 너무 먼 것만 자른다.
                q = q[np.hypot(q[:, 0], q[:, 1])
                      <= float(gp('search_fwd_max')) * 2.0]
            if len(q) == 0:
                continue
            if near is not None:
                q = q[np.hypot(q[:, 0] - near[0], q[:, 1] - near[1]) <= rad]
            q = q[q[:, 2] <= float(gp('marker_z_max'))]
            if len(q) == 0:
                continue

            # 바닥은 거리에 따라 보였다 안 보였다 하고 평탄하지도 않다.
            # 고정 z 창 대신 이 자리의 실제 바닥을 재서 그 위만 본다.
            low = q[q[:, 2] < 0.25]
            if len(low) >= 30:
                hh, ee = np.histogram(low[:, 2], bins=40)
                floor = (ee[hh.argmax()] + ee[hh.argmax() + 1]) / 2
                zmin = max(float(gp('marker_z_min')), floor + clear)
            else:
                zmin = float(gp('marker_z_min'))
            self.last_diag = (len(q), float(q[:, 2].min()),
                              float(q[:, 2].max()), zmin)
            q = q[q[:, 2] >= zmin]
            if len(q) < int(gp('min_points')):
                continue
            lab = DBSCAN(eps=float(gp('eps')),
                         min_samples=int(gp('min_samples')),
                         n_jobs=-1).fit_predict(q)
            # 잔차가 가장 작은 것을 고르면 안 된다. 점이 적고 얇은 물체일수록
            # 짧은 호가 어떤 원에도 잘 맞아 잔차가 작게 나오기 때문이다.
            # (실측: 가는 기둥 19점 잔차 2.1mm vs 물통 40점 잔차 14mm)
            # 배경을 뺀 뒤라 후보가 마커뿐이므로 점이 가장 많은 것을 고른다.
            for l in np.unique(lab):
                if l < 0:
                    continue
                c = q[lab == l]
                if len(c) < int(gp('min_points')):
                    continue
                ext = c.max(axis=0) - c.min(axis=0)
                if max(ext[0], ext[1]) > float(gp('marker_max_span')):
                    continue
                try:
                    fc = fit_circle(c[:, :2], r_fixed)
                except Exception:
                    continue
                if fc[3] > float(gp('max_rms')):
                    continue
                ext = c.max(axis=0) - c.min(axis=0)
                acc.append((fc[0], fc[1], fc[2], fc[3], len(c),
                            max(ext[0], ext[1]), ext[2]))

        if not acc:
            return []
        a = np.asarray(acc)

        # 라이다는 360도를 본다. 사람이 '옆으로 비켜도' 계속 보이므로 전경에
        # 마커 말고도 여럿 남는다. 프레임마다 잡힌 후보들을 위치로 묶어
        # 물체별로 정리한 뒤, 어느 것이 마커인지는 사람에게 묻는다.
        lab = DBSCAN(eps=0.30, min_samples=2).fit_predict(a[:, :2])
        cands = []
        for l in np.unique(lab):
            g = a[lab == l] if l >= 0 else None
            if g is None or len(g) < max(2, n // 6):
                continue
            mx, my = np.median(g[:, 0]), np.median(g[:, 1])
            d = np.hypot(g[:, 0] - mx, g[:, 1] - my)
            keep = d <= max(0.05, 3.0 * np.median(d) + 1e-6)
            gg = g[keep] if keep.sum() >= 2 else g
            cx_, cy_ = float(gg[:, 0].mean()), float(gg[:, 1].mean())
            cands.append({
                'x': cx_, 'y': cy_,
                'sx': float(gg[:, 0].std()), 'sy': float(gg[:, 1].std()),
                'r': float(gg[:, 2].mean()), 'sr': float(gg[:, 2].std()),
                'rms': float(gg[:, 3].mean()), 'n': int(gg[:, 4].mean()),
                'w': float(gg[:, 5].mean()), 'h': float(gg[:, 6].mean()),
                'frames': int(len(gg)), 'dropped': int(len(g) - len(gg)),
                'dist': float(np.hypot(cx_, cy_)),
                'fwd': float(cx_ * fdir[0] + cy_ * fdir[1]),
                'left': float(cx_ * ldir[0] + cy_ * ldir[1]),
            })
        # 전체 탐색일 때는 마커일 수 없는 것을 미리 거른다.
        if near is None:
            cands = [c for c in cands if 0.05 <= c['w'] <= 0.7]
        cands.sort(key=lambda c: c['dist'])
        return cands

    # ------------------------------------------------------------------
    def find_roi(self, yaw, lidar_z, origin):
        """벽에 막히지 않으면서 원점을 포함하는 최대 축정렬 박스를 찾는다.

        좌표계가 돌아가 있으면 방이 마름모꼴로 보여 축정렬 박스가 잘 안 맞는다.
        그래서 계산으로 최대 박스를 찾아준다.
        """
        gp = lambda k: self.get_parameter(k).value
        frames = self.grab(5)
        if not frames:
            return None
        R = rot2(yaw)
        pts = []
        for p in frames:
            q = p.copy()
            q[:, :2] = q[:, :2] @ R.T - origin
            q[:, 2] += lidar_z
            pts.append(q)
        p = np.vstack(pts)

        # 라이다 높이대만 본다. 바닥/천장을 빼야 벽이 보인다.
        band = p[np.abs(p[:, 2] - lidar_z) <= float(gp('wall_band'))]
        if len(band) == 0:
            return None

        g = float(gp('grid'))
        lim = 8.0
        nb = int(2 * lim / g)
        H, xe, ye = np.histogram2d(
            band[:, 0], band[:, 1], bins=[nb, nb],
            range=[[-lim, lim], [-lim, lim]])
        # 벽 격자를 margin 만큼 부풀려 여유를 준다.
        occ = H >= (float(gp('wall_min_points')) / max(1, len(frames)))
        pad = max(1, int(round(float(gp('wall_margin')) / g)))
        grown = occ.copy()
        for dx in range(-pad, pad + 1):
            for dy in range(-pad, pad + 1):
                grown |= np.roll(np.roll(occ, dx, axis=0), dy, axis=1)

        # 원점 칸에서 사방으로 넓힐 수 있는 만큼 넓힌다.
        i0 = int((0.0 + lim) / g)
        j0 = int((0.0 + lim) / g)
        if not (0 <= i0 < nb and 0 <= j0 < nb) or grown[i0, j0]:
            return None
        x0 = x1 = i0
        y0 = y1 = j0
        changed = True
        while changed:
            changed = False
            for side in range(4):
                if side == 0 and x0 > 0 and not grown[x0 - 1, y0:y1 + 1].any():
                    x0 -= 1; changed = True
                elif side == 1 and x1 < nb - 1 and not grown[x1 + 1, y0:y1 + 1].any():
                    x1 += 1; changed = True
                elif side == 2 and y0 > 0 and not grown[x0:x1 + 1, y0 - 1].any():
                    y0 -= 1; changed = True
                elif side == 3 and y1 < nb - 1 and not grown[x0:x1 + 1, y1 + 1].any():
                    y1 += 1; changed = True
        return {
            'x_min': round(float(xe[x0]), 2), 'x_max': round(float(xe[x1 + 1]), 2),
            'y_min': round(float(ye[y0]), 2), 'y_max': round(float(ye[y1 + 1]), 2),
        }


# ----------------------------------------------------------------------
def fwd_left_to_xy(fwd, left, front_deg):
    """정면/좌우 거리를 좌표로 바꾼다.

    사용자는 '정면 3m 왼쪽 1m' 로 생각하지 x, y 로 생각하지 않는다.
    정면 축이 어디든(+x/+y/...) 같은 말로 받을 수 있게 여기서 변환한다.
    """
    f = rot2(front_deg) @ np.array([1.0, 0.0])      # 정면 단위벡터
    l = rot2(front_deg + 90.0) @ np.array([1.0, 0.0])  # 왼쪽 단위벡터
    return tuple(fwd * f + left * l)


def ask_pos(what, rad, front_deg, front_axis):
    """마커를 대략 어디 두었는지 묻는다.

    라이다는 360도를 보므로 장면 전체에서 마커를 자동으로 찾는 것은
    생각보다 어렵다(사람, 기둥, 잡음이 전부 후보가 된다). 사용자는 어디
    놓았는지 알고 있으니, 그 한 가지만 받으면 탐색 범위가 100분의 1로 줄어든다.
    """
    print(f'\n  {what} 위치를 아시면 알려주세요. 모르면 그냥 Enter.')
    print(f'    아는 경우 : "정면거리 좌우거리" (왼쪽 +, 오른쪽 -)')
    print(f'                예) "3 0" = 정면 3m,  "2.5 -1" = 정면 2.5m 오른쪽 1m')
    print(f'                -> 그 반경 {rad:.1f} m 만 봅니다 (가장 정확)')
    print(f'    모르는 경우: 그냥 Enter -> 주변 물체를 전부 찾아 목록으로 보여드립니다')
    while True:
        try:
            raw = input('  >>> 정면 좌우 (모르면 Enter) : ').strip()
        except (EOFError, KeyboardInterrupt):
            return 'abort'
        if raw == '':
            print('    -> 전체 탐색')
            return None
        v = raw.replace(',', ' ').split()
        if len(v) == 2:
            try:
                p = fwd_left_to_xy(float(v[0]), float(v[1]), front_deg)
                print(f'    -> 좌표 ({p[0]:+.2f}, {p[1]:+.2f})  (정면={front_axis})')
                return p
            except ValueError:
                pass
        print('    숫자 두 개 또는 그냥 Enter')


def choose(cands, what):
    """후보를 보여주고 고르게 한다. 하나뿐이면 그대로 쓴다."""
    if not cands:
        return None
    if len(cands) == 1:
        c = cands[0]
        lr = c.get('left', 0.0)
        side = f'왼{abs(lr):.2f}m' if lr >= 0 else f'오{abs(lr):.2f}m'
        print(f'  {what}: 정면 {c.get("fwd", 0.0):.2f}m {side}  거리 {c["dist"]:.2f}m'
              f'  폭 {c.get("w", 0.0):.2f}m  점 {c["n"]}개'
              f'  지름 {2*c["r"]:.3f}m  잔차 {c["rms"]*1000:.1f}mm')
        return c
    print(f'\n  주변에서 {len(cands)}개 발견:')
    print(f'       {"정면":>7} {"좌우":>9} {"거리":>7} {"폭":>7} {"높이":>7} {"점":>5} {"지름":>7}')
    print('      ' + '-' * 58)
    for i, c in enumerate(cands, 1):
        lr = c.get('left', 0.0)
        side = f'왼{abs(lr):.2f}' if lr >= 0 else f'오{abs(lr):.2f}'
        print(f'   [{i}] {c.get("fwd", 0.0):7.2f} {side:>9} {c["dist"]:7.2f}'
              f' {c.get("w", 0.0):7.2f} {c.get("h", 0.0):7.2f}'
              f' {c["n"]:5d} {2*c["r"]:7.3f}')
    print('\n      폭이 마커 지름과 비슷한 것을 고르세요. 사람은 폭 0.15~0.4 에 높이가 큽니다.')
    while True:
        try:
            v = input(f'\n  >>> {what} 번호 [1]: ').strip()
        except (EOFError, KeyboardInterrupt):
            return None
        if v == '':
            return cands[0]
        if v.isdigit() and 1 <= int(v) <= len(cands):
            return cands[int(v) - 1]
        print('  범위를 벗어났습니다.')


def wait_countdown(sec, what):
    """자리로 이동할 시간을 준다. 혼자 보정할 때 필요하다."""
    if sec <= 0:
        return
    print(f'  {what} — {sec:.0f}초 뒤 측정합니다. 지금 이동하세요.')
    for i in range(int(sec), 0, -1):
        print(f'    {i}...', end='\r', flush=True)
        time.sleep(1.0)
    print('    측정 중       ')


def ask(prompt):
    try:
        input(f'\n  >>> {prompt}  [Enter] ')
        return True
    except (EOFError, KeyboardInterrupt):
        return False


def run(node):
    gp = lambda k: node.get_parameter(k).value
    n_frames = int(gp('n_frames'))
    front = str(gp('front_axis'))
    if front not in FRONT_AXIS_ROT:
        print(f'front_axis 는 {list(FRONT_AXIS_ROT)} 중 하나여야 합니다: {front}')
        return 1

    print('=' * 66)
    print('  라이다 좌표계 보정')
    print('=' * 66)
    print(f'  입력 토픽   {node._topic}')
    print(f'  축 설정     M1 -> M2 방향을 {front} 축으로 삼습니다')
    print(f'              (라이다 2대면 같은 마커 + 같은 값 -> 같은 좌표계)')

    print('\n데이터 대기 중...')
    if not node.grab(1, timeout=20):
        print(f'  {node._topic} 에서 데이터가 오지 않습니다.')
        print('  드라이버가 켜져 있는지 확인하세요:')
        print('    ros2 launch dual_ouster_driver dual_os1.launch.py')
        return 1
    print('  연결됨')

    # ---- 1) 정면 방향 -------------------------------------------------
    print('\n' + '-' * 66)
    print('[1/5] 라이다 정면 방향')
    if bool(gp('skip_front_check')):
        yaw_base = (float(gp('yaw_base')) + FRONT_AXIS_ROT[front]) % 360.0
        print(f'  건너뜀. yaw_base {gp("yaw_base"):.1f}도 + 정면축 보정 = {yaw_base:.1f}도')
    else:
        print('  라이다가 어느 쪽을 보는지 실측합니다.')
        print('  먼저 라이다 주변을 비워주세요.')
        if not ask('비웠으면 Enter'):
            return 1
        base = node.sector_ranges()
        if base is None:
            print('  측정 실패')
            return 1
        print('  기준값 기록됨.')
        cd = float(gp('countdown'))
        if cd > 0:
            print('  이제 정면이라고 생각하는 쪽으로 이동하실 차례입니다.')
            if not ask('Enter 를 누른 뒤 정면 1m 앞으로 걸어가세요'):
                return 1
            wait_countdown(cd, '정면에 서 계세요')
        else:
            print('  이제 정면이라고 생각하는 쪽으로 가세요.')
            if not ask('라이다에서 1m 앞에 선 채로 Enter'):
                return 1
        cur = node.sector_ranges()
        if cur is None:
            print('  측정 실패')
            return 1
        best, best_score = None, 0.0
        for _, ctr in SECTORS:
            d0, n0 = base[ctr]
            d1, n1 = cur[ctr]
            if not np.isfinite(d0) or not np.isfinite(d1):
                continue
            score = (d0 - d1) * 10 + (n1 - n0) / 500.0
            if score > best_score:
                best, best_score = ctr, score
        if best is None or best_score < 1.0:
            print('  변화를 못 찾았습니다. 더 가까이 서거나 -p skip_front_check:=true 로 넘기세요.')
            return 1
        d0, _ = base[best]
        d1, _ = cur[best]
        print(f'  방위 {best}도 에서 {d0:.2f}m -> {d1:.2f}m 로 변화')
        yaw_base = (-best + FRONT_AXIS_ROT[front]) % 360.0
        print(f'  => 정면이 프레임의 {best}도 방향.')
        print(f'     측정 중에는 라이다 정면을 {front} 로 놓고 표시합니다'
              f' (yaw {yaw_base:.1f}도)')
        if cd > 0:
            print('  이제 마커 측정으로 넘어갑니다.')
        if not ask('원래 자리로 돌아온 뒤 Enter'):
            return 1

    # ---- 2) 바닥 높이 -------------------------------------------------
    print('\n' + '-' * 66)
    print('[2/5] 설치 높이')
    given = float(gp('lidar_z'))
    if given > 0:
        lidar_z = given
        print(f'  lidar_z = {lidar_z:.3f} m  (직접 지정, 자동 측정 건너뜀)')
    else:
        lidar_z = node.measure_floor(yaw_base)
        if lidar_z is None:
            print('  바닥을 못 찾았습니다.')
            print('  줄자로 재서 넣으세요:  -p lidar_z:=2.05')
            return 1
        print(f'  lidar_z = {lidar_z:.3f} m  (바닥 최빈값)')
    blind = lidar_z / Calibrator.TAN_DOWN
    print(f'  수직 사각지대 반경 {blind:.2f} m — 이 안쪽 바닥은 원리적으로 안 보입니다')
    print(f'  바닥까지 보려면 {blind:.1f} m 이상 떨어져야 합니다')

    # ---- 3) 마커 반지름 ------------------------------------------------
    print('\n' + '-' * 66)
    given_r = float(gp('marker_radius'))
    if given_r > 0:
        print('[3/5] 마커 반지름 — 건너뜀')
        print(f'  지정하신 반지름 {given_r:.4f} m (지름 {2*given_r:.3f} m) 를 씁니다.')
        radius = given_r
    else:
        print('[3/5] 마커 반지름')
        print(f'  원기둥 마커를 세워 놓으세요.')
        print(f'  탐색 영역: 정면 0~{gp("search_fwd_max"):.1f} m, '
              f'좌우 +-{gp("search_side_max"):.1f} m  (직사각형)')
        print(f'  원 피팅은 바닥이 안 보여도 되지만, 마커 윗부분은 보여야 합니다.')
        for mh in (0.5, 1.0, 1.5):
            d = max(0.0, (lidar_z - mh) / Calibrator.TAN_DOWN)
            print(f'    마커 높이 {mh:.1f} m -> {d:.1f} m 이상')
        near = ask_pos('마커', float(gp('search_radius')), FRONT_AXIS_ROT[front], front)
        if near == 'abort':
            return 1
        cands = node.measure_marker(yaw_base, lidar_z, None, n_frames, near, FRONT_AXIS_ROT[front])
        m = choose(cands, '마커')
        if m is None:
            d = getattr(node, 'last_diag', None)
            if d:
                print(f'\n  그 자리 반경 {gp("search_radius"):.1f} m 안: 점 {d[0]}개, '
                      f'z {d[1]:+.2f}~{d[2]:+.2f} m, 바닥 위 {d[3]:.2f} m 부터 봄')
            else:
                print(f'\n  그 자리 반경 {gp("search_radius"):.1f} m 안에 점이 없습니다.')
            print('  마커를 못 찾았습니다. 확인할 것:')
            print(f'    - 알려주신 위치가 맞는가 (반경 {gp("search_radius"):.1f} m 안만 봅니다)')
            print(f'    - 사각지대({blind:.1f} m) 밖인가')
            print(f'    - 마커 윗부분이 z {gp("marker_z_min"):.2f}~{gp("marker_z_max"):.2f} m 안에 드는가')
            return 1
        print(f'\n  선택: 반지름 {m["r"]:.4f} m (지름 {2*m["r"]:.3f} m),  잔차 {m["rms"]*1000:.1f} mm')
        if m['rms'] > 0.03:
            print('  ! 잔차가 큽니다. 원기둥이 아니거나 기울어졌을 수 있습니다.')
            print('    사람 다리도 이 정도 잔차가 나옵니다. 지름이 맞는지 확인하세요.')
        radius = m['r']

    # ---- 4) M1, M2 ----------------------------------------------------
    print('\n' + '-' * 66)
    print('[4/5] 원점 지점 (M1)')
    print('  마커를 원점(0,0) 으로 삼을 자리에 놓으세요.')
    near1 = ask_pos('M1', float(gp('search_radius')), FRONT_AXIS_ROT[front], front)
    if near1 == 'abort':
        return 1
    m1 = choose(node.measure_marker(yaw_base, lidar_z, radius, n_frames, near1, FRONT_AXIS_ROT[front]), 'M1')
    if m1 is None:
        d = getattr(node, 'last_diag', None)
        if d:
            print(f'  그 자리 점 {d[0]}개, z {d[1]:+.2f}~{d[2]:+.2f}, 바닥 위 {d[3]:.2f} m 부터')
        print('  그 위치 반경 안에서 마커를 못 찾았습니다. 위치를 다시 확인하세요.')
        return 1
    print(f'\n  M1 = ({m1["x"]:.4f}, {m1["y"]:.4f})   흔들림 {m1["sx"]*1000:.1f}/{m1["sy"]*1000:.1f} mm'
          + (f'   [이상 {m1["dropped"]}프레임 제외]' if m1['dropped'] else ''))

    print('\n' + '-' * 66)
    print(f'[5/5] 축 방향 지점 (M2)')
    print(f'  마커를 옮기세요. M1 -> M2 방향이 최종 좌표계의 {front} 축이 됩니다.')
    print('  멀수록 정확합니다. 최소 2 m, 가능하면 3 m 이상 벌리세요.')
    near2 = ask_pos('M2', float(gp('search_radius')), FRONT_AXIS_ROT[front], front)
    if near2 == 'abort':
        return 1
    m2 = choose(node.measure_marker(yaw_base, lidar_z, radius, n_frames, near2, FRONT_AXIS_ROT[front]), 'M2')
    if m2 is None:
        d = getattr(node, 'last_diag', None)
        if d:
            print(f'  그 자리 점 {d[0]}개, z {d[1]:+.2f}~{d[2]:+.2f}, 바닥 위 {d[3]:.2f} m 부터')
        print('  그 위치 반경 안에서 마커를 못 찾았습니다. 위치를 다시 확인하세요.')
        return 1
    print(f'\n  M2 = ({m2["x"]:.4f}, {m2["y"]:.4f})   흔들림 {m2["sx"]*1000:.1f}/{m2["sy"]*1000:.1f} mm'
          + (f'   [이상 {m2["dropped"]}프레임 제외]' if m2['dropped'] else ''))

    # ---- 계산 ----------------------------------------------------------
    P1 = np.array([m1['x'], m1['y']])
    P2 = np.array([m2['x'], m2['y']])
    dv = P2 - P1
    dist = float(np.hypot(*dv))
    if dist < 0.3:
        print(f'\n  두 지점이 너무 가깝습니다 ({dist:.2f} m). 다시 하세요.')
        return 1

    # 측정 프레임이 이미 정면=front_axis 이므로, M1->M2 를 그 축에 맞추는
    # 각도만 더 돌리면 된다.
    target = FRONT_AXIS_ROT[front]
    theta = target - math.degrees(math.atan2(dv[1], dv[0]))
    yaw = yaw_base + theta
    origin = rot2(theta) @ P1

    print('\n' + '=' * 66)
    print('  계산 결과')
    print('=' * 66)
    print(f'  마커 간 거리   {dist:.4f} m')
    print(f'  회전           {theta:+.3f}도  (M1->M2 를 {front} 축에 맞춤)')
    err = max(m1['sx'], m1['sy'], m2['sx'], m2['sy'])
    print(f'  yaw 정확도     약 {math.degrees(math.atan(err/dist)):.3f}도'
          f'  (흔들림 {err*1000:.1f} mm / 간격 {dist:.2f} m)')

    # 검증
    Rf = rot2(theta)
    v1 = Rf @ P1 - origin
    v2 = Rf @ P2 - origin


    # ---- ROI ----------------------------------------------------------
    print('\n  벽 탐색 중...')
    print('  마커를 치우고 사람도 비켜주세요.')
    if not ask('비웠으면 Enter'):
        return 1
    roi = node.find_roi(yaw, lidar_z, origin)
    if roi is None:
        print('  벽 탐색 실패. ROI 는 직접 넣어야 합니다.')
        roi = {'x_min': -3.0, 'x_max': 3.0, 'y_min': -3.0, 'y_max': 3.0}
    else:
        area = (roi['x_max'] - roi['x_min']) * (roi['y_max'] - roi['y_min'])
        print(f'  ROI  x {roi["x_min"]:.2f} ~ {roi["x_max"]:.2f}'
              f'   y {roi["y_min"]:.2f} ~ {roi["y_max"]:.2f}   ({area:.1f} m2)')

    # ---- 저장 ----------------------------------------------------------
    out = str(gp('output_file'))
    if not out:
        out = os.path.join(config_dir(), f'calibration_lidar{node.lid}.yaml')

    body = f"""# 라이다{node.lid} 보정값 — calibrate 로 자동 생성
# {time.strftime('%Y-%m-%d %H:%M:%S')}
#
#   원점(0,0)   : 마커를 놓았던 첫 번째 자리 (바닥)
#   라이다 정면  : {front}
#   라이다 위치  : ({-origin[0]:+.4f}, {-origin[1]:+.4f}),  원점에서 {np.hypot(*origin):.4f} m
#   yaw 정확도   : 약 {math.degrees(math.atan(err/dist)):.3f}도  (마커 간격 {dist:.2f} m)
#   마커 지름    : {2*radius:.3f} m ({'지정값' if float(gp('marker_radius')) > 0 else '라이다 추정'})
#
# 라이다를 옮기면 다시 돌려야 한다:
#   ros2 run dual_ouster_driver calibrate --ros-args -p front_axis:={front}
# 노드 이름이 roi_filter 가 아니어도(roi_filter_1 등) 먹히도록 와일드카드.
# 'roi_filter:' 로 두면 이름이 다를 때 조용히 무시된다. 실제로 당했다.
/**:
  ros__parameters:
    lidar_z: {lidar_z:.4f}
    yaw_deg: {yaw % 360.0:.4f}
    origin_x: {origin[0]:.4f}
    origin_y: {origin[1]:.4f}

    roi_x_min: {roi['x_min']:.2f}
    roi_x_max: {roi['x_max']:.2f}
    roi_y_min: {roi['y_min']:.2f}
    roi_y_max: {roi['y_max']:.2f}
    roi_z_min: {float(gp('roi_z_min')):.2f}
    roi_z_max: {float(gp('roi_z_max')):.2f}

    min_range: 0.1
    input_topic: /lidar{node.lid}/points
    output_topic: /lidar{node.lid}/roi_points
    output_frame: map
"""
    try:
        with open(out, 'w') as f:
            f.write(body)
        install_copy(out)
    except Exception as e:
        print(f'\n  저장 실패: {e}')
        print(body)
        return 1

    print('\n\n' + '=' * 66)
    print('  보정 완료 — 결과 요약')
    print('=' * 66)

    print('\n[측정값]')
    print(f'  라이다 정면 실측     yaw_base {yaw_base:.2f}도  '
          + ('(직접 지정)' if bool(gp('skip_front_check')) else '(사람이 서서 확인)'))
    print(f'  설치 높이            {lidar_z:.4f} m  '
          + ('(직접 지정)' if float(gp('lidar_z')) > 0 else '(바닥 최빈값)'))
    print(f'  마커 반지름          {radius:.4f} m (지름 {2*radius:.3f} m)  '
          + ('(지정값)' if float(gp('marker_radius')) > 0 else '(라이다 추정)'))
    print(f'  M1 원점 지점         ({m1["x"]:+.4f}, {m1["y"]:+.4f})   '
          f'흔들림 {m1["sx"]*1000:.1f}/{m1["sy"]*1000:.1f} mm, 점 {m1["n"]}개')
    print(f'  M2 축 방향 지점      ({m2["x"]:+.4f}, {m2["y"]:+.4f})   '
          f'흔들림 {m2["sx"]*1000:.1f}/{m2["sy"]*1000:.1f} mm, 점 {m2["n"]}개')
    print(f'  두 지점 간 거리      {dist:.4f} m')

    print('\n[보정값]  이 값이 YAML 에 저장됩니다')
    print(f'  lidar_z              {lidar_z:.4f}')
    print(f'  yaw_deg              {yaw % 360.0:.4f}')
    print(f'  origin_x             {origin[0]:+.4f}')
    print(f'  origin_y             {origin[1]:+.4f}')

    print('\n[좌표계]')
    fdir = rot2(FRONT_AXIS_ROT[front]) @ np.array([1.0, 0.0])
    ldir = rot2(FRONT_AXIS_ROT[front] + 90.0) @ np.array([1.0, 0.0])
    ax = lambda v: ('+x' if v[0] > 0.5 else '-x' if v[0] < -0.5
                    else '+y' if v[1] > 0.5 else '-y')
    print(f'  원점 (0,0,0)         M1 을 놓았던 자리 (바닥)')
    print(f'  {front} 방향             M1 -> M2 (마커가 정의한 축)')
    lfwd = rot2(yaw % 360.0) @ np.array([-1.0, 0.0])   # 라이다 정면(원시 -x)
    print(f'  라이다가 보는 방향    ({lfwd[0]:+.2f}, {lfwd[1]:+.2f})')
    print(f'  z = 0                바닥')
    print(f'  라이다 위치          ({-origin[0]:+.4f}, {-origin[1]:+.4f}, {lidar_z:.3f})'
          f'   원점에서 {np.hypot(*origin):.4f} m')

    print('\n[정확도]')
    rep = math.degrees(math.atan(err / dist))
    print(f'  yaw 반복정밀도       {rep:.3f}도'
          f'   (마커 흔들림 {err*1000:.1f} mm / 간격 {dist:.2f} m)')
    print(f'                       -> 10 m 지점에서 {10*math.tan(math.radians(rep))*100:.1f} cm')
    if float(gp('marker_radius')) <= 0:
        print(f'  ! 반지름을 추정으로 썼습니다. 실제와 다르면 그만큼')
        print(f'    원점이 라이다 쪽으로 치우칩니다(계통 오차).')
        print(f'    줄자로 재서 -p marker_radius:= 로 넣으면 없앨 수 있습니다.')
    print(f'  마커 간격을 넓히면 정확도가 그만큼 좋아집니다'
          f' (지금 {dist:.1f} m -> 5 m 면 {rep*dist/5:.3f}도)')

    print('\n[검증]  보정을 적용했을 때 마커가 어디로 가는지')
    print(f'  M1 -> ({v1[0]:+.4f}, {v1[1]:+.4f})   기대 (0, 0)')
    exp_s = {'+x': f'({dist:.4f}, 0)', '+y': f'(0, {dist:.4f})',
             '-x': f'(-{dist:.4f}, 0)', '-y': f'(0, -{dist:.4f})'}[front]
    print(f'  M2 -> ({v2[0]:+.4f}, {v2[1]:+.4f})   기대 {exp_s}')
    resid = max(np.hypot(*v1), abs(np.hypot(*v2) - dist))
    print(f'  잔차 {resid*1000:.4f} mm  '
          + ('정상' if resid < 1e-6 else '! 계산 오류 의심'))

    print('\n[비행 영역]  ROI')
    area = (roi['x_max'] - roi['x_min']) * (roi['y_max'] - roi['y_min'])
    print(f'  x  {roi["x_min"]:+.2f} ~ {roi["x_max"]:+.2f} m')
    print(f'  y  {roi["y_min"]:+.2f} ~ {roi["y_max"]:+.2f} m')
    print(f'  z  {float(gp("roi_z_min")):+.2f} ~ {float(gp("roi_z_max")):+.2f} m')
    print(f'  면적 {area:.1f} m2')
    print(f'  이 밖으로 나가면 추적이 끊깁니다.')

    print('\n' + '=' * 66)
    print(f'  저장: {out}')
    print('=' * 66)
    print('\n  적용:')
    print(f'    colcon build --packages-select dual_ouster_driver --symlink-install')
    print(f'    ros2 run dual_ouster_driver roi_filter --ros-args \\')
    print(f'      --params-file $(ros2 pkg prefix dual_ouster_driver)'
          f'/share/dual_ouster_driver/config/calibration_lidar{node.lid}.yaml')
    print()
    return 0




# ======================================================================
#  라이다 여러 대 동시 보정
# ======================================================================

def install_copy(path):
    """보정 결과를 설치된 share/config 에도 복사한다.

    setup.py 의 data_files glob 은 '빌드 시점'에 평가된다. 그래서 보정으로
    새로 만든 calibration_lidarN.yaml 은 다시 빌드하기 전까지 install 트리에
    나타나지 않고, get_package_share_directory 로 찾는 런치 파일이 못 본다.
    새 PC 에서 보정을 마쳤는데 dual_view 가 '파일이 없습니다' 라고 하는 게
    이것 때문이다. 여기서 같이 써두면 다시 빌드하지 않아도 된다.
    """
    try:
        from ament_index_python.packages import get_package_share_directory
        d = os.path.join(get_package_share_directory('dual_ouster_driver'), 'config')
        if not os.path.isdir(d):
            return None
        dst = os.path.join(d, os.path.basename(path))
        if os.path.realpath(dst) == os.path.realpath(path):
            return dst          # 심볼릭 링크라 이미 같은 파일
        shutil.copyfile(path, dst)
        return dst
    except Exception:
        return None


def config_dir():
    """보정 YAML 을 쓸 폴더. 반드시 소스 트리여야 한다.

    --symlink-install 이라 __file__ 이 build/ 아래를 가리킬 때가 있다.
    거기에 쓰면 이미 있는 파일은 심볼릭 링크를 따라 src 로 가지만,
    새 파일(calibration_lidar2.yaml 처럼)은 build/ 에 갇혀서 다음 빌드에
    사라진다. 실제로 당했다. 그래서 build -> src 로 되돌려 확인한다.
    """
    here = os.path.dirname(os.path.abspath(__file__))
    cand = os.path.abspath(os.path.join(here, '..', 'config'))
    if os.sep + 'build' + os.sep in cand:
        alt = cand.replace(os.sep + 'build' + os.sep, os.sep + 'src' + os.sep)
        if os.path.isdir(alt):
            return alt
    if os.path.isdir(cand):
        return cand
    return os.getcwd()


def match_moved(m1_cands, m2_cands, ref_gap, still=0.15, gap_tol=0.12):
    """M1 목록과 M2 목록을 비교해 '움직인 것' 짝을 찾는다.

    방 안에서 M1 자리에서 M2 자리로 옮겨간 물체는 마커뿐이다. 의자도 기둥도
    두 목록에 같은 자리로 그대로 남는다. 그래서

      a: M1 목록에 있는데 M2 목록에는 그 자리가 비어 있는 것  (떠난 자리)
      b: M2 목록에 새로 생긴 것                                (도착한 자리)
      |a-b| 가 다른 라이다가 잰 M1-M2 간격과 맞을 것

    세 조건을 모두 만족하는 짝은 거의 유일하다.

    반환: [(a, b, 간격오차, 점수)] 를 좋은 순으로.
    """
    def nearest(c, others):
        if not others:
            return 9.9
        return min(math.hypot(c['x'] - o['x'], c['y'] - o['y']) for o in others)

    out = []
    for a in m1_cands:
        a_gone = nearest(a, m2_cands)       # 클수록 '떠났다'
        if a_gone < still:
            continue
        for b in m2_cands:
            b_new = nearest(b, m1_cands)    # 클수록 '새로 생겼다'
            if b_new < still:
                continue
            gap = math.hypot(a['x'] - b['x'], a['y'] - b['y'])
            err = abs(gap - ref_gap)
            if err > gap_tol:
                continue
            # 간격이 맞을수록, 확실히 움직였을수록, 잔차가 작을수록 좋다
            score = err - 0.02 * min(a_gone, b_new) \
                + 0.5 * (a['rms'] + b['rms'])
            out.append((a, b, gap, err, score))
    out.sort(key=lambda r: r[4])
    return out


def choose_free(cands, what, expect_d=None, radius=None):
    """정면 개념 없이(360도 탐색) 찾은 후보 중에서 고르게 한다.

    마주 보는 라이다는 '정면'이 서로 달라서 정면/좌우 표시가 의미가 없다.
    대신 거리·폭·지름·잔차를 보여준다. M2 단계에서는 이미 고른 M1 과의
    간격도 같이 보여주는데, 그 값이 다른 라이다가 잰 간격과 맞는 것이
    거의 확실하게 마커다.
    """
    if not cands:
        return None
    print(f'\n  {what} 후보 {len(cands)}개:')
    head = f'       {"거리":>7} {"폭":>7} {"높이":>7} {"점":>5} {"지름":>7} {"잔차":>8}'
    if expect_d is not None:
        head += f' {"M1과간격":>9}'
    print(head)
    print('      ' + '-' * (66 if expect_d is None else 76))
    best_i = 0
    best_err = None
    for i, c in enumerate(cands):
        line = (f'   [{i+1}] {c["dist"]:7.2f} {c.get("w", 0.0):7.2f}'
                f' {c.get("h", 0.0):7.2f} {c["n"]:5d} {2*c["r"]:7.3f}'
                f' {c["rms"]*1000:7.1f}mm')
        if expect_d is not None:
            gap = c.get('_gap')
            if gap is None:
                line += f' {"-":>9}'
            else:
                err = abs(gap - expect_d)
                line += f' {gap:8.3f}m'
                if best_err is None or err < best_err:
                    best_err, best_i = err, i
                if err < 0.05:
                    line += '  <== 일치'
        print(line)
    if expect_d is not None and best_err is not None:
        print(f'\n      다른 라이다가 잰 M1-M2 간격은 {expect_d:.3f} m 입니다.')
        print(f'      가장 가까운 것은 [{best_i+1}] (차이 {best_err*1000:.0f} mm).')
    else:
        print('\n      폭이 마커 지름과 비슷하고 잔차가 작은 것을 고르세요.')
    while True:
        try:
            v = input(f'\n  >>> {what} 번호 [{best_i+1}]: ').strip()
        except (EOFError, KeyboardInterrupt):
            return None
        if v == '':
            return cands[best_i]
        if v.isdigit() and 1 <= int(v) <= len(cands):
            return cands[int(v) - 1]
        print('  범위를 벗어났습니다.')


def write_yaml(node, lid, front, lidar_z, yaw, origin, roi, radius,
               dist, err, gp):
    """보정값 YAML 하나를 쓴다. 성공하면 경로, 실패하면 None."""
    out = os.path.join(config_dir(), f'calibration_lidar{lid}.yaml')

    body = f"""# 라이다{lid} 보정값 — calibrate 로 자동 생성 (다중 라이다 동시 보정)
# {time.strftime('%Y-%m-%d %H:%M:%S')}
#
#   원점(0,0)   : 마커를 놓았던 첫 번째 자리 (바닥)
#   좌표축      : M1 -> M2 방향이 {front}
#   라이다 위치  : ({-origin[0]:+.4f}, {-origin[1]:+.4f}),  원점에서 {np.hypot(*origin):.4f} m
#   yaw 정확도   : 약 {math.degrees(math.atan(err/dist)):.3f}도  (마커 간격 {dist:.2f} m)
#   마커 지름    : {2*radius:.3f} m
#
#   같은 마커를 여러 라이다가 동시에 봐서 잡은 값이므로,
#   calibration_lidar*.yaml 들은 모두 같은 좌표계를 씁니다.
#
# 라이다를 옮기면 다시 돌려야 한다:
#   ros2 run dual_ouster_driver calibrate --ros-args \\
#     -p lidar_ids:="[1,2]" -p front_axis:={front}
# 노드 이름이 roi_filter 가 아니어도(roi_filter_1 등) 먹히도록 와일드카드.
# 'roi_filter:' 로 두면 이름이 다를 때 조용히 무시된다. 실제로 당했다.
/**:
  ros__parameters:
    lidar_z: {lidar_z:.4f}
    yaw_deg: {yaw % 360.0:.4f}
    origin_x: {origin[0]:.4f}
    origin_y: {origin[1]:.4f}

    roi_x_min: {roi['x_min']:.2f}
    roi_x_max: {roi['x_max']:.2f}
    roi_y_min: {roi['y_min']:.2f}
    roi_y_max: {roi['y_max']:.2f}
    roi_z_min: {float(gp('roi_z_min')):.2f}
    roi_z_max: {float(gp('roi_z_max')):.2f}

    min_range: 0.1
    input_topic: /lidar{lid}/points
    output_topic: /lidar{lid}/roi_points
    output_frame: map
"""
    try:
        with open(out, 'w') as f:
            f.write(body)
        install_copy(out)
        return out
    except Exception as e:
        print(f'  저장 실패 (lidar{lid}): {e}')
        return None


def measure_all(node, lids, yaw_base, floor, radius, n_frames, front_deg,
                front, hint, label, expect_d=None, picks=None):
    """마커 하나를 모든 라이다가 각자 잰다. {lid: 후보} 를 돌려준다.

    첫 라이다만 사람이 위치를 알려주고(정면 상자), 나머지는 360도로 훑어
    후보 목록에서 고른다. 마주 보는 라이다끼리는 '정면'이 서로 달라서
    같은 위치 입력을 재활용할 수 없기 때문이다.
    """
    out = {}
    for k, lid in enumerate(lids):
        node._active = lid
        print(f'\n  --- 라이다{lid} 로 {label} 측정 중 ---')
        if k == 0:
            cands = node.measure_marker(
                yaw_base[lid], floor[lid], radius, n_frames, hint, front_deg,
                use_box=True)
            c = choose(cands, f'{label}(라이다{lid})')
        else:
            cands = node.measure_marker(
                yaw_base[lid], floor[lid], radius, n_frames, None, front_deg,
                use_box=False)
            # 마커일 수 없는 것 거르기
            cands = [x for x in cands if 0.03 <= x.get('w', 0) <= 0.7]
            if expect_d is not None and picks and lid in picks:
                p1 = np.array([picks[lid]['x'], picks[lid]['y']])
                for x in cands:
                    x['_gap'] = float(np.hypot(x['x'] - p1[0], x['y'] - p1[1]))
                cands.sort(key=lambda x: abs(x['_gap'] - expect_d))
            c = choose_free(cands, f'{label}(라이다{lid})', expect_d, radius)
        if c is None:
            d = getattr(node, 'last_diag', None)
            if d:
                print(f'  진단: 점 {d[0]}개, z {d[1]:+.2f}~{d[2]:+.2f}, '
                      f'바닥 위 {d[3]:.2f} m 부터')
            print(f'  라이다{lid} 가 {label} 를 못 찾았습니다.')
            return None
        print(f'  라이다{lid} {label} = ({c["x"]:.4f}, {c["y"]:.4f})'
              f'   흔들림 {c["sx"]*1000:.1f}/{c["sy"]*1000:.1f} mm'
              + (f'   [이상 {c["dropped"]}프레임 제외]' if c['dropped'] else ''))
        out[lid] = c
    return out


def run_multi(node):
    """마커 2개로 여러 라이다를 한 번에 같은 좌표계로 맞춘다."""
    gp = lambda n: node.get_parameter(n).value
    lids = node.lids
    front = str(gp('front_axis'))
    if front not in FRONT_AXIS_ROT:
        print(f'front_axis 는 {list(FRONT_AXIS_ROT)} 중 하나여야 합니다: {front}')
        return 1
    front_deg = FRONT_AXIS_ROT[front]
    n_frames = int(gp('n_frames'))

    print('=' * 66)
    print('  라이다 좌표계 보정 — 여러 대 동시')
    print('=' * 66)
    for lid in lids:
        print(f'  라이다{lid}   {node._topics[lid]}')
    print(f'  축 설정     M1 -> M2 방향을 {front} 축으로 삼습니다')
    print('              같은 마커를 모든 라이다가 동시에 보므로')
    print('              결과 좌표계가 자동으로 일치합니다.')
    print('=' * 66)

    print('\n데이터 대기 중...')
    for lid in lids:
        node._active = lid
        if not node.grab(1, timeout=15.0):
            print(f'  라이다{lid} ({node._topics[lid]}) 에서 데이터가 안 옵니다.')
            return 1
        print(f'  라이다{lid} 연결됨')

    # ---- [1/4] 정면 방향 (첫 라이다만) --------------------------------
    print('\n' + '-' * 66)
    print('[1/4] 라이다 정면 방향')
    yaw_base = {}
    if bool(gp('skip_front_check')):
        yaw_base[lids[0]] = (float(gp('yaw_base')) + front_deg) % 360.0
        print(f'  건너뜀. 라이다{lids[0]} yaw_base = {yaw_base[lids[0]]:.1f}도')
    else:
        print(f'  라이다{lids[0]} 이 어느 쪽을 보는지만 재면 됩니다.')
        print('  (이 값은 최종 결과에 영향이 없고, 화면 표시와 탐색 범위용입니다)')
        print('  먼저 라이다 주변을 비워주세요.')
        if not ask('비웠으면 Enter'):
            return 1
        node._active = lids[0]
        base = node.sector_ranges()
        if base is None:
            print('  측정 실패')
            return 1
        print('  기준값 기록됨.')
        cd = float(gp('countdown'))
        if cd > 0:
            if not ask(f'Enter 를 누른 뒤 라이다{lids[0]} 정면 1m 앞으로 걸어가세요'):
                return 1
            wait_countdown(cd, '정면에 서 계세요')
        else:
            if not ask(f'라이다{lids[0]} 정면 1m 앞에 서서 Enter'):
                return 1
        after = node.sector_ranges()
        if after is None:
            print('  측정 실패')
            return 1
        best, gain = None, 0.0
        for ctr, (d0, _) in base.items():
            d1 = after[ctr][0]
            if np.isfinite(d0) and np.isfinite(d1) and (d0 - d1) > gain:
                best, gain = ctr, d0 - d1
        if best is None:
            print('  변화를 못 찾았습니다. -p skip_front_check:=true 로 넘기세요.')
            return 1
        print(f'  방위 {best:.0f}도 에서 {gain:.2f} m 가까워짐 -> 정면')
        yaw_base[lids[0]] = (-best + front_deg) % 360.0
        if not ask('원래 자리로 돌아온 뒤 Enter'):
            return 1
    # 나머지 라이다는 정면 개념이 필요 없다 (360도 탐색)
    for lid in lids[1:]:
        yaw_base[lid] = 0.0

    # ---- [2/4] 설치 높이 ----------------------------------------------
    print('\n' + '-' * 66)
    print('[2/4] 설치 높이')
    floor = {}
    fixed = float(gp('lidar_z'))
    for lid in lids:
        node._active = lid
        if fixed > 0:
            floor[lid] = fixed
            print(f'  라이다{lid}  lidar_z = {fixed:.3f} m  (지정값)')
            continue
        h = node.measure_floor(yaw_base[lid])
        if h is None:
            print(f'  라이다{lid} 바닥 측정 실패. -p lidar_z 로 직접 넣으세요.')
            return 1
        floor[lid] = h
        blind = h / Calibrator.TAN_DOWN
        print(f'  라이다{lid}  lidar_z = {h:.3f} m   사각지대 반경 {blind:.2f} m')

    # ---- [3/4] M1 ------------------------------------------------------
    radius = float(gp('marker_radius'))
    if radius <= 0:
        print('\n  marker_radius 를 지정해 주세요 (동시 보정에서는 필수).')
        print('  예: -p marker_radius:=0.045   (지름 9cm)')
        return 1
    print(f'\n  마커 반지름 {radius:.4f} m (지름 {2*radius:.3f} m)')

    print('\n' + '-' * 66)
    print('[3/4] 원점 지점 (M1)')
    print('  마커를 원점(0,0) 으로 삼을 자리에 놓으세요.')
    print('  모든 라이다에서 보이는 자리여야 하고, 각 라이다에서 2 m 이상')
    print('  떨어져야 합니다 (가까우면 수직 시야 밖으로 나갑니다).')
    hint1 = ask_pos(f'M1(라이다{lids[0]} 기준)', float(gp('search_radius')),
                    front_deg, front)
    if hint1 == 'abort':
        return 1
    node._active = lids[0]
    print(f'\n  --- 라이다{lids[0]} 로 M1 측정 중 ---')
    c1 = choose(node.measure_marker(
        yaw_base[lids[0]], floor[lids[0]], radius, n_frames, hint1,
        front_deg, use_box=True), f'M1(라이다{lids[0]})')
    if c1 is None:
        d = getattr(node, 'last_diag', None)
        if d:
            print(f'  진단: 점 {d[0]}개, z {d[1]:+.2f}~{d[2]:+.2f}, '
                  f'바닥 위 {d[3]:.2f} m 부터')
        print(f'  라이다{lids[0]} 가 M1 을 못 찾았습니다.')
        return 1
    print(f'  라이다{lids[0]} M1 = ({c1["x"]:.4f}, {c1["y"]:.4f})'
          f'   흔들림 {c1["sx"]*1000:.1f}/{c1["sy"]*1000:.1f} mm')
    M1 = {lids[0]: c1}

    # 보조 라이다는 지금 고르지 않는다. 후보만 쌓아두고, M2 를 잰 뒤
    # '움직인 것'으로 자동 판별한다. 360도 탐색이라 후보가 10개 넘게
    # 나오는데 사람이 눈으로 고르면 틀리기 쉽다.
    m1_pool = {}
    for lid in lids[1:]:
        node._active = lid
        print(f'\n  --- 라이다{lid} 로 M1 자리 훑는 중 ---')
        cs = node.measure_marker(
            yaw_base[lid], floor[lid], radius, n_frames, None, front_deg,
            use_box=False)
        cs = [x for x in cs if 0.03 <= x.get('w', 0) <= 0.7]
        if not cs:
            print(f'  라이다{lid} 가 이 자리에서 아무것도 못 찾았습니다.')
            return 1
        m1_pool[lid] = cs
        print(f'  후보 {len(cs)}개 기록 (선택은 M2 측정 뒤에)')

    # ---- [4/4] M2 ------------------------------------------------------
    print('\n' + '-' * 66)
    print('[4/4] 축 방향 지점 (M2)')
    print(f'  마커를 옮기세요. M1 -> M2 방향이 {front} 축이 됩니다.')
    print('  멀수록 정확합니다. 최소 2 m, 가능하면 3 m 이상 벌리세요.')
    hint2 = ask_pos(f'M2(라이다{lids[0]} 기준)', float(gp('search_radius')),
                    front_deg, front)
    if hint2 == 'abort':
        return 1
    # 첫 라이다를 먼저 재서 간격을 알아낸 뒤, 나머지 라이다 후보를 그 값으로 거른다
    node._active = lids[0]
    print(f'\n  --- 라이다{lids[0]} 로 M2 측정 중 ---')
    c0 = choose(node.measure_marker(
        yaw_base[lids[0]], floor[lids[0]], radius, n_frames, hint2,
        front_deg, use_box=True), f'M2(라이다{lids[0]})')
    if c0 is None:
        print(f'  라이다{lids[0]} 가 M2 를 못 찾았습니다.')
        return 1
    p1 = np.array([M1[lids[0]]['x'], M1[lids[0]]['y']])
    ref_gap = float(np.hypot(c0['x'] - p1[0], c0['y'] - p1[1]))
    print(f'  라이다{lids[0]} M2 = ({c0["x"]:.4f}, {c0["y"]:.4f})'
          f'   흔들림 {c0["sx"]*1000:.1f}/{c0["sy"]*1000:.1f} mm')
    print(f'  M1-M2 간격 {ref_gap:.4f} m')

    M2 = {lids[0]: c0}
    for lid in lids[1:]:
        node._active = lid
        print(f'\n  --- 라이다{lid} 로 M2 자리 훑는 중 ---')
        m2 = node.measure_marker(
            yaw_base[lid], floor[lid], radius, n_frames, None, front_deg,
            use_box=False)
        m2 = [x for x in m2 if 0.03 <= x.get('w', 0) <= 0.7]
        if not m2:
            print(f'  라이다{lid} 가 이 자리에서 아무것도 못 찾았습니다.')
            return 1
        print(f'  후보 {len(m2)}개')

        pairs = match_moved(m1_pool[lid], m2, ref_gap)
        if not pairs:
            print(f'\n  라이다{lid}: 움직인 물체를 못 찾았습니다.')
            print(f'  (M1 후보 {len(m1_pool[lid])}개, M2 후보 {len(m2)}개, '
                  f'기대 간격 {ref_gap:.3f} m)')
            print('  마커가 라이다2 에서 안 보이거나 두 자리가 너무 가깝습니다.')
            return 1

        print(f'\n  라이다{lid}: M1 -> M2 로 움직인 물체를 찾았습니다.')
        print('  (방에서 움직인 건 마커뿐이므로 이게 마커입니다)')
        print(f'\n       {"M1 위치":>17} {"M2 위치":>17} {"간격":>8} {"오차":>8}')
        print('      ' + '-' * 56)
        for i, (a, b, gap, err, _) in enumerate(pairs[:5], 1):
            print(f'   [{i}] ({a["x"]:+7.3f},{a["y"]:+7.3f}) '
                  f'({b["x"]:+7.3f},{b["y"]:+7.3f}) {gap:8.3f} {err*1000:7.1f}mm')
        print(f'\n      라이다{lids[0]} 가 잰 간격은 {ref_gap:.3f} m 입니다.')
        try:
            v = input(f'\n  >>> 라이다{lid} 번호 [1]: ').strip()
        except (EOFError, KeyboardInterrupt):
            return 1
        k = int(v) - 1 if v.isdigit() and 1 <= int(v) <= len(pairs[:5]) else 0
        a, b, gap, err, _ = pairs[k]
        M1[lid], M2[lid] = a, b
        print(f'  라이다{lid} M1 = ({a["x"]:.4f}, {a["y"]:.4f})'
              f'   흔들림 {a["sx"]*1000:.1f}/{a["sy"]*1000:.1f} mm')
        print(f'  라이다{lid} M2 = ({b["x"]:.4f}, {b["y"]:.4f})'
              f'   흔들림 {b["sx"]*1000:.1f}/{b["sy"]*1000:.1f} mm')
        print(f'  간격 {gap:.4f} m   (라이다{lids[0]} 대비 {err*1000:+.1f} mm)')

    # ---- 계산 ----------------------------------------------------------
    sol = {}
    for lid in lids:
        P1 = np.array([M1[lid]['x'], M1[lid]['y']])
        P2 = np.array([M2[lid]['x'], M2[lid]['y']])
        dv = P2 - P1
        dist = float(np.hypot(*dv))
        if dist < 0.3:
            print(f'\n  라이다{lid}: 두 지점이 너무 가깝습니다 ({dist:.2f} m).')
            return 1
        theta = front_deg - math.degrees(math.atan2(dv[1], dv[0]))
        sol[lid] = {
            'yaw': yaw_base[lid] + theta,
            'origin': rot2(theta) @ P1,
            'dist': dist,
            'err': max(M1[lid]['sx'], M1[lid]['sy'],
                       M2[lid]['sx'], M2[lid]['sy']),
        }

    print('\n' + '=' * 66)
    print('  계산 결과')
    print('=' * 66)
    gaps = [sol[l]['dist'] for l in lids]
    for lid in lids:
        s = sol[lid]
        print(f'  라이다{lid}  간격 {s["dist"]:.4f} m   '
              f'yaw {s["yaw"] % 360.0:.4f}도   '
              f'원점 ({s["origin"][0]:+.4f}, {s["origin"][1]:+.4f})   '
              f'정확도 {math.degrees(math.atan(s["err"]/s["dist"])):.3f}도')

    # 라이다들이 잰 M1-M2 간격이 서로 얼마나 맞는가.
    # 기준값이 없는 상황에서 얻을 수 있는 가장 실질적인 교차검증이다.
    spread = max(gaps) - min(gaps)
    print(f'\n  [교차검증] 각 라이다가 잰 M1-M2 간격')
    print(f'    ' + '   '.join(f'라이다{l} {sol[l]["dist"]:.4f}m' for l in lids))
    print(f'    최대 차이 {spread*1000:.1f} mm', end='')
    if spread < 0.02:
        print('   두 라이다가 같은 것을 보고 있습니다.')
    elif spread < 0.05:
        print('   허용 범위지만 조금 큽니다.')
    else:
        print('   ** 큽니다. 다른 물체를 골랐거나 마커가 움직였을 수 있습니다. **')

    # ---- ROI (첫 라이다 기준, 모두 공유) -------------------------------
    print('\n  벽 탐색 중...')
    print('  마커를 치우고 사람도 비켜주세요.')
    if not ask('비웠으면 Enter'):
        return 1
    node._active = lids[0]
    roi = node.find_roi(sol[lids[0]]['yaw'], floor[lids[0]],
                        sol[lids[0]]['origin'])
    if roi is None:
        print('  벽 탐색 실패. ROI 기본값을 씁니다.')
        roi = {'x_min': -3.0, 'x_max': 3.0, 'y_min': -3.0, 'y_max': 3.0}
    area = (roi['x_max'] - roi['x_min']) * (roi['y_max'] - roi['y_min'])
    print(f'  ROI  x {roi["x_min"]:.2f} ~ {roi["x_max"]:.2f}'
          f'   y {roi["y_min"]:.2f} ~ {roi["y_max"]:.2f}   ({area:.1f} m2)')
    print('  (같은 비행 영역이므로 모든 라이다가 같은 ROI 를 씁니다)')

    # ---- 저장 ----------------------------------------------------------
    paths = []
    for lid in lids:
        s = sol[lid]
        p = write_yaml(node, lid, front, floor[lid], s['yaw'], s['origin'],
                       roi, radius, s['dist'], s['err'], gp)
        if p is None:
            return 1
        paths.append(p)

    print('\n\n' + '=' * 66)
    print('  보정 완료 — 결과 요약')
    print('=' * 66)
    for lid in lids:
        s = sol[lid]
        print(f'\n  [라이다{lid}]')
        print(f'    설치 높이     {floor[lid]:.4f} m')
        print(f'    yaw_deg       {s["yaw"] % 360.0:.4f}')
        print(f'    origin        ({s["origin"][0]:+.4f}, {s["origin"][1]:+.4f})')
        print(f'    라이다 위치   ({-s["origin"][0]:+.4f}, {-s["origin"][1]:+.4f})'
              f'   원점에서 {np.hypot(*s["origin"]):.4f} m')
        print(f'    M1 흔들림     {M1[lid]["sx"]*1000:.1f}/{M1[lid]["sy"]*1000:.1f} mm'
              f'  ({M1[lid]["n"]}점)')
        print(f'    M2 흔들림     {M2[lid]["sx"]*1000:.1f}/{M2[lid]["sy"]*1000:.1f} mm'
              f'  ({M2[lid]["n"]}점)')

    # 두 라이다가 본 '라이다 사이 거리' — 배치 확인용
    if len(lids) >= 2:
        a, b = sol[lids[0]]['origin'], sol[lids[1]]['origin']
        print(f'\n  [배치] 라이다{lids[0]} 와 라이다{lids[1]} 사이 거리 '
              f'{np.hypot(a[0]-b[0], a[1]-b[1]):.3f} m')
        print('        줄자로 재서 이 값과 맞는지 확인하면 좋습니다.')

    print(f'\n  [좌표계]  모든 라이다가 공유')
    print(f'    원점 (0,0,0)   마커를 놓았던 첫 자리 (바닥)')
    print(f'    {front} 방향     M1 -> M2')
    print(f'    비행 영역      x {roi["x_min"]:.2f}~{roi["x_max"]:.2f}'
          f'  y {roi["y_min"]:.2f}~{roi["y_max"]:.2f}'
          f'  z {float(gp("roi_z_min")):.2f}~{float(gp("roi_z_max")):.2f}')

    print('\n  저장:')
    for p in paths:
        print(f'    {p}')
    print('\n  적용:')
    print('    colcon build --packages-select dual_ouster_driver --symlink-install')
    print('=' * 66 + '\n')
    return 0


def main(args=None):
    rclpy.init(args=args)
    node = Calibrator()
    t = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    t.start()
    code = 1
    try:
        code = run_multi(node) if len(node.lids) > 1 else run(node)
    except KeyboardInterrupt:
        print('\n중단됨')
    finally:
        # spin 스레드가 돌고 있는 채로 종료하면 C++ 쪽에서 terminate 가 난다.
        # shutdown 으로 spin 을 먼저 빠져나오게 하고 스레드를 거둔 뒤 정리한다.
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass          # Ctrl+C 로 이미 내려간 경우
        t.join(timeout=2.0)
        try:
            node.destroy_node()
        except Exception:
            pass
    sys.exit(code)


if __name__ == '__main__':
    main()
