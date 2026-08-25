#!/usr/bin/env python3

"""비행 중 라이다 추적 결과를 CSV 로 남긴다.

cluster_median 이 낸 위치를 그대로(raw) 와 칼만필터를 거친 값(kf) 둘 다
기록한다. 어느 쪽이 나은지는 나중에 데이터를 보고 판단하면 된다.

  입력  /lidarN/cluster_poses   geometry_msgs/PoseArray
  출력  <out_dir>/flight_<시각>/track.csv
        <out_dir>/flight_<시각>/meta.txt

CSV 한 줄 = 라이다 한 프레임. 검출이 없었던 프레임도 detected=0 으로 남긴다.
그래야 나중에 "언제 놓쳤는지"를 알 수 있다.

시각(t_sec)은 처리 시각이 아니라 **라이다가 찍은 시각**이다. 지연이 200 ms
쯤 있는데, 처리 시각을 쓰면 그게 그대로 오차가 된다.

기준값(ground truth) 은 아직 없다. 나중에 모캡을 붙이면 같은 형식으로
truth.csv 를 남기고 flight_report 가 비교하게 하면 된다.

사용:
    ros2 run dual_ouster_driver flight_recorder
    ros2 run dual_ouster_driver flight_recorder --ros-args -p out_dir:=~/flights

Ctrl+C 로 끝내면 요약이 출력되고 파일이 닫힌다.
"""

import csv
import math
import os
import sys
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseArray
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy


class KF6:
    """등속도 모델 6상태 칼만필터 [x y z vx vy vz].

    라이다 관측은 프레임마다 몇 mm 씩 튀고, 가끔 통째로 빠진다.
    빠진 구간은 예측만으로 이어가고, 너무 오래 빠지면 초기화한다.
    """

    def __init__(self, q_pos=0.05, q_vel=0.30, r_pos=0.05):
        self.x = np.zeros(6)
        self.P = np.eye(6) * 9.0
        self.Q = np.diag([q_pos**2] * 3 + [q_vel**2] * 3)
        self.R = np.eye(3) * r_pos**2
        self.H = np.zeros((3, 6))
        self.H[0, 0] = self.H[1, 1] = self.H[2, 2] = 1.0
        self.ready = False

    def init(self, z):
        self.x[:3] = z
        self.x[3:] = 0.0
        self.P = np.eye(6) * 0.25
        self.ready = True

    def predict(self, dt):
        if not self.ready or dt <= 0:
            return
        F = np.eye(6)
        F[0, 3] = F[1, 4] = F[2, 5] = dt
        self.x = F @ self.x
        self.P = F @ self.P @ F.T + self.Q * dt

    def update(self, z):
        if not self.ready:
            self.init(z)
            return
        y = z - self.H @ self.x
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)
        self.x = self.x + K @ y
        A = np.eye(6) - K @ self.H
        self.P = A @ self.P @ A.T + K @ self.R @ K.T


class FlightRecorder(Node):

    def __init__(self):
        super().__init__('flight_recorder')
        num = ParameterDescriptor(dynamic_typing=True)

        self.declare_parameter('lidar_id', 1, num)
        self.declare_parameter('input_topic', '')
        self.declare_parameter('out_dir', '~/flights')
        self.declare_parameter('label', '')
        # 이 시간 넘게 못 잡으면 칼만필터를 초기화한다.
        self.declare_parameter('lost_reset_sec', 2.0, num)
        # 칼만필터 잡음. 드론이 급기동하면 q_vel 을 키운다.
        self.declare_parameter('q_pos', 0.05, num)
        self.declare_parameter('q_vel', 0.30, num)
        self.declare_parameter('r_pos', 0.05, num)
        self.declare_parameter('report_period', 2.0, num)

        gp = lambda n: self.get_parameter(n).value
        self.lid = int(gp('lidar_id'))
        topic = str(gp('input_topic')) or f'/lidar{self.lid}/cluster_poses'

        stamp = time.strftime('%Y%m%d_%H%M%S')
        label = str(gp('label')).strip()
        name = f'flight_{stamp}' + (f'_{label}' if label else '')
        self.out_dir = os.path.join(
            os.path.expanduser(str(gp('out_dir'))), name)
        os.makedirs(self.out_dir, exist_ok=True)

        self.csv_path = os.path.join(self.out_dir, 'track.csv')
        self._f = open(self.csv_path, 'w', newline='')
        self._w = csv.writer(self._f)
        self._w.writerow([
            't_sec', 'detected',
            'raw_x', 'raw_y', 'raw_z',
            'kf_x', 'kf_y', 'kf_z',
            'kf_vx', 'kf_vy', 'kf_vz',
            'speed', 'n_cluster',
        ])

        self._kf = KF6(float(gp('q_pos')), float(gp('q_vel')), float(gp('r_pos')))
        self._t0 = None
        self._t_last = None
        self._t_seen = None
        self._rows = 0
        self._hits = 0
        self._gaps = []          # (시작, 끝) 놓친 구간
        self._gap_start = None

        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST, depth=20)
        self.create_subscription(PoseArray, topic, self._cb, qos)
        self.create_timer(float(gp('report_period')), self._report)

        self.get_logger().info(f'기록 시작: {topic} -> {self.csv_path}')
        self.get_logger().info('Ctrl+C 로 종료하면 요약이 나옵니다.')

    # ------------------------------------------------------------------
    def _cb(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self._t0 is None:
            self._t0 = t
        rel = t - self._t0
        dt = (t - self._t_last) if self._t_last is not None else 0.0
        self._t_last = t

        det = len(msg.poses) > 0
        raw = (float('nan'),) * 3
        if det:
            p = msg.poses[0].position
            raw = (p.x, p.y, p.z)

        self._kf.predict(dt)
        if det:
            self._kf.update(np.array(raw))
            self._t_seen = t
            self._hits += 1
            if self._gap_start is not None:
                self._gaps.append((self._gap_start, rel))
                self._gap_start = None
        else:
            if self._gap_start is None and self._rows > 0:
                self._gap_start = rel
            lost = float(self.get_parameter('lost_reset_sec').value)
            if self._t_seen is not None and (t - self._t_seen) > lost:
                self._kf.ready = False

        if self._kf.ready:
            kx, ky, kz = self._kf.x[:3]
            vx, vy, vz = self._kf.x[3:]
            sp = float(np.linalg.norm(self._kf.x[3:]))
        else:
            kx = ky = kz = vx = vy = vz = sp = float('nan')

        f = lambda v: '' if (isinstance(v, float) and math.isnan(v)) else f'{v:.5f}'
        self._w.writerow([f'{rel:.4f}', 1 if det else 0,
                          f(raw[0]), f(raw[1]), f(raw[2]),
                          f(kx), f(ky), f(kz), f(vx), f(vy), f(vz), f(sp),
                          len(msg.poses)])
        self._rows += 1
        if self._rows % 50 == 0:
            self._f.flush()

    def _report(self):
        if self._rows == 0:
            self.get_logger().warn('입력 없음 — cluster_median 이 켜져 있는지 확인')
            return
        dur = self._t_last - self._t0 if self._t_last else 0.0
        rate = 100.0 * self._hits / self._rows
        pos = ''
        if self._kf.ready:
            pos = (f'   위치 ({self._kf.x[0]:+.2f},{self._kf.x[1]:+.2f},'
                   f'{self._kf.x[2]:.2f})  {np.linalg.norm(self._kf.x[3:]):.2f} m/s')
        self.get_logger().info(
            f'{dur:6.1f}s  프레임 {self._rows}  검출 {rate:.0f}%{pos}')

    # ------------------------------------------------------------------
    def close(self):
        if self._gap_start is not None and self._t_last is not None:
            self._gaps.append((self._gap_start, self._t_last - self._t0))
        try:
            self._f.flush()
            self._f.close()
        except Exception:
            pass

        dur = (self._t_last - self._t0) if self._t_last else 0.0
        rate = (100.0 * self._hits / self._rows) if self._rows else 0.0
        meta = os.path.join(self.out_dir, 'meta.txt')
        lines = [
            f'기록 시각   {time.strftime("%Y-%m-%d %H:%M:%S")}',
            f'라이다      lidar{self.lid}',
            f'비행 시간   {dur:.1f} s',
            f'프레임      {self._rows}',
            f'검출        {self._hits} ({rate:.1f}%)',
            f'놓친 구간   {len(self._gaps)}회',
        ]
        for a, b in self._gaps[:20]:
            lines.append(f'   {a:7.2f} ~ {b:7.2f} s   ({b-a:.2f} s)')
        if len(self._gaps) > 20:
            lines.append(f'   ... 외 {len(self._gaps)-20}회')
        try:
            with open(meta, 'w') as f:
                f.write('\n'.join(lines) + '\n')
        except Exception:
            pass

        print('\n' + '=' * 58)
        print('  기록 종료')
        print('=' * 58)
        for l in lines:
            print('  ' + l)
        print(f'\n  저장: {self.out_dir}')
        print(f'\n  분석:')
        print(f'    ros2 run dual_ouster_driver flight_report --ros-args '
              f'-p run_dir:={self.out_dir}')
        print()


def main(args=None):
    rclpy.init(args=args)
    node = FlightRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        # Ctrl+C 를 두 번 누르거나 launch 가 프로세스 그룹에 INT 를 보내면
        # 정리 도중에 또 인터럽트가 들어온다. KeyboardInterrupt 는
        # Exception 이 아니라 BaseException 이라 그냥 except Exception
        # 으로는 못 막는다. 파일은 close() 에서 이미 닫혔으므로
        # 여기서 죽어도 데이터는 안전하지만, 트레이스백은 보기 싫다.
        for fn in (rclpy.shutdown, node.destroy_node):
            try:
                fn()
            except BaseException:
                pass
    sys.exit(0)


if __name__ == '__main__':
    main()
