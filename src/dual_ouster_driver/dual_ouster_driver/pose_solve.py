#!/usr/bin/env python3

"""마커 두 지점 측정값으로 yaw 와 원점을 계산한다.

marker_fit 의 ~/record 서비스로 저장한 기록을 읽는다.

  1번째 기록  =  원점이 될 지점        -> (0, 0) 이 된다
  2번째 기록  =  +x 축 방향의 지점     -> (거리, 0) 이 된다

계산:
    theta   = -atan2(y2-y1, x2-x1)      돌려야 할 각도
    새 yaw  = 현재 yaw + theta
    새 원점 = R(theta) · M1             회전 후에 뺄 값

roi_filter 가 `R(yaw)·p - origin` 순서로 적용하므로 원점도 회전된 값이어야 한다.

사용:
    ros2 run dual_ouster_driver pose_solve
    ros2 run dual_ouster_driver pose_solve --ros-args -p current_yaw_deg:=180.0
"""

import math

import numpy as np
import rclpy
from rclpy.node import Node


def solve(m1, m2, current_yaw_deg):
    """두 점으로 새 yaw 와 원점을 구한다."""
    dx, dy = m2[0] - m1[0], m2[1] - m1[1]
    dist = math.hypot(dx, dy)
    theta = -math.atan2(dy, dx)

    c, s = math.cos(theta), math.sin(theta)
    R = np.array([[c, -s], [s, c]])
    origin = R @ np.asarray(m1, dtype=float)

    return {
        'theta_deg': math.degrees(theta),
        'yaw_deg': current_yaw_deg + math.degrees(theta),
        'origin_x': float(origin[0]),
        'origin_y': float(origin[1]),
        'dist': dist,
    }


class PoseSolve(Node):

    def __init__(self):
        super().__init__('pose_solve')
        self.declare_parameter('record_file', '/tmp/marker_records.txt')
        self.declare_parameter('current_yaw_deg', 180.0)

        path = str(self.get_parameter('record_file').value)
        cur_yaw = float(self.get_parameter('current_yaw_deg').value)

        try:
            rows = []
            for line in open(path):
                line = line.strip()
                if line:
                    rows.append([float(v) for v in line.split()])
        except FileNotFoundError:
            self.get_logger().error(
                f'{path} 없음. marker_fit 의 ~/record 서비스를 먼저 호출하세요.')
            return

        if len(rows) < 2:
            self.get_logger().error(
                f'기록이 {len(rows)}개뿐입니다. 2개가 필요합니다.')
            return

        if len(rows) > 2:
            self.get_logger().warn(
                f'기록이 {len(rows)}개입니다. 마지막 2개를 씁니다.')
        r1, r2 = rows[-2], rows[-1]
        m1, m2 = (r1[0], r1[1]), (r2[0], r2[1])
        res = solve(m1, m2, cur_yaw)

        # 검증: 구한 값을 적용하면 두 마커가 어디로 가는지
        c, s = math.cos(math.radians(res['theta_deg'])), math.sin(math.radians(res['theta_deg']))
        R = np.array([[c, -s], [s, c]])
        o = np.array([res['origin_x'], res['origin_y']])
        p1 = R @ np.asarray(m1) - o
        p2 = R @ np.asarray(m2) - o

        out = [
            '',
            '=' * 60,
            '  마커 기반 자세 추정 결과',
            '=' * 60,
            f'  입력 (현재 좌표계, yaw={cur_yaw:.2f}도 기준)',
            f'    M1 원점 지점    ({m1[0]:8.4f}, {m1[1]:8.4f})  흔들림 {r1[3]*1000:.1f}/{r1[4]*1000:.1f}mm',
            f'    M2 +x 방향      ({m2[0]:8.4f}, {m2[1]:8.4f})  흔들림 {r2[3]*1000:.1f}/{r2[4]*1000:.1f}mm',
            f'    두 마커 간 거리  {res["dist"]:.4f} m',
            '',
            f'  보정량',
            f'    회전 {res["theta_deg"]:+.3f}도',
            '',
            '  적용할 값',
            f'    -p yaw_deg:={res["yaw_deg"]:.3f}',
            f'    -p origin_x:={res["origin_x"]:.4f}',
            f'    -p origin_y:={res["origin_y"]:.4f}',
            '',
            '  적용 후 라이다 위치',
            f'    ({-res["origin_x"]:7.4f}, {-res["origin_y"]:7.4f})'
            f'   원점에서 {math.hypot(res["origin_x"], res["origin_y"]):.4f} m',
            '',
            '  검증 (적용 후 마커 위치)',
            f'    M1 -> ({p1[0]:7.4f}, {p1[1]:7.4f})   기대 (0, 0)',
            f'    M2 -> ({p2[0]:7.4f}, {p2[1]:7.4f})   기대 ({res["dist"]:.4f}, 0)',
            '',
            '  실행 중 적용:',
            f'    ros2 param set /roi_filter yaw_deg {res["yaw_deg"]:.3f}',
            f'    ros2 param set /roi_filter origin_x {res["origin_x"]:.4f}',
            f'    ros2 param set /roi_filter origin_y {res["origin_y"]:.4f}',
            '',
            '  yaw 정확도 추정',
        ]
        err = max(r1[3], r1[4], r2[3], r2[4])
        if res['dist'] > 0.01:
            out.append(
                f'    마커 흔들림 {err*1000:.1f}mm, 간격 {res["dist"]:.2f}m'
                f'  ->  약 {math.degrees(math.atan(err/res["dist"])):.3f}도')
        out.append('=' * 60)
        self.get_logger().info('\n'.join(out))


def main(args=None):
    rclpy.init(args=args)
    node = PoseSolve()
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
