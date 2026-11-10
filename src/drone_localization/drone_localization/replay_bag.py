#!/usr/bin/env python3
"""
rosbag을 트래커에 다시 먹여 오프라인으로 채점한다 — 런 편차 없는 비교.

왜 필요한가
-----------
같은 코드로 날려도 결과가 런마다 0.056~0.113m로 흔들린다(폭 0.057m).
파라미터 하나를 바꿔 얻는 차이는 보통 그보다 작아서, 새 비행끼리 비교하면
개선인지 그날 운인지 구분할 수 없다.

비행 한 번을 bag으로 녹화해 두고 여러 설정에 재생하면 **입력이 완전히
같으므로 차이는 전부 설정 탓**이 된다. 비행 한 번 값으로 조합을 몇십 개든
비교할 수 있다.

어떻게 동작하나
---------------
노드를 띄우되 spin하지 않는다. bag에서 꺼낸 메시지를 콜백에 직접 넣고,
타이머(_process)는 bag 시각에 맞춰 손으로 돌린다. ROS 실행기가 개입하지
않으므로 재생할 때마다 **완전히 같은 결과**가 나온다.

시계를 bag 시각으로 바꿔치기하는 것이 중요하다. 트래커는 time.monotonic()
으로 "버퍼가 오래됐나 / 관측이 끊겼나"를 판단하는데, 오프라인 재생은
실시간보다 훨씬 빠르므로 진짜 시계를 쓰면 버퍼가 절대 낡지 않아 실제
비행과 다르게 동작한다. 그래서 모듈의 time을 bag 시각을 돌려주는 것으로
갈아끼운다.

같은 bag 을 PF / MF / SU 로 각각 재생하면 같은 비행에서 세 융합 방식을 비교할 수 있다.

녹화
----
  추적 GUI 에서 "원본 점군 녹화"를 켜고 비행하면 run 폴더에 bag/ 이 생긴다.

재생 (실내 실기, 수신 시각 기준)
----
  python3 -m drone_localization.replay_bag --bag <run 폴더>/bag --out <결과 폴더> \\
      --clock arrival --quiet --ros-args --params-file <트래커 yaml> -p fusion_mode:=position
  # fusion_mode: cloud = PF, position = MF, sequential = SU

출력: estimate_details.csv, state_details.csv (KF 상태와 공분산), measurements.csv,
processing_cost.csv, manifest.json (사용한 파라미터·코드 해시).
"""

import argparse
import csv
import json
import hashlib
import shutil
import os
import sys
import time as _real_time

import rclpy
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

try:
    import rosbag2_py
except ImportError:
    raise ImportError('rosbag2_py가 없습니다. sudo apt install ros-humble-rosbag2-py')


# 이 저장소의 트래커는 lidar_drone_tracker 하나다 (시뮬 저장소의 lidar_drone_tracker_cov 와 같은 파일).
# 시뮬 저장소와 같은 명령을 쓰도록 'cov' 이름도 받는다.
TRACKERS = {
    'cov':   ('drone_localization.lidar_drone_tracker', 'LidarDroneTracker'),
    'fused': ('drone_localization.lidar_drone_tracker', 'LidarDroneTracker'),
}

DEFAULT_ACTUAL_TOPIC = '/fmu/out/vehicle_local_position_v1'
DEFAULT_GROUND_TRUTH_TOPIC = '/world/competition_map/dynamic_pose/info'


class _BagClock:
    """모듈의 time을 대신한다. monotonic()만 bag 시각으로 바꾸고 나머지는 그대로.

    트래커가 time.monotonic()으로 재는 것은 전부 '얼마나 지났나'이지 절대
    시각이 아니다. 그래서 bag 시각(초)을 그대로 돌려주면 실제 비행과 같은
    판단이 나온다."""

    def __init__(self):
        self.t = 0.0

    def monotonic(self):
        return self.t

    def __getattr__(self, name):
        return getattr(_real_time, name)


def _open_bag(path):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=path, storage_id=''),
        rosbag2_py.ConverterOptions('', ''))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    return reader, types


def main():
    ap = argparse.ArgumentParser(
        description='rosbag을 트래커에 재생해 오프라인 채점용 CSV를 만든다')
    ap.add_argument('--bag', required=True, help='rosbag 디렉터리')
    ap.add_argument('--tracker', default='cov', choices=sorted(TRACKERS),
                    help='재생할 트래커')
    ap.add_argument('--out', required=True, help='결과를 쓸 폴더')
    ap.add_argument('--actual-topic', default=DEFAULT_ACTUAL_TOPIC)
    ap.add_argument(
        '--ground-truth-topic', default=DEFAULT_GROUND_TRUTH_TOPIC,
        help='Gazebo world pose 토픽. bag에 있으면 actual_path.csv의 기준으로 '
             'PX4 위치보다 우선 사용한다.')
    ap.add_argument(
        '--ground-truth-pose-index', type=int, default=0,
        help='PoseArray에서 드론 world pose의 인덱스 (competition_map은 0)')
    ap.add_argument('--require-ground-truth', action='store_true')
    ap.add_argument('--quiet', action='store_true', help='진행 표시 끄기')
    ap.add_argument(
        '--clock', choices=('arrival', 'stamp'), default='arrival',
        help='트래커 시계. arrival = bag 수신 시각, stamp = 스캔 헤더의 측정 시각. '
             'Gazebo 는 센서를 순서대로 처리해 같은 시각의 두 스캔이 수십 ms 차이로 도착하므로, '
             '실기처럼 짝 스캔을 같은 시각에 처리하려면 stamp 를 쓴다')
    ap.add_argument('--lidar2-delay', type=float, default=0.0,
                    help='레거시 지연 주입: 도착과 헤더를 함께 이동(초). 촬영 위상 실험으로 사용 금지. '
                         '시뮬은 두 센서가 완전 동기(실측 0.00ms)지만 실기체는 '
                         '각자 자유 구동이라 0~100ms 어긋난다. 그 상황을 흉내낸다.')
    args, _ = ap.parse_known_args()

    bag = os.path.expanduser(args.bag)
    out = os.path.expanduser(args.out)
    if not os.path.isdir(bag):
        sys.exit(f'bag을 찾을 수 없습니다: {bag}')
    os.makedirs(out, exist_ok=True)

    # --ros-args는 그대로 넘겨서 --params-file / -p 가 평소처럼 먹게 한다.
    rclpy.init(args=sys.argv)

    mod_name, cls_name = TRACKERS[args.tracker]
    mod = __import__(mod_name, fromlist=[cls_name])

    # 시계 바꿔치기는 노드를 만들기 **전에** 해야 한다. 생성자가 이미
    # time.monotonic()으로 통계 시작 시각을 잡기 때문이다.
    clock = _BagClock()
    patched = []
    for mod_key, m in list(sys.modules.items()):
        if mod_key.startswith('drone_localization.') and hasattr(m, 'time'):
            m.time = clock
            patched.append(mod_key)

    node = getattr(mod, cls_name)()
    node.get_logger().info(
        f'재생 시작: {os.path.basename(bag)} → {args.tracker} '
        f'(시계 교체: {len(patched)}개 모듈)')

    # 발행되는 추정치를 가로챈다. 토픽으로 받으려면 spin이 필요한데,
    # spin을 쓰면 재생이 다시 비결정적이 된다.
    est_rows = []
    estimate_details = []
    state_details = []   # 평가 시각 예측 보정용: 출력 순간의 KF 상태(위치·속도)와 6x6 공분산
    gt_details = []
    costs = []
    orig_publish = node._publish_pose

    # D의 지연이 KF에서 생기는지, KF에 들어가기 전 LiDAR 관측에서 이미
    # 생기는지 분리할 수 있도록 단일 주 KF의 입력도 함께 저장한다.
    meas_rows = []
    if hasattr(node, '_kf') and hasattr(node._kf, 'update'):
        orig_update = node._kf.update

        def capture_update(z, *args, **kwargs):
            meas_rows.append(
                (clock.t, float(z[0]), float(z[1]), float(z[2])))
            return orig_update(z, *args, **kwargs)

        node._kf.update = capture_update

    def capture(header, *values, **kwargs):
        published = orig_publish(header, *values, **kwargs)
        # B may publish a read-only extrapolation rather than its KF state.
        p, cov = published if published is not None else (node._kf.position, node._kf.cov_pos)
        est_rows.append((clock.t, p[0], p[1], p[2]))
        estimate_details.append([clock.t,
            header.stamp.sec + header.stamp.nanosec * 1e-9,
            *p, *cov.reshape(-1)])
        if hasattr(node, '_kf') and hasattr(node._kf, 'x'):
            state_details.append([clock.t, *node._kf.x, *node._kf.P.reshape(-1)])

    node._publish_pose = capture

    reader, types = _open_bag(bag)
    l1, l2 = node._sensors[0].topic, node._sensors[1].topic
    # D(cov)는 구독을 lambda로 _store_cloud(idx, msg)에 묶어 두어 _cb_lidarN이
    # 없다. 비교 대상 트래커는 고치지 않는 것이 원칙이므로 여기서 이름만 맞춘다.
    if not hasattr(node, '_cb_lidar1'):
        if not hasattr(node, '_store_cloud'):
            sys.exit(f'{args.tracker}: _cb_lidarN 도 _store_cloud 도 없습니다')
        node._cb_lidar1 = lambda m: node._store_cloud(0, m)
        node._cb_lidar2 = lambda m: node._store_cloud(1, m)

    has_gt = args.ground_truth_topic in types
    if args.require_ground_truth and not has_gt:
        raise RuntimeError('Gazebo ground truth is required for this experiment')
    wanted = {l1: '1', l2: '2', args.actual_topic: 'a'}
    if has_gt:
        wanted[args.ground_truth_topic] = 'g'
    missing = [t for t in wanted if t not in types]
    if missing:
        node.get_logger().warn(
            f'bag에 없는 토픽: {missing}\n  bag이 가진 토픽: {sorted(types)}')

    msg_cls = {t: get_message(types[t]) for t in wanted if t in types}

    px4_rows = []
    gt_rows = []
    t0 = None
    n_scan = 0
    delay = args.lidar2_delay
    pending2 = []        # 지연 적용된 라이다2 스캔 대기열
    # 타이머(_process)를 bag 시각에 맞춰 손으로 돌린다. 특히 A(fused)는
    # 모든 처리를 이 타이머에서 하므로 이게 없으면 아무것도 나오지 않는다.
    dt = getattr(node, '_dt', 0.1)
    next_timer = None

    def timed_call(kind, fn, *values):
        start = _real_time.perf_counter()
        fn(*values)
        costs.append((kind, _real_time.perf_counter() - start))

    def run_timers(upto):
        nonlocal next_timer
        while next_timer is not None and next_timer <= upto:
            saved, clock.t = clock.t, next_timer
            try:
                timed_call('timer', node._process)
            except Exception as e:      # noqa: BLE001 - 재생은 끝까지 간다
                raise RuntimeError(f'timer failed at {next_timer:.2f}s') from e
            clock.t = saved
            next_timer += dt

    def feed_pending(upto):
        nonlocal n_scan
        while pending2 and pending2[0][0] <= upto:
            tt, m = pending2.pop(0)
            clock.t = tt
            run_timers(tt)
            timed_call('lidar2', node._cb_lidar2, m)
            n_scan += 1

    while reader.has_next():
        topic, data, stamp_ns = reader.read_next()
        if topic not in wanted:
            continue
        msg = deserialize_message(data, msg_cls[topic])
        if args.clock == 'stamp' and topic not in (l1, l2):
            if t0 is None:      # 시계는 LiDAR 스캔만 움직인다
                continue
        else:
            t = (msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
                 if args.clock == 'stamp' else stamp_ns * 1e-9)
            if t0 is None:
                t0 = t
                next_timer = 0.0
            feed_pending(t - t0)
            # 실기는 짝 스캔의 스탬프가 0.5 ms 안에서 앞뒤가 바뀐다. 시계를 되돌리면 SU 의 두 번째
            # 갱신 상태가 첫 번째보다 이른 시각으로 기록되므로 시계는 앞으로만 간다.
            clock.t = t - t0 if args.clock == 'arrival' else max(clock.t, t - t0)
            run_timers(clock.t)

        if topic == l1:
            timed_call('lidar1', node._cb_lidar1, msg)
            n_scan += 1
        elif topic == l2:
            if delay > 0.0:
                # 헤더 스탬프도 같이 밀어야 트래커가 '늦게 찍힌 스캔'으로 본다.
                ns = msg.header.stamp.nanosec + int(delay * 1e9)
                msg.header.stamp.sec += ns // 1_000_000_000
                msg.header.stamp.nanosec = ns % 1_000_000_000
                pending2.append((clock.t + delay, msg))
            else:
                timed_call('lidar2', node._cb_lidar2, msg)
                n_scan += 1
        elif topic == args.actual_topic:
            if getattr(msg, 'xy_valid', True) and getattr(msg, 'z_valid', True):
                # NED → world ENU (flight_recorder와 같은 변환)
                px4_rows.append((clock.t, msg.y, msg.x, -msg.z))
        else:
            idx = args.ground_truth_pose_index
            if idx < 0 or idx >= len(msg.poses):
                sys.exit(
                    f'ground-truth pose index {idx}가 범위를 벗어났습니다 '
                    f'(poses={len(msg.poses)})')
            p = msg.poses[idx].position
            gt_t = (msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9 - t0
                    if args.clock == 'stamp' else clock.t)
            gt_rows.append((gt_t, p.x, p.y, p.z))
            q = msg.poses[idx].orientation
            gt_details.append([clock.t,
                msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
                p.x, p.y, p.z, q.x, q.y, q.z, q.w, len(msg.poses)])

        if not args.quiet and n_scan % 500 == 0 and n_scan:
            print(f'\r  스캔 {n_scan}  추정 {len(est_rows)}  '
                  f'{clock.t:6.1f}초', end='', flush=True)

    feed_pending(float('inf'))
    if not args.quiet:
        print()

    def write(name, rows):
        p = os.path.join(out, name)
        with open(p, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['t_sec', 'world_x', 'world_y', 'world_z'])
            for r in rows:
                w.writerow([f'{r[0]:.9f}'] + [f'{v:.9f}' for v in r[1:]])
        return p

    def details(name, header, rows):
        with open(os.path.join(out, name), 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(header)
            writer.writerows(rows)
    details('estimate_details.csv', ['arrival_s', 'stamp_s', 'x', 'y', 'z'] +
            [f'p{i}{j}' for i in range(3) for j in range(3)], estimate_details)
    if state_details:
        details('state_details.csv', ['arrival_s', 'x', 'y', 'z', 'vx', 'vy', 'vz'] +
                [f'P{i}{j}' for i in range(6) for j in range(6)], state_details)
    details('gt_details.csv', ['arrival_s', 'stamp_s', 'x', 'y', 'z',
                             'qx', 'qy', 'qz', 'qw', 'pose_count'], gt_details)
    details('processing_cost.csv', ['event', 'seconds'], costs)
    params = {name: param.value for name, param in
              node.get_parameters_by_prefix('').items()}
    source = mod.__file__
    shutil.copy2(source, os.path.join(out, 'tracker_source.py'))
    shutil.copy2(__file__, os.path.join(out, 'replay_source.py'))
    with open(source, 'rb') as f:
        source_hash = hashlib.sha256(f.read()).hexdigest()
    with open(os.path.join(out, 'manifest.json'), 'w') as f:
        json.dump(dict(argv=sys.argv, tracker=args.tracker, parameters=params,
                       source=source, source_sha256=source_hash, bag=bag,
                       gt_topic=args.ground_truth_topic if has_gt else None,
                       gt_pose_index=args.ground_truth_pose_index,
                       clock='bag arrival seconds; header stamps saved separately',
                       replay_age='callback CPU duration is zero on bag clock'),
                  f, indent=2)
    actual_rows = gt_rows if gt_rows else px4_rows
    write('actual_path.csv', actual_rows)
    if gt_rows:
        write('actual_ground_truth_path.csv', gt_rows)
        write('actual_px4_path.csv', px4_rows)
    write('estimated_path.csv', est_rows)
    if meas_rows:
        write('measurements.csv', meas_rows)

    dur = clock.t if t0 is not None else 0.0
    node.get_logger().info(
        f'재생 완료: 스캔 {n_scan}  실측 {len(actual_rows)}'
        + (f' (Gazebo GT, PX4 {len(px4_rows)})' if gt_rows else ' (PX4)')
        + f'  추정 {len(est_rows)}'
        + (f' ({len(est_rows) / dur:.1f}Hz)' if dur > 0 else '')
        + f'  → {out}')

    node.close()
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
