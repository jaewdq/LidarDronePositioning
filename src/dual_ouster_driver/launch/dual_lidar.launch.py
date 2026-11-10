#!/usr/bin/env python3

"""Ouster 라이다 두 대(OS0-SR 64ch)를 띄운다. PTP 시각 + 위상 고정이 기본이다.

os_driver 는 라이프사이클 노드라 그냥 실행하면 unconfigured 에서 멈춘다
(CPU 0%, 로그도 안 나와서 죽은 것처럼 보인다). configure -> activate 를
명시적으로 쏴줘야 데이터가 나온다.

토픽 이름은 remappings 가 아니라 namespace 로 잡는다. 이 버전의 ouster-ros
는 상대 이름으로 발행해서 '/ouster/points' 같은 절대 경로 remap 이 매칭되지
않는다.

  192.168.6.11  -> /lidar1/points   (남쪽 삼각대)
  192.168.6.12  -> /lidar2/points   (북쪽 삼각대)

한 대만 띄우려면:
    ros2 launch dual_ouster_driver dual_lidar.launch.py enable_lidar2:=false
"""

import launch
import lifecycle_msgs.msg
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, LogInfo,
                            OpaqueFunction, RegisterEventHandler)
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState

SENSORS = {
    1: '192.168.6.11',
    2: '192.168.6.12',
}

# 센서가 UDP 를 쏠 목적지. 비워두면(자동) 인터페이스가 여럿일 때 센서가
# 링크로컬(169.254.x.x) 같은 엉뚱한 주소를 고르고, 그러면 드라이버가
# poll_client() 타임아웃만 반복하며 데이터가 안 온다. 실제로 겪었다.
UDP_DEST = '192.168.6.100'


def make_lidar(idx, host, lidar_mode, phase_lock, phase_offset):
    """드라이버 노드 + 전이 이벤트 3개를 묶어서 돌려준다."""
    node = LifecycleNode(
        package='ouster_ros',
        executable='os_driver',
        name=f'os_driver_lidar{idx}',
        namespace=f'lidar{idx}',
        output='screen',
        parameters=[{
            'sensor_hostname': host,
            'lidar_mode': lidar_mode,
            # PTP. 그랜드마스터는 이 PC 의 시스템 클럭이다
            # (sudo ptp4l -f ptp_gm.conf -i <랜카드> -m, priority1=0).
            # 두 센서 모두 stratum 1 / rms_offset 약 2us 로 붙는 것을
            # 확인했다 (http://<ip>/api/v1/system/time 의 tracking).
            #
            # ROS_TIME 은 패킷이 호스트에 도착한 시각을 쓰기 때문에 두 센서
            # 스탬프가 34~65ms 어긋났고, sync_tolerance_sec(10ms)에 매번
            # 걸려 융합이 한 번도 일어나지 않았다.
            #
            # ptp4l 이 죽으면 센서가 동기 대상을 잃으므로 추적 전에 항상
            # 살아 있는지 확인할 것.
            'timestamp_mode': 'TIME_FROM_PTP_1588',

            # ouster-ros 는 PTP 시각을 TAI 로 보고 기본 37초를 뺀다. 그런데
            # ptp4l 소프트웨어 타임스탬프 모드는 PC 의 UTC 시계를 그대로 보내므로
            # 빼면 스탬프가 PC 시각보다 정확히 37초 늦어진다 (2026-10-29 OS0 에서 확인).
            # 두 센서 사이 융합에는 영향이 없지만 다른 기록과 시간축이 어긋난다.
            'ptp_utc_tai_offset': 0.0,

            # 위상 고정 (기본 켬, launch 인자 phase_lock).
            #
            # PTP 는 시계만 맞추고 회전 위상은 맞추지 않는다. 그대로 두면 두
            # 센서의 스캔 시각이 0~100 ms 어긋나(2026-08 OS1: 37/62 ms) PF/MF 의
            # 10 ms 짝짓기에 걸리지 않고 매 프레임 한 센서로만 갱신된다.
            # 위상 고정을 켜면 각 센서가 PTP 정각마다 같은 방위(offset)를 지나므로
            # 두 센서 프레임 시작 시각이 같아진다. 마주 본 두 센서는 같은 offset 이면
            # 필드 가운데를 동시에 지나고, 모서리에서도 차이는 26 ms 이내다.
            #
            # 2026-08 OS1 에서는 켜면 UDP 가 멈췄지만, 2026-10-29 OS0-SR(FW 3.1)
            # 에서는 실행 중에 켜도, 드라이버 시작 때 켜도 LOCKED · 10 Hz · 오류
            # 없이 동작했다 (offset 0/90/180° -> 프레임 위상 0/25/50 ms).
            # OS1 처럼 데이터가 끊기면 phase_lock:=false 로 끈다.
            'phase_lock_enable': phase_lock,
            'phase_lock_offset': phase_offset,

            'sensor_frame': f'lidar{idx}_sensor',
            'lidar_frame': f'lidar{idx}',
            'imu_frame': f'lidar{idx}_imu',
            'point_cloud_frame': f'lidar{idx}',

            'point_type': 'xyz',
            'proc_mask': 'PCL|IMU',

            # 0 이면 클라이언트가 빈 포트를 알아서 고른다. 두 대를 띄울 때
            # 포트를 고정하면 서로 부딪힌다.
            'lidar_port': 0,
            'imu_port': 0,
            'udp_dest': UDP_DEST,

            'attempt_reconnect': True,
        }],
    )

    configure = EmitEvent(event=ChangeState(
        lifecycle_node_matcher=matches_action(node),
        transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
    ))

    activate = RegisterEventHandler(OnStateTransition(
        target_lifecycle_node=node, goal_state='inactive',
        entities=[
            LogInfo(msg=f'os_driver_lidar{idx} activating...'),
            EmitEvent(event=ChangeState(
                lifecycle_node_matcher=matches_action(node),
                transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
            )),
        ],
        handle_once=True,
    ))

    # 한 대가 응답하지 않으면 전체를 내린다. 반쪽만 뜬 상태로 두면
    # 나중에 어느 쪽 데이터가 빠진 것인지 헷갈린다.
    failed = RegisterEventHandler(OnStateTransition(
        target_lifecycle_node=node, goal_state='finalized',
        entities=[
            LogInfo(msg=f'lidar{idx} ({host}) 와 통신하지 못했습니다.'),
            EmitEvent(event=launch.events.Shutdown(
                reason=f"Couldn't communicate with lidar{idx}")),
        ],
    ))

    return [node, configure, activate, failed]


def setup(context, *args, **kwargs):
    use2 = LaunchConfiguration('enable_lidar2').perform(context)
    mode = LaunchConfiguration('lidar_mode').perform(context)
    lock = LaunchConfiguration('phase_lock').perform(context).lower() not in ('false', '0', 'no')
    offset = int(LaunchConfiguration('phase_lock_offset').perform(context))
    acts = make_lidar(1, SENSORS[1], mode, lock, offset)
    if use2.lower() not in ('false', '0', 'no'):
        acts += make_lidar(2, SENSORS[2], mode, lock, offset)
    return acts


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'enable_lidar2', default_value='true',
            description='두 번째 라이다(192.168.6.12)도 띄울지'),
        # 2048x10 은 방위각 샘플을 두 배로 늘린다. 20 m 에서 가로 점 간격이
        # 12.3 cm -> 6.2 cm 가 되어, 지름 30 cm 마커에 링당 4~5 점이 찍힌다
        # (1024 에서는 2.4 점이라 원피팅이 안 선다). 보정할 때만 쓰고 비행
        # 추적은 1024x10 으로 되돌리는 것을 전제로 한다 -- 점이 두 배면
        # 클러스터링 비용도 두 배다. 외부 파라미터는 모드와 무관하므로
        # 되돌려도 보정값은 그대로 유효하다.
        DeclareLaunchArgument(
            'lidar_mode', default_value='1024x10',
            description='512x10 | 512x20 | 1024x10 | 1024x20 | 2048x10'),
        DeclareLaunchArgument(
            'phase_lock', default_value='true',
            description='위상 고정 (PF/MF 짝짓기용). OS1 처럼 데이터가 끊기면 false'),
        # 두 센서에 같은 값을 준다. 같은 방향(원시 -x 가 필드)으로 달린 두 센서가
        # 같은 순간 같은 자기 방위를 지나게 된다. 값 자체는 프레임 경계 위치만 바꾼다.
        DeclareLaunchArgument(
            'phase_lock_offset', default_value='0',
            description='위상 고정 방위 [millidegree], 0~360000, 두 센서 공통'),
        OpaqueFunction(function=setup),
    ])
