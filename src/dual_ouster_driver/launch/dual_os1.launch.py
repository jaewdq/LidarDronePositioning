#!/usr/bin/env python3

"""OS1 두 대를 띄운다.

os_driver 는 라이프사이클 노드라 그냥 실행하면 unconfigured 에서 멈춘다
(CPU 0%, 로그도 안 나와서 죽은 것처럼 보인다). configure -> activate 를
명시적으로 쏴줘야 데이터가 나온다.

토픽 이름은 remappings 가 아니라 namespace 로 잡는다. 이 버전의 ouster-ros
는 상대 이름으로 발행해서 '/ouster/points' 같은 절대 경로 remap 이 매칭되지
않는다.

  192.168.6.11  OS1-070-128U-AX    sn 122531002331  -> /lidar1/points
  192.168.6.12  OS1-070-128U-ASR   sn 122451001190  -> /lidar2/points

한 대만 띄우려면:
    ros2 launch dual_ouster_driver dual_os1.launch.py enable_lidar2:=false
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


def make_lidar(idx, host):
    """드라이버 노드 + 전이 이벤트 3개를 묶어서 돌려준다."""
    node = LifecycleNode(
        package='ouster_ros',
        executable='os_driver',
        name=f'os_driver_lidar{idx}',
        namespace=f'lidar{idx}',
        output='screen',
        parameters=[{
            'sensor_hostname': host,
            'lidar_mode': '1024x10',
            # PTP. 그랜드마스터는 이 PC 의 시스템 클럭이다
            # (sudo ptp4l -f ~/drone_ws/ptp_gm.conf -m, priority1=0).
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

            # 위상 고정은 끈 채로 둔다.
            #
            # 두 센서의 회전 출발 각도가 달라서 PTP 로 시계를 2us 까지
            # 맞춘 뒤에도 스캔 시각 델타가 37ms 와 62ms 를 번갈아 남았다
            # (37+62 = 99ms, 10Hz 한 바퀴). 위상차이지 시계 오차가 아니다.
            #
            # 그래서 phase_lock_enable=True, offset=0 을 넣어봤더니 센서가
            # RUNNING 을 유지하면서도 UDP 를 아예 멈췄고, 드라이버가
            # poll_client() 타임아웃 -> self reset 루프에 빠졌다. 설정 자체는
            # 센서에 반영됐지만(config API 확인) 데이터가 나오지 않는다.
            #
            # 대신 트래커 쪽 sync_tolerance_sec 를 실제 델타보다 크게 잡아
            # 융합이 일어나도록 한다. 37ms 어긋남은 걷는 속도(0.7m/s)에서
            # 2.5cm 라 실내 테스트에는 충분히 작다.
            'phase_lock_enable': False,
            'phase_lock_offset': 0,

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
    acts = make_lidar(1, SENSORS[1])
    if use2.lower() not in ('false', '0', 'no'):
        acts += make_lidar(2, SENSORS[2])
    return acts


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'enable_lidar2', default_value='true',
            description='두 번째 라이다(192.168.6.12)도 띄울지'),
        OpaqueFunction(function=setup),
    ])
