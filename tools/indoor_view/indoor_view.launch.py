"""실내 케이지 추적 화면을 한 번에: 트래커 + 표시 도구 + RViz.

PTP(sudo ptp4l)와 드라이버(dual_lidar.launch.py)는 먼저 각자 터미널에 띄워 둔다.

    ros2 launch ~/drone_project/LidarDronePositioning/tools/indoor_view/indoor_view.launch.py
    ... start_tracker:=false        트래커를 따로 띄웠을 때 (표시 도구 + RViz 만)
    ... params_file:=<yaml>         다른 트래커 YAML
    ... rviz:=false
    ... view:=track                 비행·녹화용 가벼운 화면 (점군 없이 필드·ROI·추적 위치,
                                    트래커의 ROI 점군 발행도 끔)
                                    기본 view:=calib 은 캘리브레이션 확인용 (두 센서 점군 겹침)
    ... view:=replay                경로 재생 화면 (path_replay.py 가 start_tracker:=false 로 띄운다)

트래커 실행 파일 이름은 PC 마다 다르다. 실제 운용 저장소만 빌드한 PC 는
lidar_drone_tracker, 시뮬 저장소도 함께 빌드한 PC 는 lidar_drone_tracker_cov
(같은 파일; 시뮬 저장소의 lidar_drone_tracker 는 옛 비교용 A 트래커라 쓰면 안 된다).
기본값 auto 는 _cov 가 있으면 그것을 고른다.
"""

import os

from ament_index_python.packages import get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

HERE = os.path.dirname(os.path.realpath(__file__))
DEFAULT_PARAMS = os.path.normpath(os.path.join(
    HERE, '..', '..', 'src', 'drone_localization', 'params',
    'lidar_localization_os0_indoor.yaml'))


def tracker_executable(requested):
    if requested != 'auto':
        return requested
    lib = os.path.join(get_package_prefix('drone_localization'), 'lib', 'drone_localization')
    return ('lidar_drone_tracker_cov'
            if os.path.exists(os.path.join(lib, 'lidar_drone_tracker_cov'))
            else 'lidar_drone_tracker')


def launch_setup(context):
    params = os.path.expanduser(LaunchConfiguration('params_file').perform(context))
    view = LaunchConfiguration('view').perform(context)
    rviz_file = {'calib': 'indoor_view.rviz', 'track': 'indoor_track.rviz',
                 'replay': 'indoor_replay.rviz'}.get(view)
    if rviz_file is None:
        raise RuntimeError(f"view:={view} 는 없습니다 (calib | track | replay)")
    # 비행·녹화 중에는 ROI 점군을 발행할 필요가 없다 — 트래커 직렬화 부하를 줄인다
    overrides = [{'publish_filtered_clouds': False, 'publish_world_clouds': False}] \
        if view == 'track' else []
    actions = []
    if LaunchConfiguration('start_tracker').perform(context).lower() == 'true':
        actions.append(Node(
            package='drone_localization',
            executable=tracker_executable(LaunchConfiguration('tracker').perform(context)),
            # 노드 이름은 YAML 최상위 키와 같아야 파라미터가 먹는다
            name='lidar_drone_tracker',
            parameters=[params] + overrides,
            output='screen'))
    actions.append(ExecuteProcess(
        cmd=['python3', os.path.join(HERE, 'indoor_view.py'), params],
        output='screen'))
    actions.append(ExecuteProcess(
        cmd=['rviz2', '-d', os.path.join(HERE, rviz_file)],
        condition=IfCondition(LaunchConfiguration('rviz'))))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=DEFAULT_PARAMS,
                              description='트래커 파라미터 YAML (표시 도구도 같은 파일을 읽는다)'),
        DeclareLaunchArgument('start_tracker', default_value='true'),
        DeclareLaunchArgument('tracker', default_value='auto',
                              description='트래커 실행 파일 이름, auto 면 자동 선택'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('view', default_value='calib',
                              description='calib = 캘리브레이션 확인 (점군), track = 비행·녹화용 (가벼움), replay = 경로 재생'),
        OpaqueFunction(function=launch_setup),
    ])
