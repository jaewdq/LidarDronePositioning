#!/usr/bin/env python3

"""비행 기록 한 번에 띄우기 — roi_filter + cluster_median + flight_recorder + rosbag.

드라이버(dual_os1.launch.py)는 따로 켜져 있어야 한다. 재시작에 15초 걸려서
비행 사이에 껐다 켜지 않는 게 낫다.

rosbag 을 왜 같이 남기나:
  CSV 만 남기면 그때의 파라미터로 계산된 결과가 전부다. 비행 후에
  "왜 30초에 끊겼지", "min_points 를 낮췄으면 잡혔을까" 를 확인할 방법이 없다.
  원본을 남겨두면 날린 건 한 번이고 분석은 몇 번이든 다시 한다.

  /lidarN/points       약 20 MB/s   ROI·바닥컷·DBSCAN 전부 재조정 가능
  /lidarN/roi_points   약 16 KB/s   DBSCAN 파라미터만 재조정 가능

사용:
    ros2 launch dual_ouster_driver record_flight.launch.py
    ros2 launch dual_ouster_driver record_flight.launch.py label:=test1 bag:=none
"""

import os
import time

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context, *args, **kwargs):
    lid = LaunchConfiguration('lidar_id').perform(context)
    label = LaunchConfiguration('label').perform(context)
    out_dir = os.path.expanduser(
        LaunchConfiguration('out_dir').perform(context))
    bag = LaunchConfiguration('bag').perform(context)
    cfg = LaunchConfiguration('config').perform(context)
    if not cfg:
        cfg = os.path.join(
            get_package_share_directory('dual_ouster_driver'),
            'config', f'calibration_lidar{lid}.yaml')

    acts = [
        Node(package='dual_ouster_driver', executable='roi_filter',
             name='roi_filter', output='screen',
             parameters=[cfg, {
                 # YAML 에는 보정값만 들어 있다. 토픽은 lidar_id 를 따른다.
                 'input_topic': f'/lidar{lid}/points',
                 'output_topic': f'/lidar{lid}/roi_points',
             }]),
        Node(package='dual_ouster_driver', executable='cluster_median',
             name='cluster_median', output='screen',
             parameters=[{
                 'input_topic': f'/lidar{lid}/roi_points',
                 # 텔로처럼 작은 표적은 복셀을 끄고 문턱을 낮춰야 잡힌다.
                 'voxel_size': 0.0,
                 'min_points': 4,
                 'min_samples': 3,
                 'max_span': 1.5,
             }]),
        Node(package='dual_ouster_driver', executable='flight_recorder',
             name='flight_recorder', output='screen',
             parameters=[{'lidar_id': int(lid), 'out_dir': out_dir,
                          'label': label}]),
    ]

    # rosbag: raw(원본) / roi(가벼움) / both / none
    topics = {'raw': [f'/lidar{lid}/points'],
              'roi': [f'/lidar{lid}/roi_points'],
              'both': [f'/lidar{lid}/points', f'/lidar{lid}/roi_points'],
              'none': []}.get(bag, [f'/lidar{lid}/points',
                                    f'/lidar{lid}/roi_points'])
    if topics:
        stamp = time.strftime('%Y%m%d_%H%M%S')
        name = f'bag_{stamp}' + (f'_{label}' if label else '')
        acts.append(ExecuteProcess(
            cmd=['ros2', 'bag', 'record', '-o', os.path.join(out_dir, name),
                 *topics, f'/lidar{lid}/cluster_poses'],
            output='screen'))
    return acts


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('lidar_id', default_value='1'),
        DeclareLaunchArgument('label', default_value=''),
        DeclareLaunchArgument('out_dir', default_value='~/flights'),
        DeclareLaunchArgument(
            'bag', default_value='both',
            description='raw | roi | both | none'),
        DeclareLaunchArgument(
            'config', default_value='',
            description='보정 YAML 경로. 비우면 calibration_lidarN.yaml'),
        OpaqueFunction(function=setup),
    ])
