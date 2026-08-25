#!/usr/bin/env python3

"""라이다 2대를 같은 좌표계로 띄워서 겹치는지 눈으로 확인한다.

각 라이다마다 roi_filter + cluster_median 을 하나씩 돌린다. 둘 다 보정값을
읽어 map 프레임으로 내보내므로, 제대로 보정됐다면 같은 물체가 같은 자리에
겹쳐 보인다. 어긋나 보이면 보정이 틀린 것이다.

  라이다1  빨강    라이다2  파랑

원본 점군도 map 에서 보고 싶으면 static_transform_publisher 를 같이 띄운다
(tf:=true). 안 띄우면 RViz 가 Raw_lidarN 표시에 오류를 내지만 나머지는
정상 동작한다.

사용:
    ros2 launch dual_ouster_driver dual_view.launch.py
    ros2 launch dual_ouster_driver dual_view.launch.py z_min:=0.05   # 바닥 물체까지
    ros2 launch dual_ouster_driver dual_view.launch.py rviz:=false

드라이버(dual_os1.launch.py)는 따로 켜져 있어야 한다.
"""

import math
import os
import re

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# 라이다별 색 (r, g, b). ROI 점 색과 맞춰 둔다.
COLORS = {1: [1.0, 0.25, 0.25], 2: [0.25, 0.6, 1.0]}


def read_calib(path):
    """YAML 을 정규식으로 훑는다. TF 만들려고 네 값만 필요하다."""
    t = open(path).read()
    out = {}
    for k in ('lidar_z', 'yaw_deg', 'origin_x', 'origin_y'):
        m = re.search(rf'{k}:\s*([-\d.]+)', t)
        if not m:
            return None
        out[k] = float(m.group(1))
    return out


def setup(context, *args, **kwargs):
    share = get_package_share_directory('dual_ouster_driver')
    ids = [int(v) for v in
           LaunchConfiguration('lidar_ids').perform(context).split(',') if v.strip()]
    z_min = LaunchConfiguration('z_min').perform(context)
    want_tf = LaunchConfiguration('tf').perform(context).lower() not in ('false', '0')
    want_rviz = LaunchConfiguration('rviz').perform(context).lower() not in ('false', '0')

    # 모든 라이다의 map 좌표를 먼저 모은다. 마주 보게 놓으면 서로를 물체로
    # 잡으므로, 각 라이다 자리를 통째로 빼야 한다.
    calib = {}
    for lid in ids:
        cfg = os.path.join(share, 'config', f'calibration_lidar{lid}.yaml')
        if os.path.exists(cfg):
            c = read_calib(cfg)
            if c:
                calib[lid] = c
    excl = []
    for c in calib.values():
        # roi_filter 기준으로 라이다는 원점 반대편에 있다
        excl += [-c['origin_x'], -c['origin_y']]

    acts = []
    for lid in ids:
        cfg = os.path.join(share, 'config', f'calibration_lidar{lid}.yaml')
        if not os.path.exists(cfg):
            print(f'[dual_view] {cfg} 가 없습니다. 라이다{lid} 는 건너뜁니다.')
            continue

        over = {'input_topic': f'/lidar{lid}/points',
                'output_topic': f'/lidar{lid}/roi_points',
                'exclude_xy': excl,
                'exclude_radius': float(
                    LaunchConfiguration('exclude_radius').perform(context))}
        if float(z_min) > 0:
            over['roi_z_min'] = float(z_min)
        acts.append(Node(
            package='dual_ouster_driver', executable='roi_filter',
            name=f'roi_filter_{lid}', output='screen',
            parameters=[cfg, over]))

        acts.append(Node(
            package='dual_ouster_driver', executable='cluster_median',
            name=f'cluster_median_{lid}', output='screen',
            parameters=[{
                'input_topic': f'/lidar{lid}/roi_points',
                'marker_color': COLORS.get(lid, [1.0, 1.0, 1.0]),
                # 작은 표적도 잡히게. 복셀은 끈다.
                'voxel_size': 0.0,
                'min_points': 4,
                'min_samples': 3,
                'max_span': 1.5,
            }]))

        if want_tf:
            c = calib.get(lid)
            if c:
                # roi_filter:  p_map = R(yaw)·p_lidar - origin,  z += lidar_z
                # 따라서 map -> lidarN 은 회전 yaw, 이동 (-ox, -oy, lidar_z)
                acts.append(Node(
                    package='tf2_ros', executable='static_transform_publisher',
                    name=f'tf_lidar{lid}', output='log',
                    arguments=[
                        '--x', f"{-c['origin_x']:.4f}",
                        '--y', f"{-c['origin_y']:.4f}",
                        '--z', f"{c['lidar_z']:.4f}",
                        '--yaw', f"{math.radians(c['yaw_deg']):.6f}",
                        '--pitch', '0', '--roll', '0',
                        '--frame-id', 'map',
                        '--child-frame-id', f'lidar{lid}',
                    ]))

    if want_rviz:
        rv = os.path.join(share, 'rviz', 'dual_view.rviz')
        if os.path.exists(rv):
            acts.append(Node(package='rviz2', executable='rviz2', name='rviz',
                             arguments=['-d', rv], output='log'))
        else:
            print(f'[dual_view] rviz 설정을 못 찾음: {rv}')
            print('[dual_view] colcon build 를 다시 해보세요.')

    return acts


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('lidar_ids', default_value='1,2'),
        DeclareLaunchArgument(
            'z_min', default_value='0.0',
            description='ROI 바닥을 낮출 때. 0 이면 YAML 값 그대로'),
        DeclareLaunchArgument(
            'exclude_radius', default_value='0.0',
            description='각 라이다 자리 주변 이 반경을 ROI 에서 뺀다 (m). '
                        '0 이면 끔. 마주 본 라이다가 서로를 잡을 때 쓴다'),
        DeclareLaunchArgument('tf', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        OpaqueFunction(function=setup),
    ])
