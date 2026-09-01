#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node


def generate_launch_description():

    # ============================================================
    # Package paths
    # ============================================================

    drone_localization_share = get_package_share_directory(
        'drone_localization'
    )

    dual_ouster_share = get_package_share_directory(
        'dual_ouster_driver'
    )

    # ============================================================
    # Launch arguments
    # ============================================================

    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value='lidar_localization_os1.yaml',
        description=(
            'Parameter file under drone_localization/params/.'
        ),
    )

    learn_background_arg = DeclareLaunchArgument(
        'learn_background',
        default_value='false',
        description=(
            'true: collect and save static background. '
            'false: normal drone tracking.'
        ),
    )

    # Default off. The competition field will not need it -- there the ROI's
    # z band is the hover altitude only, so the ground and most structure are
    # already outside it and a learned background has almost nothing to remove.
    # Practising indoors WITH it would hide whatever it was covering up.
    #
    # It also carries a failure mode worth avoiding by default: the saved model
    # embeds a signature over the ROI and both sensor poses, so touching either
    # silently retires it. The node does log an error and continue, but the run
    # goes ahead with the walls back inside the ROI and nothing removing them.
    #
    # Turn it on (use_background:=true, after learn_background:=true) only when
    # the ROI genuinely cannot avoid the walls.
    use_background_arg = DeclareLaunchArgument(
        'use_background',
        default_value='false',
        description=(
            'true: use saved background during normal tracking.'
        ),
    )

    # Which of the two fusion strategies the tracker runs. Exposed here
    # because the GUI starts the tracker through this launch file and has no
    # other way to reach a node parameter.
    #
    #   position -- cluster each sensor, fuse the two POSITIONS (default)
    #   cloud    -- concatenate both clouds, cluster once
    #
    # See _detect_from_merged_cloud in lidar_drone_tracker.py for the tradeoff.
    fusion_mode_arg = DeclareLaunchArgument(
        'fusion_mode',
        default_value='position',
        description="Detection fusion strategy: 'position' or 'cloud'.",
    )

    tracker_executable_arg = DeclareLaunchArgument(
        'tracker_executable',
        default_value='lidar_drone_tracker',
        description=(
            'Tracker executable to run: lidar_drone_tracker or '
            'lidar_drone_tracker_ekf.'
        ),
    )

    # Whether this launch also brings up the Ouster drivers.
    #
    # true (default): one command starts everything. Convenient, but the GUI
    # runs this launch as a subprocess and kills the whole process group on
    # FINISH RUN, so the drivers are torn down and reconnected on every run.
    # Starting a driver by hand alongside the GUI then gives TWO drivers per
    # sensor, which is not an error either one reports -- they simply compete
    # for the sensor's UDP stream and both time out in poll_client().
    #
    # false: the drivers are expected to be running already, and this launch
    # brings up only the tracker and its static TF. Better for repeated runs --
    # the sensor stream stays up across START/FINISH, there is no reconnect
    # delay at the start of a run, and the same drivers serve calibration too.
    #
    #   ros2 launch drone_localization lidar_localization.launch.py \
    #       start_driver:=false
    #
    #   ros2 run competition_gui competition_gui --ros-args \
    #       -p tracker_extra_args:="['start_driver:=false']"
    start_driver_arg = DeclareLaunchArgument(
        'start_driver',
        default_value='true',
        description=(
            'true: also launch the Ouster drivers. '
            'false: assume they are already running.'
        ),
    )

    # Optional override for the saved .npz file. Resolved against the running
    # user's home -- the previous default was hardcoded to /home/drone, which
    # does not exist here, so learn_background:=true ran the full 200-frame
    # collection and then died with PermissionError trying to create it.
    background_file_arg = DeclareLaunchArgument(
        'background_file',
        default_value=os.path.join(
            os.path.expanduser('~'),
            'competition_results', 'calibration',
            'dual_ouster_background.npz',
        ),
        description='Path of the saved static-background model.',
    )

    # ============================================================
    # Tracker YAML
    # ============================================================

    params_path = PathJoinSubstitution([
        drone_localization_share,
        'params',
        LaunchConfiguration('params_file'),
    ])

    # ============================================================
    # Physical Ouster OS1-128 drivers
    #
    # Expected outputs:
    #
    # /lidar1/points
    # /lidar2/points
    # ============================================================

    dual_ouster_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                dual_ouster_share,
                'launch',
                'dual_os1.launch.py',
            ])
        ),
        condition=IfCondition(LaunchConfiguration('start_driver')),
    )

    # ============================================================
    # Drone localization / background-learning node
    # ============================================================

    tracker = Node(
        package='drone_localization',
        executable=LaunchConfiguration('tracker_executable'),
        name='lidar_drone_tracker',
        output='screen',

        # First load all normal settings from YAML.
        # Then override the three parameters controlled from
        # the launch command.
        parameters=[
            params_path,
            {
                'learn_background': LaunchConfiguration(
                    'learn_background'
                ),

                'use_background': LaunchConfiguration(
                    'use_background'
                ),

                'background_file': LaunchConfiguration(
                    'background_file'
                ),

                'fusion_mode': LaunchConfiguration(
                    'fusion_mode'
                ),
            },
        ],

        respawn=False,
    )

    # ============================================================
    # map -> world identity transform
    # ============================================================
    #
    # Nothing in this pipeline publishes TF: the tracker applies the
    # sensor->world extrinsics itself, in code, and simply stamps its output
    # with frame_id 'world'. So without this node neither 'map' nor 'world'
    # ever appears in the TF tree, RViz's Fixed Frame dropdown lists only the
    # driver's lidarN frames, and every cloud the tracker publishes piles up
    # in the message filter until it is dropped -- which surfaces as
    # "queue is full", not as a missing-frame error, so it reads like a
    # performance problem when it is really a TF one.
    #
    # Args are positional in tf2_ros on Humble: x y z yaw pitch roll parent child.
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_world_static_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'world'],
    )

    # ============================================================
    # Launch description
    # ============================================================

    return LaunchDescription([

        params_file_arg,

        learn_background_arg,
        use_background_arg,
        background_file_arg,
        fusion_mode_arg,
        tracker_executable_arg,
        start_driver_arg,

        dual_ouster_driver,

        tracker,

        static_tf,
    ])
