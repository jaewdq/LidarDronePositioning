"""Launch one calibration node per sensor.

Each node is named ``<sensor>_calibration`` so that ``config/calibration_params.yaml``
can hold a separate parameter block per sensor. The sensor list is a launch
argument, so a third sensor only needs a dataset file plus a parameter block --
no code change.

Example::

    ros2 launch grid_sensor_calibration calibration.launch.py \\
        sensors:=sensor1,sensor2 \\
        calibration_file:=$HOME/grid_calibration/sensors_calibration.yaml
"""

from __future__ import annotations

import os
from typing import List

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PACKAGE_NAME = "grid_sensor_calibration"


def _launch_setup(context: LaunchContext, *args, **kwargs) -> List[Node]:
    """Build one calibration node per requested sensor.

    Args:
        context: Launch context used to resolve the launch configurations.

    Returns:
        The list of node actions to execute.
    """
    share_dir = get_package_share_directory(PACKAGE_NAME)
    config_dir = LaunchConfiguration("config_dir").perform(context) or os.path.join(
        share_dir, "config"
    )
    params_file = LaunchConfiguration("params_file").perform(context)
    calibration_file = LaunchConfiguration("calibration_file").perform(context)
    use_sim_time = LaunchConfiguration("use_sim_time").perform(context).lower() == "true"
    auto_calibrate = LaunchConfiguration("auto_calibrate").perform(context).lower() == "true"
    shutdown_after = (
        LaunchConfiguration("shutdown_after_calibration").perform(context).lower() == "true"
    )
    sensors = [
        name.strip()
        for name in LaunchConfiguration("sensors").perform(context).split(",")
        if name.strip()
    ]

    nodes: List[Node] = []
    for sensor in sensors:
        nodes.append(
            Node(
                package=PACKAGE_NAME,
                executable="sensor_calibration_node",
                name=f"{sensor}_calibration",
                output="screen",
                emulate_tty=True,
                parameters=[
                    params_file,
                    {
                        "sensor_name": sensor,
                        "input_file": os.path.join(config_dir, f"{sensor}.yaml"),
                        "calibration_file": calibration_file,
                        "auto_calibrate": auto_calibrate,
                        "shutdown_after_calibration": shutdown_after,
                        "use_sim_time": use_sim_time,
                    },
                ],
            )
        )
    return nodes


def generate_launch_description() -> LaunchDescription:
    """Declare the launch arguments and start the calibration nodes.

    Returns:
        The complete :class:`LaunchDescription`.
    """
    share_dir = get_package_share_directory(PACKAGE_NAME)
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "sensors",
                default_value="sensor1,sensor2",
                description="Comma separated list of sensors to calibrate.",
            ),
            DeclareLaunchArgument(
                "config_dir",
                default_value=os.path.join(share_dir, "config"),
                description="Directory holding the per-sensor <sensor>.yaml datasets.",
            ),
            DeclareLaunchArgument(
                "params_file",
                default_value=os.path.join(share_dir, "config", "calibration_params.yaml"),
                description="ROS parameter file for the calibration nodes.",
            ),
            DeclareLaunchArgument(
                "calibration_file",
                default_value=os.path.join(share_dir, "config", "sensors_calibration.yaml"),
                description="Combined YAML file the estimated transforms are written to.",
            ),
            DeclareLaunchArgument(
                "auto_calibrate",
                default_value="true",
                description="Calibrate once at start-up from the dataset files.",
            ),
            DeclareLaunchArgument(
                "shutdown_after_calibration",
                default_value="false",
                description="Exit after the start-up calibration (batch mode).",
            ),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            OpaqueFunction(function=_launch_setup),
        ]
    )
