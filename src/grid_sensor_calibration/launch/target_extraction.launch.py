"""Launch one ``pointcloud_target_node`` per sensor for live calibration.

Keeps the tube's dimensions in ``config/tube_target.yaml`` instead of in a
command line that has to be retyped for every sensor. The mode that uses the
radius is not the node's default, and forgetting it does not fail -- it
silently reverts to the centroid of the visible surface, which for a 0.30 m
tube is 12 cm in front of the axis. Loading a file removes that chance.

Example::

    ros2 launch grid_sensor_calibration target_extraction.launch.py

    # a different tube: give the DIAMETER, the launch halves it
    ros2 launch grid_sensor_calibration target_extraction.launch.py \\
        tube_diameter:=0.40

    # one sensor only
    ros2 launch grid_sensor_calibration target_extraction.launch.py \\
        sensors:=sensor1
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
    """Build one target-extraction node per requested sensor.

    Args:
        context: Launch context used to resolve the launch configurations.

    Returns:
        The list of node actions to execute.
    """
    params_file = LaunchConfiguration("params_file").perform(context)
    sensors = [
        name.strip()
        for name in LaunchConfiguration("sensors").perform(context).split(",")
        if name.strip()
    ]

    # Given as a diameter because that is what a tape measure reads off a tube,
    # and because halving it by hand is exactly the mistake that would double
    # the bias this whole path exists to remove. 0 keeps the file's value.
    diameter = float(LaunchConfiguration("tube_diameter").perform(context))
    overrides = {} if diameter <= 0.0 else {"target_radius": diameter / 2.0}

    return [
        Node(
            package=PACKAGE_NAME,
            executable="pointcloud_target_node",
            name=f"{sensor}_target",
            output="screen",
            emulate_tty=True,
            parameters=[params_file, overrides],
        )
        for sensor in sensors
    ]


def generate_launch_description() -> LaunchDescription:
    """Declare the launch arguments and start the target-extraction nodes.

    Returns:
        The complete :class:`LaunchDescription`.
    """
    share_dir = get_package_share_directory(PACKAGE_NAME)
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "sensors",
                default_value="sensor1,sensor2",
                description=(
                    "Comma separated sensors. Each needs a matching "
                    "'<sensor>_target' block in params_file."
                ),
            ),
            DeclareLaunchArgument(
                "params_file",
                default_value=os.path.join(share_dir, "config", "tube_target.yaml"),
                description="Target-extraction parameters.",
            ),
            DeclareLaunchArgument(
                "tube_diameter",
                default_value="0.0",
                description=(
                    "Tube DIAMETER [m], halved into target_radius. "
                    "0 keeps the value in params_file."
                ),
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
