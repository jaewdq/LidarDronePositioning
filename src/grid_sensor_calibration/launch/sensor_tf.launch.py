"""Publish the calibrated static TFs and (optionally) the runtime helpers.

Starts:

* ``sensor_tf_publisher``   -- static ``grid -> sensorN`` transforms (always),
* ``point_transform_node``  -- sensor detections republished in ``grid``
  (``point_transform:=true``, default),
* ``calibration_viz_node``  -- RViz markers (``visualization:=false`` by default).

Example::

    ros2 launch grid_sensor_calibration sensor_tf.launch.py \\
        calibration_file:=$HOME/grid_calibration/sensors_calibration.yaml \\
        visualization:=true
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
    """Build the runtime nodes according to the launch arguments.

    Args:
        context: Launch context used to resolve the launch configurations.

    Returns:
        The list of node actions to execute.
    """
    share_dir = get_package_share_directory(PACKAGE_NAME)
    params_file = LaunchConfiguration("params_file").perform(context)
    calibration_file = LaunchConfiguration("calibration_file").perform(context)
    config_dir = LaunchConfiguration("config_dir").perform(context)
    transform_mode = LaunchConfiguration("transform_mode").perform(context)
    use_sim_time = LaunchConfiguration("use_sim_time").perform(context).lower() == "true"
    want_transform = LaunchConfiguration("point_transform").perform(context).lower() == "true"
    want_viz = LaunchConfiguration("visualization").perform(context).lower() == "true"
    sensors = [
        name.strip()
        for name in LaunchConfiguration("sensors").perform(context).split(",")
        if name.strip()
    ]

    common = {"use_sim_time": use_sim_time, "calibration_file": calibration_file}

    nodes: List[Node] = [
        Node(
            package=PACKAGE_NAME,
            executable="sensor_tf_publisher",
            name="sensor_tf_publisher",
            output="screen",
            emulate_tty=True,
            parameters=[params_file, dict(common, sensors=sensors or [""])],
        )
    ]

    if want_transform:
        nodes.append(
            Node(
                package=PACKAGE_NAME,
                executable="point_transform_node",
                name="point_transform_node",
                output="screen",
                emulate_tty=True,
                parameters=[
                    params_file,
                    dict(common, transform_mode=transform_mode, **(
                        {"sensors": sensors} if sensors else {}
                    )),
                ],
            )
        )

    if want_viz:
        input_files = [os.path.join(config_dir, f"{sensor}.yaml") for sensor in sensors]
        nodes.append(
            Node(
                package=PACKAGE_NAME,
                executable="calibration_viz_node",
                name="calibration_viz_node",
                output="screen",
                emulate_tty=True,
                parameters=[params_file, dict(common, input_files=input_files or [""])],
            )
        )
    return nodes


def generate_launch_description() -> LaunchDescription:
    """Declare the launch arguments and start the runtime nodes.

    Returns:
        The complete :class:`LaunchDescription`.
    """
    share_dir = get_package_share_directory(PACKAGE_NAME)
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "sensors",
                default_value="sensor1,sensor2",
                description="Comma separated sensor list; empty publishes every "
                "sensor found in the calibration file.",
            ),
            DeclareLaunchArgument(
                "calibration_file",
                default_value=os.path.join(share_dir, "config", "sensors_calibration.yaml"),
                description="Calibration result file to read the transforms from.",
            ),
            DeclareLaunchArgument(
                "params_file",
                default_value=os.path.join(share_dir, "config", "runtime_params.yaml"),
                description="ROS parameter file for the runtime nodes.",
            ),
            DeclareLaunchArgument(
                "config_dir",
                default_value=os.path.join(share_dir, "config"),
                description="Directory holding the per-sensor datasets (visualisation).",
            ),
            DeclareLaunchArgument(
                "transform_mode",
                default_value="tf2",
                description="'tf2' uses the TF tree, 'direct' applies Pg = R Ps + t.",
            ),
            DeclareLaunchArgument(
                "point_transform",
                default_value="true",
                description="Start point_transform_node.",
            ),
            DeclareLaunchArgument(
                "visualization",
                default_value="false",
                description="Start calibration_viz_node (RViz markers).",
            ),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            OpaqueFunction(function=_launch_setup),
        ]
    )
