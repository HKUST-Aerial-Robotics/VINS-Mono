"""Launch file for vins_autotune node.

Run in monitor mode (default) or tune mode.

Usage:
  ros2 launch vins_autotune autotune.launch.py
  ros2 launch vins_autotune autotune.launch.py mode:=tune config_file:=/path/to/config.yaml
  ros2 launch vins_autotune autotune.launch.py ground_truth_topic:=/lio/odom
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    share_dir = get_package_share_directory("vins_autotune")
    default_params = os.path.join(share_dir, "config", "autotune_params.yaml")

    return LaunchDescription([
        DeclareLaunchArgument("params_file", default_value=default_params),
        DeclareLaunchArgument("mode", default_value="monitor"),
        DeclareLaunchArgument("config_file", default_value=""),
        DeclareLaunchArgument("ground_truth_topic", default_value="/ground_truth/odom"),
        DeclareLaunchArgument("vins_odom_topic", default_value="/vins_estimator/odometry"),
        DeclareLaunchArgument("eval_window", default_value="15.0"),
        DeclareLaunchArgument("max_passes", default_value="3"),

        Node(
            package="vins_autotune",
            executable="vins_autotune_node",
            name="vins_autotune",
            output="screen",
            parameters=[
                {"vins_odom_topic": LaunchConfiguration("vins_odom_topic")},
                {"ground_truth_topic": LaunchConfiguration("ground_truth_topic")},
                {"mode": LaunchConfiguration("mode")},
                {"config_file": LaunchConfiguration("config_file")},
                {"eval_window": LaunchConfiguration("eval_window")},
                {"max_passes": LaunchConfiguration("max_passes")},
            ],
        ),
    ])