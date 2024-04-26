

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

config_path_arg = DeclareLaunchArgument('config_path', default_value = os.path.join(get_package_share_directory('config'), 
                                            'config', 'realsense', 'realsense_fisheye_config.yaml'))
vins_path_arg = DeclareLaunchArgument('vins_path', default_value = os.path.join(get_package_share_directory('config'), '..'))

feature_tracker_node = Node(
        package='feature_tracker',
        executable='feature_tracker_node',
        output='log',
        parameters=[{'config_file': config_path_arg},
                    {'vins_folder': vins_path_arg}]
    )

vins_estimator_node = Node(
        package='vins_estimator',
        executable='vins_estimator_node',
        output='screen',
        parameters=[{'config_file': config_path_arg},
                    {'vins_folder': vins_path_arg}]
    )

pose_graph_node = Node(
        package='pose_graph',
        executable='pose_graph_node',
        output='screen',
        parameters=[{'config_file': config_path_arg},
                    {'visualization_shift_x': 0},
                    {'visualization_shift_y': 0},
                    {'skip_cnt': 0},
                    {'skip_dis': 0}]
    )

def generate_launch_description():
    return LaunchDescription([
        feature_tracker_node,
        vins_estimator_node,
        pose_graph_node
    ])