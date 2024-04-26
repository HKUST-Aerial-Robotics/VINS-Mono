
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

config_path_arg = DeclareLaunchArgument('config_path', default_value = os.path.join(get_package_share_directory('config'), 
                                            'config', '3dm', '3dm_config.yaml'))
vins_path_arg = DeclareLaunchArgument('vins_path', default_value = os.path.join(get_package_share_directory('config'), '..'))

feature_tracker_node = Node(
            package='feature_tracker',
            executable='feature_tracker_node',
            output='log',
            parameters=[
                {'config_file': config_path_arg},
                {'vins_folder': vins_path_arg}
            ]
)

vins_estimator_node = Node(
            package='vins_estimator',
            executable='vins_estimator_node',
            output='screen',
            parameters=[
                {'config_file': config_path_arg},
                {'vins_folder': vins_path_arg}
            ]
)

def generate_launch_description():
    return LaunchDescription([
        feature_tracker_node,
        vins_estimator_node
    ])