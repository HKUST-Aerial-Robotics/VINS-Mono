import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

sequence_name_arg = DeclareLaunchArgument('sequence_name', default_value='MH_01_easy')


benchmark_publisher_node = Node(
            # name = 'benchmark_publisher',
            package = 'benchmark_publisher',
            executable = 'benchmark_publisher_node',
            output = 'screen',
            remappings=[
                ('estimated_odometry', '/vins_estimator/odometry'),
            ],
            parameters=[
                {'data_name': os.path.join(get_package_share_directory('benchmark_publisher'), 
                                                    'config', sequence_name_arg, 'data.csv')},
            ]
)

def generate_launch_description():
    return LaunchDescription([
        benchmark_publisher_node,
    ])