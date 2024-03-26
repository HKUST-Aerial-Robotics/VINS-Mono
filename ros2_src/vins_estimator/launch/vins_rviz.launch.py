import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='log',
        arguments=['-d', os.path.join(get_package_share_directory('config'), 'config', 'vins_rviz_config.rviz')]
    )

def generate_launch_description():
    return LaunchDescription([
        rviz_node,
    ])