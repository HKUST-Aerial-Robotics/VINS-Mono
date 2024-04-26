import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


vins_estimator_path = get_package_share_directory('vins_estimator')

vins_estimator_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(vins_estimator_path,'launch','3dm.launch.py')
        )
)

ar_demo_node = Node(
            package='ar_demo',
            executable='ar_demo_node',
            output='screen',
            remappings=[
                ('image_raw', '/mv_25001498/image_raw'),
                ('camera_pose', '/vins_estimator/camera_pose'),
                ('pointcloud', '/vins_estimator/point_cloud'),
            ],
            parameters=[
                {'calib_file': os.path.join(get_package_share_directory('config'), 
                                                    'config', '3dm', '3dm_config.yaml')},
                {'use_undistored_img': False}
            ]
)

def generate_launch_description():
    return LaunchDescription([
        ar_demo_node,
        vins_estimator_node,
    ])