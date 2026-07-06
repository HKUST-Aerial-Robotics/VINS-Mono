import os
import sys

from launch import LaunchDescription

sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from common import common_nodes


def generate_launch_description():
    return LaunchDescription(common_nodes("fisheye_bag/fisheye_bag_config.yaml"))
