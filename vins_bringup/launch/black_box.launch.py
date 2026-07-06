import os
import sys

from launch import LaunchDescription

sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from common import common_nodes


def generate_launch_description():
    return LaunchDescription(
        common_nodes(
            "black_box/black_box_config.yaml",
            pose_graph_skip_dis=0.1,
            feature_tracker_prefix="taskset -c 3",
            vins_estimator_prefix="taskset -c 1",
            pose_graph_prefix="taskset -c 2",
        )
    )
