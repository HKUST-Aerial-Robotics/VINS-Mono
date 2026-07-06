from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node


def _share_dir() -> Path:
    return Path(get_package_share_directory("vins_bringup"))


def config_file_path(filename: str) -> str:
    return str(_share_dir() / "config" / filename)


def vins_folder_path() -> str:
    return str(_share_dir()) + "/"


def common_nodes(
    config_filename: str,
    include_pose_graph: bool = True,
    pose_graph_skip_dis: float = 0.0,
    feature_tracker_prefix: str | None = None,
    vins_estimator_prefix: str | None = None,
    pose_graph_prefix: str | None = None,
):
    config_path = config_file_path(config_filename)
    vins_path = vins_folder_path()
    feature_tracker_kwargs = {
        "package": "feature_tracker",
        "executable": "feature_tracker",
        "name": "feature_tracker",
        "output": "screen",
        "parameters": [
            {"config_file": config_path},
            {"vins_folder": vins_path},
        ],
    }
    if feature_tracker_prefix:
        feature_tracker_kwargs["prefix"] = feature_tracker_prefix

    vins_estimator_kwargs = {
        "package": "vins_estimator",
        "executable": "vins_estimator",
        "name": "vins_estimator",
        "output": "screen",
        "parameters": [
            {"config_file": config_path},
            {"vins_folder": vins_path},
        ],
    }
    if vins_estimator_prefix:
        vins_estimator_kwargs["prefix"] = vins_estimator_prefix

    nodes = [
        Node(**feature_tracker_kwargs),
        Node(**vins_estimator_kwargs),
    ]

    if include_pose_graph:
        pose_graph_kwargs = {
            "package": "pose_graph",
            "executable": "pose_graph",
            "name": "pose_graph",
            "output": "screen",
            "parameters": [
                {"config_file": config_path},
                {"visualization_shift_x": 0},
                {"visualization_shift_y": 0},
                {"skip_cnt": 0},
                {"skip_dis": pose_graph_skip_dis},
            ],
        }
        if pose_graph_prefix:
            pose_graph_kwargs["prefix"] = pose_graph_prefix
        nodes.append(Node(**pose_graph_kwargs))

    return nodes
