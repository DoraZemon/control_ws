from pathlib import Path

from launch import LaunchDescription
from launch_param_builder import load_yaml
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("control1", package_name="my_arm_moveit_config")
        .planning_pipelines(pipelines=["ompl"])
        .trajectory_execution(
            file_path="config/moveit_controllers.yaml",
            moveit_manage_controllers=False,
        )
        .to_moveit_configs()
    )

    params = [
        moveit_config.to_dict(),
        load_yaml(Path(moveit_config.package_path) / "config" / "move_group.yaml"),
        load_yaml(
            Path(moveit_config.package_path)
            / "config"
            / "planning_scene_monitor_parameters.yaml"
        ),
        load_yaml(
            Path(moveit_config.package_path) / "config" / "trajectory_execution.yaml"
        ),
    ]

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=params,
    )

    return LaunchDescription([move_group_node])
