from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("control1", package_name="my_arm_moveit_config")
        .to_moveit_configs()
    )

    # move_group 需要的参数集合（URDF/SRDF/kinematics/planning pipelines/limits等）
    params = []
    params.append(moveit_config.robot_description)
    params.append(moveit_config.robot_description_semantic)
    params.append(moveit_config.robot_description_kinematics)
    params.append(moveit_config.planning_pipelines)
    params.append(moveit_config.joint_limits)

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=params,
        remappings=[
            # 关键：让 move_group 用独立关节状态作为 current state
            ("/joint_states", "/joint_states_moveit"),
        ],
    )

    return LaunchDescription([move_group_node])
