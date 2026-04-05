from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("control1", package_name="my_arm_moveit_config")
        .to_moveit_configs()
    )

    # robot_description（URDF）由 MoveItConfigsBuilder 提供
    robot_description = moveit_config.robot_description

    # 1) GUI 发布到 /joint_states_raw（通过 remap）
    jsp_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        output="screen",
        remappings=[
            ("/joint_states", "/joint_states_raw"),
        ],
    )

    # 2) robot_state_publisher 订阅 /joint_states（默认就是这个）
    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
        # 默认订阅 /joint_states，不需要 remap
    )

    return LaunchDescription([jsp_gui, rsp])
