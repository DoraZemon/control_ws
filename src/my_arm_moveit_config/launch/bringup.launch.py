from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory("my_arm_moveit_config")
    launch_dir = os.path.join(pkg_share, "launch")

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, "rsp_raw.launch.py"))
    )

    coupler = Node(
        package="my_arm_coupling",
        executable="joint_coupler",
        name="joint_coupler",
        output="screen",
        parameters=[{
            "in_topic": "/joint_states_raw",
            "out_topic": "/joint_states",
            "joint2_name": "joint2",
            "joint3_name": "joint3",
            "joint4_name": "joint4",
            "offset": 0.0
        }],
    )

    filter_moveit = Node(
        package="my_arm_coupling",
        executable="joint_state_filter",
        name="joint_state_filter",
        output="screen",
        parameters=[{
            "in_topic": "/joint_states",
            "out_topic": "/joint_states_moveit",
            "allow": ["joint1", "joint2", "joint3", "joint5", "joint6", "joint7"]
        }],
    )

    display_coupler = Node(
        package="my_arm_coupling",
        executable="display_trajectory_coupler",
        name="display_trajectory_coupler",
        output="screen",
        parameters=[{
            "in_topic": "/display_planned_path",
            "out_topic": "/display_planned_path_coupled",
            "joint2_name": "joint2",
            "joint3_name": "joint3",
            "joint4_name": "joint4",
            "offset": 0.0
        }],
    )

    move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, "move_group.launch.py"))
    )

    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, "moveit_rviz.launch.py"))
    )

    return LaunchDescription([rsp, coupler, filter_moveit, display_coupler, move_group, rviz])
