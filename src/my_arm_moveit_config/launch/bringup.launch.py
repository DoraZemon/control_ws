from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory("my_arm_moveit_config")
    launch_dir = os.path.join(pkg_share, "launch")

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, "rsp_raw.launch.py"))
    )

    move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, "move_group.launch.py"))
    )

    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, "moveit_rviz.launch.py"))
    )

    return LaunchDescription([rsp, move_group, rviz])
