import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable


def generate_launch_description():
    # MAIN PARAMETERS TO CHANGE HERE

    current_dir = get_package_share_directory("warehousebot_navigation")
    twist_mux_dir = get_package_share_directory("twist_mux")
    joy_teleop_dir = get_package_share_directory("teleop_twist_joy")

    ld = LaunchDescription()

    # launch twist mux
    launch_twist_mux = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(twist_mux_dir, "launch", "twist_mux_launch.py"),
        )
    )
    # launch teleop twist joy
    launch_twist_joy = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(joy_teleop_dir, "launch", "teleop-launch.py"),
        ),
        launch_arguments={
            "joy_vel": "joy_vel",
        }.items(),
    )

    # launch navigation stack
    launch_nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(current_dir, "launch", "bringup.launch.py"),
        )
    )

    ld.add_action(launch_twist_mux)
    ld.add_action(launch_twist_joy)
    ld.add_action(launch_nav2)

    return ld
