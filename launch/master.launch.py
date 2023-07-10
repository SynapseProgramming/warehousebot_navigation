import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition


def generate_launch_description():
    # MAIN PARAMETERS TO CHANGE HERE

    current_dir = get_package_share_directory("warehousebot_navigation")
    twist_mux_dir = get_package_share_directory("twist_mux")
    joy_teleop_dir = get_package_share_directory("teleop_twist_joy")
    rviz_config_dir = LaunchConfiguration("rviz_config_dir")

    ld = LaunchDescription()

    run_rviz2 = LaunchConfiguration("run_rviz2")

    declare_rviz_launch = DeclareLaunchArgument(
        "run_rviz2", default_value="false", description="run rviz"
    )

    declare_rviz_config = DeclareLaunchArgument(
        "rviz_config_dir",
        default_value=os.path.join(current_dir, "rviz", "live_nav.rviz"),
        description="default path to rviz config file",
    )

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

    # launch rviz2
    launch_rviz2 = Node(
        condition=IfCondition(run_rviz2),
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config_dir],
        output="screen",
    )

    # launch navigation stack
    launch_nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(current_dir, "launch", "bringup.launch.py"),
        )
    )

    ld.add_action(launch_twist_mux)
    ld.add_action(declare_rviz_launch)
    ld.add_action(declare_rviz_config)
    ld.add_action(launch_twist_joy)
    ld.add_action(launch_rviz2)
    ld.add_action(launch_nav2)

    return ld
