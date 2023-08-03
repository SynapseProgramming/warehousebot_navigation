import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition


def generate_launch_description():
    # MAIN PARAMETERS TO CHANGE HERE

    current_dir = get_package_share_directory("warehousebot_navigation")
    bot_desc_dir = get_package_share_directory("warehousebot_description")
    twist_mux_dir = get_package_share_directory("twist_mux")
    joy_teleop_dir = get_package_share_directory("teleop_twist_joy")
    r2_inter_dir = get_package_share_directory("labview_r2interface")
    # realsense_dir = get_package_share_directory("realsense2_camera")
    lidar_odom_dir = get_package_share_directory("rf2o_laser_odometry")

    ld = LaunchDescription()

    slam = LaunchConfiguration("slam")
    rviz_nav_config_dir = LaunchConfiguration("rviz_nav_config_dir")
    rviz_slam_config_dir = LaunchConfiguration("rviz_slam_config_dir")
    teleop_config = LaunchConfiguration("teleop_config")

    declare_rviz_nav_config = DeclareLaunchArgument(
        "rviz_nav_config_dir",
        default_value=os.path.join(current_dir, "rviz", "live_nav.rviz"),
        description="default path to rviz nav config file",
    )

    declare_rviz_slam_config = DeclareLaunchArgument(
        "rviz_slam_config_dir",
        default_value=os.path.join(current_dir, "rviz", "slam_config.rviz"),
        description="default path to rviz slam config file",
    )

    declare_teleop_config = DeclareLaunchArgument(
        "teleop_config",
        default_value=os.path.join(current_dir, "params", "teleop_params.yaml"),
        description="default path to teleop config file",
    )

    declare_slam_cmd = DeclareLaunchArgument(
        "slam", default_value="False", description="Whether run SLAM"
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
            "config_filepath": teleop_config,
        }.items(),
    )

    # launch rviz2 for nav
    launch_rviz2_nav = Node(
        condition=IfCondition(PythonExpression(["not ", slam])),
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_nav_config_dir],
        output="screen",
    )

    # launch rviz2 for slam
    launch_rviz2_slam = Node(
        condition=IfCondition(slam),
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_slam_config_dir],
        output="screen",
    )

    # launch navigation stack
    launch_nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(current_dir, "launch", "bringup.launch.py"),
        ),
        condition=IfCondition(PythonExpression(["not ", slam])),
    )

    # launch robot state publisher
    launch_bot_desc = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bot_desc_dir, "launch", "bot_description.py"),
        ),
    )

    # launch hardware drivers
    launch_r2inter = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(r2_inter_dir, "launch", "lidar_unfiltered_odom.launch.py"),
        ),
    )

    # launch intel realsense depth camera drivers
    # launch_realsense = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(realsense_dir, "launch", "rs_launch.py"),
    #     ),
    # )

    # launch imu filters
    launch_imu_filter = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(current_dir, "launch", "imu_filter.launch.py"),
        ),
    )

    # launch ekf node
    launch_ekf = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(current_dir, "launch", "ekf.launch.py"),
        ),
    )

    # launch  lidar odometry
    launch_lidar_odom = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(lidar_odom_dir, "launch", "rf2o_laser_odometry.launch.py"),
        ),
    )

    ld.add_action(declare_rviz_nav_config)
    ld.add_action(declare_rviz_slam_config)
    ld.add_action(declare_teleop_config)
    ld.add_action(declare_slam_cmd)
    ld.add_action(launch_twist_mux)
    ld.add_action(launch_twist_joy)
    ld.add_action(launch_rviz2_nav)
    ld.add_action(launch_rviz2_slam)
    ld.add_action(launch_nav2)
    ld.add_action(launch_bot_desc)
    ld.add_action(launch_r2inter)
    ld.add_action(launch_lidar_odom)
    # ld.add_action(launch_realsense)
    ld.add_action(launch_imu_filter)
    ld.add_action(launch_ekf)

    return ld
