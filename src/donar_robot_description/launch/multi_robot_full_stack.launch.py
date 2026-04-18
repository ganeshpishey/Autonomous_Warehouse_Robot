import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_share = get_package_share_directory("donar_robot_description")

    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, "launch", "multi_robot_gazebo.launch.py")
        )
    )

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, "launch", "multi_robot_nav2.launch.py")
        ),
        launch_arguments={
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "autostart": LaunchConfiguration("autostart"),
            "slam": LaunchConfiguration("slam"),
            "map": LaunchConfiguration("map"),
            "params_file": LaunchConfiguration("params_file"),
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use simulation time across the full stack.",
            ),
            DeclareLaunchArgument(
                "autostart",
                default_value="true",
                description="Automatically transition Nav2 lifecycle nodes.",
            ),
            DeclareLaunchArgument(
                "slam",
                default_value="false",
                description="Use SLAM mode in Nav2 bringup instead of AMCL+map.",
            ),
            DeclareLaunchArgument(
                "map",
                default_value="",
                description="Map YAML file for the final post-mapping integration.",
            ),
            DeclareLaunchArgument(
                "params_file",
                default_value=os.path.join(
                    pkg_share, "config", "nav2_params_multi_robot.yaml"
                ),
                description="Nav2 parameter file for both robots.",
            ),
            sim_launch,
            nav2_launch,
        ]
    )
