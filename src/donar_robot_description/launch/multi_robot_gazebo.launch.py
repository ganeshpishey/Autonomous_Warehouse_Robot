import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_share = get_package_share_directory("donar_robot_description")
    single_robot_launch = os.path.join(pkg_share, "launch", "gazebo_sdf.launch.py")

    robot1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(single_robot_launch),
        launch_arguments={
            "launch_gazebo": "true",
            "robot_name": LaunchConfiguration("robot1_name"),
            "frame_prefix": LaunchConfiguration("robot1_frame_prefix"),
            "spawn_x": LaunchConfiguration("robot1_spawn_x"),
            "spawn_y": LaunchConfiguration("robot1_spawn_y"),
            "spawn_z": LaunchConfiguration("robot1_spawn_z"),
            "spawn_yaw": LaunchConfiguration("robot1_spawn_yaw"),
        }.items(),
    )

    robot2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(single_robot_launch),
        launch_arguments={
            "launch_gazebo": "false",
            "robot_name": LaunchConfiguration("robot2_name"),
            "frame_prefix": LaunchConfiguration("robot2_frame_prefix"),
            "spawn_x": LaunchConfiguration("robot2_spawn_x"),
            "spawn_y": LaunchConfiguration("robot2_spawn_y"),
            "spawn_z": LaunchConfiguration("robot2_spawn_z"),
            "spawn_yaw": LaunchConfiguration("robot2_spawn_yaw"),
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "robot1_name",
            default_value="robot1",
            description="Gazebo entity name for the first robot.",
        ),
        DeclareLaunchArgument(
            "robot1_frame_prefix",
            default_value="robot1_",
            description="TF and joint name prefix for the first robot.",
        ),
        DeclareLaunchArgument(
            "robot1_spawn_x",
            default_value="-10.9",
            description="Spawn X position for the first robot inside the left security fence.",
        ),
        DeclareLaunchArgument(
            "robot1_spawn_y",
            default_value="-5.45",
            description="Spawn Y position for the first robot inside the left security fence.",
        ),
        DeclareLaunchArgument(
            "robot1_spawn_z",
            default_value="0.0",
            description="Spawn Z position for the first robot.",
        ),
        DeclareLaunchArgument(
            "robot1_spawn_yaw",
            default_value="1.57",
            description="Spawn yaw for the first robot.",
        ),
        DeclareLaunchArgument(
            "robot2_name",
            default_value="robot2",
            description="Gazebo entity name for the second robot.",
        ),
        DeclareLaunchArgument(
            "robot2_frame_prefix",
            default_value="robot2_",
            description="TF and joint name prefix for the second robot.",
        ),
        DeclareLaunchArgument(
            "robot2_spawn_x",
            default_value="10.9",
            description="Spawn X position for the second robot inside the right security fence.",
        ),
        DeclareLaunchArgument(
            "robot2_spawn_y",
            default_value="-5.45",
            description="Spawn Y position for the second robot inside the right security fence.",
        ),
        DeclareLaunchArgument(
            "robot2_spawn_z",
            default_value="0.0",
            description="Spawn Z position for the second robot.",
        ),
        DeclareLaunchArgument(
            "robot2_spawn_yaw",
            default_value="1.57",
            description="Spawn yaw for the second robot.",
        ),
        robot1,
        robot2,
    ])
