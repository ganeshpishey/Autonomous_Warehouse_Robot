import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("donar_robot_description")
    slam_share = get_package_share_directory("slam_toolbox")
    slam_params = os.path.join(pkg_share, "config", "slam_params_3d_lidar.yaml")

    spawn_x = LaunchConfiguration("spawn_x")
    spawn_y = LaunchConfiguration("spawn_y")
    spawn_z = LaunchConfiguration("spawn_z")
    spawn_yaw = LaunchConfiguration("spawn_yaw")
    use_rviz = LaunchConfiguration("use_rviz")

    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, "launch", "gazebo_sdf.launch.py")
        ),
        launch_arguments={
            "spawn_x": spawn_x,
            "spawn_y": spawn_y,
            "spawn_z": spawn_z,
            "spawn_yaw": spawn_yaw,
        }.items(),
    )

    lidar_sensor_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="lidar_sensor_tf",
        parameters=[{"use_sim_time": True}],
        arguments=[
            "0.24",
            "0.0",
            "0.43",
            "0.0",
            "0.0",
            "0.0",
            "base_link",
            "donar_robot/base_link/front_lidar",
        ],
        output="screen",
    )

    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_share, "launch", "online_async_launch.py")
        ),
        launch_arguments={
            "use_sim_time": "true",
            "slam_params_file": slam_params,
        }.items(),
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", os.path.join(pkg_share, "launch", "slam.rviz")],
        condition=IfCondition(use_rviz),
        output="screen",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("spawn_x", default_value="0.0"),
            DeclareLaunchArgument("spawn_y", default_value="-18.0"),
            DeclareLaunchArgument("spawn_z", default_value="0.15"),
            DeclareLaunchArgument("spawn_yaw", default_value="1.57"),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            sim_launch,
            TimerAction(
                period=8.0,
                actions=[
                    lidar_sensor_tf,
                    slam_launch,
                    rviz,
                ],
            ),
        ]
    )