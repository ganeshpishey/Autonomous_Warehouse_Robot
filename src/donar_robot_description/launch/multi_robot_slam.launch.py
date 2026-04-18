import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("donar_robot_description")

    use_rviz = LaunchConfiguration("use_rviz")
    launch_sim = LaunchConfiguration("launch_sim")
    slam_params_file = LaunchConfiguration("slam_params_file")
    use_sim_time = LaunchConfiguration("use_sim_time")
    enable_robot1 = LaunchConfiguration("enable_robot1")
    enable_robot2 = LaunchConfiguration("enable_robot2")

    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, "launch", "multi_robot_gazebo.launch.py")
        ),
        condition=IfCondition(launch_sim),
    )

    robot1_slam = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        namespace="robot1",
        name="slam_toolbox",
        output="screen",
        parameters=[slam_params_file, {"use_sim_time": use_sim_time}],
        condition=IfCondition(enable_robot1),
    )

    robot2_slam = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        namespace="robot2",
        name="slam_toolbox",
        output="screen",
        parameters=[slam_params_file, {"use_sim_time": use_sim_time}],
        condition=IfCondition(enable_robot2),
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
            DeclareLaunchArgument(
                "launch_sim",
                default_value="true",
                description="Launch the multi-robot Gazebo simulation before SLAM.",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use simulation time.",
            ),
            DeclareLaunchArgument(
                "use_rviz",
                default_value="true",
                description="Launch RViz with the SLAM configuration.",
            ),
            DeclareLaunchArgument(
                "enable_robot1",
                default_value="true",
                description="Enable slam_toolbox for robot1.",
            ),
            DeclareLaunchArgument(
                "enable_robot2",
                default_value="false",
                description="Enable slam_toolbox for robot2.",
            ),
            DeclareLaunchArgument(
                "slam_params_file",
                default_value=os.path.join(
                    pkg_share, "config", "slam_params_multi_robot.yaml"
                ),
                description="SLAM Toolbox parameter file with robot1/robot2 namespaces.",
            ),
            sim_launch,
            robot1_slam,
            robot2_slam,
            rviz,
        ]
    )
