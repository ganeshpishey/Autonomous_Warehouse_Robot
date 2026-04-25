import math
import os
import textwrap
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
 

def build_obstacle_sdf(name: str, color: tuple[float, float, float]) -> str:
    r, g, b = color
    return textwrap.dedent(
        f"""
        <sdf version="1.8">
          <model name="{name}">
            <static>false</static>
            <kinematic>true</kinematic>
            <gravity>false</gravity>
            <self_collide>false</self_collide>
            <link name="body">
              <inertial>
                <mass>8.0</mass>
                <inertia>
                  <ixx>0.5</ixx><iyy>0.5</iyy><izz>0.5</izz>
                  <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz>
                </inertia>
              </inertial>
              <collision name="collision">
                <geometry>
                  <box>
                    <size>0.6 0.6 0.9</size>
                  </box>
                </geometry>
              </collision>
              <visual name="visual">
                <geometry>
                  <box>
                    <size>0.6 0.6 0.9</size>
                  </box>
                </geometry>
                <material>
                  <ambient>{r} {g} {b} 1</ambient>
                  <diffuse>{r} {g} {b} 1</diffuse>
                </material>
              </visual>
            </link>
          </model>
        </sdf>
        """
    ).strip()


def generate_launch_description():
    pkg_share = get_package_share_directory("donar_robot_description")
    obstacle_z = 0.48
    obstacle_specs = [
        {
            "name": "predefined_route_obstacle_1",
            "color": (0.90, 0.18, 0.18),
            "spawn_delay": 8.0,
            "motion_delay": 2.0,
            "start": (11.491, -21.099),
            "end": (14.612, -18.443),
        },
        {
            "name": "predefined_route_obstacle_2",
            "color": (0.95, 0.58, 0.16),
            "spawn_delay": 10.0,
            "motion_delay": 4.0,
            "start": (14.586, -10.098),
            "end": (11.356, -6.334),
        },
        {
            "name": "predefined_route_obstacle_3",
            "color": (0.18, 0.55, 0.92),
            "spawn_delay": 12.0,
            "motion_delay": 6.0,
            "start": (13.014, -1.430),
            "end": (8.386, 2.434),
        },
        {
            "name": "predefined_route_obstacle_4",
            "color": (0.28, 0.80, 0.38),
            "spawn_delay": 14.0,
            "motion_delay": 8.0,
            "start": (2.858, -3.392),
            "end": (-1.697, -6.272),
        },
    ]

    def obstacle_initial_xy(spec: dict) -> tuple[float, float]:
        if spec.get("motion_mode", "linear") == "orbit":
            center_x = float(spec["center"][0])
            center_y = float(spec["center"][1])
            radius = float(spec["radius"])
            start_angle = float(spec.get("start_angle", 0.0))
            return (
                center_x + radius * math.cos(start_angle),
                center_y + radius * math.sin(start_angle),
            )
        return float(spec["start"][0]), float(spec["start"][1])
    
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, "launch", "gazebo_sdf.launch.py")
        ),
        launch_arguments={
            "spawn_x": LaunchConfiguration("spawn_x"),
            "spawn_y": LaunchConfiguration("spawn_y"),
            "spawn_z": LaunchConfiguration("spawn_z"),
            "spawn_yaw": LaunchConfiguration("spawn_yaw"),
            "robot_name": "donar_robot",
        }.items(),
    )
    
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, "launch", "single_robot_nav2.launch.py")
        ),
        launch_arguments={
            "map": LaunchConfiguration("map"),
            "use_rviz": LaunchConfiguration("use_rviz"),
            "launch_sim": "false",
            "nav2_start_delay": "0.0",
            "initial_pose_x": LaunchConfiguration("initial_pose_x"),
            "initial_pose_y": LaunchConfiguration("initial_pose_y"),
            "initial_pose_yaw": LaunchConfiguration("initial_pose_yaw"),
            "publish_initial_pose": LaunchConfiguration("publish_initial_pose"),
        }.items(),
    )
    
    mission_planner = Node(
        package="donar_robot_description",
        executable="mission_planner.py",
        name="predefined_multi_goal_mission_planner",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "goals_file": LaunchConfiguration("goals_file"),
                "start_delay_sec": LaunchConfiguration("mission_start_delay"),
                "max_retries": LaunchConfiguration("max_retries"),
                "stop_on_failure": False,
                "wait_for_initial_pose": True,
                "initial_pose_topic": "/initialpose",
                "amcl_pose_topic": "/amcl_pose",
                "accept_amcl_pose_as_initial_pose": False,
                "wait_for_localization": True,
                "goal_markers_topic": "/mission_goals",
                "status_topic": "/mission_status",
                "align_yaw_to_path": True,
                "start_from_nearest_goal": LaunchConfiguration("start_from_nearest_goal"),
                "skip_goal_count": LaunchConfiguration("skip_goal_count"),
                "optimize_route_order": LaunchConfiguration("optimize_route_order"),
            }
        ],
    )

    dynamic_obstacle_actions = []
    for spec in obstacle_specs:
        linear_start = spec.get("start", obstacle_initial_xy(spec))
        linear_end = spec.get("end", linear_start)
        controller_parameters = {
            "use_sim_time": True,
            "world_name": "warehouse_world",
            "obstacle_name": spec["name"],
            "spawn_on_start": True,
            "start_delay_sec": spec["motion_delay"],
            "position_frame": "map",
            "target_frame": "odom",
            "motion_mode": spec.get("motion_mode", "linear"),
            "start_x": linear_start[0],
            "start_y": linear_start[1],
            "end_x": linear_end[0],
            "end_y": linear_end[1],
            "z": obstacle_z,
            "color_r": spec["color"][0],
            "color_g": spec["color"][1],
            "color_b": spec["color"][2],
        }
        if spec.get("motion_mode") == "orbit":
            controller_parameters.update(
                {
                    "center_x": spec["center"][0],
                    "center_y": spec["center"][1],
                    "orbit_radius": spec["radius"],
                    "angular_speed_radps": spec.get("angular_speed_radps", 0.6),
                    "orbit_start_angle": spec.get("start_angle", 0.0),
                }
            )

        obstacle_controller = Node(
            package="donar_robot_description",
            executable="dynamic_obstacle_controller.py",
            name=f"{spec['name']}_controller",
            output="screen",
            condition=IfCondition(LaunchConfiguration("enable_dynamic_obstacle")),
            parameters=[controller_parameters],
        )

        dynamic_obstacle_actions.extend(
            [
                TimerAction(period=spec["spawn_delay"], actions=[obstacle_controller]),
            ]
        )
    
    delayed_nav2_launch = TimerAction(
        period=15.0,
        actions=[nav2_launch],
    )
    
    delayed_mission_planner = TimerAction(
        period=26.0,
        actions=[mission_planner],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "map",
                default_value="/home/ganesh/robot_ws/warehouse_map.yaml",
            ),
            DeclareLaunchArgument(
                "goals_file",
                default_value=os.path.join(
                    pkg_share, "config", "warehouse_route.json"
                ),
            ),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument("spawn_x", default_value="-11.4"),
            DeclareLaunchArgument("spawn_y", default_value="-4.9"),
            DeclareLaunchArgument("spawn_z", default_value="0.15"),
            DeclareLaunchArgument("spawn_yaw", default_value="-1.57"),
            DeclareLaunchArgument("initial_pose_x", default_value="-11.4"),
            DeclareLaunchArgument("initial_pose_y", default_value="-4.9"),
            DeclareLaunchArgument("initial_pose_yaw", default_value="-1.57"),
            DeclareLaunchArgument("publish_initial_pose", default_value="true"),
            DeclareLaunchArgument("mission_start_delay", default_value="10.0"),
            DeclareLaunchArgument("max_retries", default_value="2"),
            DeclareLaunchArgument("start_from_nearest_goal", default_value="false"),
            DeclareLaunchArgument("skip_goal_count", default_value="1"),
            DeclareLaunchArgument("optimize_route_order", default_value="false"),
            DeclareLaunchArgument("enable_dynamic_obstacle", default_value="false"),
            gazebo_launch,
            delayed_nav2_launch,
            delayed_mission_planner,
            *dynamic_obstacle_actions,
        ]
    )
 
