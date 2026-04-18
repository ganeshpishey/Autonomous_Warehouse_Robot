import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
    SetEnvironmentVariable,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    pkg_name = "donar_robot_description"
    pkg_share = get_package_share_directory(pkg_name)
    use_sim_time = {"use_sim_time": True}
    spawn_x = LaunchConfiguration("spawn_x")
    spawn_y = LaunchConfiguration("spawn_y")
    spawn_z = LaunchConfiguration("spawn_z")
    spawn_yaw = LaunchConfiguration("spawn_yaw")
    robot_name = LaunchConfiguration("robot_name")
    frame_prefix = LaunchConfiguration("frame_prefix")
    launch_gazebo = LaunchConfiguration("launch_gazebo")

    # Resource Path for Meshes
    pkg_parent = os.path.dirname(pkg_share)
    ign_resource_path = SetEnvironmentVariable(
        name="IGN_GAZEBO_RESOURCE_PATH",
        value=pkg_parent + ":" + os.environ.get("IGN_GAZEBO_RESOURCE_PATH", ""),
    )
    gz_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=pkg_parent + ":" + os.environ.get("GZ_SIM_RESOURCE_PATH", ""),
    )

    xacro_file = os.path.join(pkg_share, "urdf", "donar_robot.xacro")

    def launch_setup(context, *args, **kwargs):
        robot_name_val = context.launch_configurations["robot_name"]
        frame_prefix_val = context.launch_configurations["frame_prefix"]

        robot_description_config = xacro.process_file(
            xacro_file,
            mappings={
                "robot_name": robot_name_val,
                "frame_prefix": frame_prefix_val,
            },
        )
        robot_description = {"robot_description": robot_description_config.toxml()}

        robot_state_publisher = Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            namespace=robot_name_val,
            output="screen",
            parameters=[robot_description, use_sim_time],
        )

        joint_state_publisher = Node(
            package="joint_state_publisher",
            executable="joint_state_publisher",
            namespace=robot_name_val,
            output="screen",
            parameters=[
                robot_description,
                use_sim_time,
                {
                    "rate": 50.0,
                    "publish_default_positions": True,
                    "zeros": {
                        f"{frame_prefix_val}left_wheel_joint": 0.0,
                        f"{frame_prefix_val}right_wheel_joint": 0.0,
                    },
                },
            ],
        )

        spawn_robot = RegisterEventHandler(
            event_handler=OnProcessStart(
                target_action=robot_state_publisher,
                on_start=[
                    TimerAction(
                        period=5.0,
                        actions=[
                            Node(
                                package="ros_gz_sim",
                                executable="create",
                                arguments=[
                                    "-topic",
                                    f"/{robot_name_val}/robot_description",
                                    "-name",
                                    robot_name_val,
                                    "-x",
                                    context.launch_configurations["spawn_x"],
                                    "-y",
                                    context.launch_configurations["spawn_y"],
                                    "-z",
                                    context.launch_configurations["spawn_z"],
                                    "-Y",
                                    context.launch_configurations["spawn_yaw"],
                                ],
                                output="screen",
                            )
                        ],
                    )
                ],
            )
        )

        parameter_bridge = Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            parameters=[use_sim_time],
            arguments=[
                "/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock",
                f"/{robot_name_val}/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist",
                f"/{robot_name_val}/odometry@nav_msgs/msg/Odometry[ignition.msgs.Odometry",
                "/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V",
                f"/{robot_name_val}/lidar@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan",
                f"/{robot_name_val}/lidar/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked",
            ],
            output="screen",
        )

        pointcloud_to_scan = Node(
            package="pointcloud_to_laserscan",
            executable="pointcloud_to_laserscan_node",
            remappings=[
                ("cloud_in", f"/{robot_name_val}/lidar/points"),
                ("scan", f"/{robot_name_val}/scan"),
            ],
            parameters=[{
                "use_sim_time": True,
                "min_height": -0.5,
                "max_height": 0.5,
                "angle_min": -3.14159,
                "angle_max": 3.14159,
                "angle_increment": 0.00435,
                "scan_time": 0.1,
                "range_min": 0.1,
                "range_max": 10.0,
            }],
            output="screen",
        )

        return [
            robot_state_publisher,
            joint_state_publisher,
            spawn_robot,
            parameter_bridge,
            pointcloud_to_scan,
        ]

    # Gazebo Sim
    ros_gz_sim_share = get_package_share_directory("ros_gz_sim")
    gazebo_launch = os.path.join(ros_gz_sim_share, "launch", "gz_sim.launch.py")
    world_file = os.path.join(pkg_share, "worlds", "warehouse_world.sdf")
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch),
        launch_arguments={"gz_args": f"-r {world_file}"}.items(),
        condition=IfCondition(launch_gazebo),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "spawn_x",
                default_value="0.0",
                description="Robot spawn X position in the warehouse world.",
            ),
            DeclareLaunchArgument(
                "spawn_y",
                default_value="-5.45",
                description="Robot spawn Y position between the two security fences.",
            ),
            DeclareLaunchArgument(
                "spawn_z",
                default_value="0.0",
                description="Robot spawn Z position above the floor.",
            ),
            DeclareLaunchArgument(
                "spawn_yaw",
                default_value="1.57",
                description="Robot spawn yaw in radians.",
            ),
            DeclareLaunchArgument(
                "robot_name",
                default_value="donar_robot",
                description="Gazebo entity and topic prefix for this robot.",
            ),
            DeclareLaunchArgument(
                "frame_prefix",
                default_value="",
                description="Prefix added to TF frame and joint names.",
            ),
            DeclareLaunchArgument(
                "launch_gazebo",
                default_value="true",
                description="Whether to launch Gazebo alongside the robot nodes.",
            ),
            ign_resource_path,
            gz_resource_path,
            gazebo,
            OpaqueFunction(function=launch_setup),
        ]
    )
