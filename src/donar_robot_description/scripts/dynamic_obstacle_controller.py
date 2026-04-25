#!/usr/bin/env python3

import math
import textwrap

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from ros_gz_interfaces.msg import Entity
from ros_gz_interfaces.srv import SetEntityPose, SpawnEntity
from tf2_ros import Buffer, TransformException, TransformListener


class DynamicObstacleController(Node):
    def __init__(self) -> None:
        super().__init__("dynamic_obstacle_controller")

        self.declare_parameter("world_name", "warehouse_world")
        self.declare_parameter("obstacle_name", "final_demo_obstacle")
        self.declare_parameter("spawn_on_start", True)
        self.declare_parameter("start_delay_sec", 20.0)
        self.declare_parameter("update_period_sec", 0.2)
        self.declare_parameter("position_frame", "world")
        self.declare_parameter("target_frame", "world")
        self.declare_parameter("speed_mps", 0.35)
        self.declare_parameter("motion_mode", "linear")
        self.declare_parameter("start_x", -2.0)
        self.declare_parameter("start_y", -0.5)
        self.declare_parameter("end_x", 2.5)
        self.declare_parameter("end_y", -0.5)
        self.declare_parameter("center_x", 0.0)
        self.declare_parameter("center_y", 0.0)
        self.declare_parameter("orbit_radius", 1.0)
        self.declare_parameter("angular_speed_radps", 0.6)
        self.declare_parameter("orbit_start_angle", 0.0)
        self.declare_parameter("z", 0.45)
        self.declare_parameter("size_x", 0.6)
        self.declare_parameter("size_y", 0.6)
        self.declare_parameter("size_z", 0.9)
        self.declare_parameter("color_r", 0.9)
        self.declare_parameter("color_g", 0.15)
        self.declare_parameter("color_b", 0.15)

        self.world_name = str(self.get_parameter("world_name").value)
        self.obstacle_name = str(self.get_parameter("obstacle_name").value)
        self.spawn_on_start = bool(self.get_parameter("spawn_on_start").value)
        self.start_delay_sec = float(self.get_parameter("start_delay_sec").value)
        self.update_period_sec = float(self.get_parameter("update_period_sec").value)
        self.position_frame = str(self.get_parameter("position_frame").value)
        self.target_frame = str(self.get_parameter("target_frame").value)
        self.speed_mps = float(self.get_parameter("speed_mps").value)
        self.motion_mode = str(self.get_parameter("motion_mode").value).strip().lower()
        self.start_x = float(self.get_parameter("start_x").value)
        self.start_y = float(self.get_parameter("start_y").value)
        self.end_x = float(self.get_parameter("end_x").value)
        self.end_y = float(self.get_parameter("end_y").value)
        self.center_x = float(self.get_parameter("center_x").value)
        self.center_y = float(self.get_parameter("center_y").value)
        self.orbit_radius = float(self.get_parameter("orbit_radius").value)
        self.angular_speed_radps = float(self.get_parameter("angular_speed_radps").value)
        self.orbit_start_angle = float(self.get_parameter("orbit_start_angle").value)
        self.z = float(self.get_parameter("z").value)
        self.size_x = float(self.get_parameter("size_x").value)
        self.size_y = float(self.get_parameter("size_y").value)
        self.size_z = float(self.get_parameter("size_z").value)
        self.color_r = float(self.get_parameter("color_r").value)
        self.color_g = float(self.get_parameter("color_g").value)
        self.color_b = float(self.get_parameter("color_b").value)

        self.spawn_client = self.create_client(
            SpawnEntity, f"/world/{self.world_name}/create"
        )
        self.set_pose_client = self.create_client(
            SetEntityPose, f"/world/{self.world_name}/set_pose"
        )
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.spawned = not self.spawn_on_start
        self.spawn_request_in_flight = False
        self.pose_request_in_flight = False
        self._reported_spawn_service_wait = False
        self._reported_pose_service_wait = False
        self._reported_pose_failure = False
        self._reported_motion_start = False
        self._reported_invalid_mode = False
        self._reported_transform_wait = False
        self.start_time = self.get_clock().now()
        self.timer = self.create_timer(self.update_period_sec, self._tick)

    def _sdf(self) -> str:
        mass = 8.0
        r = self.color_r
        g = self.color_g
        b = self.color_b
        return textwrap.dedent(
            f"""
            <sdf version="1.8">
              <model name="{self.obstacle_name}">
                <static>false</static>
                <kinematic>true</kinematic>
                <gravity>false</gravity>
                <self_collide>false</self_collide>
                <link name="body">
                  <inertial>
                    <mass>{mass}</mass>
                    <inertia>
                      <ixx>0.5</ixx><iyy>0.5</iyy><izz>0.5</izz>
                      <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz>
                    </inertia>
                  </inertial>
                  <collision name="collision">
                    <pose>0 0 0 0 0 0</pose>
                    <geometry>
                      <box>
                        <size>{self.size_x} {self.size_y} {self.size_z}</size>
                      </box>
                    </geometry>
                  </collision>
                  <visual name="visual">
                    <pose>0 0 0 0 0 0</pose>
                    <geometry>
                      <box>
                        <size>{self.size_x} {self.size_y} {self.size_z}</size>
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

    def _elapsed(self) -> float:
        return (self.get_clock().now() - self.start_time).nanoseconds / 1e9

    def _raw_initial_position(self) -> tuple[float, float]:
        if self.motion_mode == "orbit":
            return (
                self.center_x + self.orbit_radius * math.cos(self.orbit_start_angle),
                self.center_y + self.orbit_radius * math.sin(self.orbit_start_angle),
            )
        return self.start_x, self.start_y

    def _raw_target_position(self, motion_elapsed: float) -> tuple[float, float]:
        if self.motion_mode == "orbit":
            theta = self.orbit_start_angle + self.angular_speed_radps * motion_elapsed
            return (
                self.center_x + self.orbit_radius * math.cos(theta),
                self.center_y + self.orbit_radius * math.sin(theta),
            )

        if self.motion_mode != "linear":
            if not self._reported_invalid_mode:
                self.get_logger().warn(
                    f"Unknown motion_mode '{self.motion_mode}' for "
                    f"'{self.obstacle_name}', falling back to linear"
                )
                self._reported_invalid_mode = True

        dx = self.end_x - self.start_x
        dy = self.end_y - self.start_y
        length = max(math.hypot(dx, dy), 1e-6)
        travel_time = length / max(self.speed_mps, 1e-3)
        phase = (motion_elapsed / travel_time) % 2.0
        alpha = phase if phase <= 1.0 else 2.0 - phase
        return self.start_x + dx * alpha, self.start_y + dy * alpha

    def _world_position(self, x: float, y: float) -> tuple[float, float] | None:
        if self.position_frame == self.target_frame:
            return x, y
        try:
            transform = self.tf_buffer.lookup_transform(
                self.target_frame,
                self.position_frame,
                Time(),
                timeout=Duration(seconds=0.1),
            )
        except TransformException:
            if not self._reported_transform_wait:
                self.get_logger().warn(
                    f"Waiting for transform {self.position_frame} -> {self.target_frame} "
                    f"before spawning '{self.obstacle_name}'"
                )
                self._reported_transform_wait = True
            return None

        self._reported_transform_wait = False
        tx = float(transform.transform.translation.x)
        ty = float(transform.transform.translation.y)
        qz = float(transform.transform.rotation.z)
        qw = float(transform.transform.rotation.w)
        yaw = 2.0 * math.atan2(qz, qw)
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        return (
            tx + x * cos_yaw - y * sin_yaw,
            ty + x * sin_yaw + y * cos_yaw,
        )

    def _spawn(self) -> None:
        if self.spawned or self.spawn_request_in_flight:
            return
        if not self.spawn_client.wait_for_service(timeout_sec=0.1):
            if not self._reported_spawn_service_wait:
                self.get_logger().warn(
                    f"Waiting for Gazebo spawn service /world/{self.world_name}/create"
                )
                self._reported_spawn_service_wait = True
            return

        req = SpawnEntity.Request()
        req.entity_factory.name = self.obstacle_name
        req.entity_factory.allow_renaming = False
        req.entity_factory.sdf = self._sdf()
        req.entity_factory.relative_to = "world"
        initial_position = self._world_position(*self._raw_initial_position())
        if initial_position is None:
            return
        req.entity_factory.pose.position.x = initial_position[0]
        req.entity_factory.pose.position.y = initial_position[1]
        req.entity_factory.pose.position.z = self.z
        req.entity_factory.pose.orientation.w = 1.0
        self.spawn_request_in_flight = True
        future = self.spawn_client.call_async(req)
        future.add_done_callback(self._handle_spawn_response)

    def _handle_spawn_response(self, future) -> None:
        self.spawn_request_in_flight = False
        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().error(f"Failed to spawn obstacle: {exc}")
            return
        if response.success:
            self.spawned = True
            self.get_logger().info(f"Spawned dynamic obstacle '{self.obstacle_name}'")
        else:
            self.get_logger().warn(
                f"Gazebo rejected obstacle spawn for '{self.obstacle_name}'. Retrying..."
            )

    def _set_pose(self, x: float, y: float) -> None:
        if self.pose_request_in_flight:
            return
        if not self.set_pose_client.wait_for_service(timeout_sec=0.1):
            if not self._reported_pose_service_wait:
                self.get_logger().warn(
                    f"Waiting for Gazebo pose service /world/{self.world_name}/set_pose"
                )
                self._reported_pose_service_wait = True
            return
        req = SetEntityPose.Request()
        req.entity.name = self.obstacle_name
        req.entity.type = Entity.MODEL
        req.pose.position.x = x
        req.pose.position.y = y
        req.pose.position.z = self.z
        req.pose.orientation.w = 1.0
        self.pose_request_in_flight = True
        future = self.set_pose_client.call_async(req)
        future.add_done_callback(self._handle_set_pose_response)

    def _handle_set_pose_response(self, future) -> None:
        self.pose_request_in_flight = False
        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().error(f"Failed to move obstacle: {exc}")
            return
        if not response.success and not self._reported_pose_failure:
            self.get_logger().warn(
                f"Gazebo rejected pose updates for obstacle '{self.obstacle_name}'"
            )
            self._reported_pose_failure = True

    def _tick(self) -> None:
        if self.get_clock().now().nanoseconds == 0:
            return

        elapsed = self._elapsed()
        if not self.spawned:
            self._spawn()
            return
        if elapsed < self.start_delay_sec:
            return
        if not self._reported_motion_start:
            self.get_logger().info(
                f"Starting obstacle motion for '{self.obstacle_name}'"
            )
            self._reported_motion_start = True

        world_position = self._world_position(
            *self._raw_target_position(elapsed - self.start_delay_sec)
        )
        if world_position is None:
            return
        self._set_pose(world_position[0], world_position[1])


def main() -> None:
    rclpy.init()
    node = DynamicObstacleController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
