#!/usr/bin/env python3

import json
from pathlib import Path

import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.node import Node


class ObstaclePointCapture(Node):
    def __init__(self) -> None:
        super().__init__("capture_obstacle_points")

        if not self.has_parameter("use_sim_time"):
            self.declare_parameter("use_sim_time", True)
        self.declare_parameter("input_topic", "/clicked_point")
        self.declare_parameter(
            "output_file",
            "/home/ganesh/robot_ws/src/donar_robot_description/config/obstacle_points.json",
        )

        input_topic = str(self.get_parameter("input_topic").value)
        self.output_file = Path(str(self.get_parameter("output_file").value))
        self.pending_start: dict | None = None
        self.obstacles: list[dict] = []

        self.create_subscription(PointStamped, input_topic, self._handle_point, 10)
        self.get_logger().info(
            "Listening on '%s'. Use RViz Publish Point and click start then end for each obstacle."
            % input_topic
        )

    def _handle_point(self, msg: PointStamped) -> None:
        point = {
            "x": round(float(msg.point.x), 3),
            "y": round(float(msg.point.y), 3),
        }

        if self.pending_start is None:
            self.pending_start = point
            self.get_logger().info(
                "Captured obstacle_%d start at x=%.3f y=%.3f"
                % (len(self.obstacles) + 1, point["x"], point["y"])
            )
            return

        obstacle = {
            "name": f"obstacle_{len(self.obstacles) + 1}",
            "start": self.pending_start,
            "end": point,
        }
        self.obstacles.append(obstacle)
        self.pending_start = None
        self._write_output()
        self.get_logger().info(
            "Captured %s end at x=%.3f y=%.3f"
            % (obstacle["name"], point["x"], point["y"])
        )
        self.get_logger().info(
            "%s snippet: \"start\": (%.3f, %.3f), \"end\": (%.3f, %.3f)"
            % (
                obstacle["name"],
                obstacle["start"]["x"],
                obstacle["start"]["y"],
                obstacle["end"]["x"],
                obstacle["end"]["y"],
            )
        )

    def _write_output(self) -> None:
        self.output_file.parent.mkdir(parents=True, exist_ok=True)
        payload = {"obstacles": self.obstacles}
        if self.pending_start is not None:
            payload["pending_start"] = self.pending_start
        with self.output_file.open("w", encoding="utf-8") as outfile:
            json.dump(payload, outfile, indent=2)
            outfile.write("\n")


def main() -> None:
    rclpy.init()
    node = ObstaclePointCapture()
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
