from __future__ import annotations

import json
import threading

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String

from .app.server import (
    app,
    ensure_background_started,
    register_mission_publishers,
    socketio,
    update_annotated_frame_from_ros,
    update_aruco_state_from_ros,
    update_mission_state_from_ros,
    update_telemetry_state_from_ros,
)


class WebServerNode(Node):
    def __init__(self) -> None:
        super().__init__("web_server_node")

        self.declare_parameter("host", "0.0.0.0")
        self.declare_parameter("port", 5000)

        self.host = str(self.get_parameter("host").value)
        self.port = int(self.get_parameter("port").value)

        self.goal_pub = self.create_publisher(String, "/mission/goal_json", 10)
        self.control_pub = self.create_publisher(String, "/mission/control", 10)

        self.telemetry_sub = self.create_subscription(
            String,
            "/flight/telemetry_json",
            self._on_telemetry,
            10,
        )

        self.aruco_sub = self.create_subscription(
            String,
            "/vision/aruco_detections_json",
            self._on_aruco,
            10,
        )

        self.image_sub = self.create_subscription(
            CompressedImage,
            "/vision/image_annotated/compressed",
            self._on_annotated_image,
            10,
        )

        self.mission_state_sub = self.create_subscription(
            String,
            "/mission/state",
            self._on_mission_state,
            10,
        )

        register_mission_publishers(
            goal_cb=self._publish_goal,
            control_cb=self._publish_control,
        )

        self._server_thread = threading.Thread(target=self._run_server, daemon=True)
        self._server_thread.start()

        self.get_logger().info(f"Web server starting at http://{self.host}:{self.port}")

    def _publish_goal(self, payload: dict) -> None:
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self.goal_pub.publish(msg)
        self.get_logger().info(f"Published /mission/goal_json: {payload}")

    def _publish_control(self, cmd: str) -> None:
        msg = String()
        msg.data = str(cmd)
        self.control_pub.publish(msg)
        self.get_logger().info(f"Published /mission/control: {cmd}")

    def _on_telemetry(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
            if isinstance(payload, dict):
                update_telemetry_state_from_ros(payload)
        except Exception as e:
            self.get_logger().warn(f"Telemetry JSON parse error: {e}")

    def _on_aruco(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
            if isinstance(payload, dict):
                update_aruco_state_from_ros(payload)
        except Exception as e:
            self.get_logger().warn(f"Aruco JSON parse error: {e}")

    def _on_annotated_image(self, msg: CompressedImage) -> None:
        try:
            update_annotated_frame_from_ros(bytes(msg.data))
        except Exception as e:
            self.get_logger().warn(f"Annotated image update error: {e}")

    def _on_mission_state(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
            if isinstance(payload, dict):
                update_mission_state_from_ros(payload)
        except Exception as e:
            self.get_logger().warn(f"Mission state JSON parse error: {e}")

    def _run_server(self) -> None:
        ensure_background_started()
        socketio.run(
            app,
            host=self.host,
            port=self.port,
            debug=False,
            use_reloader=False,
            allow_unsafe_werkzeug=True,
        )


def main(args=None):
    rclpy.init(args=args)
    node = WebServerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass

        try:
            rclpy.shutdown()
        except Exception:
            pass