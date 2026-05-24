from __future__ import annotations

import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage


class CameraNode(Node):
    def __init__(self) -> None:
        super().__init__("camera_node")

        self.declare_parameter("camera_index", 0)
        self.declare_parameter("width", 640)
        self.declare_parameter("height", 480)
        self.declare_parameter("fps", 15)
        self.declare_parameter("jpeg_quality", 70)

        self.camera_index = int(self.get_parameter("camera_index").value)
        self.width = int(self.get_parameter("width").value)
        self.height = int(self.get_parameter("height").value)
        self.fps = int(self.get_parameter("fps").value)
        self.jpeg_quality = int(self.get_parameter("jpeg_quality").value)

        self.compressed_pub = self.create_publisher(
            CompressedImage,
            "/camera/image/compressed",
            10,
        )

        self.cap = cv2.VideoCapture(self.camera_index, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            raise RuntimeError(f"Không mở được camera index={self.camera_index}")

        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self.cap.set(cv2.CAP_PROP_FPS, self.fps)

        try:
            self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        except Exception:
            pass

        period = 1.0 / max(self.fps, 1)
        self.timer = self.create_timer(period, self.timer_callback)

        self.get_logger().info(
            f"CameraNode started: index={self.camera_index}, "
            f"{self.width}x{self.height}@{self.fps}"
        )

    def timer_callback(self) -> None:
        ok, frame = self.cap.read()
        if not ok or frame is None:
            self.get_logger().warn("Không đọc được frame từ camera")
            return

        ok_jpg, buffer = cv2.imencode(
            ".jpg",
            frame,
            [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality],
        )
        if not ok_jpg:
            self.get_logger().warn("Không encode được JPEG")
            return

        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "camera"
        msg.format = "jpeg"
        msg.data = buffer.tobytes()
        self.compressed_pub.publish(msg)

    def destroy_node(self):
        try:
            if self.cap is not None:
                self.cap.release()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
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