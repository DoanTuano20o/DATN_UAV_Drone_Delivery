from __future__ import annotations

import json
import threading
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String


def load_calibration(calib_path: str):
    path = Path(calib_path)
    if not path.exists():
        raise FileNotFoundError(f"Calibration file not found: {calib_path}")

    suffix = path.suffix.lower()

    if suffix == ".json":
        data = json.loads(path.read_text(encoding="utf-8"))

        camera_matrix = None
        dist_coeffs = None

        if "camera_matrix" in data:
            camera_matrix = np.array(data["camera_matrix"], dtype=np.float64)
        elif "cameraMatrix" in data:
            camera_matrix = np.array(data["cameraMatrix"], dtype=np.float64)

        if "dist_coeffs" in data:
            dist_coeffs = np.array(data["dist_coeffs"], dtype=np.float64)
        elif "distCoeffs" in data:
            dist_coeffs = np.array(data["distCoeffs"], dtype=np.float64)

        if camera_matrix is None or dist_coeffs is None:
            raise ValueError("JSON calibration missing camera_matrix/dist_coeffs")

        return camera_matrix, dist_coeffs.reshape(-1, 1)

    fs = cv2.FileStorage(str(path), cv2.FILE_STORAGE_READ)
    if not fs.isOpened():
        raise RuntimeError(f"Cannot open calibration file: {calib_path}")

    camera_matrix = fs.getNode("camera_matrix").mat()
    if camera_matrix is None:
        camera_matrix = fs.getNode("cameraMatrix").mat()

    dist_coeffs = fs.getNode("dist_coeffs").mat()
    if dist_coeffs is None:
        dist_coeffs = fs.getNode("distCoeffs").mat()

    fs.release()

    if camera_matrix is None or dist_coeffs is None:
        raise ValueError("YAML calibration missing camera_matrix/dist_coeffs")

    return camera_matrix.astype(np.float64), dist_coeffs.astype(np.float64)


def estimate_pose_single_marker(corner_4x2, marker_size_m, camera_matrix, dist_coeffs):
    half = marker_size_m / 2.0

    obj_points = np.array(
        [
            [-half,  half, 0.0],
            [ half,  half, 0.0],
            [ half, -half, 0.0],
            [-half, -half, 0.0],
        ],
        dtype=np.float32,
    )

    img_points = np.array(corner_4x2, dtype=np.float32)

    ok, rvec, tvec = cv2.solvePnP(
        obj_points,
        img_points,
        camera_matrix,
        dist_coeffs,
        flags=cv2.SOLVEPNP_IPPE_SQUARE,
    )

    if not ok:
        ok, rvec, tvec = cv2.solvePnP(
            obj_points,
            img_points,
            camera_matrix,
            dist_coeffs,
            flags=cv2.SOLVEPNP_ITERATIVE,
        )

    if not ok:
        raise RuntimeError("solvePnP failed for marker pose")

    return rvec.reshape(3, 1), tvec.reshape(3, 1)


class ArucoDetectorNode(Node):
    def __init__(self) -> None:
        super().__init__("aruco_detector_node")

        self.declare_parameter("image_topic", "/camera/image/compressed")
        self.declare_parameter("annotated_topic", "/vision/image_annotated/compressed")
        self.declare_parameter("detections_topic", "/vision/aruco_detections_json")

        self.declare_parameter("aruco_dict_name", "DICT_4X4_250")

        self.declare_parameter("big_marker_id", 150)
        self.declare_parameter("big_marker_size_m", 0.70)
        self.declare_parameter("small_marker_id", 40)
        self.declare_parameter("small_marker_size_m", 0.08)

        self.declare_parameter("calib_path", "")
        self.declare_parameter("draw_axes", False)
        self.declare_parameter("jpeg_quality", 60)

        self.declare_parameter("process_rate_hz", 12.0)
        self.declare_parameter("annotated_rate_hz", 4.0)

        self.image_topic = str(self.get_parameter("image_topic").value)
        self.annotated_topic = str(self.get_parameter("annotated_topic").value)
        self.detections_topic = str(self.get_parameter("detections_topic").value)

        self.aruco_dict_name = str(self.get_parameter("aruco_dict_name").value)
        self.big_marker_id = int(self.get_parameter("big_marker_id").value)
        self.big_marker_size_m = float(self.get_parameter("big_marker_size_m").value)
        self.small_marker_id = int(self.get_parameter("small_marker_id").value)
        self.small_marker_size_m = float(self.get_parameter("small_marker_size_m").value)
        self.marker_sizes_m = {
            self.big_marker_id: self.big_marker_size_m,
            self.small_marker_id: self.small_marker_size_m,
        }

        self.calib_path = str(self.get_parameter("calib_path").value)
        self.draw_axes = bool(self.get_parameter("draw_axes").value)
        self.jpeg_quality = int(self.get_parameter("jpeg_quality").value)

        self.process_rate_hz = float(self.get_parameter("process_rate_hz").value)
        self.annotated_rate_hz = float(self.get_parameter("annotated_rate_hz").value)

        self.camera_matrix, self.dist_coeffs = load_calibration(self.calib_path)

        aruco = cv2.aruco
        if not hasattr(aruco, self.aruco_dict_name):
            raise ValueError(f"Unsupported ArUco dictionary: {self.aruco_dict_name}")

        self.aruco_dict = aruco.getPredefinedDictionary(getattr(aruco, self.aruco_dict_name))

        try:
            self.detector_params = aruco.DetectorParameters()
        except Exception:
            self.detector_params = aruco.DetectorParameters_create()

        self._frame_lock = threading.Lock()
        self._latest_msg: CompressedImage | None = None
        self._last_annotated_pub_s = 0.0

        self.image_sub = self.create_subscription(
            CompressedImage,
            self.image_topic,
            self.on_image,
            1,
        )

        self.annotated_pub = self.create_publisher(CompressedImage, self.annotated_topic, 1)
        self.detections_pub = self.create_publisher(String, self.detections_topic, 10)

        self.process_timer = self.create_timer(
            1.0 / max(self.process_rate_hz, 1.0),
            self.process_latest_frame,
        )

        self.get_logger().info(
            f"ArucoDetectorNode started. input={self.image_topic}, "
            f"annotated={self.annotated_topic}, detections={self.detections_topic}"
        )
        self.get_logger().info(
            f"Loaded calibration: {self.calib_path} | "
            f"markers={self.marker_sizes_m}"
        )

    def detect_markers(self, gray):
        aruco = cv2.aruco

        if hasattr(aruco, "ArucoDetector"):
            detector = aruco.ArucoDetector(self.aruco_dict, self.detector_params)
            corners, ids, rejected = detector.detectMarkers(gray)
        else:
            corners, ids, rejected = aruco.detectMarkers(
                gray,
                self.aruco_dict,
                parameters=self.detector_params,
            )

        return corners, ids, rejected

    def on_image(self, msg: CompressedImage) -> None:
        with self._frame_lock:
            self._latest_msg = msg

    def should_publish_annotated(self) -> bool:
        now_s = time.time()
        if now_s - self._last_annotated_pub_s >= (1.0 / max(self.annotated_rate_hz, 1.0)):
            self._last_annotated_pub_s = now_s
            return True
        return False

    def process_latest_frame(self) -> None:
        with self._frame_lock:
            msg = self._latest_msg
            self._latest_msg = None

        if msg is None:
            return

        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            if frame is None:
                return

            publish_annotated = self.should_publish_annotated()
            annotated = frame.copy() if publish_annotated else None

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            corners, ids, _ = self.detect_markers(gray)

            detections = []
            h, w = frame.shape[:2]
            img_cx = w * 0.5
            img_cy = h * 0.5

            ids_flat = []
            if ids is not None and len(ids) > 0:
                ids_flat = ids.flatten().tolist()
                self.get_logger().debug(f"RAW detected IDs: {ids_flat}")

                matched_corners = []
                matched_ids = []

                for i, marker_id in enumerate(ids_flat):
                    if int(marker_id) in self.marker_sizes_m:
                        matched_corners.append(corners[i])
                        matched_ids.append(int(marker_id))

                if matched_corners:
                    if publish_annotated and annotated is not None:
                        cv2.aruco.drawDetectedMarkers(
                            annotated,
                            matched_corners,
                            np.array(matched_ids).reshape(-1, 1),
                        )

                    for i, marker_id in enumerate(matched_ids):
                        pts = matched_corners[i][0]
                        marker_size_m = self.marker_sizes_m[marker_id]
                        cx = float(np.mean(pts[:, 0]))
                        cy = float(np.mean(pts[:, 1]))
                        err_x = float(cx - img_cx)
                        err_y = float(cy - img_cy)

                        rvec, tvec = estimate_pose_single_marker(
                            pts,
                            marker_size_m,
                            self.camera_matrix,
                            self.dist_coeffs,
                        )

                        x_m = float(tvec[0][0])
                        y_m = float(tvec[1][0])
                        z_m = float(tvec[2][0])
                        distance_m = float(np.linalg.norm(tvec))

                        if publish_annotated and annotated is not None:
                            if self.draw_axes:
                                cv2.drawFrameAxes(
                                    annotated,
                                    self.camera_matrix,
                                    self.dist_coeffs,
                                    rvec,
                                    tvec,
                                    marker_size_m * 0.5,
                                )

                            cv2.circle(annotated, (int(cx), int(cy)), 5, (0, 255, 255), -1)
                            cv2.line(
                                annotated,
                                (int(img_cx), int(img_cy)),
                                (int(cx), int(cy)),
                                (255, 255, 0),
                                2,
                            )

                            text1 = f"ID {marker_id} err=({err_x:.1f},{err_y:.1f})"
                            text2 = f"x={x_m:.2f} y={y_m:.2f} z={z_m:.2f} d={distance_m:.2f}m"

                            cv2.putText(
                                annotated,
                                text1,
                                (int(cx) + 10, int(cy) - 10),
                                cv2.FONT_HERSHEY_SIMPLEX,
                                0.5,
                                (0, 255, 0),
                                2,
                            )
                            cv2.putText(
                                annotated,
                                text2,
                                (int(cx) + 10, int(cy) + 10),
                                cv2.FONT_HERSHEY_SIMPLEX,
                                0.5,
                                (0, 255, 0),
                                2,
                            )

                        detections.append(
                            {
                                "marker_id": marker_id,
                                "center_x": int(round(cx)),
                                "center_y": int(round(cy)),
                                "err_x": err_x,
                                "err_y": err_y,
                                "x_m": x_m,
                                "y_m": y_m,
                                "z_m": z_m,
                                "distance_m": distance_m,
                                "rvec": [float(rvec[0][0]), float(rvec[1][0]), float(rvec[2][0])],
                                "tvec": [x_m, y_m, z_m],
                            }
                        )

            out = {
                "count": len(detections),
                "detections": detections,
            }
            out_msg = String()
            out_msg.data = json.dumps(out, ensure_ascii=False)
            self.detections_pub.publish(out_msg)

            if publish_annotated and annotated is not None:
                ok, jpg = cv2.imencode(
                    ".jpg",
                    annotated,
                    [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality],
                )
                if ok:
                    out_img = CompressedImage()
                    out_img.header = msg.header
                    out_img.format = "jpeg"
                    out_img.data = jpg.tobytes()
                    self.annotated_pub.publish(out_img)

        except Exception as e:
            self.get_logger().warn(f"Aruco processing error: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetectorNode()
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
