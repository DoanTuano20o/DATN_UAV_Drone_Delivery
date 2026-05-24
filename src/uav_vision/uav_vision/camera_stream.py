from __future__ import annotations

import threading
import time

import cv2

from .aruco_web_detector import ArucoWebDetector


class CameraStream:
    def __init__(
        self,
        camera_index: int = 0,
        width: int = 640,
        height: int = 480,
        fps: int = 15,
        jpeg_quality: int = 65,
        detect_every_n: int = 1,
    ) -> None:
        self.camera_index = camera_index
        self.width = width
        self.height = height
        self.fps = fps
        self.jpeg_quality = jpeg_quality
        self.detect_every_n = max(1, int(detect_every_n))

        self.cap = None
        self.running = False
        self.thread = None
        self.lock = threading.Lock()

        self.frame_jpeg = None
        self.last_detections = []
        self.frame_count = 0

        # Web chỉ cần detect nhẹ, không pose
        self.detector = ArucoWebDetector(
            dict_id=cv2.aruco.DICT_4X4_1000,
            allowed_ids={40, 150},
            min_perimeter_px=40.0,
        )

    def start(self) -> None:
        if self.running:
            return

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

        self.running = True
        self.thread = threading.Thread(target=self._reader_loop, daemon=True)
        self.thread.start()

    def _reader_loop(self) -> None:
        frame_interval = 1.0 / max(self.fps, 1)

        while self.running:
            loop_start = time.time()

            if self.cap is None:
                time.sleep(0.05)
                continue

            ok, frame = self.cap.read()
            if not ok or frame is None:
                time.sleep(0.02)
                continue

            self.frame_count += 1

            if self.frame_count % self.detect_every_n == 0:
                annotated, detections = self.detector.process(frame)
            else:
                annotated = frame
                detections = self.last_detections

            ok, buffer = cv2.imencode(
                ".jpg",
                annotated,
                [int(cv2.IMWRITE_JPEG_QUALITY), int(self.jpeg_quality)],
            )
            if not ok:
                continue

            with self.lock:
                self.frame_jpeg = buffer.tobytes()
                self.last_detections = detections

            elapsed = time.time() - loop_start
            sleep_time = frame_interval - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    def get_frame(self):
        with self.lock:
            return self.frame_jpeg

    def get_latest_detections(self):
        with self.lock:
            return list(self.last_detections)

    def generate(self):
        while True:
            frame = self.get_frame()
            if frame is None:
                time.sleep(0.02)
                continue

            yield (
                b"--frame\r\n"
                b"Content-Type: image/jpeg\r\n\r\n" + frame + b"\r\n"
            )

    def stop(self) -> None:
        self.running = False
        if self.thread is not None:
            self.thread.join(timeout=1.0)
            self.thread = None

        if self.cap is not None:
            self.cap.release()
            self.cap = None