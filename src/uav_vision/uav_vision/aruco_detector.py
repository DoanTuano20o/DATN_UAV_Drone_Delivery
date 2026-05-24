from __future__ import annotations

import json
import time
from pathlib import Path

import cv2
import numpy as np


class ArucoDetector:
    def __init__(
        self,
        calib_json: str = "calibration/camera_calibration_charuco.json",
        dict_id: int = cv2.aruco.DICT_4X4_1000,
        markers: dict[int, float] | None = None,
        pose_ema_alpha: float = 0.35,
        min_marker_perimeter_px: float = 40.0,
    ) -> None:
        self.markers = markers or {
            150: 0.20,
            40: 0.08,
        }
        self.pose_ema_alpha = pose_ema_alpha
        self.min_marker_perimeter_px = min_marker_perimeter_px
        self.pose_ema: dict[int, np.ndarray] = {}

        self.aruco_dict = cv2.aruco.getPredefinedDictionary(dict_id)

        if hasattr(cv2.aruco, "DetectorParameters"):
            self.detector_params = cv2.aruco.DetectorParameters()
        else:
            self.detector_params = cv2.aruco.DetectorParameters_create()

        if hasattr(self.detector_params, "cornerRefinementMethod"):
            self.detector_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        if hasattr(self.detector_params, "cornerRefinementWinSize"):
            self.detector_params.cornerRefinementWinSize = 5
        if hasattr(self.detector_params, "cornerRefinementMaxIterations"):
            self.detector_params.cornerRefinementMaxIterations = 50
        if hasattr(self.detector_params, "cornerRefinementMinAccuracy"):
            self.detector_params.cornerRefinementMinAccuracy = 0.01

        if hasattr(self.detector_params, "adaptiveThreshWinSizeMin"):
            self.detector_params.adaptiveThreshWinSizeMin = 3
        if hasattr(self.detector_params, "adaptiveThreshWinSizeMax"):
            self.detector_params.adaptiveThreshWinSizeMax = 31
        if hasattr(self.detector_params, "adaptiveThreshWinSizeStep"):
            self.detector_params.adaptiveThreshWinSizeStep = 4

        if hasattr(self.detector_params, "minMarkerPerimeterRate"):
            self.detector_params.minMarkerPerimeterRate = 0.01
        if hasattr(self.detector_params, "maxMarkerPerimeterRate"):
            self.detector_params.maxMarkerPerimeterRate = 4.0
        if hasattr(self.detector_params, "polygonalApproxAccuracyRate"):
            self.detector_params.polygonalApproxAccuracyRate = 0.05
        if hasattr(self.detector_params, "minDistanceToBorder"):
            self.detector_params.minDistanceToBorder = 2

        if hasattr(cv2.aruco, "ArucoDetector"):
            self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.detector_params)
        else:
            self.detector = None

        self.clahe = cv2.createCLAHE(clipLimit=2.5, tileGridSize=(8, 8))

        self.camera_matrix = None
        self.dist_coeffs = None
        self.new_camera_matrix = None
        self.map1 = None
        self.map2 = None
        self.pose_camera_matrix = None
        self.pose_dist_coeffs = None

        self._load_calibration(calib_json)

    def _load_calibration(self, calib_json: str) -> None:
        project_root = Path(__file__).resolve().parents[1]
        calib_path = project_root / calib_json

        if not calib_path.exists():
            print(f"[ARUCO] Calibration file not found: {calib_path}")
            return

        try:
            with calib_path.open("r", encoding="utf-8") as f:
                calib = json.load(f)

            self.camera_matrix = np.array(calib["camera_matrix"], dtype=np.float64)
            self.dist_coeffs = np.array(calib["dist_coeffs"], dtype=np.float64)
            print(f"[ARUCO] Loaded calibration: {calib_path}")
        except Exception as e:
            print(f"[ARUCO] Failed to load calibration: {e}")

    def _ensure_undistort_maps(self, width: int, height: int) -> None:
        if self.camera_matrix is None or self.dist_coeffs is None:
            return
        if self.map1 is not None and self.map2 is not None:
            return

        self.new_camera_matrix, _ = cv2.getOptimalNewCameraMatrix(
            self.camera_matrix,
            self.dist_coeffs,
            (width, height),
            1,
            (width, height),
        )
        self.map1, self.map2 = cv2.initUndistortRectifyMap(
            self.camera_matrix,
            self.dist_coeffs,
            None,
            self.new_camera_matrix,
            (width, height),
            cv2.CV_16SC2,
        )
        self.pose_camera_matrix = self.new_camera_matrix
        self.pose_dist_coeffs = np.zeros_like(self.dist_coeffs)

    def _detect_markers(self, gray):
        if self.detector is not None:
            return self.detector.detectMarkers(gray)
        return cv2.aruco.detectMarkers(
            gray,
            self.aruco_dict,
            parameters=self.detector_params,
        )

    def _preprocess_frame(self, frame):
        h, w = frame.shape[:2]
        self._ensure_undistort_maps(w, h)

        if self.map1 is not None and self.map2 is not None:
            undist = cv2.remap(frame, self.map1, self.map2, interpolation=cv2.INTER_LINEAR)
        else:
            undist = frame.copy()

        gray = cv2.cvtColor(undist, cv2.COLOR_BGR2GRAY)
        eq = self.clahe.apply(gray)
        med = cv2.medianBlur(eq, 3)
        blur = cv2.GaussianBlur(med, (0, 0), 1.0)
        sharp = cv2.addWeighted(med, 1.6, blur, -0.6, 0)

        return undist, {
            "gray": gray,
            "eq": eq,
            "sharp": sharp,
        }

    @staticmethod
    def _corner_perimeter(corner) -> float:
        pts = corner[0]
        peri = 0.0
        for i in range(4):
            p1 = pts[i]
            p2 = pts[(i + 1) % 4]
            peri += np.linalg.norm(p1 - p2)
        return float(peri)

    def _detect_multi_pass(self, gray_variants):
        detections = {}

        for src_name, img in gray_variants.items():
            corners, ids, _ = self._detect_markers(img)
            if ids is None or len(ids) == 0:
                continue

            for i, mid in enumerate(ids.flatten()):
                if mid not in self.markers:
                    continue

                corner = corners[i].copy()
                score = self._corner_perimeter(corner)

                if score < self.min_marker_perimeter_px:
                    continue

                if mid not in detections or score > detections[mid]["score"]:
                    detections[mid] = {
                        "corner": corner,
                        "score": score,
                        "src": src_name,
                    }

        return detections

    def _estimate_pose(self, corner, marker_length_m):
        if self.pose_camera_matrix is None or self.pose_dist_coeffs is None:
            return None, None

        marker_corners = np.array([corner[0]], dtype=np.float32)
        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
            marker_corners,
            marker_length_m,
            self.pose_camera_matrix,
            self.pose_dist_coeffs,
        )
        rvec = np.asarray(rvecs[0]).reshape(3)
        tvec = np.asarray(tvecs[0]).reshape(3)
        return rvec, tvec

    def _ema_update(self, key: int, vec: np.ndarray):
        vec = np.asarray(vec, dtype=np.float64).reshape(-1)
        if key not in self.pose_ema:
            self.pose_ema[key] = vec.copy()
        else:
            self.pose_ema[key] = self.pose_ema_alpha * vec + (1.0 - self.pose_ema_alpha) * self.pose_ema[key]
        return self.pose_ema[key]

    def process(self, frame):
        vis, variants = self._preprocess_frame(frame)
        detections = self._detect_multi_pass(variants)

        h, w = vis.shape[:2]
        cx_img = w // 2
        cy_img = h // 2
        cv2.circle(vis, (cx_img, cy_img), 4, (255, 255, 255), -1)

        if len(detections) > 0:
            draw_corners = []
            draw_ids = []
            for mid, data in detections.items():
                draw_corners.append(data["corner"])
                draw_ids.append([mid])
            cv2.aruco.drawDetectedMarkers(vis, draw_corners, np.array(draw_ids, dtype=np.int32))

        out = []

        for marker_id, data in detections.items():
            corner = data["corner"]
            src_name = data["src"]
            pts = corner[0]

            center_x = int(np.mean(pts[:, 0]))
            center_y = int(np.mean(pts[:, 1]))
            err_x = center_x - cx_img
            err_y = center_y - cy_img
            perimeter_px = float(data["score"])

            color = (0, 255, 0)
            if marker_id == 150:
                color = (0, 255, 255)
            elif marker_id == 40:
                color = (255, 255, 0)

            distance_m = None
            z_m = None

            rvec, tvec = self._estimate_pose(corner, self.markers[marker_id])
            if rvec is not None and tvec is not None:
                tvec_smooth = self._ema_update(marker_id, tvec)
                distance_m = float(np.linalg.norm(tvec_smooth))
                z_m = float(tvec_smooth[2])

                cv2.drawFrameAxes(
                    vis,
                    self.pose_camera_matrix,
                    self.pose_dist_coeffs,
                    rvec.reshape(3, 1),
                    tvec_smooth.reshape(3, 1),
                    self.markers[marker_id] * 0.5,
                )

                cv2.putText(
                    vis,
                    f"ID={marker_id} dist={distance_m:.3f}m z={z_m:.3f}m",
                    (center_x + 8, center_y - 12),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    color,
                    2,
                )
            else:
                cv2.putText(
                    vis,
                    f"ID={marker_id}",
                    (center_x + 8, center_y - 12),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    color,
                    2,
                )

            cv2.putText(
                vis,
                f"err_x={err_x} err_y={err_y} src={src_name}",
                (center_x + 8, center_y + 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.45,
                color,
                2,
            )
            cv2.circle(vis, (center_x, center_y), 4, color, -1)

            out.append(
                {
                    "marker_id": int(marker_id),
                    "center_x": int(center_x),
                    "center_y": int(center_y),
                    "err_x": int(err_x),
                    "err_y": int(err_y),
                    "perimeter_px": perimeter_px,
                    "distance_m": distance_m,
                    "z_m": z_m,
                    "src": src_name,
                    "timestamp": time.time(),
                }
            )

        return vis, out