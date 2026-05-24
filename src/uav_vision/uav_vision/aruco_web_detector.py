from __future__ import annotations

import time

import cv2
import numpy as np


class ArucoWebDetector:
    def __init__(
        self,
        dict_id: int = cv2.aruco.DICT_4X4_1000,
        allowed_ids: set[int] | None = None,
        min_perimeter_px: float = 40.0,
    ) -> None:
        self.allowed_ids = allowed_ids
        self.min_perimeter_px = min_perimeter_px

        self.aruco_dict = cv2.aruco.getPredefinedDictionary(dict_id)

        if hasattr(cv2.aruco, "DetectorParameters"):
            self.detector_params = cv2.aruco.DetectorParameters()
        else:
            self.detector_params = cv2.aruco.DetectorParameters_create()

        if hasattr(self.detector_params, "cornerRefinementMethod"):
            self.detector_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX

        if hasattr(self.detector_params, "adaptiveThreshWinSizeMin"):
            self.detector_params.adaptiveThreshWinSizeMin = 5
        if hasattr(self.detector_params, "adaptiveThreshWinSizeMax"):
            self.detector_params.adaptiveThreshWinSizeMax = 23
        if hasattr(self.detector_params, "adaptiveThreshWinSizeStep"):
            self.detector_params.adaptiveThreshWinSizeStep = 4

        if hasattr(self.detector_params, "minMarkerPerimeterRate"):
            self.detector_params.minMarkerPerimeterRate = 0.02

        if hasattr(cv2.aruco, "ArucoDetector"):
            self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.detector_params)
        else:
            self.detector = None

    def _detect_markers(self, gray):
        if self.detector is not None:
            return self.detector.detectMarkers(gray)

        return cv2.aruco.detectMarkers(
            gray,
            self.aruco_dict,
            parameters=self.detector_params,
        )

    @staticmethod
    def _corner_perimeter(corner) -> float:
        pts = corner[0].astype(np.float32)
        return float(cv2.arcLength(pts, True))

    def process(self, frame):
        vis = frame.copy()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        corners, ids, _ = self._detect_markers(gray)

        h, w = vis.shape[:2]
        cx_img = w // 2
        cy_img = h // 2
        cv2.circle(vis, (cx_img, cy_img), 4, (255, 255, 255), -1)

        if ids is None or len(ids) == 0:
            return vis, []

        ids = ids.flatten()

        draw_corners = []
        draw_ids = []
        detections = []

        for corner, marker_id in zip(corners, ids):
            marker_id = int(marker_id)

            if self.allowed_ids is not None and marker_id not in self.allowed_ids:
                continue

            perimeter_px = self._corner_perimeter(corner)
            if perimeter_px < self.min_perimeter_px:
                continue

            pts = corner[0]
            center_x = int(np.mean(pts[:, 0]))
            center_y = int(np.mean(pts[:, 1]))
            err_x = center_x - cx_img
            err_y = center_y - cy_img

            draw_corners.append(corner)
            draw_ids.append([marker_id])

            cv2.circle(vis, (center_x, center_y), 4, (0, 255, 255), -1)
            cv2.putText(
                vis,
                f"ID={marker_id} err=({err_x},{err_y})",
                (center_x + 8, center_y - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 255, 255),
                2,
            )

            detections.append(
                {
                    "marker_id": marker_id,
                    "center_x": center_x,
                    "center_y": center_y,
                    "err_x": err_x,
                    "err_y": err_y,
                    "perimeter_px": perimeter_px,
                    "distance_m": None,
                    "z_m": None,
                    "src": "web_fast",
                    "timestamp": time.time(),
                }
            )

        if draw_ids:
            cv2.aruco.drawDetectedMarkers(
                vis,
                draw_corners,
                np.array(draw_ids, dtype=np.int32),
            )

        return vis, detections