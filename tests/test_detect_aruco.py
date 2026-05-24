import cv2
import json
import numpy as np

# =========================
# CAMERA
# =========================
CAM_INDEX = 0
WIDTH = 640
HEIGHT = 480
FPS = 25
CALIB_JSON = "calibration/camera_calibration_charuco.json"

# =========================
# ARUCO
# =========================
# SUA CHO DUNG DICTIONARY CUA 2 MARKER BAN IN
DICT_ID = cv2.aruco.DICT_4X4_1000

# ID : kich thuoc that cua marker (met)
MARKERS = {
    150: 0.70,  # marker to, ví dụ 20 cm
    40: 0.08,
        30:0.08    # marker nhỏ, ví dụ 8 cm
}

# =========================
# LOAD CALIB
# =========================
with open(CALIB_JSON, "r", encoding="utf-8") as f:
    calib = json.load(f)

camera_matrix = np.array(calib["camera_matrix"], dtype=np.float64)
dist_coeffs = np.array(calib["dist_coeffs"], dtype=np.float64)

# =========================
# ARUCO DETECTOR
# =========================
aruco_dict = cv2.aruco.getPredefinedDictionary(DICT_ID)

if hasattr(cv2.aruco, "ArucoDetector"):
    detector_params = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(aruco_dict, detector_params)

    def detect_markers(gray):
        return detector.detectMarkers(gray)
else:
    detector_params = cv2.aruco.DetectorParameters_create()

    def detect_markers(gray):
        return cv2.aruco.detectMarkers(gray, aruco_dict, parameters=detector_params)

# =========================
# CAMERA OPEN
# =========================
cap = cv2.VideoCapture(CAM_INDEX, cv2.CAP_V4L2)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, WIDTH)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)
cap.set(cv2.CAP_PROP_FPS, FPS)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))

if not cap.isOpened():
    raise RuntimeError(f"Khong mo duoc camera index {CAM_INDEX}")

cv2.namedWindow("Two ArUco Test", cv2.WINDOW_NORMAL)

print("=" * 60)
print("TEST 2 MA ARUCO")
print(f"Dang theo doi cac ID: {list(MARKERS.keys())}")
print("q : thoat")
print("=" * 60)

try:
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Khong doc duoc frame")
            continue

        # Giam nhieu/gon song nhe
        frame = cv2.GaussianBlur(frame, (3, 3), 0)

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = detect_markers(gray)

        vis = frame.copy()

        h, w = gray.shape[:2]
        cx_img = w // 2
        cy_img = h // 2
        cv2.circle(vis, (cx_img, cy_img), 4, (255, 255, 255), -1)

        found_big = False
        found_small = False

        if ids is not None and len(ids) > 0:
            ids_flat = ids.flatten()
            cv2.aruco.drawDetectedMarkers(vis, corners, ids)

            for i, marker_id in enumerate(ids_flat):
                if marker_id not in MARKERS:
                    continue

                marker_length_m = MARKERS[marker_id]
                marker_corners = [corners[i]]

                rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                    marker_corners,
                    marker_length_m,
                    camera_matrix,
                    dist_coeffs
                )

                rvec = rvecs[0]
                tvec = tvecs[0]
                c = marker_corners[0][0]

                center_x = int(np.mean(c[:, 0]))
                center_y = int(np.mean(c[:, 1]))
                err_x = center_x - cx_img
                err_y = center_y - cy_img

                x = float(tvec[0][0])
                y = float(tvec[0][1])
                z = float(tvec[0][2])
                dist = float(np.linalg.norm(tvec[0]))

                cv2.drawFrameAxes(
                    vis,
                    camera_matrix,
                    dist_coeffs,
                    rvec,
                    tvec,
                    marker_length_m * 0.5
                )

                color = (0, 255, 0)
                label = f"ID={marker_id}"

                if marker_id == 150:
                    color = (0, 255, 255)
                    label += " BIG"
                    found_big = True
                elif marker_id == 40:
                    color = (255, 255, 0)
                    label += " SMALL"
                    found_small = True

                cv2.circle(vis, (center_x, center_y), 4, color, -1)
                cv2.putText(
                    vis,
                    label,
                    (center_x + 8, center_y - 28),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    color,
                    2
                )
                cv2.putText(
                    vis,
                    f"Dist={dist:.3f}m Z={z:.3f}m",
                    (center_x + 8, center_y - 6),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.55,
                    color,
                    2
                )
                cv2.putText(
                    vis,
                    f"err_x={err_x} err_y={err_y}",
                    (center_x + 8, center_y + 16),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    color,
                    2
                )
                cv2.putText(
                    vis,
                    f"X={x:.3f} Y={y:.3f}",
                    (center_x + 8, center_y + 36),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    color,
                    2
                )

            status = []
            status.append("BIG:OK" if found_big else "BIG:MISS")
            status.append("SMALL:OK" if found_small else "SMALL:MISS")

            cv2.putText(
                vis,
                " | ".join(status),
                (20, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 255, 0),
                2
            )
        else:
            cv2.putText(
                vis,
                "Khong detect duoc marker nao",
                (20, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 0, 255),
                2
            )

        cv2.imshow("Two ArUco Test", vis)
        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break

finally:
    cap.release()
    cv2.destroyAllWindows()
    print("Da thoat test 2 marker.")