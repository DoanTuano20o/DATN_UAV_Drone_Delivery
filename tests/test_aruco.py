import cv2
import json
import numpy as np

# =========================
# CAMERA + CALIB
# =========================
CAM_INDEX = 0
WIDTH = 640
HEIGHT = 480
CALIB_JSON = "calibration/camera_calibration_charuco.json"

# =========================
# ARUCO TEST MARKER
# =========================
# PDF ban gui cho thay ID = 30
TARGET_ID =150

# SUA 2 DONG DUOI DAY CHO DUNG VOI MARKER BAN DA TAO
DICT_ID = cv2.aruco.DICT_4X4_1000   # vi du: DICT_4X4_50, DICT_4X4_1000...
MARKER_LENGTH_M = 0.70              # vi du: 0.20 neu marker canh 20 cm

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
# CAMERA
# =========================
cap = cv2.VideoCapture(CAM_INDEX, cv2.CAP_V4L2)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, WIDTH)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))

if not cap.isOpened():
    raise RuntimeError(f"Khong mo duoc camera index {CAM_INDEX}")

cv2.namedWindow("ArUco Test", cv2.WINDOW_NORMAL)

print("=" * 60)
print("TEST DETECT ARUCO")
print(f"TARGET_ID = {TARGET_ID}")
print("q : thoat")
print("=" * 60)

try:
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Khong doc duoc frame")
            continue

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = detect_markers(gray)

        vis = frame.copy()

        if ids is not None and len(ids) > 0:
            ids_flat = ids.flatten()
            cv2.aruco.drawDetectedMarkers(vis, corners, ids)

            # chi xu ly marker dung TARGET_ID
            target_indices = [i for i, mid in enumerate(ids_flat) if mid == TARGET_ID]

            if len(target_indices) > 0:
                target_corners = [corners[i] for i in target_indices]

                rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                    target_corners,
                    MARKER_LENGTH_M,
                    camera_matrix,
                    dist_coeffs
                )

                for j, idx in enumerate(target_indices):
                    rvec = rvecs[j]
                    tvec = tvecs[j]

                    # ve truc toa do
                    cv2.drawFrameAxes(
                        vis,
                        camera_matrix,
                        dist_coeffs,
                        rvec,
                        tvec,
                        MARKER_LENGTH_M * 0.5
                    )

                    # tinh tam marker
                    c = target_corners[j][0]
                    center_x = int(np.mean(c[:, 0]))
                    center_y = int(np.mean(c[:, 1]))

                    # khoang cach xap xi
                    distance_m = float(np.linalg.norm(tvec[0]))

                    # hien thi thong tin
                    text1 = f"ID={TARGET_ID}"
                    text2 = f"Dist={distance_m:.3f} m"
                    text3 = f"X={tvec[0][0]:.3f}  Y={tvec[0][1]:.3f}  Z={tvec[0][2]:.3f}"

                    cv2.circle(vis, (center_x, center_y), 4, (0, 255, 255), -1)
                    cv2.putText(vis, text1, (center_x + 10, center_y - 20),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                    cv2.putText(vis, text2, (center_x + 10, center_y + 5),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                    cv2.putText(vis, text3, (20, 30),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
            else:
                cv2.putText(vis, f"Khong thay TARGET_ID={TARGET_ID}", (20, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        else:
            cv2.putText(vis, "Khong detect duoc marker", (20, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

        cv2.imshow("ArUco Test", vis)
        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break

finally:
    cap.release()
    cv2.destroyAllWindows()
    print("Da thoat test ArUco.")