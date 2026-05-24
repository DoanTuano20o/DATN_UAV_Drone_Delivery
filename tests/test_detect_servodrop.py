#!/usr/bin/env python3
import cv2
import json
import numpy as np
import time
import threading
from smbus2 import SMBus

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
DICT_ID = cv2.aruco.DICT_4X4_1000
MARKERS = {
    150: 0.20,   # marker to, ví dụ 20 cm
    40: 0.08,    # marker nhỏ, ví dụ 8 cm
}
TARGET_ID = 150

# =========================
# LOCK / TRACK
# =========================
LOCK_FRAMES = 5
MISS_HOLD_FRAMES = 2        # mất 1-2 frame vẫn chưa reset lock ngay
CENTER_TOL_PX = None        # ví dụ 80 nếu muốn thêm điều kiện gần tâm
REARM_AFTER_DROP = False
ROI_SEARCH_SCALE = 3.0      # ROI quanh target cũ
ROI_UPSCALE = 2.5           # phóng to ROI để detect lại
POSE_EMA_ALPHA = 0.35       # làm mượt pose hiển thị
MIN_TARGET_PERIM_PX = 40.0  # bỏ qua marker quá bé / quá mờ

# =========================
# SERVO / PCA9685
# =========================
BUS_ID = 5
I2C_ADDRESS = 0x40
SERVO_CHANNEL = 3
PWM_FREQUENCY = 50

SERVO_MIN_US = 500
SERVO_MAX_US = 2500
SERVO_CLOSE_ANGLE = 0
SERVO_OPEN_ANGLE = 38
SERVO_OPEN_HOLD_S = 5.0

PCA9685_MODE1 = 0x00
PCA9685_PRESCALE = 0xFE
LED0_ON_L = 0x06


class PCA9685:
    def __init__(self, bus_id=BUS_ID, address=I2C_ADDRESS, frequency=PWM_FREQUENCY):
        self.bus_id = bus_id
        self.address = address
        self.frequency = frequency
        self.bus = SMBus(bus_id)

        self.write8(PCA9685_MODE1, 0x00)
        time.sleep(0.01)
        self.set_pwm_freq(frequency)

    def write8(self, reg, value):
        self.bus.write_byte_data(self.address, reg, value & 0xFF)

    def read8(self, reg):
        return self.bus.read_byte_data(self.address, reg)

    def set_pwm_freq(self, freq_hz):
        prescaleval = 25000000.0 / 4096.0 / float(freq_hz) - 1.0
        prescale = int(prescaleval + 0.5)

        oldmode = self.read8(PCA9685_MODE1)
        sleep_mode = (oldmode & 0x7F) | 0x10

        self.write8(PCA9685_MODE1, sleep_mode)
        self.write8(PCA9685_PRESCALE, prescale)
        self.write8(PCA9685_MODE1, oldmode)
        time.sleep(0.005)
        self.write8(PCA9685_MODE1, oldmode | 0xA1)

    def set_pwm(self, channel, on_tick, off_tick):
        reg = LED0_ON_L + 4 * channel
        self.write8(reg + 0, on_tick & 0xFF)
        self.write8(reg + 1, (on_tick >> 8) & 0xFF)
        self.write8(reg + 2, off_tick & 0xFF)
        self.write8(reg + 3, (off_tick >> 8) & 0xFF)

    def release_channel(self, channel):
        self.set_pwm(channel, 0, 0)

    def close(self):
        self.bus.close()


class ServoDropper:
    def __init__(
        self,
        bus_id=BUS_ID,
        address=I2C_ADDRESS,
        channel=SERVO_CHANNEL,
        min_us=SERVO_MIN_US,
        max_us=SERVO_MAX_US,
        close_angle=SERVO_CLOSE_ANGLE,
        open_angle=SERVO_OPEN_ANGLE,
    ):
        self.channel = channel
        self.min_us = min_us
        self.max_us = max_us
        self.close_angle = close_angle
        self.open_angle = open_angle
        self.pca = PCA9685(bus_id=bus_id, address=address, frequency=PWM_FREQUENCY)

    def angle_to_ticks(self, angle_deg):
        angle_deg = max(0.0, min(180.0, float(angle_deg)))
        pulse_us = self.min_us + (self.max_us - self.min_us) * (angle_deg / 180.0)
        period_us = 1_000_000.0 / PWM_FREQUENCY
        ticks = int((pulse_us / period_us) * 4096)
        return max(0, min(4095, ticks))

    def set_angle(self, angle_deg):
        ticks = self.angle_to_ticks(angle_deg)
        self.pca.set_pwm(self.channel, 0, ticks)
        print(f"[servo] ch={self.channel} angle={angle_deg:.1f} ticks={ticks}")

    def open(self):
        self.set_angle(self.open_angle)

    def close_servo(self):
        self.set_angle(self.close_angle)

    def drop(self, hold_s=SERVO_OPEN_HOLD_S):
        print(f"[dropper] OPEN {self.open_angle} deg")
        self.open()
        time.sleep(hold_s)
        print(f"[dropper] CLOSE {self.close_angle} deg")
        self.close_servo()

    def cleanup(self, release_pwm=False):
        try:
            if release_pwm:
                self.pca.release_channel(self.channel)
        finally:
            self.pca.close()


# =========================
# LOAD CALIB
# =========================
with open(CALIB_JSON, "r", encoding="utf-8") as f:
    calib = json.load(f)

camera_matrix = np.array(calib["camera_matrix"], dtype=np.float64)
dist_coeffs = np.array(calib["dist_coeffs"], dtype=np.float64)

# precompute undistort map
new_camera_matrix, _ = cv2.getOptimalNewCameraMatrix(
    camera_matrix, dist_coeffs, (WIDTH, HEIGHT), 1, (WIDTH, HEIGHT)
)
map1, map2 = cv2.initUndistortRectifyMap(
    camera_matrix, dist_coeffs, None, new_camera_matrix, (WIDTH, HEIGHT), cv2.CV_16SC2
)

pose_camera_matrix = new_camera_matrix
pose_dist_coeffs = np.zeros_like(dist_coeffs)

# =========================
# DETECTOR
# =========================
aruco_dict = cv2.aruco.getPredefinedDictionary(DICT_ID)

if hasattr(cv2.aruco, "DetectorParameters"):
    detector_params = cv2.aruco.DetectorParameters()
else:
    detector_params = cv2.aruco.DetectorParameters_create()

# tinh chỉnh cho marker nhỏ
if hasattr(detector_params, "cornerRefinementMethod"):
    detector_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
if hasattr(detector_params, "cornerRefinementWinSize"):
    detector_params.cornerRefinementWinSize = 5
if hasattr(detector_params, "cornerRefinementMaxIterations"):
    detector_params.cornerRefinementMaxIterations = 50
if hasattr(detector_params, "cornerRefinementMinAccuracy"):
    detector_params.cornerRefinementMinAccuracy = 0.01

if hasattr(detector_params, "adaptiveThreshWinSizeMin"):
    detector_params.adaptiveThreshWinSizeMin = 3
if hasattr(detector_params, "adaptiveThreshWinSizeMax"):
    detector_params.adaptiveThreshWinSizeMax = 31
if hasattr(detector_params, "adaptiveThreshWinSizeStep"):
    detector_params.adaptiveThreshWinSizeStep = 4

if hasattr(detector_params, "minMarkerPerimeterRate"):
    detector_params.minMarkerPerimeterRate = 0.01
if hasattr(detector_params, "maxMarkerPerimeterRate"):
    detector_params.maxMarkerPerimeterRate = 4.0
if hasattr(detector_params, "polygonalApproxAccuracyRate"):
    detector_params.polygonalApproxAccuracyRate = 0.05
if hasattr(detector_params, "minDistanceToBorder"):
    detector_params.minDistanceToBorder = 2

if hasattr(cv2.aruco, "ArucoDetector"):
    detector = cv2.aruco.ArucoDetector(aruco_dict, detector_params)

    def detect_markers(gray):
        return detector.detectMarkers(gray)
else:
    def detect_markers(gray):
        return cv2.aruco.detectMarkers(gray, aruco_dict, parameters=detector_params)


clahe = cv2.createCLAHE(clipLimit=2.5, tileGridSize=(8, 8))


def load_camera():
    cap = cv2.VideoCapture(CAM_INDEX, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)
    cap.set(cv2.CAP_PROP_FPS, FPS)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    if not cap.isOpened():
        raise RuntimeError(f"Khong mo duoc camera index {CAM_INDEX}")
    return cap


def preprocess_frame(frame):
    undist = cv2.remap(frame, map1, map2, interpolation=cv2.INTER_LINEAR)
    gray = cv2.cvtColor(undist, cv2.COLOR_BGR2GRAY)

    eq = clahe.apply(gray)
    med = cv2.medianBlur(eq, 3)

    # unsharp nhe de giu canh marker
    blur = cv2.GaussianBlur(med, (0, 0), 1.0)
    sharp = cv2.addWeighted(med, 1.6, blur, -0.6, 0)

    return undist, {
        "gray": gray,
        "eq": eq,
        "sharp": sharp,
    }


def corner_perimeter(corner):
    pts = corner[0]
    peri = 0.0
    for i in range(4):
        p1 = pts[i]
        p2 = pts[(i + 1) % 4]
        peri += np.linalg.norm(p1 - p2)
    return float(peri)


def detect_multi_pass(gray_variants, wanted_ids=None):
    detections = {}

    for src_name, img in gray_variants.items():
        corners, ids, _ = detect_markers(img)
        if ids is None or len(ids) == 0:
            continue

        for i, mid in enumerate(ids.flatten()):
            if mid not in MARKERS:
                continue
            if wanted_ids is not None and mid not in wanted_ids:
                continue

            corner = corners[i].copy()
            score = corner_perimeter(corner)

            if mid not in detections or score > detections[mid]["score"]:
                detections[mid] = {
                    "corner": corner,
                    "score": score,
                    "src": src_name,
                }

    return detections


def corner_bbox(corner):
    pts = corner[0]
    x0 = int(np.min(pts[:, 0]))
    y0 = int(np.min(pts[:, 1]))
    x1 = int(np.max(pts[:, 0]))
    y1 = int(np.max(pts[:, 1]))
    return x0, y0, x1, y1


def make_roi(corner, img_w, img_h, scale=3.0):
    x0, y0, x1, y1 = corner_bbox(corner)
    cx = 0.5 * (x0 + x1)
    cy = 0.5 * (y0 + y1)
    bw = max(1, x1 - x0)
    bh = max(1, y1 - y0)

    half_w = 0.5 * bw * scale
    half_h = 0.5 * bh * scale

    rx0 = max(0, int(cx - half_w))
    ry0 = max(0, int(cy - half_h))
    rx1 = min(img_w, int(cx + half_w))
    ry1 = min(img_h, int(cy + half_h))
    return rx0, ry0, rx1, ry1


def detect_target_in_roi(gray_variants, last_corner, img_w, img_h):
    rx0, ry0, rx1, ry1 = make_roi(last_corner, img_w, img_h, ROI_SEARCH_SCALE)
    if rx1 <= rx0 or ry1 <= ry0:
        return None, None

    best = None
    best_score = -1.0

    for src_name, img in gray_variants.items():
        roi = img[ry0:ry1, rx0:rx1]
        if roi.size == 0:
            continue

        up = cv2.resize(roi, None, fx=ROI_UPSCALE, fy=ROI_UPSCALE, interpolation=cv2.INTER_CUBIC)
        corners, ids, _ = detect_markers(up)

        if ids is None or len(ids) == 0:
            continue

        for i, mid in enumerate(ids.flatten()):
            if mid != TARGET_ID:
                continue

            c = corners[i].copy()
            c[:, :, 0] = c[:, :, 0] / ROI_UPSCALE + rx0
            c[:, :, 1] = c[:, :, 1] / ROI_UPSCALE + ry0

            score = corner_perimeter(c)
            if score > best_score:
                best_score = score
                best = {
                    "corner": c,
                    "score": score,
                    "src": f"roi_{src_name}",
                }

    return best, (rx0, ry0, rx1, ry1)


def estimate_pose(corner, marker_length_m):
    marker_corners = np.array([corner[0]], dtype=np.float32)
    rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
        marker_corners,
        marker_length_m,
        pose_camera_matrix,
        pose_dist_coeffs,
    )
    rvec = np.asarray(rvecs[0]).reshape(3)
    tvec = np.asarray(tvecs[0]).reshape(3)
    return rvec, tvec


def ema_update(state, key, vec, alpha=0.35):
    vec = np.asarray(vec, dtype=np.float64).reshape(-1)
    if key not in state:
        state[key] = vec.copy()
    else:
        state[key] = alpha * vec + (1.0 - alpha) * state[key]
    return state[key]


def draw_marker_info(vis, marker_id, corner, rvec, tvec, color, cx_img, cy_img, src_name):
    pts = corner[0]
    center_x = int(np.mean(pts[:, 0]))
    center_y = int(np.mean(pts[:, 1]))
    err_x = center_x - cx_img
    err_y = center_y - cy_img

    x, y, z = [float(v) for v in tvec]
    dist = float(np.linalg.norm(tvec))

    cv2.drawFrameAxes(
        vis,
        pose_camera_matrix,
        pose_dist_coeffs,
        rvec.reshape(3, 1),
        tvec.reshape(3, 1),
        MARKERS[marker_id] * 0.5,
    )

    cv2.circle(vis, (center_x, center_y), 4, color, -1)
    cv2.putText(vis, f"ID={marker_id} src={src_name}", (center_x + 8, center_y - 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2)
    cv2.putText(vis, f"Dist={dist:.3f}m Z={z:.3f}m", (center_x + 8, center_y - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.50, color, 2)
    cv2.putText(vis, f"err_x={err_x} err_y={err_y}", (center_x + 8, center_y + 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.48, color, 2)

    return center_x, center_y, err_x, err_y, dist, z


def main():
    cap = load_camera()
    dropper = None
    drop_in_progress = False
    drop_done = False
    consec_target = 0
    target_miss_frames = 0
    last_target_corner = None
    pose_ema = {}

    try:
        dropper = ServoDropper()
        dropper.close_servo()
        print(f"[config] servo close={SERVO_CLOSE_ANGLE} open={SERVO_OPEN_ANGLE} hold={SERVO_OPEN_HOLD_S}s")
    except Exception as e:
        print(f"[WARN] Khoi tao servo that bai: {e}")
        print("[WARN] Van tiep tuc detect, nhung se khong drop duoc")

    cv2.namedWindow("ArUco Servo Lock Denoise", cv2.WINDOW_NORMAL)

    print("=" * 72)
    print("TEST DETECT ARUCO + SERVO + DENOISE / ROI TRACK")
    print(f"TARGET_ID = {TARGET_ID}")
    print(f"LOCK_FRAMES = {LOCK_FRAMES}")
    print(f"MISS_HOLD_FRAMES = {MISS_HOLD_FRAMES}")
    print("r : re-arm sau khi da drop")
    print("q : thoat")
    print("=" * 72)

    def do_drop():
        nonlocal drop_in_progress, drop_done, consec_target
        try:
            if dropper is not None:
                dropper.drop(SERVO_OPEN_HOLD_S)
                drop_done = True
        finally:
            drop_in_progress = False
            consec_target = 0

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                print("Khong doc duoc frame")
                continue

            undist, variants = preprocess_frame(frame)
            vis = undist.copy()
            h, w = vis.shape[:2]

            cx_img = w // 2
            cy_img = h // 2
            cv2.circle(vis, (cx_img, cy_img), 4, (255, 255, 255), -1)

            detections = detect_multi_pass(variants)

            roi_box = None
            if TARGET_ID not in detections and last_target_corner is not None and target_miss_frames <= MISS_HOLD_FRAMES:
                roi_det, roi_box = detect_target_in_roi(variants, last_target_corner, w, h)
                if roi_det is not None:
                    detections[TARGET_ID] = roi_det

            if roi_box is not None:
                rx0, ry0, rx1, ry1 = roi_box
                cv2.rectangle(vis, (rx0, ry0), (rx1, ry1), (255, 0, 255), 1)
                cv2.putText(vis, "ROI target search", (rx0, max(18, ry0 - 5)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 0, 255), 1)

            if len(detections) > 0:
                draw_corners = []
                draw_ids = []
                for mid, data in detections.items():
                    draw_corners.append(data["corner"])
                    draw_ids.append([mid])
                cv2.aruco.drawDetectedMarkers(vis, draw_corners, np.array(draw_ids, dtype=np.int32))

            target_seen_this_frame = False
            target_center_ok = True

            for marker_id, data in detections.items():
                corner = data["corner"]
                score = data["score"]
                src_name = data["src"]

                if marker_id == TARGET_ID and score < MIN_TARGET_PERIM_PX:
                    continue

                rvec, tvec = estimate_pose(corner, MARKERS[marker_id])
                tvec_smooth = ema_update(pose_ema, marker_id, tvec, POSE_EMA_ALPHA)

                color = (0, 255, 0)
                if marker_id == TARGET_ID:
                    color = (0, 255, 255)
                elif marker_id == 40:
                    color = (255, 255, 0)

                center_x, center_y, err_x, err_y, dist, z = draw_marker_info(
                    vis, marker_id, corner, rvec, tvec_smooth, color, cx_img, cy_img, src_name
                )

                if marker_id == TARGET_ID:
                    target_seen_this_frame = True
                    last_target_corner = corner.copy()
                    target_miss_frames = 0

                    if CENTER_TOL_PX is not None:
                        target_center_ok = abs(err_x) <= CENTER_TOL_PX and abs(err_y) <= CENTER_TOL_PX
                        if not target_center_ok:
                            cv2.putText(vis, "TARGET seen but out of center tolerance",
                                        (20, 95), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

            if not target_seen_this_frame:
                target_miss_frames += 1
                if target_miss_frames > MISS_HOLD_FRAMES:
                    last_target_corner = None
                    if not drop_in_progress:
                        consec_target = 0

            if target_seen_this_frame and target_center_ok and not drop_in_progress and not drop_done:
                consec_target += 1

            if consec_target >= LOCK_FRAMES and not drop_in_progress and not drop_done:
                if dropper is not None:
                    print(f"[LOCK] TARGET_ID={TARGET_ID} thay on dinh -> DROP")
                    drop_in_progress = True
                    threading.Thread(target=do_drop, daemon=True).start()
                else:
                    print("[WARN] Da lock target nhung servo chua san sang")
                    drop_done = True

            status1 = f"target={TARGET_ID} consec={consec_target}/{LOCK_FRAMES}"
            status2 = f"miss_hold={target_miss_frames}/{MISS_HOLD_FRAMES} drop={drop_in_progress} done={drop_done}"
            cv2.putText(vis, status1, (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                        (0, 255, 0) if consec_target > 0 else (0, 0, 255), 2)
            cv2.putText(vis, status2, (20, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.65,
                        (255, 255, 0), 2)

            if drop_in_progress:
                cv2.putText(vis, "DROPPING...", (20, 125), cv2.FONT_HERSHEY_SIMPLEX, 0.95, (0, 255, 255), 3)
            elif drop_done:
                cv2.putText(vis, "DROP DONE", (20, 125), cv2.FONT_HERSHEY_SIMPLEX, 0.95, (0, 255, 0), 3)

            cv2.imshow("ArUco Servo Lock Denoise", vis)
            key = cv2.waitKey(1) & 0xFF

            if key == ord("q"):
                break
            elif key == ord("r"):
                if not drop_in_progress:
                    drop_done = False
                    consec_target = 0
                    target_miss_frames = 0
                    last_target_corner = None
                    print("[INFO] Re-armed. Co the drop lai.")
                    if dropper is not None:
                        dropper.close_servo()

            if REARM_AFTER_DROP and drop_done and not drop_in_progress:
                drop_done = False

    finally:
        cap.release()
        cv2.destroyAllWindows()
        if dropper is not None:
            dropper.cleanup(release_pwm=False)
        print("Da thoat test detect + servo denoise.")


if __name__ == "__main__":
    main()