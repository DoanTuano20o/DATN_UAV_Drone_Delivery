import cv2
import os
import datetime

SAVE_DIR = "calibration_images"
CAM_INDEX = 0          # doi thanh 1 hoac 2 neu cam khong len
WIDTH = 640
HEIGHT = 480
MAX_IMAGES = 40
MIN_MARKERS = 6
BLUR_THRESHOLD = 80.0

os.makedirs(SAVE_DIR, exist_ok=True)

# Dung dung board hien tai cua ban
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_1000)
detector_params = cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(aruco_dict, detector_params)

# Khoi tao USB camera
cap = cv2.VideoCapture(CAM_INDEX)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, WIDTH)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)

if not cap.isOpened():
    raise RuntimeError(f"Khong mo duoc camera index {CAM_INDEX}")

image_count = 0

print("=" * 60)
print("Chup anh ChArUco bang tay")
print("c : chup va luu anh")
print("q : thoat")
print("Meo: canh bang o nhieu goc khac nhau, nghieng, gan, xa, ra 4 goc khung hinh")
print("=" * 60)

try:
    while image_count < MAX_IMAGES:
        ret, frame = cap.read()
        if not ret:
            print("Khong doc duoc frame")
            continue

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = detector.detectMarkers(gray)

        marker_count = 0 if ids is None else len(ids)
        blur_score = cv2.Laplacian(gray, cv2.CV_64F).var()
        can_save = marker_count >= MIN_MARKERS and blur_score >= BLUR_THRESHOLD

        vis = frame.copy()
        if marker_count > 0:
            cv2.aruco.drawDetectedMarkers(vis, corners, ids)

        status = f"saved={image_count}/{MAX_IMAGES} | markers={marker_count} | sharp={blur_score:.1f}"
        cv2.putText(
            vis,
            status,
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 255, 0) if can_save else (0, 0, 255),
            2,
        )
        cv2.putText(
            vis,
            "Nhan c de chup | q de thoat",
            (10, 60),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (255, 255, 0),
            2,
        )

        cv2.imshow("USB Camera - ChArUco Capture", vis)
        key = cv2.waitKey(1) & 0xFF

        if key == ord("q"):
            break

        elif key == ord("c"):
            if can_save:
                timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S_%f")
                image_name = os.path.join(SAVE_DIR, f"image_{timestamp}.jpg")
                ok = cv2.imwrite(image_name, frame)
                if ok:
                    image_count += 1
                    print(f"Da luu: {image_name}")
                else:
                    print("Loi khi luu anh")
            else:
                print(f"Khung hinh chua dat. markers={marker_count}, sharpness={blur_score:.1f}")

finally:
    cap.release()
    cv2.destroyAllWindows()
    print("Hoan thanh chup anh calibration.")