import cv2
import glob
import json
import numpy as np
from pathlib import Path

# =========================
# THONG SO BOARD DA IN
# =========================
DICT_ID = cv2.aruco.DICT_4X4_1000
SQUARES_X = 7
SQUARES_Y = 5
SQUARE_LENGTH = 0.030   # 30 mm
MARKER_LENGTH = 0.022   # 22 mm
START_ID = 30

# Thu muc anh da chup
INPUT_DIR = "calibration_images"

# File ket qua
OUTPUT_YAML = "calibration/camera_calibration_charuco.yaml"
OUTPUT_JSON = "calibration/camera_calibration_charuco.json"

MIN_CHARUCO_CORNERS = 12


def detect_markers(gray, aruco_dict):
    if hasattr(cv2.aruco, "ArucoDetector"):
        detector_params = cv2.aruco.DetectorParameters()
        detector = cv2.aruco.ArucoDetector(aruco_dict, detector_params)
        corners, ids, rejected = detector.detectMarkers(gray)
    else:
        detector_params = cv2.aruco.DetectorParameters_create()
        corners, ids, rejected = cv2.aruco.detectMarkers(
            gray, aruco_dict, parameters=detector_params
        )
    return corners, ids, rejected


def create_charuco_board():
    aruco_dict = cv2.aruco.getPredefinedDictionary(DICT_ID)

    # So marker tren board 7x5
    num_markers = (SQUARES_X * SQUARES_Y) // 2
    ids = np.arange(START_ID, START_ID + num_markers, dtype=np.int32)

    if hasattr(cv2.aruco, "CharucoBoard"):
        try:
            board = cv2.aruco.CharucoBoard(
                (SQUARES_X, SQUARES_Y),
                SQUARE_LENGTH,
                MARKER_LENGTH,
                aruco_dict,
                ids
            )
            return aruco_dict, board
        except Exception:
            pass

    if hasattr(cv2.aruco, "CharucoBoard_create"):
        board = cv2.aruco.CharucoBoard_create(
            SQUARES_X,
            SQUARES_Y,
            SQUARE_LENGTH,
            MARKER_LENGTH,
            aruco_dict
        )
        if hasattr(board, "setIds"):
            board.setIds(ids)
        return aruco_dict, board

    raise RuntimeError("OpenCV hien tai khong ho tro CharucoBoard.")


def main():
    image_paths = sorted(glob.glob(f"{INPUT_DIR}/*.jpg"))
    if not image_paths:
        raise RuntimeError(f"Khong tim thay anh trong thu muc: {INPUT_DIR}")

    aruco_dict, board = create_charuco_board()

    all_charuco_corners = []
    all_charuco_ids = []
    image_size = None
    accepted = 0

    print(f"[INFO] Tim thay {len(image_paths)} anh")

    for img_path in image_paths:
        image = cv2.imread(img_path)
        if image is None:
            print(f"[WARN] Khong doc duoc: {img_path}")
            continue

        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        if image_size is None:
            image_size = (gray.shape[1], gray.shape[0])

        corners, ids, _ = detect_markers(gray, aruco_dict)

        if ids is None or len(ids) == 0:
            print(f"[SKIP] Khong thay marker: {img_path}")
            continue

        retval, charuco_corners, charuco_ids = cv2.aruco.interpolateCornersCharuco(
            markerCorners=corners,
            markerIds=ids,
            image=gray,
            board=board
        )

        if retval is None or retval < MIN_CHARUCO_CORNERS or charuco_ids is None:
            print(f"[SKIP] Qua it ChArUco corners ({0 if retval is None else int(retval)}): {img_path}")
            continue

        all_charuco_corners.append(charuco_corners)
        all_charuco_ids.append(charuco_ids)
        accepted += 1
        print(f"[OK] {img_path} -> corners = {int(retval)}")

    if accepted < 8:
        raise RuntimeError(f"Qua it anh hop le: {accepted}. Nen co it nhat 12-20 anh tot.")

    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_COUNT, 100, 1e-9)

    rms, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.aruco.calibrateCameraCharuco(
        charucoCorners=all_charuco_corners,
        charucoIds=all_charuco_ids,
        board=board,
        imageSize=image_size,
        cameraMatrix=None,
        distCoeffs=None,
        flags=0,
        criteria=criteria
    )

    print("\n===== KET QUA CALIB =====")
    print(f"RMS reprojection error: {rms:.6f}")
    print("Camera matrix:")
    print(camera_matrix)
    print("Distortion coefficients:")
    print(dist_coeffs.ravel())

    Path("calibration").mkdir(parents=True, exist_ok=True)

    fs = cv2.FileStorage(OUTPUT_YAML, cv2.FILE_STORAGE_WRITE)
    fs.write("image_width", int(image_size[0]))
    fs.write("image_height", int(image_size[1]))
    fs.write("camera_matrix", camera_matrix)
    fs.write("dist_coeffs", dist_coeffs)
    fs.write("rms", float(rms))
    fs.write("used_images", int(accepted))
    fs.release()

    with open(OUTPUT_JSON, "w", encoding="utf-8") as f:
        json.dump({
            "image_width": int(image_size[0]),
            "image_height": int(image_size[1]),
            "camera_matrix": camera_matrix.tolist(),
            "dist_coeffs": dist_coeffs.tolist(),
            "rms": float(rms),
            "used_images": int(accepted),
            "dictionary": "DICT_4X4_1000",
            "squares_x": SQUARES_X,
            "squares_y": SQUARES_Y,
            "square_length_m": SQUARE_LENGTH,
            "marker_length_m": MARKER_LENGTH,
            "start_id": START_ID
        }, f, indent=2)

    print(f"\nDa luu: {OUTPUT_YAML}")
    print(f"Da luu: {OUTPUT_JSON}")


if __name__ == "__main__":
    main()