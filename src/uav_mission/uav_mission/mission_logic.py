from __future__ import annotations

import math


def haversine_distance_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    r = 6371000.0

    p1 = math.radians(lat1)
    p2 = math.radians(lat2)
    dp = math.radians(lat2 - lat1)
    dl = math.radians(lon2 - lon1)

    a = math.sin(dp / 2.0) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2.0) ** 2
    c = 2.0 * math.atan2(math.sqrt(a), math.sqrt(1.0 - a))
    return r * c


def select_detection_by_id(detections: list[dict], marker_id: int) -> dict | None:
    candidates = [d for d in detections if int(d.get("marker_id", -1)) == int(marker_id)]
    if not candidates:
        return None

    # Chọn marker gần tâm ảnh nhất
    def score(item: dict) -> float:
        ex = float(item.get("err_x", 999999))
        ey = float(item.get("err_y", 999999))
        return ex * ex + ey * ey

    return min(candidates, key=score)


def is_centered(det: dict | None, tol_px: float) -> bool:
    if det is None:
        return False

    ex = abs(float(det.get("err_x", 999999)))
    ey = abs(float(det.get("err_y", 999999)))
    return ex <= tol_px and ey <= tol_px