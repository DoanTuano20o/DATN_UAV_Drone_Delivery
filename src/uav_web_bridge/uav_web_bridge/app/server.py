from __future__ import annotations

import hmac
import json
import os
import threading
import time
from functools import wraps
from pathlib import Path

from flask import Flask, Response, jsonify, redirect, render_template, request, session, url_for
from flask_socketio import SocketIO

APP_DIR = Path(__file__).resolve().parent
WORKSPACE_ROOT = Path(__file__).resolve().parents[4]

DATA_DIR = WORKSPACE_ROOT / "data" / "missions"
GPS_STATIONS_FILE = DATA_DIR / "gps_stations.json"

app = Flask(
    __name__,
    template_folder=str(APP_DIR / "templates"),
    static_folder=str(APP_DIR / "static"),
)

ADMIN_USERNAME = os.environ.get("UAV_ADMIN_USERNAME", "admin")
ADMIN_PASSWORD = os.environ.get("UAV_ADMIN_PASSWORD", "Drone111")
SECRET_KEY = os.environ.get("UAV_WEB_SECRET_KEY", "dev-uav-web-secret-change-me")
COOKIE_SECURE = os.environ.get("UAV_COOKIE_SECURE", "").strip().lower() in {"1", "true", "yes", "on"}

app.config.update(
    SECRET_KEY=SECRET_KEY,
    SESSION_COOKIE_HTTPONLY=True,
    SESSION_COOKIE_SAMESITE="Lax",
    SESSION_COOKIE_SECURE=COOKIE_SECURE,
)

if "UAV_ADMIN_USERNAME" not in os.environ or "UAV_ADMIN_PASSWORD" not in os.environ:
    app.logger.warning(
        "WARNING: Using default admin credentials. Change UAV_ADMIN_USERNAME and UAV_ADMIN_PASSWORD before public deployment."
    )

if "UAV_WEB_SECRET_KEY" not in os.environ:
    app.logger.warning("WARNING: Using development UAV_WEB_SECRET_KEY fallback. Set UAV_WEB_SECRET_KEY before public deployment.")

socketio = SocketIO(app, cors_allowed_origins="*", async_mode="threading")

telemetry_lock = threading.Lock()
aruco_lock = threading.Lock()
vision_lock = threading.Lock()
mission_lock = threading.Lock()

_background_started = False
_last_marker_signature = None

_publish_mission_goal_cb = None
_publish_mission_control_cb = None


def make_default_telemetry_state():
    return {
        "connected": False,
        "mode": "DISCONNECTED",
        "armed": False,
        "battery_v": None,
        "current_a": None,
        "gps_fix": None,
        "satellites": None,
        "rel_alt_m": None,
        "speed_mps": None,
        "roll_deg": None,
        "pitch_deg": None,
        "yaw_deg": None,
        "heading_deg": None,
        "lat": None,
        "lon": None,
        "error": None,
        "last_msg": None,
        "last_update_unix": None,
    }


telemetry_state = make_default_telemetry_state()

aruco_state = {
    "markers": [],
}

vision_state = {
    "frame_jpeg": None,
    "last_frame_unix": None,
}

VIDEO_STREAM_TARGET_FPS = 15.0
VIDEO_STREAM_MIN_INTERVAL = 1.0 / VIDEO_STREAM_TARGET_FPS
VIDEO_STREAM_KEEPALIVE_INTERVAL = 1.0

mission_state = {
    "state": "N/A",
    "goal": None,
    "mode": None,
    "armed": None,

    "big_marker_seen": False,
    "big_marker_id": 150,
    "big_marker": None,

    "small_marker_seen": False,
    "small_marker_id": 40,
    "small_marker": None,
}


def register_mission_publishers(goal_cb=None, control_cb=None):
    global _publish_mission_goal_cb, _publish_mission_control_cb
    _publish_mission_goal_cb = goal_cb
    _publish_mission_control_cb = control_cb


def update_telemetry_state_from_ros(payload: dict) -> None:
    global telemetry_state

    new_state = make_default_telemetry_state()
    new_state.update(payload or {})

    with telemetry_lock:
        telemetry_state = new_state
        snapshot = dict(telemetry_state)

    socketio.emit("telemetry", snapshot)


def _build_marker_signature(markers: list[dict]) -> tuple:
    signature = []
    for d in markers:
        signature.append(
            (
                int(d.get("marker_id", d.get("id", -1))),
                int(d.get("center_x", 0)),
                int(d.get("center_y", 0)),
            )
        )
    return tuple(sorted(signature))


def update_aruco_state_from_ros(payload: dict) -> None:
    global _last_marker_signature

    markers = list((payload or {}).get("detections", []))
    signature = _build_marker_signature(markers)

    with aruco_lock:
        aruco_state["markers"] = markers

    if signature != _last_marker_signature:
        socketio.emit("aruco_markers", markers)
        _last_marker_signature = signature


def update_annotated_frame_from_ros(frame_jpeg: bytes) -> None:
    if not frame_jpeg:
        return

    with vision_lock:
        vision_state["frame_jpeg"] = frame_jpeg
        vision_state["last_frame_unix"] = time.time()


def update_mission_state_from_ros(payload: dict) -> None:
    global mission_state

    with mission_lock:
        mission_state = dict(payload or {})
        snapshot = dict(mission_state)

    socketio.emit("mission_state", snapshot)


def load_gps_stations():
    if not GPS_STATIONS_FILE.exists():
        return []

    with GPS_STATIONS_FILE.open("r", encoding="utf-8") as f:
        return json.load(f)


def generate_video_stream():
    last_sent_stamp = None
    last_sent_mono = 0.0

    try:
        while True:
            with vision_lock:
                frame = vision_state["frame_jpeg"]
                frame_stamp = vision_state["last_frame_unix"]

            if frame is None:
                time.sleep(0.08)
                continue

            now = time.monotonic()
            elapsed = now - last_sent_mono
            same_frame = frame_stamp == last_sent_stamp

            if elapsed < VIDEO_STREAM_MIN_INTERVAL:
                time.sleep(min(VIDEO_STREAM_MIN_INTERVAL - elapsed, 0.03))
                continue

            if same_frame and elapsed < VIDEO_STREAM_KEEPALIVE_INTERVAL:
                time.sleep(0.03)
                continue

            last_sent_stamp = frame_stamp
            last_sent_mono = time.monotonic()

            yield (
                b"--frame\r\n"
                b"Content-Type: image/jpeg\r\n"
                b"Cache-Control: no-cache\r\n"
                b"Content-Length: " + str(len(frame)).encode("ascii") + b"\r\n\r\n" +
                frame +
                b"\r\n"
            )
    except GeneratorExit:
        return


def ensure_background_started():
    global _background_started
    if _background_started:
        return
    _background_started = True


def is_admin_authenticated():
    return bool(session.get("admin_authenticated"))


def is_safe_next_path(next_path: str | None) -> bool:
    if not next_path:
        return False
    return next_path.startswith("/") and not next_path.startswith("//") and "\r" not in next_path and "\n" not in next_path


def admin_required(view):
    @wraps(view)
    def wrapped(*args, **kwargs):
        if is_admin_authenticated():
            return view(*args, **kwargs)

        if request.path.startswith("/api/") or request.method != "GET":
            return jsonify({"ok": False, "error": "admin_auth_required"}), 401

        next_path = request.full_path if request.query_string else request.path
        return redirect(url_for("admin_login", next=next_path))

    return wrapped


@app.route("/admin/login", methods=["GET", "POST"])
def admin_login():
    next_path = request.values.get("next") or "/dashboard"
    if not is_safe_next_path(next_path):
        next_path = "/dashboard"

    if is_admin_authenticated():
        return redirect(next_path)

    error = ""
    username = ""

    if request.method == "POST":
        username = (request.form.get("username") or "").strip()
        password = request.form.get("password") or ""

        if not username or not password:
            error = "Vui lòng nhập đầy đủ tài khoản và mật khẩu."
        elif hmac.compare_digest(username, ADMIN_USERNAME) and hmac.compare_digest(password, ADMIN_PASSWORD):
            session.clear()
            session["admin_authenticated"] = True
            session["admin_username"] = username
            return redirect(next_path)
        else:
            error = "Mật khẩu hoặc tài khoản không đúng."

    return render_template("login.html", error=error, username=username, next_path=next_path)


@app.route("/admin/logout")
def admin_logout():
    session.clear()
    return redirect(url_for("index"))


@app.route("/")
def index():
    ensure_background_started()
    return render_template("landing.html")


@app.route("/dashboard")
@admin_required
def dashboard():
    ensure_background_started()
    return render_template("dashboard.html")


@app.route("/tracking")
def tracking():
    ensure_background_started()
    return render_template("tracking.html")


@app.route("/legacy")
@admin_required
def legacy_dashboard():
    ensure_background_started()
    return render_template("dashboard.html")


@app.route("/health")
def health():
    with telemetry_lock:
        connected = telemetry_state["connected"]

    return jsonify(
        {
            "status": "ok",
            "service": "DATN_UAV web",
            "fc_connected": connected,
        }
    )


@app.route("/api/gps-stations")
def get_gps_stations():
    return jsonify(load_gps_stations())


@app.route("/api/aruco-markers")
def get_aruco_markers():
    with aruco_lock:
        markers = list(aruco_state["markers"])
    return jsonify(markers)


@app.route("/api/telemetry")
def get_telemetry():
    with telemetry_lock:
        snapshot = dict(telemetry_state)
    return jsonify(snapshot)


@app.route("/api/drone-position")
def get_drone_position():
    with telemetry_lock:
        lat = telemetry_state["lat"]
        lon = telemetry_state["lon"]
        mode = telemetry_state["mode"]
        armed = telemetry_state["armed"]
        rel_alt_m = telemetry_state["rel_alt_m"]
        satellites = telemetry_state["satellites"]

    if lat is None or lon is None:
        return jsonify({"ok": False, "lat": None, "lon": None})

    return jsonify(
        {
            "ok": True,
            "lat": lat,
            "lon": lon,
            "mode": mode,
            "armed": armed,
            "rel_alt_m": rel_alt_m,
            "satellites": satellites,
        }
    )


@app.route("/api/mission/state")
def api_get_mission_state():
    with mission_lock:
        snapshot = dict(mission_state)
    return jsonify(snapshot)


@app.route("/api/mission/goal", methods=["POST"])
@admin_required
def api_set_mission_goal():
    data = request.get_json(silent=True) or {}

    try:
        lat = float(data["lat"])
        lon = float(data["lon"])
        search_alt_m = float(data.get("search_alt_m", 4.0))
        small_alt_m = float(data.get("small_alt_m", 2.0))
    except Exception as e:
        return jsonify({"ok": False, "error": f"bad payload: {e}"}), 400

    payload = {
        "lat": lat,
        "lon": lon,
        "search_alt_m": search_alt_m,
        "small_alt_m": small_alt_m,
    }

    if _publish_mission_goal_cb is None:
        return jsonify({"ok": False, "error": "mission goal publisher not ready"}), 503

    _publish_mission_goal_cb(payload)
    return jsonify({"ok": True, "goal": payload})


@app.route("/api/mission/control", methods=["POST"])
@admin_required
def api_mission_control():
    data = request.get_json(silent=True) or {}

    try:
        cmd = str(data["cmd"]).strip().upper()
    except Exception as e:
        return jsonify({"ok": False, "error": f"bad payload: {e}"}), 400

    if _publish_mission_control_cb is None:
        return jsonify({"ok": False, "error": "mission control publisher not ready"}), 503

    _publish_mission_control_cb(cmd)
    return jsonify({"ok": True, "cmd": cmd})


@app.route("/video_feed")
@admin_required
def video_feed():
    response = Response(
        generate_video_stream(),
        mimetype="multipart/x-mixed-replace; boundary=frame",
    )
    response.headers["Cache-Control"] = "no-cache, no-store, must-revalidate"
    response.headers["Pragma"] = "no-cache"
    response.headers["Expires"] = "0"
    response.headers["X-Accel-Buffering"] = "no"
    return response


@socketio.on("connect")
def handle_connect():
    ensure_background_started()

    with telemetry_lock:
        telemetry_snapshot = dict(telemetry_state)

    with aruco_lock:
        markers_snapshot = list(aruco_state["markers"])

    with mission_lock:
        mission_snapshot = dict(mission_state)

    socketio.emit("telemetry", telemetry_snapshot)
    socketio.emit("aruco_markers", markers_snapshot)
    socketio.emit("mission_state", mission_snapshot)
