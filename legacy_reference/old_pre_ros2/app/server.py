from __future__ import annotations

import json
import math
import sys
import threading
import time
from pathlib import Path

from flask import Flask, Response, jsonify, render_template
from flask_socketio import SocketIO
from pymavlink import mavutil

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from flight.connection import MavlinkConnection
from vision.camera_stream import CameraStream

DATA_DIR = PROJECT_ROOT / "data" / "missions"
GPS_STATIONS_FILE = DATA_DIR / "gps_stations.json"
FLIGHT_CONFIG_FILE = PROJECT_ROOT / "configs" / "flight.yaml"

app = Flask(
    __name__,
    template_folder=str(PROJECT_ROOT / "app" / "templates"),
    static_folder=str(PROJECT_ROOT / "app" / "static"),
)

socketio = SocketIO(app, cors_allowed_origins="*", async_mode="threading")


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
aruco_state = {"markers": []}

camera_stream = None
CAMERA_INDEX = 0
CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480
CAMERA_FPS = 20

_last_marker_signature = None
_background_started = False


COPTER_MODE_MAP = {
    0: "STABILIZE",
    1: "ACRO",
    2: "ALT_HOLD",
    3: "AUTO",
    4: "GUIDED",
    5: "LOITER",
    6: "RTL",
    7: "CIRCLE",
    9: "LAND",
    11: "DRIFT",
    13: "SPORT",
    14: "FLIP",
    15: "AUTOTUNE",
    16: "POSHOLD",
    17: "BRAKE",
    18: "THROW",
    19: "AVOID_ADSB",
    20: "GUIDED_NOGPS",
    21: "SMART_RTL",
    22: "FLOWHOLD",
    23: "FOLLOW",
    24: "ZIGZAG",
    25: "SYSTEMID",
    26: "AUTOROTATE",
    27: "AUTO_RTL",
    28: "TURTLE",
    29: "RATE_ACRO",
}


def load_gps_stations():
    if not GPS_STATIONS_FILE.exists():
        return []

    with GPS_STATIONS_FILE.open("r", encoding="utf-8") as f:
        return json.load(f)


def ensure_camera_started():
    global camera_stream

    if camera_stream is not None:
        return camera_stream

    camera_stream = CameraStream(
        camera_index=CAMERA_INDEX,
        width=CAMERA_WIDTH,
        height=CAMERA_HEIGHT,
        fps=CAMERA_FPS,
    )
    camera_stream.start()
    print(f"[WEB] Camera started: index={CAMERA_INDEX}")
    return camera_stream


def build_marker_signature(detections):
    signature = []
    for d in detections:
        signature.append(
            (
                int(d.get("marker_id", -1)),
                int(d.get("center_x", 0)),
                int(d.get("center_y", 0)),
            )
        )
    return tuple(sorted(signature))


def update_aruco_state():
    global _last_marker_signature

    cam = ensure_camera_started()
    detections = cam.get_latest_detections()
    aruco_state["markers"] = detections

    signature = build_marker_signature(detections)

    if signature != _last_marker_signature:
        socketio.emit("aruco_markers", detections)
        _last_marker_signature = signature


def request_message_interval(master, msg_id: int, hz: float):
    interval_us = int(1_000_000 / hz)
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
        0,
        msg_id,
        interval_us,
        0,
        0,
        0,
        0,
        0,
    )


def request_required_messages(master):
    try:
        request_message_interval(master, mavutil.mavlink.MAVLINK_MSG_ID_HEARTBEAT, 1)
        request_message_interval(master, mavutil.mavlink.MAVLINK_MSG_ID_SYS_STATUS, 2)
        request_message_interval(master, mavutil.mavlink.MAVLINK_MSG_ID_GPS_RAW_INT, 2)
        request_message_interval(master, mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, 5)
        request_message_interval(master, mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, 5)
        request_message_interval(master, mavutil.mavlink.MAVLINK_MSG_ID_VFR_HUD, 2)
    except Exception as e:
        print(f"[WEB] SET_MESSAGE_INTERVAL warning: {e}")

    stream_rates = {
        mavutil.mavlink.MAV_DATA_STREAM_EXTENDED_STATUS: 2,
        mavutil.mavlink.MAV_DATA_STREAM_POSITION: 5,
        mavutil.mavlink.MAV_DATA_STREAM_EXTRA1: 5,
        mavutil.mavlink.MAV_DATA_STREAM_EXTRA2: 2,
        mavutil.mavlink.MAV_DATA_STREAM_EXTRA3: 2,
    }

    for stream_id, rate in stream_rates.items():
        try:
            master.mav.request_data_stream_send(
                master.target_system,
                master.target_component,
                stream_id,
                rate,
                1,
            )
        except Exception as e:
            print(f"[WEB] request_data_stream warning stream={stream_id}: {e}")


def send_gcs_heartbeat(master):
    try:
        master.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_GCS,
            mavutil.mavlink.MAV_AUTOPILOT_INVALID,
            0,
            0,
            mavutil.mavlink.MAV_STATE_ACTIVE,
        )
    except Exception as e:
        print(f"[WEB] heartbeat send warning: {e}")


def update_state_from_msg(msg):
    global telemetry_state

    msg_type = msg.get_type()
    telemetry_state["last_msg"] = msg_type
    telemetry_state["last_update_unix"] = time.time()

    if msg_type == "HEARTBEAT":
        custom_mode = getattr(msg, "custom_mode", None)
        mode_name = COPTER_MODE_MAP.get(int(custom_mode), "UNKNOWN") if custom_mode is not None else "UNKNOWN"
        telemetry_state["mode"] = mode_name
        telemetry_state["armed"] = bool(
            msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
        )

    elif msg_type == "SYS_STATUS":
        voltage_battery = getattr(msg, "voltage_battery", None)
        current_battery = getattr(msg, "current_battery", None)

        if voltage_battery not in (None, -1, 65535):
            telemetry_state["battery_v"] = round(voltage_battery / 1000.0, 2)

        if current_battery not in (None, -1):
            telemetry_state["current_a"] = round(current_battery / 100.0, 2)

    elif msg_type == "GPS_RAW_INT":
        telemetry_state["gps_fix"] = int(getattr(msg, "fix_type", 0) or 0)
        telemetry_state["satellites"] = int(getattr(msg, "satellites_visible", 0) or 0)

    elif msg_type == "GLOBAL_POSITION_INT":
        lat = getattr(msg, "lat", None)
        lon = getattr(msg, "lon", None)
        rel_alt = getattr(msg, "relative_alt", None)
        vx = getattr(msg, "vx", None)
        vy = getattr(msg, "vy", None)

        if lat not in (None, 0) and lon not in (None, 0):
            telemetry_state["lat"] = lat / 1e7
            telemetry_state["lon"] = lon / 1e7

        if rel_alt is not None:
            telemetry_state["rel_alt_m"] = round(rel_alt / 1000.0, 2)

        if vx is not None and vy is not None:
            telemetry_state["speed_mps"] = round(math.sqrt(vx * vx + vy * vy) / 100.0, 2)

    elif msg_type == "ATTITUDE":
        telemetry_state["roll_deg"] = round(math.degrees(getattr(msg, "roll", 0.0)), 2)
        telemetry_state["pitch_deg"] = round(math.degrees(getattr(msg, "pitch", 0.0)), 2)
        telemetry_state["yaw_deg"] = round(math.degrees(getattr(msg, "yaw", 0.0)), 2)

    elif msg_type == "VFR_HUD":
        heading = getattr(msg, "heading", None)
        if heading is not None:
            telemetry_state["heading_deg"] = int(heading)


@app.route("/")
def index():
    ensure_camera_started()
    ensure_background_started()
    return render_template("index.html")


@app.route("/health")
def health():
    return jsonify(
        {
            "status": "ok",
            "service": "DATN_UAV web",
            "fc_connected": telemetry_state["connected"],
        }
    )


@app.route("/api/gps-stations")
def get_gps_stations():
    return jsonify(load_gps_stations())


@app.route("/api/aruco-markers")
def get_aruco_markers():
    update_aruco_state()
    return jsonify(aruco_state["markers"])


@app.route("/api/telemetry")
def get_telemetry():
    return jsonify(telemetry_state)


@app.route("/api/drone-position")
def get_drone_position():
    if telemetry_state["lat"] is None or telemetry_state["lon"] is None:
        return jsonify({"ok": False, "lat": None, "lon": None})

    return jsonify(
        {
            "ok": True,
            "lat": telemetry_state["lat"],
            "lon": telemetry_state["lon"],
            "mode": telemetry_state["mode"],
            "armed": telemetry_state["armed"],
        }
    )


@app.route("/video_feed")
def video_feed():
    cam = ensure_camera_started()
    return Response(
        cam.generate(),
        mimetype="multipart/x-mixed-replace; boundary=frame",
    )


def camera_background_loop():
    ensure_camera_started()

    while True:
        try:
            update_aruco_state()
        except Exception as e:
            print(f"[WEB] camera loop error: {e}")
        time.sleep(0.10)


def mavlink_background_loop():
    global telemetry_state

    while True:
        mav = None
        try:
            print("[WEB] Connecting to flight controller...")
            mav = MavlinkConnection(FLIGHT_CONFIG_FILE)
            master, _ = mav.connect()

            hb = master.wait_heartbeat(timeout=10)
            if hb is None:
                raise RuntimeError("No HEARTBEAT from FC")

            print(
                f"[WEB] FC heartbeat ok: sys={master.target_system} comp={master.target_component}"
            )

            telemetry_state = make_default_telemetry_state()
            telemetry_state["connected"] = True
            telemetry_state["error"] = None
            socketio.emit("telemetry", telemetry_state)

            request_required_messages(master)

            last_gcs_heartbeat = 0.0
            last_emit = 0.0
            last_msg_time = time.time()

            while True:
                now = time.time()

                if now - last_gcs_heartbeat >= 1.0:
                    send_gcs_heartbeat(master)
                    last_gcs_heartbeat = now

                msg = master.recv_match(blocking=True, timeout=1.0)
                if msg is None:
                    if now - last_msg_time > 5.0:
                        raise RuntimeError("Telemetry timeout: no MAVLink messages for 5s")
                    continue

                msg_type = msg.get_type()
                if msg_type == "BAD_DATA":
                    continue

                last_msg_time = now
                update_state_from_msg(msg)
                telemetry_state["connected"] = True
                telemetry_state["error"] = None

                if now - last_emit >= 0.2:
                    socketio.emit("telemetry", telemetry_state)
                    last_emit = now

        except Exception as e:
            telemetry_state = make_default_telemetry_state()
            telemetry_state["error"] = str(e)
            print(f"[WEB] MAVLink error: {e}")
            socketio.emit("telemetry", telemetry_state)
            time.sleep(2.0)

        finally:
            try:
                if mav is not None:
                    mav.close()
            except Exception:
                pass


def ensure_background_started():
    global _background_started

    if _background_started:
        return

    t1 = threading.Thread(target=mavlink_background_loop, daemon=True)
    t2 = threading.Thread(target=camera_background_loop, daemon=True)
    t1.start()
    t2.start()

    _background_started = True


@socketio.on("connect")
def handle_connect():
    ensure_background_started()
    socketio.emit("telemetry", telemetry_state)
    socketio.emit("aruco_markers", aruco_state["markers"])


if __name__ == "__main__":
    ensure_background_started()
    socketio.run(app, host="0.0.0.0", port=5000, debug=False, use_reloader=False)