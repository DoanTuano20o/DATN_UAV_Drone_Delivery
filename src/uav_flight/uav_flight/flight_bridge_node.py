from __future__ import annotations

import json
import math
import threading
import time
from typing import Any

import rclpy
from pymavlink import mavutil
from rclpy.node import Node
from std_msgs.msg import String

from .connection import DEFAULT_CONFIG_PATH, MavlinkConnection
from .telemetry import TelemetryReader


class FlightBridgeNode(Node):
    def __init__(self) -> None:
        super().__init__("flight_bridge_node")

        self.declare_parameter("config_path", str(DEFAULT_CONFIG_PATH))
        self.config_path = str(self.get_parameter("config_path").value)

        self.telemetry_pub = self.create_publisher(String, "/flight/telemetry_json", 10)
        self.status_pub = self.create_publisher(String, "/flight/status", 10)

        self.create_subscription(String, "/flight/cmd_takeoff_json", self.on_cmd_takeoff, 10)
        self.create_subscription(String, "/flight/cmd_goto_global_json", self.on_cmd_goto_global, 10)
        self.create_subscription(String, "/flight/cmd_target_altitude_json", self.on_cmd_target_altitude, 10)
        self.create_subscription(String, "/flight/cmd_vel_body_json", self.on_cmd_vel_body, 10)
        self.create_subscription(String, "/flight/cmd_vel_local_json", self.on_cmd_vel_local, 10)
        self.create_subscription(String, "/flight/cmd_hold_yaw_json", self.on_cmd_hold_yaw, 10)
        self.create_subscription(String, "/flight/cmd_land", self.on_cmd_land, 10)
        self.create_subscription(String, "/flight/cmd_rtl", self.on_cmd_rtl, 10)

        self.master = None
        self.master_lock = threading.Lock()

        self.telemetry_lock = threading.Lock()
        self.latest_telemetry: dict[str, Any] = {
            "connected": False,
            "mode": "DISCONNECTED",
            "armed": False,
            "lat": None,
            "lon": None,
            "rel_alt_m": None,
            "heading_deg": None,
            "yaw_deg": None,
        }

        self._worker = threading.Thread(target=self._worker_loop, daemon=True)
        self._worker.start()

        self.get_logger().info(f"FlightBridgeNode started with config: {self.config_path}")

    def _publish_status(self, text: str) -> None:
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)

    def _publish_telemetry(self, data: dict) -> None:
        msg = String()
        msg.data = json.dumps(data, ensure_ascii=False)
        self.telemetry_pub.publish(msg)

    def _get_master(self):
        with self.master_lock:
            return self.master

    def _set_master(self, master) -> None:
        with self.master_lock:
            self.master = master

    def _current_mode(self) -> str:
        with self.telemetry_lock:
            return str(self.latest_telemetry.get("mode", "")).upper()

    def _is_armed(self) -> bool:
        with self.telemetry_lock:
            return bool(self.latest_telemetry.get("armed", False))

    def _current_heading_deg(self) -> float | None:
        with self.telemetry_lock:
            val = self.latest_telemetry.get("heading_deg")
        if val is None:
            return None
        try:
            return float(val)
        except Exception:
            return None

    def _auto_allowed(self) -> bool:
        return self._current_mode() == "GUIDED" and self._is_armed()

    def _normalize_heading_deg(self, heading_deg: float) -> float:
        value = float(heading_deg) % 360.0
        if value < 0.0:
            value += 360.0
        return value

    def _yaw_direction_shortest(self, current_deg: float, target_deg: float) -> float:
        diff = (target_deg - current_deg + 540.0) % 360.0 - 180.0
        return 1.0 if diff >= 0.0 else -1.0

    def _set_guided_speed(self, speed_mps: float) -> bool:
        master = self._get_master()
        if master is None or not self._auto_allowed():
            return False

        try:
            master.mav.command_long_send(
                master.target_system,
                master.target_component,
                mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED,
                0,
                1,
                float(speed_mps),
                -1,
                0,
                0,
                0,
                0,
            )
            self.get_logger().info(f"Requested GUIDED speed: {speed_mps:.2f} m/s")
            self._publish_status(f"SPEED_SET:{speed_mps:.2f}")
            return True
        except Exception as e:
            self.get_logger().warn(f"Set speed failed: {e}")
            self._publish_status(f"SPEED_ERROR:{e}")
            return False

    def _send_takeoff(self, alt_m: float) -> bool:
        master = self._get_master()
        if master is None:
            self._publish_status("TAKEOFF_ERROR:NO_CONNECTION")
            return False

        if not self._auto_allowed():
            self._publish_status("CMD_REJECTED:NOT_GUIDED_OR_NOT_ARMED")
            return False

        try:
            master.mav.command_long_send(
                master.target_system,
                master.target_component,
                mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                float(alt_m),
            )
            self.get_logger().info(f"TAKEOFF sent: alt={alt_m:.2f}")
            self._publish_status(f"TAKEOFF_SENT:{alt_m:.2f}")
            return True
        except Exception as e:
            self.get_logger().error(f"TAKEOFF failed: {e}")
            self._publish_status(f"TAKEOFF_ERROR:{e}")
            return False

    def _send_goto_global(
        self,
        lat: float,
        lon: float,
        alt_m: float,
        speed_mps: float | None = None,
        heading_deg: float | None = None,
    ) -> bool:
        master = self._get_master()
        if master is None:
            self._publish_status("GOTO_ERROR:NO_CONNECTION")
            return False

        if not self._auto_allowed():
            self._publish_status("CMD_REJECTED:NOT_GUIDED_OR_NOT_ARMED")
            return False

        if speed_mps is not None and speed_mps > 0.0:
            self._set_guided_speed(speed_mps)

        lat_int = int(lat * 1e7)
        lon_int = int(lon * 1e7)

        type_mask = (
            (1 << 3)
            | (1 << 4)
            | (1 << 5)
            | (1 << 6)
            | (1 << 7)
            | (1 << 8)
            | (1 << 11)
        )
        yaw_rad = 0.0
        if heading_deg is None:
            type_mask |= 1 << 10
        else:
            yaw_rad = math.radians(self._normalize_heading_deg(heading_deg))

        try:
            for _ in range(5):
                master.mav.set_position_target_global_int_send(
                    int(time.time() * 1000) & 0xFFFFFFFF,
                    master.target_system,
                    master.target_component,
                    mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                    type_mask,
                    lat_int,
                    lon_int,
                    float(alt_m),
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    0.0,
                    yaw_rad,
                    0.0,
                )
                time.sleep(0.1)

            self.get_logger().info(
                f"GOTO sent: lat={lat:.7f}, lon={lon:.7f}, alt={alt_m:.2f}, "
                f"speed={speed_mps}, heading={heading_deg}"
            )
            self._publish_status(f"GOTO_SENT:{lat:.7f},{lon:.7f},{alt_m:.2f}")
            return True
        except Exception as e:
            self.get_logger().error(f"GOTO failed: {e}")
            self._publish_status(f"GOTO_ERROR:{e}")
            return False

    def _send_target_altitude(self, alt_m: float) -> bool:
        with self.telemetry_lock:
            lat = self.latest_telemetry.get("lat")
            lon = self.latest_telemetry.get("lon")

        if lat is None or lon is None:
            self._publish_status("ALT_ERROR:NO_POSITION")
            return False

        return self._send_goto_global(float(lat), float(lon), float(alt_m), None)

    def _send_vel_body(
        self,
        vx: float,
        vy: float,
        vz: float,
        yaw_rate: float = 0.0,
        heading_deg: float | None = None,
    ) -> bool:
        master = self._get_master()
        if master is None:
            self._publish_status("VEL_ERROR:NO_CONNECTION")
            return False

        if not self._auto_allowed():
            self._publish_status("CMD_REJECTED:NOT_GUIDED_OR_NOT_ARMED")
            return False

        type_mask = (
            (1 << 0)
            | (1 << 1)
            | (1 << 2)
            | (1 << 6)
            | (1 << 7)
            | (1 << 8)
        )
        yaw_rad = 0.0
        if heading_deg is None:
            type_mask |= 1 << 10
        else:
            type_mask |= 1 << 11
            yaw_rad = math.radians(self._normalize_heading_deg(heading_deg))

        try:
            master.mav.set_position_target_local_ned_send(
                int(time.time() * 1000) & 0xFFFFFFFF,
                master.target_system,
                master.target_component,
                mavutil.mavlink.MAV_FRAME_BODY_NED,
                type_mask,
                0.0,
                0.0,
                0.0,
                float(vx),
                float(vy),
                float(vz),
                0.0,
                0.0,
                0.0,
                yaw_rad,
                float(yaw_rate),
            )
            self._publish_status(
                f"VEL_SENT:{vx:.2f},{vy:.2f},{vz:.2f},yr={yaw_rate:.2f},hdg={heading_deg}"
            )
            return True
        except Exception as e:
            self.get_logger().error(f"VEL failed: {e}")
            self._publish_status(f"VEL_ERROR:{e}")
            return False

    def _send_vel_local(
        self,
        vn: float,
        ve: float,
        vd: float,
        heading_deg: float | None = None,
    ) -> bool:
        master = self._get_master()
        if master is None:
            self._publish_status("VEL_LOCAL_ERROR:NO_CONNECTION")
            return False

        if not self._auto_allowed():
            self._publish_status("CMD_REJECTED:NOT_GUIDED_OR_NOT_ARMED")
            return False

        type_mask = (
            (1 << 0)
            | (1 << 1)
            | (1 << 2)
            | (1 << 6)
            | (1 << 7)
            | (1 << 8)
            | (1 << 11)
        )
        yaw_rad = 0.0
        if heading_deg is None:
            type_mask |= 1 << 10
        else:
            yaw_rad = math.radians(self._normalize_heading_deg(heading_deg))

        try:
            master.mav.set_position_target_local_ned_send(
                int(time.time() * 1000) & 0xFFFFFFFF,
                master.target_system,
                master.target_component,
                mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                type_mask,
                0.0,
                0.0,
                0.0,
                float(vn),
                float(ve),
                float(vd),
                0.0,
                0.0,
                0.0,
                yaw_rad,
                0.0,
            )
            self._publish_status(f"VEL_LOCAL_SENT:{vn:.2f},{ve:.2f},{vd:.2f},hdg={heading_deg}")
            return True
        except Exception as e:
            self.get_logger().error(f"VEL_LOCAL failed: {e}")
            self._publish_status(f"VEL_LOCAL_ERROR:{e}")
            return False

    def _send_hold_yaw(self, heading_deg: float, yaw_rate_dps: float = 10.0) -> bool:
        master = self._get_master()
        if master is None:
            self._publish_status("YAW_HOLD_ERROR:NO_CONNECTION")
            return False

        if not self._auto_allowed():
            self._publish_status("CMD_REJECTED:NOT_GUIDED_OR_NOT_ARMED")
            return False

        try:
            target_heading = self._normalize_heading_deg(heading_deg)
            current_heading = self._current_heading_deg()
            if current_heading is None:
                current_heading = target_heading
            else:
                current_heading = self._normalize_heading_deg(current_heading)

            direction = self._yaw_direction_shortest(current_heading, target_heading)

            master.mav.command_long_send(
                master.target_system,
                master.target_component,
                mavutil.mavlink.MAV_CMD_CONDITION_YAW,
                0,
                float(target_heading),
                float(yaw_rate_dps),
                float(direction),
                0.0,
                0.0,
                0.0,
                0.0,
            )
            self.get_logger().info(
                f"YAW_HOLD sent: target={target_heading:.1f} current={current_heading:.1f} dir={direction:+.0f}"
            )
            self._publish_status(f"YAW_HOLD_SENT:{target_heading:.1f}")
            return True
        except Exception as e:
            self.get_logger().error(f"YAW_HOLD failed: {e}")
            self._publish_status(f"YAW_HOLD_ERROR:{e}")
            return False

    def _send_land(self) -> bool:
        master = self._get_master()
        if master is None:
            self._publish_status("LAND_ERROR:NO_CONNECTION")
            return False

        if not self._auto_allowed():
            self._publish_status("CMD_REJECTED:NOT_GUIDED_OR_NOT_ARMED")
            return False

        try:
            mode_map = master.mode_mapping()
            if mode_map and "LAND" in mode_map:
                mode_id = mode_map["LAND"]
                master.mav.set_mode_send(
                    master.target_system,
                    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                    mode_id,
                )
            else:
                master.mav.command_long_send(
                    master.target_system,
                    master.target_component,
                    mavutil.mavlink.MAV_CMD_NAV_LAND,
                    0,
                    0,
                    0,
                    0,
                    0,
                    0,
                    0,
                    0,
                )

            self.get_logger().info("LAND sent")
            self._publish_status("LAND_SENT")
            return True
        except Exception as e:
            self.get_logger().error(f"LAND failed: {e}")
            self._publish_status(f"LAND_ERROR:{e}")
            return False

    def _send_rtl(self) -> bool:
        master = self._get_master()
        if master is None:
            self._publish_status("RTL_ERROR:NO_CONNECTION")
            return False

        if not self._auto_allowed():
            self._publish_status("CMD_REJECTED:NOT_GUIDED_OR_NOT_ARMED")
            return False

        try:
            mode_map = master.mode_mapping()
            if mode_map and "RTL" in mode_map:
                mode_id = mode_map["RTL"]
                master.mav.set_mode_send(
                    master.target_system,
                    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                    mode_id,
                )
            else:
                master.mav.command_long_send(
                    master.target_system,
                    master.target_component,
                    mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH,
                    0,
                    0,
                    0,
                    0,
                    0,
                    0,
                    0,
                    0,
                )

            self.get_logger().info("RTL sent")
            self._publish_status("RTL_SENT")
            return True
        except Exception as e:
            self.get_logger().error(f"RTL failed: {e}")
            self._publish_status(f"RTL_ERROR:{e}")
            return False

    def on_cmd_takeoff(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
            self._send_takeoff(float(data["alt_m"]))
        except Exception as e:
            self._publish_status(f"TAKEOFF_ERROR:BAD_JSON:{e}")

    def on_cmd_goto_global(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
            self._send_goto_global(
                float(data["lat"]),
                float(data["lon"]),
                float(data["alt_m"]),
                float(data["speed_mps"]) if "speed_mps" in data else None,
                float(data["heading_deg"]) if data.get("heading_deg") is not None else None,
            )
        except Exception as e:
            self._publish_status(f"GOTO_ERROR:BAD_JSON:{e}")

    def on_cmd_target_altitude(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
            self._send_target_altitude(float(data["alt_m"]))
        except Exception as e:
            self._publish_status(f"ALT_ERROR:BAD_JSON:{e}")

    def on_cmd_vel_body(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
            self._send_vel_body(
                float(data.get("vx", 0.0)),
                float(data.get("vy", 0.0)),
                float(data.get("vz", 0.0)),
                float(data.get("yaw_rate", 0.0)),
                float(data["heading_deg"]) if data.get("heading_deg") is not None else None,
            )
        except Exception as e:
            self._publish_status(f"VEL_ERROR:BAD_JSON:{e}")

    def on_cmd_vel_local(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
            self._send_vel_local(
                float(data.get("vn", 0.0)),
                float(data.get("ve", 0.0)),
                float(data.get("vd", 0.0)),
                float(data["heading_deg"]) if data.get("heading_deg") is not None else None,
            )
        except Exception as e:
            self._publish_status(f"VEL_LOCAL_ERROR:BAD_JSON:{e}")

    def on_cmd_hold_yaw(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
            self._send_hold_yaw(
                float(data["heading_deg"]),
                float(data.get("yaw_rate_dps", 10.0)),
            )
        except Exception as e:
            self._publish_status(f"YAW_HOLD_ERROR:BAD_JSON:{e}")

    def on_cmd_land(self, msg: String) -> None:
        _ = msg.data
        self._send_land()

    def on_cmd_rtl(self, msg: String) -> None:
        _ = msg.data
        self._send_rtl()

    def _worker_loop(self) -> None:
        while rclpy.ok():
            mav = None
            try:
                self.get_logger().info("Connecting MAVLink...")
                self._publish_status("CONNECTING")

                mav = MavlinkConnection(self.config_path)
                master, _ = mav.connect()
                telemetry = TelemetryReader(master)

                self._set_master(master)

                self.get_logger().info("MAVLink connected")
                self._publish_status("CONNECTED")

                last_emit = 0.0

                while rclpy.ok():
                    msg = telemetry.read_message(timeout_s=mav.telemetry_cfg.read_timeout_s)
                    if msg is None:
                        continue

                    now = time.time()
                    if now - last_emit >= 0.2:
                        payload = telemetry.state.to_dict()
                        payload["connected"] = True
                        payload["error"] = None

                        with self.telemetry_lock:
                            self.latest_telemetry = dict(payload)

                        self._publish_telemetry(payload)
                        last_emit = now

            except Exception as e:
                self.get_logger().error(f"MAVLink error: {e}")
                self._publish_status(f"ERROR:{e}")

                payload = {
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
                    "error": str(e),
                }

                with self.telemetry_lock:
                    self.latest_telemetry = dict(payload)

                self._publish_telemetry(payload)
                time.sleep(2.0)

            finally:
                try:
                    if mav is not None:
                        mav.close()
                except Exception:
                    pass
                self._set_master(None)


def main(args=None):
    rclpy.init(args=args)
    node = FlightBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass
