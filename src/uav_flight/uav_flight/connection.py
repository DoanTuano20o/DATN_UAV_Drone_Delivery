from __future__ import annotations

import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml
from pymavlink import mavutil

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CONFIG_PATH = PACKAGE_ROOT / "config" / "flight.yaml"

MSG_NAME_TO_ID = {
    "HEARTBEAT": 0,
    "SYS_STATUS": 1,
    "GPS_RAW_INT": 24,
    "ATTITUDE": 30,
    "GLOBAL_POSITION_INT": 33,
}


@dataclass
class ConnectionConfig:
    port: str
    baud: int
    heartbeat_timeout_s: float
    reconnect_delay_s: float


@dataclass
class TelemetryConfig:
    read_timeout_s: float
    monitor_period_s: float


def load_flight_config(config_path: str | Path = DEFAULT_CONFIG_PATH) -> dict[str, Any]:
    config_path = Path(config_path)
    if not config_path.exists():
        raise FileNotFoundError(f"Không tìm thấy file config: {config_path}")

    with config_path.open("r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}

    if "connection" not in data or "telemetry" not in data:
        raise ValueError("flight.yaml phải có 2 khối: connection và telemetry")

    return data


class MavlinkConnection:
    def __init__(self, config_path: str | Path = DEFAULT_CONFIG_PATH) -> None:
        raw_cfg = load_flight_config(config_path)

        self.connection_cfg = ConnectionConfig(**raw_cfg["connection"])
        self.telemetry_cfg = TelemetryConfig(**raw_cfg["telemetry"])
        self.master = None

    def _is_valid_autopilot_heartbeat(self, msg) -> bool:
        if msg is None or msg.get_type() != "HEARTBEAT":
            return False

        src_sys = msg.get_srcSystem()
        src_comp = msg.get_srcComponent()

        if src_sys is None or src_sys <= 0:
            return False

        if src_comp is None:
            return False

        return True

    def connect(self):
        print(
            f"[INFO] Connecting to {self.connection_cfg.port} "
            f"at {self.connection_cfg.baud}..."
        )

        self.master = mavutil.mavlink_connection(
            self.connection_cfg.port,
            baud=self.connection_cfg.baud,
        )

        print("[INFO] Waiting for valid autopilot heartbeat...")

        heartbeat = None
        start = time.time()

        while time.time() - start < self.connection_cfg.heartbeat_timeout_s:
            msg = self.master.recv_match(type="HEARTBEAT", blocking=True, timeout=1)
            if not self._is_valid_autopilot_heartbeat(msg):
                continue

            src_sys = msg.get_srcSystem()
            src_comp = msg.get_srcComponent()

            self.master.target_system = src_sys
            self.master.target_component = src_comp
            heartbeat = msg
            break

        if heartbeat is None:
            raise TimeoutError("Không nhận được heartbeat hợp lệ từ autopilot")

        print("[OK] HEARTBEAT received")
        print(f"    sysid   : {self.master.target_system}")
        print(f"    compid  : {self.master.target_component}")
        print(f"    type    : {getattr(heartbeat, 'type', None)}")
        print(f"    autoplt : {getattr(heartbeat, 'autopilot', None)}")
        print(f"    mode    : {mavutil.mode_string_v10(heartbeat)}")

        self.configure_default_message_intervals()
        return self.master, heartbeat

    def configure_default_message_intervals(self) -> None:
        default_rates_hz = {
            "HEARTBEAT": 1,
            "SYS_STATUS": 2,
            "GPS_RAW_INT": 2,
            "GLOBAL_POSITION_INT": 5,
            "ATTITUDE": 5,
        }

        for msg_name, hz in default_rates_hz.items():
            ok = self.set_message_interval(msg_name, hz)
            if ok:
                print(f"[INFO] Requested {msg_name} at {hz} Hz")
            else:
                print(f"[WARN] Could not request {msg_name} at {hz} Hz")

    def set_message_interval(self, message_name: str, frequency_hz: float) -> bool:
        if self.master is None:
            raise RuntimeError("Chưa connect MAVLink")

        msg_id = MSG_NAME_TO_ID.get(message_name)
        if msg_id is None:
            raise ValueError(f"Không hỗ trợ message: {message_name}")

        if frequency_hz <= 0:
            interval_us = -1
        else:
            interval_us = int(1_000_000 / frequency_hz)

        try:
            self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
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
            time.sleep(0.05)
            return True
        except Exception:
            return False

    def close(self) -> None:
        if self.master is not None:
            try:
                self.master.close()
            except Exception:
                pass
            self.master = None