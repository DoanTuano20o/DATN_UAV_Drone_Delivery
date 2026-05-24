from __future__ import annotations

import math
from dataclasses import asdict, dataclass
from typing import Optional

from pymavlink import mavutil


@dataclass
class TelemetryState:
    mode: str = "UNKNOWN"
    armed: bool = False
    battery_v: Optional[float] = None
    current_a: Optional[float] = None
    gps_fix: Optional[int] = None
    satellites: Optional[int] = None
    rel_alt_m: Optional[float] = None
    speed_mps: Optional[float] = None
    roll_deg: Optional[float] = None
    pitch_deg: Optional[float] = None
    yaw_deg: Optional[float] = None
    heading_deg: Optional[float] = None
    lat: Optional[float] = None
    lon: Optional[float] = None

    def to_dict(self) -> dict:
        return asdict(self)


class TelemetryReader:
    def __init__(self, master) -> None:
        self.master = master
        self.state = TelemetryState()
        self.target_system = getattr(master, "target_system", None)
        self.target_component = getattr(master, "target_component", None)

    def _is_from_autopilot_system(self, msg) -> bool:
        if msg is None:
            return False

        src_sys = msg.get_srcSystem()
        if self.target_system is None:
            return True
        return src_sys == self.target_system

    def _is_from_autopilot_heartbeat(self, msg) -> bool:
        if not self._is_from_autopilot_system(msg):
            return False

        src_comp = msg.get_srcComponent()
        if self.target_component is None:
            return True
        return src_comp == self.target_component

    def read_message(self, timeout_s: float = 1.0):
        msg = self.master.recv_match(blocking=True, timeout=timeout_s)
        if msg is None:
            return None

        self.update_from_message(msg)
        return msg

    def update_from_message(self, msg) -> TelemetryState:
        mtype = msg.get_type()

        if mtype == "BAD_DATA":
            return self.state

        if mtype == "HEARTBEAT":
            if not self._is_from_autopilot_heartbeat(msg):
                return self.state

            mode_str = mavutil.mode_string_v10(msg)
            if not mode_str.startswith("Mode(0x00000000)"):
                self.state.mode = mode_str

            self.state.armed = bool(
                msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
            )
            return self.state

        if not self._is_from_autopilot_system(msg):
            return self.state

        if mtype == "SYS_STATUS":
            voltage_mv = getattr(msg, "voltage_battery", None)
            current_ca = getattr(msg, "current_battery", None)

            if voltage_mv is not None and voltage_mv != 65535:
                self.state.battery_v = voltage_mv / 1000.0

            if current_ca is not None and current_ca != -1:
                self.state.current_a = current_ca / 100.0

        elif mtype == "GPS_RAW_INT":
            self.state.gps_fix = getattr(msg, "fix_type", None)
            self.state.satellites = getattr(msg, "satellites_visible", None)

        elif mtype == "GLOBAL_POSITION_INT":
            relative_alt_mm = getattr(msg, "relative_alt", None)
            if relative_alt_mm is not None:
                rel_alt = relative_alt_mm / 1000.0
                if (not self.state.armed) and (-2.0 < rel_alt < 2.0):
                    rel_alt = 0.0
                self.state.rel_alt_m = rel_alt

            lat = getattr(msg, "lat", None)
            lon = getattr(msg, "lon", None)
            if lat not in (None, 0):
                self.state.lat = lat / 1e7
            if lon not in (None, 0):
                self.state.lon = lon / 1e7

            hdg_cdeg = getattr(msg, "hdg", None)
            if hdg_cdeg is not None and hdg_cdeg != 65535:
                self.state.heading_deg = hdg_cdeg / 100.0

            vx = getattr(msg, "vx", None)
            vy = getattr(msg, "vy", None)
            if vx is not None and vy is not None:
                self.state.speed_mps = math.sqrt(vx * vx + vy * vy) / 100.0

        elif mtype == "ATTITUDE":
            roll = getattr(msg, "roll", None)
            pitch = getattr(msg, "pitch", None)
            yaw = getattr(msg, "yaw", None)

            if roll is not None:
                self.state.roll_deg = math.degrees(roll)
            if pitch is not None:
                self.state.pitch_deg = math.degrees(pitch)
            if yaw is not None:
                self.state.yaw_deg = math.degrees(yaw)

        return self.state

    def format_status_line(self) -> str:
        s = self.state

        def fmt(value, precision: int = 2, unit: str = "") -> str:
            if value is None:
                return "None"
            if isinstance(value, float):
                return f"{value:.{precision}f}{unit}"
            return f"{value}{unit}"

        return (
            f"mode={s.mode} | "
            f"armed={s.armed} | "
            f"battery={fmt(s.battery_v, 2, 'V')} | "
            f"current={fmt(s.current_a, 2, 'A')} | "
            f"gps_fix={fmt(s.gps_fix)} | "
            f"sats={fmt(s.satellites)} | "
            f"rel_alt={fmt(s.rel_alt_m, 2, 'm')} | "
            f"speed={fmt(s.speed_mps, 2, 'm/s')} | "
            f"latlon=({fmt(s.lat, 6)}, {fmt(s.lon, 6)}) | "
            f"rpy=({fmt(s.roll_deg, 1)}, {fmt(s.pitch_deg, 1)}, {fmt(s.yaw_deg, 1)})"
        )
