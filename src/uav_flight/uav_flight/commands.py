from __future__ import annotations

import time
from pymavlink import mavutil


class FlightCommands:
    def __init__(self, master) -> None:
        self.master = master

    def _normalize_mode_name(self, mode_name: str) -> str:
        mode_name = mode_name.strip().upper().replace(" ", "_")
        aliases = {
            "ALTHOLD": "ALT_HOLD",
        }
        return aliases.get(mode_name, mode_name)

    def get_mode_mapping(self) -> dict:
        mapping = getattr(self.master, "mode_mapping", lambda: None)()
        return mapping or {}

    def print_available_modes(self) -> None:
        mapping = self.get_mode_mapping()
        if not mapping:
            print("[WARN] Không lấy được mode mapping từ FC.")
            return

        print("[INFO] Available modes:")
        for name, mode_id in mapping.items():
            print(f"  - {name}: {mode_id}")

    def set_mode(self, mode_name: str, timeout_s: float = 8.0) -> bool:
        requested_mode = self._normalize_mode_name(mode_name)

        mapping = self.get_mode_mapping()
        if not mapping:
            print("[ERROR] FC không trả về mode mapping.")
            return False

        if requested_mode not in mapping:
            print(f"[ERROR] Mode '{requested_mode}' không tồn tại trong mapping.")
            return False

        mode_id = mapping[requested_mode]

        print(f"[INFO] Requesting mode change -> {requested_mode}")
        self.master.set_mode(mode_id)

        start = time.time()
        while time.time() - start < timeout_s:
            msg = self.master.recv_match(type="HEARTBEAT", blocking=True, timeout=1)
            if msg is None:
                continue

            if msg.get_srcSystem() != self.master.target_system:
                continue

            if msg.get_srcComponent() != self.master.target_component:
                continue

            current_mode = mavutil.mode_string_v10(msg)
            print(f"[INFO] Current mode: {current_mode}")

            if current_mode == requested_mode:
                print(f"[OK] Mode changed to {requested_mode}")
                return True

        print(f"[WARN] Không xác nhận được mode {requested_mode} trong {timeout_s}s")
        return False
