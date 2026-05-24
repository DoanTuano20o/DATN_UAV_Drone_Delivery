from __future__ import annotations

import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from flight.connection import MavlinkConnection
from flight.commands import FlightCommands


def main() -> None:
    mav = MavlinkConnection(PROJECT_ROOT / "configs" / "flight.yaml")
    master, _ = mav.connect()

    cmd = FlightCommands(master)
    cmd.print_available_modes()

    print("\n[INFO] Test đổi mode bằng code")
    print("[INFO] Ví dụ nhập: STABILIZE hoặc ALT_HOLD hoặc ACRO")
    mode_name = input("Nhập mode muốn chuyển: ").strip().upper()

    try:
        cmd.set_mode(mode_name)
    finally:
        mav.close()


if __name__ == "__main__":
    main()
