from __future__ import annotations

import sys
import time
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from flight.connection import MavlinkConnection
from flight.telemetry import TelemetryReader


def main() -> None:
    mav = MavlinkConnection(PROJECT_ROOT / "configs" / "flight.yaml")
    master, _ = mav.connect()

    telemetry = TelemetryReader(master)
    last_print = 0.0
    monitor_period = mav.telemetry_cfg.monitor_period_s
    read_timeout = mav.telemetry_cfg.read_timeout_s

    print("[INFO] Telemetry monitor started. Press Ctrl+C to stop.")

    try:
        while True:
            telemetry.read_message(timeout_s=read_timeout)

            now = time.time()
            if now - last_print >= monitor_period:
                print(telemetry.format_status_line())
                last_print = now

    except KeyboardInterrupt:
        print("\n[INFO] Stopped by user.")
    finally:
        mav.close()


if __name__ == "__main__":
    main()
