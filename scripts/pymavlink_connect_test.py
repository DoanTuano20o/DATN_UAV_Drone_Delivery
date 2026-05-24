from pymavlink import mavutil
import time

# Ưu tiên dùng đường dẫn /dev/serial/by-id nếu có
PORT = "/dev/ttyACM0"
BAUD = 115200

print(f"[INFO] Connecting to {PORT} at {BAUD}...")

master = mavutil.mavlink_connection(PORT, baud=BAUD)

print("[INFO] Waiting for heartbeat...")
hb = master.wait_heartbeat(timeout=15)

print("[OK] HEARTBEAT received")
print(f"    sysid   : {master.target_system}")
print(f"    compid  : {master.target_component}")
print(f"    mode    : {mavutil.mode_string_v10(hb)}")

print("[INFO] Reading some messages...")
start = time.time()

while time.time() - start < 10:
    msg = master.recv_match(blocking=True, timeout=1)
    if msg is None:
        continue

    mtype = msg.get_type()

    if mtype == "HEARTBEAT":
        print(f"[HEARTBEAT] mode={mavutil.mode_string_v10(msg)}")
    elif mtype == "SYS_STATUS":
        voltage = getattr(msg, "voltage_battery", -1)
        current = getattr(msg, "current_battery", -1)
        print(f"[SYS_STATUS] voltage={voltage}mV current={current}cA")
    elif mtype == "GPS_RAW_INT":
        fix_type = getattr(msg, "fix_type", -1)
        sats = getattr(msg, "satellites_visible", -1)
        print(f"[GPS] fix_type={fix_type} sats={sats}")
    elif mtype == "ATTITUDE":
        print(f"[ATTITUDE] roll={msg.roll:.2f} pitch={msg.pitch:.2f} yaw={msg.yaw:.2f}")

print("[DONE] Test finished.")
