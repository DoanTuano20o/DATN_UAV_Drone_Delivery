#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="${UAV_ROOT_DIR:-/media/orangepi/nvme_data/home/DATN_UAV}"
WEB_PORT="${UAV_WEB_PORT:-5000}"
LOCAL_WEB_URL="http://127.0.0.1:${WEB_PORT}"
NGROK_API="http://127.0.0.1:4040/api/tunnels"
ROS_LOG="${UAV_ROS_LOG:-/tmp/uav_full_system.launch.log}"
NGROK_LOG="${UAV_NGROK_LOG:-/tmp/uav_public_web.ngrok.log}"

# Project-local runtime folders.
# Force ROS2 to write logs inside project instead of /home/orangepi/.ros/log.
RUNTIME_DIR="${UAV_RUNTIME_DIR:-${ROOT_DIR}/runtime}"
ROS_RUNTIME_HOME="${RUNTIME_DIR}/ros_home"
ROS_RUNTIME_LOG_DIR="${RUNTIME_DIR}/ros_logs"

mkdir -p "${ROS_RUNTIME_HOME}" "${ROS_RUNTIME_LOG_DIR}"

export ROS_HOME="${ROS_RUNTIME_HOME}"
export ROS_LOG_DIR="${ROS_RUNTIME_LOG_DIR}"
export RCUTILS_LOGGING_DIRECTORY="${ROS_RUNTIME_LOG_DIR}"

ROS_PID=""
NGROK_PID=""
ROS_WITH_SETSID=0
NGROK_WITH_SETSID=0
CLEANED_UP=0

require_file() {
  local path="$1"
  if [[ ! -f "$path" ]]; then
    echo "Required file not found: $path"
    exit 1
  fi
}

need_cmd() {
  if ! command -v "$1" >/dev/null 2>&1; then
    echo "Missing required command: $1"
    exit 1
  fi
}

stop_process() {
  local pid="$1"
  local use_setsid="$2"

  if [[ -z "$pid" ]]; then
    return
  fi

  if [[ "$use_setsid" == "1" ]]; then
    kill -TERM -- "-$pid" >/dev/null 2>&1 || true
  fi

  pkill -TERM -P "$pid" >/dev/null 2>&1 || true
  kill -TERM "$pid" >/dev/null 2>&1 || true

  sleep 0.4

  if [[ "$use_setsid" == "1" ]]; then
    kill -KILL -- "-$pid" >/dev/null 2>&1 || true
  fi

  if kill -0 "$pid" >/dev/null 2>&1; then
    pkill -KILL -P "$pid" >/dev/null 2>&1 || true
    kill -KILL "$pid" >/dev/null 2>&1 || true
  fi
}

cleanup() {
  if [[ "$CLEANED_UP" == "1" ]]; then
    return
  fi
  CLEANED_UP=1

  echo
  echo "Stopping ngrok and ROS2 launch..."
  stop_process "$NGROK_PID" "$NGROK_WITH_SETSID"
  stop_process "$ROS_PID" "$ROS_WITH_SETSID"
}

handle_signal() {
  cleanup
  exit 130
}

trap cleanup EXIT
trap handle_signal INT TERM

cd "$ROOT_DIR"

require_file "$ROOT_DIR/.venv/bin/activate"
require_file "/opt/ros/humble/setup.bash"
require_file "$ROOT_DIR/install/setup.bash"

# BEGIN safe source env: ROS setup may reference unset variables
set +u
# shellcheck disable=SC1091
source "$ROOT_DIR/.venv/bin/activate"
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source "$ROOT_DIR/install/setup.bash"
set -u
# END safe source env

export UAV_ADMIN_USERNAME="${UAV_ADMIN_USERNAME:-admin}"
export UAV_ADMIN_PASSWORD="${UAV_ADMIN_PASSWORD:-Drone111}"
export UAV_WEB_SECRET_KEY="${UAV_WEB_SECRET_KEY:-drone-delivery-secret-key}"

need_cmd python3
need_cmd ros2

if ! command -v ngrok >/dev/null 2>&1; then
  echo "ngrok is not installed."
  echo "Please run:"
  echo "  ./tools/install_ngrok_arm64.sh"
  echo "Then add your real authtoken:"
  echo "  ngrok config add-authtoken <YOUR_REAL_NGROK_AUTHTOKEN>"
  exit 1
fi

if command -v pkill >/dev/null 2>&1; then
  pkill ngrok >/dev/null 2>&1 || true
  sleep 0.5
fi

rm -f "$ROS_LOG" "$NGROK_LOG"

echo "Starting ROS2 full system launch..."
echo "ROS2 log: $ROS_LOG"
if command -v setsid >/dev/null 2>&1; then
  setsid ros2 launch uav_bringup full_system.launch.py >"$ROS_LOG" 2>&1 &
  ROS_WITH_SETSID=1
else
  ros2 launch uav_bringup full_system.launch.py >"$ROS_LOG" 2>&1 &
fi
ROS_PID="$!"

web_is_ready() {
  python3 - "$LOCAL_WEB_URL" <<'PY'
import sys
import urllib.request

base = sys.argv[1].rstrip("/")
for path in ("/health", "/"):
    try:
        with urllib.request.urlopen(base + path, timeout=2) as response:
            if 200 <= response.status < 500:
                sys.exit(0)
    except Exception:
        pass
sys.exit(1)
PY
}

echo "Waiting for local web: ${LOCAL_WEB_URL}/"
WEB_READY=0
for _ in $(seq 1 60); do
  if web_is_ready; then
    WEB_READY=1
    break
  fi

  if ! kill -0 "$ROS_PID" >/dev/null 2>&1; then
    echo "ROS2 launch stopped before the web became ready."
    echo "Last ROS2 log lines:"
    tail -n 80 "$ROS_LOG" || true
    exit 1
  fi

  sleep 1
done

if [[ "$WEB_READY" != "1" ]]; then
  echo "Local web did not become ready after 60 seconds."
  echo "Last ROS2 log lines:"
  tail -n 80 "$ROS_LOG" || true
  exit 1
fi

NGROK_ARGS=(http "$WEB_PORT" "--log=stdout")
if [[ -n "${NGROK_DOMAIN:-}" ]]; then
  NGROK_ARGS=(http "--domain=${NGROK_DOMAIN}" "$WEB_PORT" "--log=stdout")
fi

echo "Starting ngrok tunnel..."
echo "ngrok log: $NGROK_LOG"
if command -v setsid >/dev/null 2>&1; then
  setsid ngrok "${NGROK_ARGS[@]}" >"$NGROK_LOG" 2>&1 &
  NGROK_WITH_SETSID=1
else
  ngrok "${NGROK_ARGS[@]}" >"$NGROK_LOG" 2>&1 &
fi
NGROK_PID="$!"

read_ngrok_url() {
  python3 - "$NGROK_API" <<'PY'
import json
import sys
import urllib.request

api_url = sys.argv[1]
try:
    with urllib.request.urlopen(api_url, timeout=1) as response:
        data = json.load(response)
except Exception:
    sys.exit(1)

urls = [
    tunnel.get("public_url", "")
    for tunnel in data.get("tunnels", [])
    if tunnel.get("public_url", "").startswith("https://")
]
if not urls:
    sys.exit(1)

print(urls[0])
PY
}

PUBLIC_URL=""
for _ in $(seq 1 40); do
  if ! kill -0 "$NGROK_PID" >/dev/null 2>&1; then
    echo "ngrok stopped before a public URL was created."
    echo "Last ngrok log lines:"
    tail -n 80 "$NGROK_LOG" || true
    echo
    echo "If this is an auth issue, run:"
    echo "  ngrok config add-authtoken <YOUR_REAL_NGROK_AUTHTOKEN>"
    exit 1
  fi

  PUBLIC_URL="$(read_ngrok_url || true)"
  if [[ -n "$PUBLIC_URL" ]]; then
    break
  fi

  sleep 0.5
done

if [[ -z "$PUBLIC_URL" ]]; then
  echo "Could not read the public ngrok URL from ${NGROK_API}."
  echo "Last ngrok log lines:"
  tail -n 80 "$NGROK_LOG" || true
  echo
  echo "If this is an auth issue, run:"
  echo "  ngrok config add-authtoken <YOUR_REAL_NGROK_AUTHTOKEN>"
  exit 1
fi

LAN_IP="$(hostname -I 2>/dev/null | tr ' ' '\n' | awk '/^[0-9]+\./ { print; exit }' || true)"
if [[ -n "$LAN_IP" ]]; then
  LAN_URL="http://${LAN_IP}:${WEB_PORT}/"
else
  LAN_URL="http://192.168.1.27:${WEB_PORT}/"
fi

printf '%s\n' "$PUBLIC_URL" > "$ROOT_DIR/.uav_public_url"

echo
echo "========================================"
echo "UAV PUBLIC WEB IS READY"
echo "========================================"
echo "PUBLIC WEB URL:"
echo "$PUBLIC_URL"
echo
echo "Landing:"
echo "${PUBLIC_URL}/"
echo
echo "Tracking:"
echo "${PUBLIC_URL}/tracking"
echo
echo "Admin Dashboard:"
echo "${PUBLIC_URL}/dashboard"
echo
echo "Admin Login:"
echo "Username: $UAV_ADMIN_USERNAME"
echo "Password: $UAV_ADMIN_PASSWORD"
echo
echo "Local LAN:"
echo "$LAN_URL"
echo "========================================"
echo
echo "Press Ctrl+C to stop ROS2 and ngrok."

while true; do
  if ! kill -0 "$ROS_PID" >/dev/null 2>&1; then
    echo "ROS2 launch stopped. Last ROS2 log lines:"
    tail -n 80 "$ROS_LOG" || true
    exit 1
  fi

  if ! kill -0 "$NGROK_PID" >/dev/null 2>&1; then
    echo "ngrok stopped. Last ngrok log lines:"
    tail -n 80 "$NGROK_LOG" || true
    exit 1
  fi

  sleep 2
done
