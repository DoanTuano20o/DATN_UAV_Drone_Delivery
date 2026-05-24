#!/usr/bin/env bash
set -euo pipefail

WEB_PORT="${UAV_WEB_PORT:-5000}"
LOCAL_SERVICE="${UAV_LOCAL_SERVICE:-http://127.0.0.1:${WEB_PORT}}"
NGROK_API="http://127.0.0.1:4040/api/tunnels"
LOG_FILE="${UAV_NGROK_LOG_FILE:-/tmp/uav_public_web.ngrok.log}"
PUBLIC_URL=""
NGROK_PID=""

cleanup() {
  if [[ -n "$NGROK_PID" ]] && kill -0 "$NGROK_PID" >/dev/null 2>&1; then
    echo
    echo "Stopping ngrok..."
    kill "$NGROK_PID" >/dev/null 2>&1 || true
    wait "$NGROK_PID" 2>/dev/null || true
  fi
}
trap cleanup INT TERM EXIT

need_cmd() {
  if ! command -v "$1" >/dev/null 2>&1; then
    echo "Missing required command: $1"
    exit 1
  fi
}

need_cmd python3

if ! command -v ngrok >/dev/null 2>&1; then
  echo "ngrok is not installed."
  echo "Install it first:"
  echo "  chmod +x tools/install_ngrok_arm64.sh"
  echo "  ./tools/install_ngrok_arm64.sh"
  echo "Then add your token:"
  echo "  ngrok config add-authtoken <YOUR_NGROK_AUTHTOKEN>"
  exit 1
fi

if ! python3 - "$LOCAL_SERVICE" <<'PY'
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
then
  echo "Local UAV web is not reachable at ${LOCAL_SERVICE}."
  echo "Start the web first, for example:"
  echo "  ros2 launch uav_bringup full_system.launch.py"
  exit 1
fi

if python3 - <<'PY'
import socket
import sys

sock = socket.socket()
sock.settimeout(0.5)
try:
    sys.exit(0 if sock.connect_ex(("127.0.0.1", 4040)) == 0 else 1)
finally:
    sock.close()
PY
then
  echo "Port 4040 is already in use."
  echo "If another ngrok is running, stop it first:"
  echo "  pkill ngrok"
  exit 1
fi

rm -f "$LOG_FILE"

NGROK_ARGS=(http "$WEB_PORT" "--log=stdout")
if [[ -n "${NGROK_DOMAIN:-}" ]]; then
  NGROK_ARGS=(http "--domain=${NGROK_DOMAIN}" "$WEB_PORT" "--log=stdout")
fi

echo "Starting ngrok tunnel to ${LOCAL_SERVICE}..."
echo "Reading public URL from ngrok local API: ${NGROK_API}"
echo "ngrok log: ${LOG_FILE}"
ngrok "${NGROK_ARGS[@]}" >"$LOG_FILE" 2>&1 &
NGROK_PID="$!"

for _ in $(seq 1 20); do
  if ! kill -0 "$NGROK_PID" >/dev/null 2>&1; then
    echo "ngrok stopped before a public URL was created."
    echo "If this is an authtoken issue, run:"
    echo "  ngrok config add-authtoken <YOUR_NGROK_AUTHTOKEN>"
    echo
    tail -n 40 "$LOG_FILE" || true
    exit 1
  fi

  PUBLIC_URL="$(python3 - "$NGROK_API" <<'PY' || true
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
)"

  if [[ -n "$PUBLIC_URL" ]]; then
    break
  fi

  sleep 0.5
done

if [[ -z "$PUBLIC_URL" ]]; then
  echo "Could not read the public ngrok URL from ${NGROK_API}."
  echo "Check the ngrok terminal/log:"
  echo "  ${LOG_FILE}"
  echo "If this is an authtoken issue, run:"
  echo "  ngrok config add-authtoken <YOUR_NGROK_AUTHTOKEN>"
  echo
  tail -n 40 "$LOG_FILE" || true
  exit 1
fi

echo
echo "========================================"
echo "PUBLIC WEB URL:"
echo "$PUBLIC_URL"
echo "========================================"
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
echo "Username: admin"
echo "Password: Drone111"
echo
echo "Press Ctrl+C to stop ngrok."

wait "$NGROK_PID"
