#!/usr/bin/env bash
set -euo pipefail

NGROK_URL="https://bin.equinox.io/c/bNyj1mQVY4c/ngrok-v3-stable-linux-arm64.tgz"

if command -v ngrok >/dev/null 2>&1; then
  echo "ngrok already installed:"
  ngrok version
  exit 0
fi

ARCH="$(uname -m)"
if [[ "$ARCH" != "aarch64" && "$ARCH" != "arm64" ]]; then
  echo "This script is for Linux ARM64. Current arch: $ARCH"
  exit 1
fi

if ! command -v wget >/dev/null 2>&1 && ! command -v curl >/dev/null 2>&1; then
  echo "wget or curl is required to download ngrok."
  exit 1
fi

TMP_DIR="$(mktemp -d)"
cleanup() {
  rm -rf "$TMP_DIR"
}
trap cleanup EXIT

ARCHIVE="$TMP_DIR/ngrok.tgz"
echo "Downloading ngrok Linux ARM64..."
if command -v wget >/dev/null 2>&1; then
  wget -O "$ARCHIVE" "$NGROK_URL"
else
  curl -L -o "$ARCHIVE" "$NGROK_URL"
fi

if [[ "$(id -u)" -eq 0 ]]; then
  tar xvzf "$ARCHIVE" -C /usr/local/bin ngrok
else
  sudo tar xvzf "$ARCHIVE" -C /usr/local/bin ngrok
fi

echo
echo "Installed ngrok:"
ngrok version

echo
echo "Next step:"
echo "ngrok config add-authtoken <YOUR_NGROK_AUTHTOKEN>"
