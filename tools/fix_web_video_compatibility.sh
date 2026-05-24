#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
VIDEO_DIR="$ROOT_DIR/src/uav_web_bridge/uav_web_bridge/app/static/assets/videos"
IMAGE_DIR="$ROOT_DIR/src/uav_web_bridge/uav_web_bridge/app/static/assets/images"
BACKUP_DIR="$VIDEO_DIR/original_backup"

HERO_MAX_WIDTH="${HERO_MAX_WIDTH:-1920}"
HERO_CRF="${HERO_CRF:-23}"
SECTION_MAX_WIDTH="${SECTION_MAX_WIDTH:-1440}"
SECTION_CRF="${SECTION_CRF:-24}"

VIDEO_FILES=(
  "delivery-hero.mp4"
  "fleet-dashboard.mp4"
  "mission-planning.mp4"
  "operations-monitoring.mp4"
  "platform-overview.mp4"
  "precision-landing.mp4"
  "route-planning.mp4"
)

require_tool() {
  if ! command -v "$1" >/dev/null 2>&1; then
    printf 'Missing required tool: %s\n' "$1" >&2
    exit 1
  fi
}

probe_video() {
  local file="$1"
  local label="$2"
  local audio
  local moov

  printf '\n[%s] %s\n' "$label" "$(basename "$file")"
  stat -c 'size_bytes=%s' "$file"
  ffprobe -v error -select_streams v:0 \
    -show_entries stream=codec_name,profile,width,height,r_frame_rate,pix_fmt,bits_per_raw_sample \
    -of default=noprint_wrappers=1 \
    "$file"

  audio="$(ffprobe -v error -select_streams a:0 -show_entries stream=codec_name -of csv=p=0 "$file" || true)"
  if [[ -n "$audio" ]]; then
    printf 'audio=%s\n' "$audio"
  else
    printf 'audio=none\n'
  fi

  moov="$(grep -abo -m1 'moov' "$file" | cut -d: -f1 || true)"
  if [[ -n "$moov" ]]; then
    printf 'moov_offset=%s\n' "$moov"
  else
    printf 'moov_offset=not_found\n'
  fi
}

encode_video() {
  local name="$1"
  local input="$VIDEO_DIR/$name"
  local backup="$BACKUP_DIR/$name"
  local output="${input%.mp4}.compatible.mp4"
  local poster="$IMAGE_DIR/${name%.mp4}.jpg"
  local max_width="$SECTION_MAX_WIDTH"
  local crf="$SECTION_CRF"

  if [[ "$name" == "delivery-hero.mp4" ]]; then
    max_width="$HERO_MAX_WIDTH"
    crf="$HERO_CRF"
  fi

  if [[ ! -f "$input" ]]; then
    printf 'Required video missing: %s\n' "$input" >&2
    exit 1
  fi

  mkdir -p "$BACKUP_DIR" "$IMAGE_DIR"
  if [[ ! -f "$backup" ]]; then
    cp -p "$input" "$backup"
    printf '\nCreated backup: %s\n' "$backup"
  else
    printf '\nBackup already exists: %s\n' "$backup"
  fi

  probe_video "$input" "before"

  rm -f "$output"
  ffmpeg -y -i "$backup" \
    -vf "scale='min(${max_width},iw)':-2:flags=lanczos,fps=24" \
    -an \
    -c:v libx264 \
    -preset medium \
    -crf "$crf" \
    -pix_fmt yuv420p \
    -profile:v high \
    -level:v 4.1 \
    -movflags +faststart \
    "$output"

  if [[ ! -s "$output" ]]; then
    printf 'Encoded output is empty, keeping original: %s\n' "$output" >&2
    rm -f "$output"
    exit 1
  fi

  mv "$output" "$input"

  if ! ffmpeg -y -ss 00:00:02 -i "$input" -frames:v 1 -q:v 3 "$poster"; then
    ffmpeg -y -ss 00:00:00.5 -i "$input" -frames:v 1 -q:v 3 "$poster"
  fi

  if [[ ! -s "$poster" ]]; then
    printf 'Poster output is empty: %s\n' "$poster" >&2
    exit 1
  fi

  probe_video "$input" "after"
  printf 'Poster: %s\n' "$poster"
}

require_tool ffmpeg
require_tool ffprobe

for video in "${VIDEO_FILES[@]}"; do
  encode_video "$video"
done

printf '\nWeb video compatibility fix completed.\n'
