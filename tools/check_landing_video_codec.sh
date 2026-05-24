#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
VIDEO_DIR="$ROOT_DIR/src/uav_web_bridge/uav_web_bridge/app/static/assets/videos"

VIDEO_FILES=(
  "delivery-hero.mp4"
  "fleet-dashboard.mp4"
  "mission-planning.mp4"
  "operations-monitoring.mp4"
  "platform-overview.mp4"
  "precision-landing.mp4"
  "route-planning.mp4"
)

if ! command -v ffprobe >/dev/null 2>&1; then
  printf 'Missing required tool: ffprobe\n' >&2
  exit 1
fi

print_encode_suggestion() {
  local input="$1"
  local output="${input%.mp4}.compatible.mp4"

  printf 'Suggested encode command:\n'
  printf 'ffmpeg -y -i "%s" \\\n' "$input"
  printf '  -vf "scale='\''min(1920,iw)'\'':-2:flags=lanczos,fps=24" \\\n'
  printf '  -an \\\n'
  printf '  -c:v libx264 \\\n'
  printf '  -preset medium \\\n'
  printf '  -crf 23 \\\n'
  printf '  -pix_fmt yuv420p \\\n'
  printf '  -profile:v high \\\n'
  printf '  -level:v 4.1 \\\n'
  printf '  -movflags +faststart \\\n'
  printf '  "%s"\n' "$output"
}

failed=0

for name in "${VIDEO_FILES[@]}"; do
  input="$VIDEO_DIR/$name"
  printf '\n[%s]\n' "$name"

  if [[ ! -f "$input" ]]; then
    printf 'missing=%s\n' "$input"
    failed=1
    continue
  fi

  codec_name="$(ffprobe -v error -select_streams v:0 -show_entries stream=codec_name -of csv=p=0 "$input")"
  profile="$(ffprobe -v error -select_streams v:0 -show_entries stream=profile -of csv=p=0 "$input")"
  pix_fmt="$(ffprobe -v error -select_streams v:0 -show_entries stream=pix_fmt -of csv=p=0 "$input")"
  width="$(ffprobe -v error -select_streams v:0 -show_entries stream=width -of csv=p=0 "$input")"
  height="$(ffprobe -v error -select_streams v:0 -show_entries stream=height -of csv=p=0 "$input")"
  r_frame_rate="$(ffprobe -v error -select_streams v:0 -show_entries stream=r_frame_rate -of csv=p=0 "$input")"
  duration="$(ffprobe -v error -show_entries format=duration -of csv=p=0 "$input")"
  audio_streams="$(ffprobe -v error -select_streams a -show_entries stream=index,codec_name -of csv=p=0 "$input" || true)"

  printf 'codec_name=%s\n' "$codec_name"
  printf 'profile=%s\n' "$profile"
  printf 'pix_fmt=%s\n' "$pix_fmt"
  printf 'width=%s\n' "$width"
  printf 'height=%s\n' "$height"
  printf 'r_frame_rate=%s\n' "$r_frame_rate"
  printf 'duration=%s\n' "$duration"
  if [[ -n "$audio_streams" ]]; then
    printf 'audio_streams=%s\n' "$audio_streams"
  else
    printf 'audio_streams=none\n'
  fi

  if [[ "$codec_name" != "h264" || "$pix_fmt" != "yuv420p" ]]; then
    printf 'WARNING: expected codec_name=h264 and pix_fmt=yuv420p for Chrome-safe MP4.\n'
    print_encode_suggestion "$input"
    failed=1
  else
    printf 'OK: codec/pixel format compatible.\n'
  fi
done

exit "$failed"
