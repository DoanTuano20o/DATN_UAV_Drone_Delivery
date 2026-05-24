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

fps_is_compatible() {
  local rate="$1"
  awk -F/ '
    NF == 2 && $2 != 0 {
      fps = $1 / $2
      exit !(fps >= 23.5 && fps <= 24.5)
    }
    NF == 1 {
      fps = $1
      exit !(fps >= 23.5 && fps <= 24.5)
    }
    { exit 1 }
  ' <<<"$rate"
}

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
    printf 'FAIL missing file: %s\n' "$input"
    failed=1
    continue
  fi

  codec="$(ffprobe -v error -select_streams v:0 -show_entries stream=codec_name -of csv=p=0 "$input")"
  pix_fmt="$(ffprobe -v error -select_streams v:0 -show_entries stream=pix_fmt -of csv=p=0 "$input")"
  rate="$(ffprobe -v error -select_streams v:0 -show_entries stream=r_frame_rate -of csv=p=0 "$input")"
  profile="$(ffprobe -v error -select_streams v:0 -show_entries stream=profile -of csv=p=0 "$input")"
  resolution="$(ffprobe -v error -select_streams v:0 -show_entries stream=width,height -of csv=s=x:p=0 "$input")"
  audio="$(ffprobe -v error -select_streams a:0 -show_entries stream=codec_name -of csv=p=0 "$input" || true)"

  printf 'codec_name=%s\n' "$codec"
  printf 'profile=%s\n' "$profile"
  printf 'resolution=%s\n' "$resolution"
  printf 'pix_fmt=%s\n' "$pix_fmt"
  printf 'r_frame_rate=%s\n' "$rate"
  if [[ -n "$audio" ]]; then
    printf 'audio=%s\n' "$audio"
  else
    printf 'audio=none\n'
  fi

  file_failed=0
  [[ "$codec" == "h264" ]] || file_failed=1
  [[ "$codec" != "hevc" && "$codec" != "h265" ]] || file_failed=1
  [[ "$pix_fmt" == "yuv420p" ]] || file_failed=1
  [[ "$pix_fmt" != "yuv420p10le" && "$pix_fmt" != "yuv422p" && "$pix_fmt" != "yuv444p" ]] || file_failed=1
  [[ -z "$audio" ]] || file_failed=1
  fps_is_compatible "$rate" || file_failed=1

  if [[ "$file_failed" -eq 0 ]]; then
    printf 'PASS Chrome-compatible landing video.\n'
  else
    printf 'FAIL video should be H.264/yuv420p/no-audio/about-24fps.\n'
    print_encode_suggestion "$input"
    failed=1
  fi
done

exit "$failed"
