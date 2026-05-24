#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
VIDEO_DIR="$ROOT_DIR/src/uav_web_bridge/uav_web_bridge/app/static/assets/videos"
IMAGE_DIR="$ROOT_DIR/src/uav_web_bridge/uav_web_bridge/app/static/assets/images"
BACKUP_DIR="$VIDEO_DIR/original_backup"
IMAGE_BACKUP_DIR="$IMAGE_DIR/original_backup"

FPS="${FPS:-24}"
PRESET="${PRESET:-medium}"

VIDEO_SPECS=(
  "delivery-hero.mp4|1920|25"
  "fleet-dashboard.mp4|1440|25"
  "mission-planning.mp4|1440|24"
  "operations-monitoring.mp4|1440|24"
  "platform-overview.mp4|1440|24"
  "precision-landing.mp4|1440|25"
  "route-planning.mp4|1440|25"
)

require_command() {
  if ! command -v "$1" >/dev/null 2>&1; then
    echo "ERROR: $1 chưa được cài. Chạy: sudo apt install ffmpeg -y" >&2
    exit 1
  fi
}

file_size_bytes() {
  stat -c%s "$1"
}

format_mb() {
  awk -v bytes="$1" 'BEGIN { printf "%.2f MB", bytes / 1048576 }'
}

probe_stream_field() {
  local file="$1"
  local field="$2"

  ffprobe -v error \
    -select_streams v:0 \
    -show_entries "stream=$field" \
    -of default=noprint_wrappers=1:nokey=1 \
    "$file" | head -n 1
}

video_resolution() {
  local file="$1"
  local width
  local height

  width="$(probe_stream_field "$file" width)"
  height="$(probe_stream_field "$file" height)"
  printf "%sx%s" "$width" "$height"
}

video_fps() {
  local file="$1"
  local rate

  rate="$(probe_stream_field "$file" r_frame_rate)"
  awk -v rate="$rate" 'BEGIN {
    split(rate, parts, "/");
    if (parts[2] > 0) {
      printf "%.2f", parts[1] / parts[2];
    } else {
      printf "%s", rate;
    }
  }'
}

verify_optimized_video() {
  local output="$1"
  local codec
  local pix_fmt

  if [[ ! -s "$output" ]]; then
    echo "ERROR: Output lỗi hoặc rỗng: $output" >&2
    return 1
  fi

  codec="$(probe_stream_field "$output" codec_name)"
  pix_fmt="$(probe_stream_field "$output" pix_fmt)"

  if [[ "$codec" != "h264" ]]; then
    echo "ERROR: Codec không hợp lệ cho $output: $codec" >&2
    return 1
  fi

  if [[ "$pix_fmt" != "yuv420p" ]]; then
    echo "ERROR: Pixel format không hợp lệ cho $output: $pix_fmt" >&2
    return 1
  fi

  if ffprobe -v error -select_streams a -show_entries stream=index -of csv=p=0 "$output" | grep -q .; then
    echo "ERROR: Output vẫn còn audio: $output" >&2
    return 1
  fi
}

print_summary_header() {
  printf "%-28s %12s %12s %9s %-12s %-7s %-7s %-9s\n" \
    "File" "Before" "After" "Reduced" "Resolution" "FPS" "Codec" "PixFmt"
}

print_summary_line() {
  local file="$1"
  local before_file="$2"
  local after_file="$3"
  local before_bytes
  local after_bytes
  local reduced
  local resolution
  local fps
  local codec
  local pix_fmt

  before_bytes="$(file_size_bytes "$before_file")"
  after_bytes="$(file_size_bytes "$after_file")"
  reduced="$(awk -v before="$before_bytes" -v after="$after_bytes" 'BEGIN {
    if (before > 0) printf "%.1f%%", (1 - after / before) * 100;
    else printf "0.0%%";
  }')"
  resolution="$(video_resolution "$after_file")"
  fps="$(video_fps "$after_file")"
  codec="$(probe_stream_field "$after_file" codec_name)"
  pix_fmt="$(probe_stream_field "$after_file" pix_fmt)"

  printf "%-28s %12s %12s %9s %-12s %-7s %-7s %-9s\n" \
    "$file" \
    "$(format_mb "$before_bytes")" \
    "$(format_mb "$after_bytes")" \
    "$reduced" \
    "$resolution" \
    "$fps" \
    "$codec" \
    "$pix_fmt"
}

optimize_video() {
  local file="$1"
  local max_width="$2"
  local crf="$3"
  local base="${file%.mp4}"
  local input="$VIDEO_DIR/$file"
  local backup="$BACKUP_DIR/$file"
  local output="$VIDEO_DIR/$base.optimized.mp4"
  local poster="$IMAGE_DIR/$base.jpg"
  local poster_backup="$IMAGE_BACKUP_DIR/$base.jpg"
  local before_size
  local after_size

  if [[ ! -f "$input" ]]; then
    echo "SKIP: Không tìm thấy $input"
    return
  fi

  if [[ ! -f "$backup" ]]; then
    cp -p "$input" "$backup"
    echo "Backup video: $backup"
  else
    echo "Backup video đã tồn tại: $backup"
  fi

  if [[ -f "$poster" && ! -f "$poster_backup" ]]; then
    cp -p "$poster" "$poster_backup"
    echo "Backup poster: $poster_backup"
  fi

  before_size="$(format_mb "$(file_size_bytes "$backup")")"

  echo "========================================"
  echo "Optimizing: $file"
  echo "Before: $before_size"
  echo "Max width: $max_width | FPS: $FPS | CRF: $crf | Preset: $PRESET"

  rm -f "$output"

  ffmpeg -hide_banner -y \
    -i "$backup" \
    -vf "scale='min(${max_width},iw)':-2:flags=lanczos,fps=${FPS}" \
    -an \
    -c:v libx264 \
    -preset "$PRESET" \
    -crf "$crf" \
    -pix_fmt yuv420p \
    -movflags +faststart \
    "$output"

  verify_optimized_video "$output"
  mv "$output" "$input"

  ffmpeg -hide_banner -y \
    -ss 00:00:02 \
    -i "$input" \
    -frames:v 1 \
    -q:v 3 \
    "$poster" || true

  after_size="$(format_mb "$(file_size_bytes "$input")")"
  echo "After:  $after_size"
  echo "Codec check:"
  ffprobe -v error -select_streams v:0 \
    -show_entries stream=codec_name,width,height,r_frame_rate,pix_fmt \
    -of default=noprint_wrappers=1 "$input"
  echo
}

require_command ffmpeg
require_command ffprobe

mkdir -p "$BACKUP_DIR" "$IMAGE_DIR" "$IMAGE_BACKUP_DIR"

echo "Video directory: $VIDEO_DIR"
echo "Image directory: $IMAGE_DIR"
echo "Backup video directory: $BACKUP_DIR"
echo "Backup poster directory: $IMAGE_BACKUP_DIR"
echo

for spec in "${VIDEO_SPECS[@]}"; do
  IFS="|" read -r file max_width crf <<< "$spec"
  optimize_video "$file" "$max_width" "$crf"
done

echo "========================================"
echo "Optimization summary"
print_summary_header
for spec in "${VIDEO_SPECS[@]}"; do
  IFS="|" read -r file _max_width _crf <<< "$spec"
  backup="$BACKUP_DIR/$file"
  optimized="$VIDEO_DIR/$file"
  if [[ -f "$backup" && -f "$optimized" ]]; then
    print_summary_line "$file" "$backup" "$optimized"
  else
    printf "%-28s %12s\n" "$file" "SKIPPED"
  fi
done

echo "========================================"
echo "DONE. Backup video gốc nằm tại:"
echo "$BACKUP_DIR"
echo "Backup poster cũ nằm tại:"
echo "$IMAGE_BACKUP_DIR"
