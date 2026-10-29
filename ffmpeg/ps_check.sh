#!/bin/bash
# Each corpus stream written as a GB/T 28181 program stream (ps_write.py),
# read back by FFmpeg's MPEG-PS demuxer and held to the golden framemd5.
#
# usage: ffmpeg/ps_check.sh path/to/ffmpeg
set -euo pipefail

ffmpeg=$1
here=$(cd "$(dirname "$0")" && pwd)
root=$(dirname "$here")
tmp=$(mktemp -d)
trap 'rm -rf "$tmp"' EXIT

for g in "$root"/tests/golden/*.framemd5; do
  name=$(basename "$g" .framemd5)
  s="$root/tests/streams/$name.svac3"
  [ -f "$s" ] || continue
  python3 "$here/ps_write.py" "$s" "$tmp/s.ps"
  "$ffmpeg" -y -hide_banner -loglevel warning -f mpeg -i "$tmp/s.ps" \
    -fps_mode passthrough -f framemd5 "$tmp/out.framemd5" </dev/null
  echo "$name (program stream):"
  python3 "$here/check_framemd5.py" "$g" "$tmp/out.framemd5"
done
