#!/bin/bash
# Each corpus stream sent as RTP over local UDP (rtp_send.py), received by
# FFmpeg through an SDP, decoded, and held to the same golden framemd5 the
# raw demuxer is held to: the depacketiser has to give back the stream it was
# given, byte for byte as far as the decoder can tell.
#
# usage: ffmpeg/rtp_check.sh path/to/ffmpeg [mtu [split]]
set -euo pipefail

ffmpeg=$1
mtu=${2:-1200}
split=${3:-}
here=$(cd "$(dirname "$0")" && pwd)
root=$(dirname "$here")
tmp=$(mktemp -d)
trap 'rm -rf "$tmp"' EXIT
port=15004

for g in "$root"/tests/golden/*.framemd5; do
  name=$(basename "$g" .framemd5)
  s="$root/tests/streams/$name.svac3"
  [ -f "$s" ] || continue

  mkfifo "$tmp/go"
  python3 "$here/rtp_send.py" "$s" "$tmp/s.sdp" "$port" "$mtu" $split \
    > "$tmp/ready" < "$tmp/go" &
  sender=$!
  exec 3> "$tmp/go"
  # Ready when it has written the SDP; a sender that dies first, or never
  # gets there, fails the check rather than hanging it.
  for _ in $(seq 100); do
    grep -q sdp "$tmp/ready" 2>/dev/null && break
    kill -0 "$sender" 2>/dev/null || break
    sleep 0.1
  done
  if ! grep -q sdp "$tmp/ready" 2>/dev/null; then
    echo "$name: the sender did not start" >&2
    kill "$sender" 2>/dev/null || true
    exit 1
  fi

  # Ends when no packet has arrived for three seconds.
  "$ffmpeg" -y -hide_banner -loglevel warning \
    -protocol_whitelist file,udp,rtp -rw_timeout 3000000 \
    -i "$tmp/s.sdp" -fps_mode passthrough -f framemd5 "$tmp/out.framemd5" &
  ff=$!
  sleep 1
  echo go >&3
  exec 3>&-
  wait "$sender"
  wait "$ff" || true
  rm -f "$tmp/go" "$tmp/ready"

  echo "$name (mtu $mtu${split:+, $split fmtp}):"
  python3 "$here/check_framemd5.py" "$g" "$tmp/out.framemd5"
  port=$((port + 2))
done
