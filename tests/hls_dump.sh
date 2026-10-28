#!/usr/bin/env bash
# Compare the C header dump with the reference parser on every stream given.
# Both parsers must succeed on a stream and produce identical output.
# usage: tests/hls_dump.sh <svac3dump binary> <stream>...
set -u
dump=$1; shift
ref="$(dirname "$0")/../tools/svac3_parse.py"
tmp=$(mktemp -d)
trap 'rm -rf "$tmp"' EXIT
rc=0
for s in "$@"; do
    python3 "$ref" "$s" > "$tmp/ref.txt" 2> "$tmp/ref.err"; ref_rc=$?
    "$dump" "$s" > "$tmp/c.txt" 2> "$tmp/c.err"; c_rc=$?
    if [ $ref_rc -ne 0 ] || [ $c_rc -ne 0 ]; then
        echo "FAIL: $s (svac3_parse.py exit $ref_rc, svac3dump exit $c_rc)"
        tail -3 "$tmp/ref.err" "$tmp/c.err"
        rc=1
    elif ! diff -u --label "svac3_parse.py $s" --label "svac3dump $s" \
            "$tmp/ref.txt" "$tmp/c.txt" > "$tmp/diff.txt"; then
        echo "MISMATCH: $s"
        head -20 "$tmp/diff.txt"
        rc=1
    else
        echo "ok: $s ($(grep -c '^\[nal' "$tmp/c.txt") NAL units)"
    fi
done
exit $rc
