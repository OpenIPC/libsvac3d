#!/usr/bin/env bash
# Compare the C header dump with the reference parser on every stream given.
# usage: tests/hls_dump.sh <svac3dump binary> <stream>...
set -eu
dump=$1; shift
here=$(dirname "$0")
rc=0
for s in "$@"; do
    if ! diff -u "--label=svac3_parse.py $s" "--label=svac3dump $s" \
            <(python3 "$here/../tools/svac3_parse.py" "$s") <("$dump" "$s") > /dev/null; then
        echo "MISMATCH: $s"
        diff -u <(python3 "$here/../tools/svac3_parse.py" "$s") <("$dump" "$s") | head -20
        rc=1
    else
        echo "ok: $s ($(grep -c '^\[nal' <("$dump" "$s")) NAL units)"
    fi
done
exit $rc
