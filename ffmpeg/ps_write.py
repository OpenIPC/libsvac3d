#!/usr/bin/env python3
"""Write an SVAC3 byte stream as an MPEG-2 program stream, the way GB/T 28181
carries it: one pack per access unit; a system header and a program stream
map (stream_type 0x80, stream id 0xE0) ahead of every access unit that holds
an SPS or a random-access patch; the access unit, start codes included, in
PES packets with the PTS on the first.

usage: ps_write.py stream.svac3 out.ps
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from rtp_send import access_units, nal_units  # noqa: E402

STREAM_ID = 0xE0
STREAM_TYPE = 0x80
MUX_RATE = 50000  # units of 50 bytes/s; informative only
PES_MAX = 65535


def crc32_mpeg(data):
    crc = 0xFFFFFFFF
    for b in data:
        crc ^= b << 24
        for _ in range(8):
            crc = ((crc << 1) ^ 0x04C11DB7) if crc & 0x80000000 else crc << 1
            crc &= 0xFFFFFFFF
    return crc


def pack_header(scr):
    return bytes([
        0, 0, 1, 0xBA,
        0x44 | ((scr >> 27) & 0x38) | ((scr >> 28) & 0x03),
        (scr >> 20) & 0xFF,
        0x04 | ((scr >> 12) & 0xF8) | ((scr >> 13) & 0x03),
        (scr >> 5) & 0xFF,
        0x04 | ((scr << 3) & 0xF8),
        0x01,
        (MUX_RATE >> 14) & 0xFF, (MUX_RATE >> 6) & 0xFF,
        ((MUX_RATE << 2) & 0xFC) | 0x03,
        0xF8,
    ])


def system_header():
    body = bytes([
        0x80 | (MUX_RATE >> 15), (MUX_RATE >> 7) & 0xFF,
        ((MUX_RATE << 1) & 0xFE) | 1,
        0x00, 0xE1, 0x7F,
        STREAM_ID, 0xE0 | (1024 >> 8), 1024 & 0xFF,
    ])
    return bytes([0, 0, 1, 0xBB]) + len(body).to_bytes(2, "big") + body


def stream_map():
    body = bytes([0xE0, 0xFF, 0, 0, 0, 4, STREAM_TYPE, STREAM_ID, 0, 0])
    head = bytes([0, 0, 1, 0xBC]) + (len(body) + 4).to_bytes(2, "big") + body
    return head + crc32_mpeg(head).to_bytes(4, "big")


def pes(data, pts):
    if pts is None:
        hdr = bytes([0x80, 0x00, 0])
    else:
        hdr = bytes([
            0x84, 0x80, 5,
            0x21 | ((pts >> 29) & 0x0E), (pts >> 22) & 0xFF,
            ((pts >> 14) & 0xFE) | 1, (pts >> 7) & 0xFF, ((pts << 1) & 0xFE) | 1,
        ])
    return bytes([0, 0, 1, STREAM_ID]) + (len(hdr) + len(data)).to_bytes(2, "big") + hdr + data


def pack(au, n):
    """Access unit n as its pack: the map ahead of a random access point,
    then the unit in PES packets, the PTS on the first."""
    pts = (n * 3000) & 0x1FFFFFFFF
    types = {nal[1] & 0x1F for nal in au}
    out = bytearray(pack_header(pts))
    if types & {2, 4, 7}:
        out += system_header() + stream_map()
    data = b"".join(b"\x00\x00\x00\x01" + nal for nal in au)
    first = True
    while data or first:
        room = PES_MAX - (8 if first else 3)
        out += pes(data[:room], pts if first else None)
        data = data[room:]
        first = False
    return bytes(out)


def main():
    src, dst = sys.argv[1], sys.argv[2]
    aus = list(access_units(nal_units(open(src, "rb").read())))
    with open(dst, "wb") as f:
        for n, au in enumerate(aus):
            f.write(pack(au, n))
    print(f"{len(aus)} access units", file=sys.stderr)


if __name__ == "__main__":
    main()
