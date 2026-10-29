#!/usr/bin/env python3
"""Send an SVAC3 byte stream as RTP over UDP, and write the SDP to receive it.

The payload format is the one rtpdec_svac3.c reads (see the comment at its
top): single NAL unit packets, and fragmentation units of type 28 with an
S|E|R|FuType header. One access unit per RTP timestamp, the marker on its
last packet.

usage: rtp_send.py stream.svac3 out.sdp port [mtu [split]]
"split" puts sprop-sps and sprop-pps on fmtp lines of their own and leaves
them out of the first access unit, so the first picture decodes only from
what the SDP gave. Later units keep theirs: a stream with library pictures
re-sends its SPS between the library and the pictures that use it.
Writes the SDP, then waits for a line on stdin (the receiver is up), then
sends, paced so a local receiver does not drop.
"""
import base64
import socket
import sys
import time

FU = 28
PT = 102


def nal_units(data):
    """Start-code-stripped NAL units of an Annex-B stream."""
    starts = []
    i = 0
    while True:
        i = data.find(b"\x00\x00\x01", i)
        if i < 0:
            break
        starts.append(i + 3)
        i += 3
    for k, s in enumerate(starts):
        end = starts[k + 1] - 3 if k + 1 < len(starts) else len(data)
        nal = data[s:end]
        while nal.endswith(b"\x00") and k + 1 < len(starts):
            nal = nal[:-1]  # the next unit's 4-byte start code
        if len(nal) >= 2:
            yield nal


PATCHES = {1, 2, 4, 12, 14, 17, 18, 19}
# What opens an access unit once its patches have been seen: the rule of
# ff_svac3_nal_starts_au() in 0001's parser.
STARTERS = {3, 5, 6, 7, 8, 9}


def access_units(nals):
    """Group NAL units into access units the way the FFmpeg parser does: a
    picture header, extension, SEI, SPS, PPS or security parameter set that
    follows a patch opens the next one."""
    au, patch_seen = [], False
    for nal in nals:
        t = nal[1] & 0x1F
        if patch_seen and t in STARTERS:
            yield au
            au, patch_seen = [], False
        au.append(nal)
        patch_seen = patch_seen or t in PATCHES
    if au:
        yield au


def payloads(nal, mtu):
    if len(nal) <= mtu:
        yield nal
        return
    hdr = bytes([nal[0], (nal[1] & 0xE0) | FU])
    t = nal[1] & 0x1F
    body = nal[2:]
    chunk = mtu - 3
    for off in range(0, len(body), chunk):
        s = 0x80 if off == 0 else 0
        e = 0x40 if off + chunk >= len(body) else 0
        yield hdr + bytes([s | e | t]) + body[off:off + chunk]


def main():
    path, sdp_path, port = sys.argv[1], sys.argv[2], int(sys.argv[3])
    mtu = int(sys.argv[4]) if len(sys.argv) > 4 else 1200
    split = len(sys.argv) > 5 and sys.argv[5] == "split"
    data = open(path, "rb").read()
    aus = list(access_units(nal_units(data)))
    sps = next(n for au in aus for n in au if n[1] & 0x1F == 7)
    pps = next(n for au in aus for n in au if n[1] & 0x1F == 8)
    sprop_sps = f"sprop-sps={base64.b64encode(sps).decode()}"
    sprop_pps = f"sprop-pps={base64.b64encode(pps).decode()}"
    fmtp = (f"a=fmtp:{PT} {sprop_sps}\r\na=fmtp:{PT} {sprop_pps}\r\n" if split
            else f"a=fmtp:{PT} {sprop_sps};{sprop_pps}\r\n")
    with open(sdp_path, "w") as f:
        f.write(
            "v=0\r\no=- 0 0 IN IP4 127.0.0.1\r\ns=svac3\r\n"
            "c=IN IP4 127.0.0.1\r\nt=0 0\r\n"
            f"m=video {port} RTP/AVP {PT}\r\n"
            f"a=rtpmap:{PT} SVAC3/90000\r\n" + fmtp)
    sys.stdout.write("sdp\n")
    sys.stdout.flush()
    sys.stdin.readline()

    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 1 << 22)
    seq, ssrc = 0, 0x5AC3
    if split:
        aus[0] = [nal for nal in aus[0] if nal[1] & 0x1F not in (7, 8)]
    for n, au in enumerate(aus):
        ts = (n * 3000) & 0xFFFFFFFF
        pkts = [p for nal in au for p in payloads(nal, mtu)]
        for i, p in enumerate(pkts):
            marker = 0x80 if i == len(pkts) - 1 else 0
            hdr = bytes([0x80, marker | PT]) + seq.to_bytes(2, "big") + \
                ts.to_bytes(4, "big") + ssrc.to_bytes(4, "big")
            s.sendto(hdr + p, ("127.0.0.1", port))
            seq = (seq + 1) & 0xFFFF
            if i % 4 == 3:
                time.sleep(0.001)
        time.sleep(0.004)
    print(f"sent {len(aus)} access units", file=sys.stderr)


if __name__ == "__main__":
    main()
