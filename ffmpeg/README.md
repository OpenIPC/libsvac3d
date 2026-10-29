# FFmpeg support

Three patches, applied in order to FFmpeg 9.0 (tag `n9.0.2`).
`0001-avcodec-add-SVAC-3.0-decoding-via-libsvac3d.patch` adds SVAC 3.0
decoding, in the shape of the `libuavs3d` wrapper.
`0002-avformat-rtpdec-add-an-RTP-depacketizer-for-SVAC-3.0.patch` adds
receiving it over RTP and RTSP.
`0003-avformat-mpeg-read-SVAC-video-from-a-GB-T-28181-program-stream.patch`
reads it from an MPEG-2 program stream, as GB/T 28181 carries it.

| Piece | Files |
|---|---|
| codec ID and descriptor | `AV_CODEC_ID_SVAC3` in `codec_id.h`, `codec_desc.c` |
| decoder | `libavcodec/libsvac3d.c`, `--enable-libsvac3d` |
| parser, one access unit per packet | `libavcodec/svac3_parser.c`, `svac3.h` |
| raw demuxer, `.svac` / `.svac3` | `libavformat/svac3dec.c` |
| RTP depacketizer, `SVAC3/90000` | `libavformat/rtpdec_svac3.c` |
| program stream map type 0x80 | `libavformat/mpeg.c`, `mpeg.h` |

## Build

```sh
# libsvac3d, into a prefix
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=$PWD/prefix
cmake --build build && cmake --install build

# FFmpeg with the patch
git clone --depth 1 --branch n9.0.2 https://github.com/FFmpeg/FFmpeg.git
for p in ffmpeg/0*.patch; do git -C FFmpeg apply "$PWD/$p"; done && cd FFmpeg
PKG_CONFIG_PATH=$PWD/../prefix/lib/pkgconfig ./configure --enable-libsvac3d
make
```

`./ffmpeg -i stream.svac3 out.mp4`, `ffplay stream.svac3` and anything else
built on libavcodec (mpv, go2rtc's ffmpeg mode) then decode SVAC 3.0.

## Check

```sh
./ffmpeg -i tests/streams/cv608_synth_1080p_cbr2m.svac3 -f framemd5 out.framemd5
ffmpeg/check_framemd5.py tests/golden/cv608_synth_1080p_cbr2m.framemd5 out.framemd5
```

`ffmpeg/rtp_check.sh path/to/ffmpeg` sends every corpus stream as RTP over
local UDP (`ffmpeg/rtp_send.py`), receives it through an SDP, and holds the
result to the same golden hashes.

`ffmpeg/ps_check.sh path/to/ffmpeg` writes every corpus stream as a
program stream (`ffmpeg/ps_write.py`) and reads it back with `-f mpeg`,
against the same hashes.

CI builds a minimal FFmpeg this way and runs all three checks on every
change.

## RTP

The SVAC 3.0 draft defines a byte stream and no RTP mapping. The
depacketizer reads the one OpenIPC cameras send over RTSP, which is RFC
7798's shape on SVAC3's two-byte NAL header:

- a single NAL unit packet is the NAL unit, header included;
- a fragmentation unit carries that header with its type replaced by 28,
  which the draft reserves. An FU header follows (`S|E|R|FuType`, with the
  5-bit original type), then a fragment of the bytes after the NAL unit's
  header. The other header fields (security idcs, layer, temporal id) stay
  as they were;
- the marker bit ends an access unit;
- SDP: `a=rtpmap:<pt> SVAC3/90000`, and `sprop-sps` / `sprop-pps` in `fmtp`
  (base64 of whole NAL units), which become the decoder's extradata.

`ffplay rtsp://camera/stream=0` then plays a camera streaming SVAC 3.0.

## Notes

- The demuxer only claims a file whose extension is `.svac` or `.svac3`
  and whose first NAL units are an SPS of profile 0x30/0x40/0x42, a picture
  header and a patch: the start codes and NAL headers alone are shared with
  other codecs.
- The parser cuts before an SPS, PPS, picture header, SEI, extension or
  security parameter set that follows a patch; key frames are pictures
  with an IDR or RAP-I patch, and displayed library pictures.
- Pictures come out in decoding order (low delay, no B pictures).
- A packet may hold any whole number of NAL units and complete several
  pictures (the parser makes it one); each picture is returned. When a
  packet completes some pictures and then fails, the pictures are returned
  first and the error after them.
- A P picture whose reference was lost (decoding from the middle of a
  stream, or after a seek) is an `AVERROR_INVALIDDATA` until the next I
  picture.

## Program stream (GB/T 28181)

GB/T 28181 carries video as an MPEG-2 program stream, with SVAC named
`stream_type` 0x80 in the program stream map. That value is user-private in
ISO/IEC 13818-1, so 0003 reads it as SVAC only on a video stream id
(0xE0-0xEF). `ffmpeg -f mpeg -i capture.ps` then decodes a capture. RTP
carrying a program stream (`PS/90000`) has no depacketizer in FFmpeg; save
the payloads and read the result as a file.
