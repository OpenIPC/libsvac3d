# FFmpeg support

`0001-avcodec-add-SVAC-3.0-decoding-via-libsvac3d.patch` adds SVAC 3.0 to
FFmpeg 9.0 (tag `n9.0.2`). It is written in the shape of the `libuavs3d`
wrapper:

| Piece | Files |
|---|---|
| codec ID and descriptor | `AV_CODEC_ID_SVAC3` in `codec_id.h`, `codec_desc.c` |
| decoder | `libavcodec/libsvac3d.c`, `--enable-libsvac3d` |
| parser, one access unit per packet | `libavcodec/svac3_parser.c`, `svac3.h` |
| raw demuxer, `.svac` / `.svac3` | `libavformat/svac3dec.c` |

## Build

```sh
# libsvac3d, into a prefix
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=$PWD/prefix
cmake --build build && cmake --install build

# FFmpeg with the patch
git clone --depth 1 --branch n9.0.2 https://github.com/FFmpeg/FFmpeg.git
git -C FFmpeg apply "$PWD"/ffmpeg/0001-*.patch && cd FFmpeg
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

CI builds a minimal FFmpeg this way and runs the check on every change.

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
