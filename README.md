# libsvac3d

An open-source SVAC 3.0 video decoder, with FFmpeg support.

SVAC 3.0 is the Chinese surveillance video codec (GB/T 25724, *Surveillance
Video and Audio Coding*), the successor of SVAC 2.0. Its third edition is
still a public-comment draft, national standard plan 20243294-T-312, and
the explanatory note describes it as built on modified AVS3 core coding
technology. This decoder is written from that draft. Where the draft's text
disagrees with what reference streams need, `docs/errata.md` says so and
says what the decoder does instead.

## What it decodes

The Main profile (0x30) tool set that HiSilicon encoders emit:
- 8-bit 4:2:0;
- I and P pictures, one patch per picture;
- MAEC entropy coding, SRCC coefficients, EIPM (66 intra modes) and MIPF;
- TMVP, the median motion vector predictor, and skip/direct;
- deblocking, SAO and CU delta QP.

Every picture of the test corpus is bit-exact against the reference
decoder (`tests/golden/*.framemd5`).

Anything else (B pictures, library pictures, several patches, the
security layer, the High profile's tools) is refused with
`SVAC3D_ERR_UNSUPPORTED` rather than decoded wrongly.

## Build

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build
ctest --test-dir build
cmake --install build          # libsvac3d.a, svac3d.h, libsvac3d.pc
```

`build/svac3dec stream.svac3 out.yuv` decodes a raw stream to I420.

## API

`source/svac3/svac3d.h` is the installed interface:

```c
svac3d_t *d = svac3d_open();
svac3d_decode(d, buf, len, on_picture, opaque);   /* Annex-B, any whole NAL units */
svac3d_close(d);
```

`on_picture` receives each decoded picture, cropped to the display size.

## FFmpeg

`ffmpeg/` holds a patch for FFmpeg 9.0 that adds:
- `AV_CODEC_ID_SVAC3`;
- a parser;
- a raw `.svac`/`.svac3` demuxer;
- the `libsvac3d` decoder.

Its README has the build steps. CI builds it and checks its framemd5
output against the golden hashes.

## Layout

| Path | What |
|---|---|
| `source/svac3/` | the decoder: NAL and header parsing, MAEC, patch decoding, intra and inter prediction, transforms, loop filters, the picture buffer |
| `tools/` | `svac3dec`, `svac3dump` (headers), `svac3patch` (patch statistics), `svac3_parse.py` (an independent header parser), `gen_tables.py` (regenerates `svac3_tables.h` from the draft's text) |
| `tests/` | unit tests, the corpus, golden per-frame MD5s, fuzz targets |
| `ffmpeg/` | the FFmpeg patch |
| `source/decoder`, `source/decore` | uavs3d, the AVS3 decoder this repository was forked from; built as `libuavs3d`, unchanged and unused by libsvac3d |

## Licence

BSD 3-clause; see `COPYING`. The uavs3d code is copyright Peking University
Shenzhen Graduate School, Peng Cheng Laboratory and Guangdong Bohua UHD
Innovation Corporation. The SVAC3 decoder is copyright OpenIPC
contributors. The FFmpeg patch is LGPL-2.1-or-later, as FFmpeg is.

The SVAC 3.0 draft itself is not in this repository; the code cites its
sections and tables.
