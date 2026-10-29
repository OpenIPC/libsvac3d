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
- I and P pictures, in one patch or several (uniform patches, with the
  loop filters crossing patch boundaries);
- MAEC entropy coding, SRCC coefficients, EIPM (66 intra modes) and MIPF;
- TMVP, the median motion vector predictor, and skip/direct;
- several reference pictures, and library pictures (the library stream
  HiSilicon's SmartP mode interleaves, with RL pictures referencing it);
- deblocking, SAO and CU delta QP.

Every picture of the test corpus is bit-exact against the reference
decoder (`tests/golden/*.framemd5`).

Anything else (B pictures, non-displayed library pictures, non-uniform
patches or loop filters stopping at patch boundaries, the security layer,
the High profile's tools) is refused with
`SVAC3D_ERR_UNSUPPORTED` rather than decoded wrongly.

## Build

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build
ctest --test-dir build
cmake --install build          # libsvac3d.a, svac3d.h, libsvac3d.pc
```

`build/svac3dec stream.svac3 out.yuv` decodes a raw stream to I420.

## Performance

The sample kernels (inverse transform, motion compensation, adding the
residual, deblocking, angular intra prediction, SAO) have AVX2 versions on x86-64, chosen at run time when
the CPU has it, and NEON versions on AArch64; `-DSVAC3D_SIMD=OFF` builds the
C ones only. Every SIMD kernel gives the C kernel's output bits:
`test_dsp` compares them on random inputs, and the corpus checks the
decoded pictures. On the 1080p CBR corpus stream, one core decodes about
200 pictures a second on a Cortex-X925 and about 110 on a Core i9-13900;
at high bit rates the entropy decoder, which is serial, sets the pace.

`svac3d_set_threads()` (`svac3dec -t N`, FFmpeg's `-threads`) decodes N
pictures at a time. A P picture waits only for the rows of its reference
that its motion reaches, as the reference's thread finishes them, and
each picture has its reconstruction and loop filters follow its parser on
two threads of its own, so up to three threads per picture are busy.
Pictures come out in the same order and with the same bits whatever the
number, up to N pictures later than with one thread; `svac3d_flush()`
returns the last of them at the end of the stream.

On a Core Ultra 5 245K (six P-cores):

| stream | 1 thread | 2 threads | 4 threads |
|---|---|---|---|
| 1080p CBR, 90 pictures | 0.45 s | 0.13 s | 0.13 s |
| 2560x1440, 295 pictures | 3.25 s | 0.89 s | 0.79 s |
| 1080p QP 22, 120 pictures, an I picture in six | 1.58 s | 0.76 s | 0.68 s |

Threads cost 10-30% more CPU time than one thread does. Streams with many I
pictures gain least: nothing overlaps an I picture's entropy decoding.

## API

`source/svac3/svac3d.h` is the installed interface:

```c
svac3d_t *d = svac3d_open();
svac3d_set_threads(d, 4);                         /* optional */
svac3d_decode(d, buf, len, on_picture, opaque);   /* Annex-B, any whole NAL units */
svac3d_flush(d, on_picture, opaque);              /* at the end, with threads */
svac3d_close(d);
```

`on_picture` receives each decoded picture, cropped to the display size.
`svac3d_decode_tag()` gives each picture a value from the buffer its first
patch came in, which is how the FFmpeg wrapper keeps packet timestamps on
pictures that come out later.

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
