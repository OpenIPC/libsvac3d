# Test corpus

| Stream | Source | Content |
|---|---|---|
| `streams/cv608_synth_1080p_cbr2m.svac3` | HiSilicon Hi3516CV608 hardware SVAC3 encoder, profile 0x30, 1920x1080, 90 frames, CBR 2 Mbit/s, GOP 30, I/P only | synthetic: moving diagonal gradient XOR 3-bit noise, a bouncing 200x200 white box, fixed chroma ramps |
| `streams/cv608_cbr_720p_gop10.svac3` | the same encoder, 1280x720, 40 frames, CBR 1 Mbit/s, GOP 10 | the same synthetic content |
| `streams/cv608_dualp_1080p.svac3` | 1920x1080, 40 frames, CBR 2 Mbit/s, GOP 20, DualP (a refresh P every 4 frames) | the same |
| `streams/cv608_fixqp22_1080p.svac3` | 1920x1080, first 6 of 30 frames, fixed QP 22/24 (I/P): dense coefficients | the same |
| `streams/cv608_fixqp48_1080p.svac3` | 1920x1080, 30 frames, fixed QP 48/50: coarse, mostly skipped | the same |
| `streams/cv608_smartp_1080p.svac3` | 1920x1080, first 45 of 60 frames, CBR 2 Mbit/s, GOP 20, SmartP with a 40-frame background interval: library pictures (NAL type 17, a library-stream SPS) every 40 frames, RL pictures (type 14) referencing them every 20, and P pictures with two references (the previous picture and the library picture) | the same |
| `streams/cv608_patch4_1080p.svac3` | 1920x1080, 30 frames, CBR 2 Mbit/s, GOP 15, slice split every 4 LCU rows: 9 patches per picture | the same |
| `streams/cv608_patch2_720p.svac3` | 1280x720, 20 frames, CBR 1 Mbit/s, GOP 10, split every 2 LCU rows: 12 patches per picture | the same |

The cv608 streams other than the first were captured with a small program that feeds synthetic
frames to the encoder through the SDK's video buffer pool, with the
encoder's rate-control and GOP modes set as above. The golden hashes of all
of them come from the reference decoder. A stream cut short ("first N of M
frames") ends on an access-unit boundary, and its hashes are those of the
same pictures of the whole stream.

`golden/*.framemd5` holds the per-frame MD5 of the decoded I420 picture
(Y, then U, then V, cropped to the display size, no padding). Decoding must
be bit-exact against these.

`local/` is git-ignored: streams we may use for testing but not redistribute.
