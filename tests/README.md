# Test corpus

| Stream | Source | Content |
|---|---|---|
| `streams/cv608_synth_1080p_cbr2m.svac3` | HiSilicon Hi3516CV608 hardware SVAC3 encoder, profile 0x30, 1920x1080, 90 frames, CBR 2 Mbit/s, GOP 30, I/P only | synthetic: moving diagonal gradient XOR 3-bit noise, a bouncing 200x200 white box, fixed chroma ramps |

`golden/*.framemd5` holds the per-frame MD5 of the decoded I420 picture
(Y, then U, then V, cropped to the display size, no padding). Decoding must
be bit-exact against these.

`local/` is git-ignored: streams we may use for testing but not redistribute.
