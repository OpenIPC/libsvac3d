# Draft errata

Defects found in the SVAC 3.0 public-comment draft (national standard plan
20243294-T-312) while implementing it, and how this decoder treats them.

| Where | Defect | Treatment |
|---|---|---|
| 表31 picture header | `picture_privacy_enable_flag` has no descriptor | read as u(1) |
| 表31 picture header | condition spelled `DeblckingFilterDisableFlag` | read as `DeblockingFilterDisableFlag` from the PPS |
| 5.2.1.7.2 heading | carries the wrong table title (表21) | none (editorial) |
| 表13 / `low_delay` semantics | `low_delay` is sent only for profiles 0x40/0x42; when absent it is 1, and Main (0x30) requires 1 | inferred as 1; reading it as 0 desynchronises the SPS tail by 5 bits |
| 表31 `ref_pic_list_set_flag[1]` | absent when `rpl1_index_exist_flag` is 0 | inferred equal to `ref_pic_list_set_flag[0]` |
| 5.3.3.3.3.5 `update_ctx` | when a probability passes 1023 it is reflected (2047 − p), but neither branch flips the MPS | the MPS flips; with MAEC it flips when **either** `lgPmps0` or `lgPmps1` reflects. Only this reading parses the corpus: with no flip, or a flip on `lgPmps0` alone, the first IDR picture desynchronises |
| 表36 `block()`, SRCC branch | chroma blocks are scanned with the CU's luma `blockWidth`/`blockHeight`; only the run-level branch halves them | chroma blocks are half the CU size (4:2:0); the unhalved reading desynchronises the first IDR picture |
| 5.3.3.3.3.2 `decode_decision` | the MAEC `lgPmps` expressions have unbalanced parentheses | `lgPmps = (lgPmps0 + lgPmps1 + 1) >> 3`; for `CtxWeight`, each side is `(lgPmps0 + lgPmps1 + 1) >> 1` |
| 5.4.11.1 deblocking order | "for each filter block, first the vertical then the horizontal boundary", filter blocks in raster order | all vertical edges of the picture are filtered before any horizontal edge. The per-block order leaves 65 000 luma samples of the first IDR picture different from the reference decoder; the picture-wide order is bit-exact |
| 5.4.7.3.2, 表137, 5.4.8.1 | superscripts and Greek letters are lost in the text: `(V + 24) >> 5`, `2BitDepth-1`, the α/β of the Bs derivation | read as `(V + 2^4) >> 5`, `2^(BitDepth−1)`, and β for the flatness tests (`|p0−p1|`, `|p0−p2|`, `|p1−q1|`) with α only for `|p0−q0|`, as in AVS3 |
| 5.4.8.1.4.2 / 5.4.8.1.5.2 clipping | luma angular output is clipped only with MIPF, chroma only without | always clipped; without MIPF the filter taps are non-negative, so the two readings are identical |
