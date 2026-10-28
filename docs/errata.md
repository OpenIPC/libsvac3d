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
