#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-3-Clause
"""Reference parser for SVAC3 high-level syntax.

Dumps every NAL unit header plus the SPS / PPS / picture header / patch
header fields as "key=value" lines, one block per NAL. The C decoder's
header dump (svac3dec --dump-headers) must produce the same text; this
script is the oracle for that test.

Section and table numbers refer to the SVAC 3.0 public-comment draft,
national standard plan 20243294-T-312 ("公共安全视频监控数字视音频编解码
技术要求"), which revises GB/T 25724-2017:
  5.2.1.2  表12  NAL unit
  5.2.1.3  表13  sequence parameter set
  5.2.1.4  表14  reference picture list set
  5.2.1.5  表15  weight quant matrix
  5.2.1.8  表30  picture parameter set
  5.2.1.9  表31  picture header
  5.2.1.10 表32  patch data (header part only)

Sub-structures this script does not parse yet raise NotImplementedError
instead of guessing, so a stream that needs them fails loudly.
"""
import sys

NAL_NAMES = {
    0: "FORBIDDEN", 1: "NRAP_PATCH", 2: "RAPI_PATCH", 3: "PIC_HEAD",
    4: "IDR_PATCH", 5: "EXTENSION", 6: "SEI", 7: "SPS", 8: "PPS",
    9: "SEC_PS", 10: "AUTH_DATA", 11: "EOS", 12: "CRR_NDP_PATCH",
    14: "RL_PATCH", 15: "PIC_DL", 16: "EOCVS", 17: "CRR_DP_PATCH",
    18: "CRR_DL_PATCH", 19: "CRR_PRIVACY",
}
PATCH_TYPES = (1, 2, 4, 12, 14, 17, 18, 19)


def split_annexb(data):
    """Yield NAL unit payloads (without start codes)."""
    starts = []
    i, n = 0, len(data)
    while i + 3 <= n:
        if data[i] == 0 and data[i + 1] == 0 and data[i + 2] == 1:
            starts.append(i + 3)
            i += 3
        else:
            i += 1
    for k, s in enumerate(starts):
        e = starts[k + 1] - 3 if k + 1 < len(starts) else n
        # trailing_zero_8bits and the zero_byte of a 4-byte start code
        while e > s and data[e - 1] == 0:
            e -= 1
        yield data[s:e]


def unescape(payload):
    """Remove emulation_prevention_three_byte (表12): 00 00 03 -> 00 00."""
    out = bytearray()
    zeros = 0
    for b in payload:
        if zeros >= 2 and b == 3:
            zeros = 0
            continue
        out.append(b)
        zeros = zeros + 1 if b == 0 else 0
    return bytes(out)


class Bits:
    def __init__(self, data):
        self.d, self.p = data, 0

    def u(self, n):
        v = 0
        for _ in range(n):
            if self.p >= len(self.d) * 8:
                raise EOFError("read past end of RBSP")
            v = (v << 1) | ((self.d[self.p >> 3] >> (7 - (self.p & 7))) & 1)
            self.p += 1
        return v

    def ue(self):
        z = 0
        while self.u(1) == 0:
            z += 1
            if z > 32:
                raise ValueError("ue(v) longer than 32 bits")
        return (1 << z) - 1 + self.u(z)

    def se(self):
        k = self.ue()
        return (k + 1) // 2 if k & 1 else -(k // 2)

    def byte_aligned(self):
        return self.p % 8 == 0

    def trailing_ok(self):
        """rbsp_trailing_bits: a 1 then zeros to the end of the RBSP."""
        rest = len(self.d) * 8 - self.p
        if rest <= 0:
            return False
        if self.u(1) != 1:
            return False
        return all(self.u(1) == 0 for _ in range(len(self.d) * 8 - self.p))


class Parser:
    def __init__(self, out):
        self.out = out
        self.sps = None
        self.pps = None
        self.ph = None

    def emit(self, k, v):
        self.out.write(f"{k}={v}\n")

    # 表14
    def rpl(self, b, sps, lst, idx, prefix):
        r = {}
        if sps["library_picture_enable_flag"] and not sps["library_stream_flag"]:
            r["reference_to_library_enable_flag"] = b.u(1)
        else:
            r["reference_to_library_enable_flag"] = 0
        n = b.ue()
        r["num_of_ref_pic"] = n
        deltas = []
        for i in range(n):
            lib = b.u(1) if r["reference_to_library_enable_flag"] else 0
            if lib:
                deltas.append(("lib", b.ue()))
            else:
                a = b.ue()
                s = b.u(1) if a > 0 else 0
                deltas.append(-a if s else a)
        r["delta_doi"] = deltas
        self.emit(f"{prefix}.num_of_ref_pic", n)
        self.emit(f"{prefix}.delta_doi", ",".join(map(str, deltas)))
        return r

    # 表15
    def wq_matrix(self, b, prefix):
        m = [b.ue() for _ in range(16 + 64)]
        self.emit(f"{prefix}.weight_quant_matrix", ",".join(map(str, m)))

    # 表13
    def parse_sps(self, b):
        s = {}
        e = self.emit

        def f(name, n):
            s[name] = b.u(n)
            e(name, s[name])
            return s[name]

        f("profile_id", 8)
        f("level_id", 8)
        f("temporal_nesting_flag", 1)
        f("library_picture_enable_flag", 1)
        f("library_stream_flag", 1)
        if s["library_picture_enable_flag"]:
            f("duplicate_sequence_header_flag", 1)
            f("library_picture_mode_index", 2)
        f("rapi_nalu_type_enable_flag", 1)
        high = s["profile_id"] in (0x40, 0x42)
        s.update(num_of_layers_minus1=0, ssvc_enable_flag=0, privacy_enable_flag=0,
                 privacy_mode_index=0, reference_picture_resampling_enable_flag=0,
                 rcm_enable_flag=0)
        if high:
            f("num_of_layers_minus1", 2)
            f("ssvc_enable_flag", 1)
            f("privacy_enable_flag", 1)
            if s["privacy_enable_flag"]:
                f("privacy_mode_index", 2)
            if not s["ssvc_enable_flag"]:
                f("reference_picture_resampling_enable_flag", 1)
            f("rcm_enable_flag", 1)
            if s["rcm_enable_flag"]:
                f("rcm_mvp_enable_flag", 1)
                f("rcm_mv_precision_type", 1)
        layers = s["num_of_layers_minus1"] + 1
        if layers > 1:
            raise NotImplementedError("multi-layer SPS (SSVC/TSVC layers)")
        f("horizontal_size", 14)
        f("vertical_size", 14)
        f("pic_width_in_luma", 14)
        f("pic_height_in_luma", 14)
        if s["reference_picture_resampling_enable_flag"]:
            f("rpr_pic_width_in_luma", 14)
            f("rpr_pic_height_in_luma", 14)
        f("chroma_format", 2)
        f("sample_precision", 3)
        if s["profile_id"] == 0x42:
            f("encoding_precision", 3)
        f("aspect_ratio", 4)
        f("frame_rate_code", 4)
        f("bit_rate_lower", 18)
        f("bit_rate_upper", 12)
        if high:
            f("low_delay", 1)
        else:
            s["low_delay"] = 1  # inferred when absent (semantics of low_delay)
            e("low_delay", "1(inferred)")
        f("temporal_id_enable_flag", 1)
        f("bbv_buffer_size", 18)
        f("max_dpb_size_minus1", 4)
        f("rpl1_index_exist_flag", 1)
        f("rpl1_same_as_rpl0_flag", 1)
        s["rpls"] = [[], []]
        n0 = b.ue()
        e("num_ref_pic_list_set[0]", n0)
        for j in range(n0):
            s["rpls"][0].append(self.rpl(b, s, 0, j, f"rpls[0][{j}]"))
        if not s["rpl1_same_as_rpl0_flag"]:
            n1 = b.ue()
            e("num_ref_pic_list_set[1]", n1)
            for j in range(n1):
                s["rpls"][1].append(self.rpl(b, s, 1, j, f"rpls[1][{j}]"))
        else:
            s["rpls"][1] = s["rpls"][0]
        s["num_ref_default_active_minus1"] = [b.ue(), b.ue()]
        e("num_ref_default_active_minus1", ",".join(map(str, s["num_ref_default_active_minus1"])))
        f("log2_lcu_size_minus2", 3)
        f("log2_min_cu_size_minus2", 2)
        f("log2_max_part_ratio_minus2", 2)
        f("max_split_times_minus6", 3)
        f("log2_min_qt_size_minus2", 3)
        f("log2_max_bt_size_minus2", 3)
        f("log2_max_eqt_size_minus3", 2)
        f("weight_quant_enable_flag", 1)
        if s["weight_quant_enable_flag"]:
            f("load_seq_weight_quant_data_flag", 1)
            if s["load_seq_weight_quant_data_flag"]:
                self.wq_matrix(b, "sps")
        for name in ("st_enable_flag", "sao_enable_flag", "alf_enable_flag",
                     "affine_enable_flag", "smvd_enable_flag", "ipcm_enable_flag",
                     "amvr_enable_flag"):
            f(name, 1)
        f("num_of_hmvp_cand", 4)
        f("umve_enable_flag", 1)
        s["emvr_enable_flag"] = 0
        if s["num_of_hmvp_cand"] != 0 and s["amvr_enable_flag"]:
            f("emvr_enable_flag", 1)
        f("intra_pf_enable_flag", 1)
        f("tscpm_enable_flag", 1)
        f("dt_enable_flag", 1)
        if s["dt_enable_flag"]:
            f("log2_max_dt_size_minus4", 2)
        f("pbt_enable_flag", 1)
        s["eipm_enable_flag"] = 1
        e("eipm_enable_flag", "1(implicit)")
        f("pmc_enable_flag", 1)
        f("iip_enable_flag", 1)
        f("sawp_enable_flag", 1)
        f("mipf_enable_flag", 1)
        s["asr_enable_flag"] = 0
        if s["affine_enable_flag"]:
            f("asr_enable_flag", 1)
        f("awp_enable_flag", 1)
        f("tmvp_enable_flag", 1)
        s["sbtmvp_enable_flag"] = s["etmvp_mvap_enable_flag"] = 0
        if s["tmvp_enable_flag"]:
            f("sbtmvp_enable_flag", 1)
            f("etmvp_mvap_enable_flag", 1)
        for name in ("dmvr_enable_flag", "bio_enable_flag", "bgc_enable_flag",
                     "inter_pf_enable_flag", "inter_pc_enable_flag", "obmc_enable_flag",
                     "sbt_enable_flag", "ist_enable_flag"):
            f(name, 1)
        s["ts_enable_flag"] = 0
        if s["ist_enable_flag"]:
            f("ts_enable_flag", 1)
        for name in ("cu_dqp_enable_flag", "srcc_enable_flag", "maec_enable_flag",
                     "dbr_enable_flag", "esao_enable_flag", "ccsao_enable_flag"):
            f(name, 1)
        if s["esao_enable_flag"]:
            s["sao_enable_flag"] = 0
            e("sao_enable_flag", "0(forced by esao)")
        s["ealf_enable_flag"] = 0
        if s["alf_enable_flag"]:
            f("ealf_enable_flag", 1)
        s.update(ibc_enable_flag=0, isc_enable_flag=0, fimc_enable_flag=0, nn_tools_set_hook=0)
        if high:
            f("ibc_enable_flag", 1)
            f("isc_enable_flag", 1)
            if s["ibc_enable_flag"] or s["isc_enable_flag"]:
                f("num_of_intra_hmvp_cand", 4)
            f("fimc_enable_flag", 1)
            f("nn_tools_set_hook", 8)
            if s["nn_tools_set_hook"] & 1:
                s["num_of_nn_filter_minus1"] = b.ue()
                e("num_of_nn_filter_minus1", s["num_of_nn_filter_minus1"])
        if s["low_delay"] == 0:
            f("output_reorder_delay", 5)
        f("cross_patch_loop_filter_enable_flag", 1)
        f("ref_colocated_patch_flag", 1)
        f("stable_patch_flag", 1)
        if s["stable_patch_flag"]:
            f("uniform_patch_flag", 1)
            if s["uniform_patch_flag"]:
                s["patch_width_minus1"] = b.ue()
                s["patch_height_minus1"] = b.ue()
                e("patch_width_minus1", s["patch_width_minus1"])
                e("patch_height_minus1", s["patch_height_minus1"])
        f("reserved_bits", 2)
        self.sps = s
        return s

    # 表30
    def parse_pps(self, b):
        s, p, e = self.sps, {}, self.emit

        def f(name, n):
            p[name] = b.u(n)
            e(name, p[name])
            return p[name]

        f("fixed_picture_qp_flag", 1)
        p["cu_delta_qp_flag"] = 0
        if not p["fixed_picture_qp_flag"] and s["cu_dqp_enable_flag"]:
            f("cu_delta_qp_flag", 1)
            if p["cu_delta_qp_flag"]:
                f("cu_qp_group_size_log2_minus3", 2)
        f("deblocking_filter_disable_flag", 1)
        p["deblocking_filter_parameter_flag"] = 0
        if not p["deblocking_filter_disable_flag"]:
            f("deblocking_filter_type", 1)
            f("deblocking_filter_parameter_flag", 1)
        if s["weight_quant_enable_flag"]:
            f("picture_weight_quant_enable_flag", 1)
            if p["picture_weight_quant_enable_flag"]:
                f("picture_weight_quant_data_index", 2)
                if p["picture_weight_quant_data_index"] == 1:
                    f("reserved_bits", 1)
                    f("weight_quant_param_index", 2)
                    f("weight_quant_model", 2)
                    if p["weight_quant_param_index"] in (1, 2):
                        d = [b.se() for _ in range(6)]
                        e(f"weight_quant_param_delta{p['weight_quant_param_index']}",
                          ",".join(map(str, d)))
                elif p["picture_weight_quant_data_index"] == 2:
                    self.wq_matrix(b, "pps")
        if s["ibc_enable_flag"]:
            f("picture_ibc_enable_flag", 1)
        if s["isc_enable_flag"]:
            f("picture_isc_os_enable_flag", 1)
            f("picture_isc_evs_ubvs_enable_flag", 1)
        if s["fimc_enable_flag"]:
            f("picture_fimc_enable_flag", 1)
        if s["affine_enable_flag"]:
            f("affine_subblock_size_flag", 1)
        if s["awp_enable_flag"] or s["sawp_enable_flag"]:
            f("picture_awp_refine_index", 1)
        self.pps = p
        return p

    # 表31
    def parse_ph(self, b):
        s, p, h, e = self.sps, self.pps, {}, self.emit

        def f(name, n):
            h[name] = b.u(n)
            e(name, h[name])
            return h[name]

        f("decode_order_index", 8)
        if s["library_stream_flag"]:
            h["library_picture_index"] = b.ue()
            e("library_picture_index", h["library_picture_index"])
            f("library_patch_num_minus1", 8)
        t = f("picture_coding_type", 2)
        if s["reference_picture_resampling_enable_flag"]:
            f("picture_rpr_flag", 1)
        if s["profile_id"] != 0x30 and s["tmvp_enable_flag"]:
            f("picture_tmvp_enable_flag", 1)
        if t != 1:
            f("random_access_decodable_flag", 1)
        f("bbv_delay", 32)
        if t == 1:
            f("time_code_flag", 1)
            if h["time_code_flag"]:
                f("time_code", 24)
        if s["low_delay"] == 0:
            h["picture_output_delay"] = b.ue()
            e("picture_output_delay", h["picture_output_delay"])
        else:
            h["bbv_check_times"] = b.ue()
            e("bbv_check_times", h["bbv_check_times"])
        f("ref_pic_list_set_flag[0]", 1)
        if h["ref_pic_list_set_flag[0]"]:
            if len(s["rpls"][0]) > 1:
                h["ref_pic_list_set_index[0]"] = b.ue()
                e("ref_pic_list_set_index[0]", h["ref_pic_list_set_index[0]"])
        else:
            self.rpl(b, s, 0, len(s["rpls"][0]), "ph.rpl[0]")
        if s["rpl1_index_exist_flag"]:
            f("ref_pic_list_set_flag[1]", 1)
        else:
            # inferred equal to ref_pic_list_set_flag[0] (semantics, 5.2.2)
            h["ref_pic_list_set_flag[1]"] = h["ref_pic_list_set_flag[0]"]
            e("ref_pic_list_set_flag[1]", f"{h['ref_pic_list_set_flag[1]']}(inferred)")
        if h["ref_pic_list_set_flag[1]"]:
            if s["rpl1_index_exist_flag"] and len(s["rpls"][1]) > 1:
                h["ref_pic_list_set_index[1]"] = b.ue()
                e("ref_pic_list_set_index[1]", h["ref_pic_list_set_index[1]"])
        else:
            self.rpl(b, s, 1, len(s["rpls"][1]), "ph.rpl[1]")
        if t != 1:
            f("num_ref_active_override_flag", 1)
            if h["num_ref_active_override_flag"]:
                h["num_ref_active_minus1[0]"] = b.ue()
                e("num_ref_active_minus1[0]", h["num_ref_active_minus1[0]"])
                if t == 3:
                    h["num_ref_active_minus1[1]"] = b.ue()
                    e("num_ref_active_minus1[1]", h["num_ref_active_minus1[1]"])
        f("picture_qp", 7)
        if s["privacy_mode_index"] in (2, 3):
            # 表31 gives this element no descriptor; u(1) assumed (docs/errata.md)
            f("picture_privacy_enable_flag", 1)
        if t == 2:
            f("reserved_bits", 1)
        if not p["deblocking_filter_disable_flag"]:
            if p["deblocking_filter_parameter_flag"]:
                h["alpha_c_offset"] = b.se()
                h["beta_offset"] = b.se()
                e("alpha_c_offset", h["alpha_c_offset"])
                e("beta_offset", h["beta_offset"])
            if s["dbr_enable_flag"]:
                for d in ("v", "h"):
                    en = f(f"picture_dbr_{d}_enable_flag", 1)
                    if en:
                        f(f"dbr_{d}_offset0_minus1", 2)
                        f(f"dbr_{d}_offset1_minus1", 2)
                    alt = f(f"picture_alt_dbr_{d}_enable_flag", 1)
                    if alt:
                        f(f"dbr_{d}_alt_offset0_minus1", 2)
                        f(f"dbr_{d}_alt_offset1_minus1", 2)
                    if en or alt:
                        f(f"dbr_{d}_threshold_minus1", 1)
        f("chroma_quant_param_disable_flag", 1)
        if not h["chroma_quant_param_disable_flag"]:
            h["chroma_quant_param_delta_cb"] = b.se()
            h["chroma_quant_param_delta_cr"] = b.se()
            e("chroma_quant_param_delta_cb", h["chroma_quant_param_delta_cb"])
            e("chroma_quant_param_delta_cr", h["chroma_quant_param_delta_cr"])
        if s["esao_enable_flag"]:
            raise NotImplementedError("esao_parameter_picture_header_set")
        if s["ccsao_enable_flag"]:
            raise NotImplementedError("ccsao_parameter_picture_header_set")
        if s["alf_enable_flag"]:
            en = [f(f"picture_alf_enable_flag[{c}]", 1) for c in range(3)]
            if any(en):
                raise NotImplementedError("alf_parameter_set")
        if s["pmc_enable_flag"]:
            f("picture_pmc_param_index", 1)
        if s["umve_enable_flag"] and t != 1:
            f("picture_umve_step_set_index", 1)
        if s["obmc_enable_flag"] and t == 3:
            f("picture_obmc_blending_set_index", 1)
        if t != 1 and s["inter_pc_enable_flag"]:
            f("picture_inter_pc_skip_flag", 1)
        if s["ts_enable_flag"]:
            f("picture_ts_enable_flag", 1)
        if s["nn_tools_set_hook"] & 1:
            raise NotImplementedError("picture NN filter flags")
        if s["rcm_enable_flag"] and t != 1:
            raise NotImplementedError("RCM picture parameters")
        self.ph = h
        return h

    # 表32, header part only
    def parse_patch(self, b):
        s, p, e = self.sps, self.pps, self.emit
        e("patch_index", b.u(8))
        if not p["fixed_picture_qp_flag"]:
            e("fixed_patch_qp_flag", b.u(1))
            e("patch_qp", b.u(7))
        if s["sao_enable_flag"]:
            e("patch_sao_enable_flag", ",".join(str(b.u(1)) for _ in range(3)))
        align_ok = True
        while not b.byte_aligned():
            if b.u(1) != 1:
                align_ok = False
        e("aec_byte_alignment", "ok" if align_ok else "BAD")
        e("aec_data_offset", b.p // 8)

    def nal(self, idx, raw):
        e = self.emit
        hdr = (raw[0] << 8) | raw[1]
        t = hdr & 31
        self.out.write(f"[nal {idx} {NAL_NAMES.get(t, 'RESERVED')}]\n")
        e("forbidden_zero_bit", hdr >> 15)
        e("encryption_idc", (hdr >> 13) & 3)
        e("authentication_idc", (hdr >> 11) & 3)
        e("authentication_data_id", (hdr >> 10) & 1)
        e("layer_id", (hdr >> 8) & 3)
        e("temporal_id", (hdr >> 5) & 7)
        e("nal_unit_type", t)
        if (hdr >> 13) & 3:
            e("payload", "encrypted (not parsed)")
            return
        b = Bits(unescape(raw[2:]))
        if t == 7:
            self.parse_sps(b)
        elif t == 8:
            self.parse_pps(b)
        elif t == 3:
            self.parse_ph(b)
        elif t in PATCH_TYPES:
            self.parse_patch(b)
            return  # AEC data follows; no RBSP trailing check here
        else:
            e("payload_bytes", len(b.d))
            return
        e("rbsp_trailing_bits", "ok" if b.trailing_ok() else "BAD")


def main():
    if len(sys.argv) != 2:
        sys.exit(f"usage: {sys.argv[0]} stream.svac3")
    data = open(sys.argv[1], "rb").read()
    p = Parser(sys.stdout)
    for i, raw in enumerate(split_annexb(data)):
        if len(raw) < 2:
            continue
        p.nal(i, raw)


if __name__ == "__main__":
    main()
