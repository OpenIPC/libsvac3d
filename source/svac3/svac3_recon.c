/* SPDX-License-Identifier: BSD-3-Clause
 * Copyright (c) 2026 OpenIPC contributors
 *
 * SVAC3 intra prediction, dequantisation and inverse DCT2 for 8-bit
 * samples, written from 5.4.8.1, 5.4.7.2 and 5.4.7.3.2.
 */
#include "svac3_recon.h"
#include "svac3_tables.h"
#include "svac3_dsp.h"

#include <string.h>

#define BIT_DEPTH 8
#define MAX_REF   (2 * 64 + 4)

static inline int clip3(int lo, int hi, int v) { return v < lo ? lo : v > hi ? hi : v; }
static inline int clip1(int v) { return clip3(0, (1 << BIT_DEPTH) - 1, v); }

static int ilog2(int v)
{
    int l = 0;
    while ((1 << l) < v)
        l++;
    return l;
}

/* ------------------------------------------------------------------------ */
/* Reference samples, 5.4.8.1.2 / 5.4.8.1.3. r[] above (r[0] the corner), c[]
 * left; both are stored with an offset of 2 so r[-2] and r[-1] exist. */

typedef struct {
    int r[MAX_REF + 2], c[MAX_REF + 2];
    int any_r, any_c;            /* some r[1..M] / c[1..N] available */
    int M, N;
} refs_t;

#define R(t, i) ((t)->r[(i) + 2])
#define C(t, j) ((t)->c[(j) + 2])

static void get_refs(const svac3_frame_t *f, int comp, int x0, int y0, int M, int N,
                     svac3_avail_fn avail, void *opaque, refs_t *t)
{
    const uint8_t *I = f->plane[comp];
    const int s = f->stride[comp], sc = comp ? 1 : 0;   /* chroma -> luma shift */
    const int mid = 1 << (BIT_DEPTH - 1);
#define AV(x, y) ((x) >= 0 && (y) >= 0 && (x) < f->width[comp] && (y) < f->height[comp] && \
                  avail(opaque, (x) << sc, (y) << sc))

    t->M = M;
    t->N = N;
    for (int i = 0; i <= 2 * M; i++)
        R(t, i) = mid;
    for (int j = 0; j <= 2 * N; j++)
        C(t, j) = mid;
    t->any_r = t->any_c = 0;

    for (int i = 1; i <= M; i++)
        if (AV(x0 + i - 1, y0 - 1)) {
            R(t, i) = I[(y0 - 1) * s + x0 + i - 1];
            t->any_r = 1;
        }
    for (int i = M + 1; i <= 2 * M; i++)
        R(t, i) = (t->any_r && AV(x0 + i - 1, y0 - 1)) ? I[(y0 - 1) * s + x0 + i - 1] : R(t, i - 1);
    for (int j = 1; j <= N; j++)
        if (AV(x0 - 1, y0 + j - 1)) {
            C(t, j) = I[(y0 + j - 1) * s + x0 - 1];
            t->any_c = 1;
        }
    for (int j = N + 1; j <= 2 * N; j++)
        C(t, j) = (t->any_c && AV(x0 - 1, y0 + j - 1)) ? I[(y0 + j - 1) * s + x0 - 1] : C(t, j - 1);

    if (AV(x0 - 1, y0 - 1))
        R(t, 0) = I[(y0 - 1) * s + x0 - 1];
    else if (t->any_r)
        R(t, 0) = R(t, 1);           /* cases f) 1) and 3) */
    else if (t->any_c)
        R(t, 0) = C(t, 1);
    C(t, 0) = R(t, 0);
#undef AV
}

/* r[i] for i > 2M is r[2M], likewise for c */
static inline int rr(const refs_t *t, int i) { return R(t, i > 2 * t->M ? 2 * t->M : i); }
static inline int cc(const refs_t *t, int j) { return C(t, j > 2 * t->N ? 2 * t->N : j); }

/* ------------------------------------------------------------------------ */
/* The prediction methods, 5.4.8.1.4.2 (luma) and 5.4.8.1.5.2 (chroma). mode
 * uses the luma numbering: 0 DC, 1 plane, 2 bilinear, 12 vertical,
 * 24 horizontal, others angular. */

static void pred_dc(const refs_t *t, uint8_t *p)
{
    const int M = t->M, N = t->N;
    int v;
    if (t->any_r && t->any_c) {
        int s = 0;
        for (int i = 1; i <= M; i++)
            s += R(t, i);
        for (int j = 1; j <= N; j++)
            s += C(t, j);
        v = ((s + ((M + N) >> 1)) * (4096 / (M + N))) >> 12;
    } else if (t->any_r) {
        int s = 0;
        for (int i = 1; i <= M; i++)
            s += R(t, i);
        v = (s + (M >> 1)) >> ilog2(M);
    } else if (t->any_c) {
        int s = 0;
        for (int j = 1; j <= N; j++)
            s += C(t, j);
        v = (s + (N >> 1)) >> ilog2(N);
    } else {
        v = 1 << (BIT_DEPTH - 1);
    }
    memset(p, v, (size_t)M * N);
}

static void pred_plane(const refs_t *t, uint8_t *p)
{
    static const int ibMult[5] = { 13, 17, 5, 11, 23 };
    static const int ibShift[5] = { 7, 10, 11, 15, 19 };
    const int M = t->M, N = t->N;
    const int imh = ibMult[ilog2(M) - 2], ish = ibShift[ilog2(M) - 2];
    const int imv = ibMult[ilog2(N) - 2], isv = ibShift[ilog2(N) - 2];
    int ih = 0, iv = 0;
    for (int i = 0; i < (M >> 1); i++)
        ih += (i + 1) * (R(t, (M >> 1) + 1 + i) - R(t, (M >> 1) - 1 - i));
    for (int i = 0; i < (N >> 1); i++)
        iv += (i + 1) * (C(t, (N >> 1) + 1 + i) - C(t, (N >> 1) - 1 - i));
    const int ia = (R(t, M) + C(t, N)) << 4;
    /* ih, iv and the products below can be negative: multiply rather than
     * shift left (a left shift of a negative value is undefined in C) */
    const int ib = (ih * 32 * imh + (1 << (ish - 1))) >> ish;
    const int ic = (iv * 32 * imv + (1 << (isv - 1))) >> isv;
    for (int y = 0; y < N; y++)
        for (int x = 0; x < M; x++)
            p[y * M + x] = (uint8_t)clip1((ia + (x - ((M >> 1) - 1)) * ib +
                                           (y - ((N >> 1) - 1)) * ic + 16) >> 5);
}

static void pred_bilinear(const refs_t *t, uint8_t *p)
{
    static const int bilinearWeight[3] = { 21, 13, 7 };
    const int M = t->M, N = t->N, lM = ilog2(M), lN = ilog2(N);
    const int ia = R(t, M), ib = C(t, N);
    int ic;
    if (M == N) {
        ic = (ia + ib + 1) >> 1;
    } else {
        /* ratios 2, 4 and 8; svac3_patch_supported() refuses MaxPartRatio > 8,
         * as the Main profile requires, so the clamp never changes a result */
        int wi = ilog2((M > N ? M : N) / (M < N ? M : N)) - 1;
        if (wi > 2)
            wi = 2;
        const int weight = bilinearWeight[wi];
        const int ishift = ilog2(M < N ? M : N);
        ic = (((ia << lM) + (ib << lN)) * weight + (1 << (ishift + 5))) >> (ishift + 6);
    }
    for (int y = 0; y < N; y++)
        for (int x = 0; x < M; x++) {
            int v = (((ia - C(t, y + 1)) * (x + 1)) * (1 << lN) +
                     ((ib - R(t, x + 1)) * (y + 1)) * (1 << lM) +
                     ((R(t, x + 1) + C(t, y + 1)) << (lM + lN)) +
                     ((ic << 1) - ia - ib) * x * y + (1 << (lM + lN))) >> (lM + lN + 1);
            p[y * M + x] = (uint8_t)clip1(v);
        }
}

static void pred_angular(refs_t *t, int mode, int chroma, int mipf, uint8_t *p)
{
    static const int filterBit[4] = { 7, 7, 7, 6 };
    static const int filterOffset[4] = { 64, 64, 64, 32 };
    const int M = t->M, N = t->N;
    const int16_t *d = svac3_dir_tab[mode];
    const int xyAxis = d[0], xySign = d[1], imx = d[2], isx = d[3], imy = d[4], isy = d[5];
    /* filterIndex: luma switches on the first row/column and M*N <= 64,
     * chroma on the first two and K*L <= 32 */
    const int small = chroma ? (M * N <= 32) : (M * N <= 64);
    const int edge_f = mipf ? (small ? 3 : 2) : 0, in_f = mipf ? (small ? 2 : 1) : 0;

    R(t, -1) = C(t, 1);
    R(t, -2) = C(t, 2);
    C(t, -1) = R(t, 1);
    C(t, -2) = R(t, 2);

    for (int y = 0; y < N; y++)
        for (int x = 0; x < M; x++) {
            int offset, iX, iY, fi;
            const int first = xyAxis == 0 ? (chroma ? y <= 1 : y == 0) : (chroma ? x <= 1 : x == 0);
            fi = first ? edge_f : in_f;
            if (xySign < 0) {
                if (xyAxis == 0) {
                    offset = (((y + 1) * imx * 32) >> isx) - (((y + 1) * imx) >> isx) * 32;
                    iX = x + (((y + 1) * imx) >> isx);
                    iY = -1;
                } else {
                    offset = (((x + 1) * imy * 32) >> isy) - (((x + 1) * imy) >> isy) * 32;
                    iX = -1;
                    iY = y + (((x + 1) * imy) >> isy);
                }
            } else {
                const int offsetx = (((y + 1) * imx * 32) >> isx) - (((y + 1) * imx) >> isx) * 32;
                const int iXx = x - (((y + 1) * imx) >> isx);
                const int offsety = (((x + 1) * imy * 32) >> isy) - (((x + 1) * imy) >> isy) * 32;
                const int iYy = y - (((x + 1) * imy) >> isy);
                if (iYy <= -1) {
                    offset = offsetx;
                    iX = iXx;
                    iY = -1;
                } else {
                    offset = offsety;
                    iX = -1;
                    iY = iYy;
                }
            }
            const int8_t *f = svac3_ipf_tab[fi][offset];
            int v;
            if (iY == -1) {
                const int n = xySign < 0 ? iX + 1 : iX - 1;
                const int n2 = xySign < 0 ? iX + 2 : iX - 2;
                const int n1 = xySign < 0 ? iX - 1 : iX + 1;
                v = (rr(t, n1 + 1) * f[0] + rr(t, iX + 1) * f[1] + rr(t, n + 1) * f[2] +
                     rr(t, n2 + 1) * f[3] + filterOffset[fi]) >> filterBit[fi];
            } else {
                const int n = xySign < 0 ? iY + 1 : iY - 1;
                const int n2 = xySign < 0 ? iY + 2 : iY - 2;
                const int n1 = xySign < 0 ? iY - 1 : iY + 1;
                v = (cc(t, n1 + 1) * f[0] + cc(t, iY + 1) * f[1] + cc(t, n + 1) * f[2] +
                     cc(t, n2 + 1) * f[3] + filterOffset[fi]) >> filterBit[fi];
            }
            /* the draft clips MIPF output (luma) or all output (chroma); without
             * MIPF the taps are non-negative, so clipping always is identical */
            p[y * M + x] = (uint8_t)clip1(v);
        }
}

void svac3_intra_pred(const svac3_frame_t *f, int c, int x0, int y0, int M, int N,
                      int mode, int luma_mode, int mipf,
                      svac3_avail_fn avail, void *opaque, uint8_t *pred)
{
    refs_t t;
    int m = mode;

    if (c) {
        /* IntraChromaPredMode to the luma numbering; 0 (DM) takes the luma mode.
         * A conforming stream cannot produce more than 4 (5.2.2.12 forbids the
         * index that would); clamp so that malformed input stays in bounds. */
        static const int map[5] = { -1, 0, 24, 12, 2 };
        m = mode == 0 ? luma_mode : map[mode >= 1 && mode <= 4 ? mode : 1];
    }
    if (m < 0 || m > 65)
        m = 0;
    get_refs(f, c, x0, y0, M, N, avail, opaque, &t);
    switch (m) {
    case 0:  pred_dc(&t, pred); break;
    case 1:  pred_plane(&t, pred); break;
    case 2:  pred_bilinear(&t, pred); break;
    case 12:
        for (int y = 0; y < N; y++)
            for (int x = 0; x < M; x++)
                pred[y * M + x] = (uint8_t)R(&t, x + 1);
        break;
    case 24:
        for (int y = 0; y < N; y++)
            for (int x = 0; x < M; x++)
                pred[y * M + x] = (uint8_t)C(&t, y + 1);
        break;
    case 33:
        memset(pred, 0, (size_t)M * N);    /* PCM */
        break;
    default:
        pred_angular(&t, m, c != 0, mipf, pred);
        break;
    }
}

/* ------------------------------------------------------------------------ */
/* Dequantisation (5.4.7.2, 表117) and the DCT2 inverse transform. */

static const uint16_t dq_tab[80] = {
    32768, 36061, 38968, 42495, 46341, 50535, 55437, 60424, 32932, 35734, 38968, 42495, 46177,
    50535, 55109, 59933, 65535, 35734, 38968, 42577, 46341, 50617, 55027, 60097, 32809, 35734,
    38968, 42454, 46382, 50576, 55109, 60056, 65535, 35734, 38968, 42495, 46320, 50515, 55109,
    60076, 65535, 35744, 38968, 42495, 46341, 50535, 55099, 60087, 65535, 35734, 38973, 42500,
    46341, 50535, 55109, 60097, 32771, 35734, 38965, 42497, 46341, 50535, 55109, 60099, 32768,
    36061, 38968, 42495, 46341, 50535, 55437, 60424, 32932, 35734, 38968, 42495, 46177, 50535,
    55109, 59933,
};
static const uint8_t dq_shift[80] = {
    14, 14, 14, 14, 14, 14, 14, 14, 13, 13, 13, 13, 13, 13, 13, 13, 13, 12, 12, 12, 12, 12, 12,
    12, 11, 11, 11, 11, 11, 11, 11, 11, 11, 10, 10, 10, 10, 10, 10, 10, 10, 9, 9, 9, 9, 9, 9, 9,
    9, 8, 8, 8, 8, 8, 8, 8, 7, 7, 7, 7, 7, 7, 7, 7, 6, 6, 6, 6, 6, 6, 6, 6, 5, 5, 5, 5, 5, 5, 5, 5,
};

static const int8_t *dct2(int n)
{
    switch (n) {
    case 4:  return &svac3_dct2_4[0][0];
    case 8:  return &svac3_dct2_8[0][0];
    case 16: return &svac3_dct2_16[0][0];
    case 32: return &svac3_dct2_32[0][0];
    default: return &svac3_dct2_64[0][0];
    }
}

int svac3_chroma_qp(int qp, int delta)
{
    static const uint8_t tab[21] = { 42, 43, 43, 44, 44, 45, 45, 46, 46, 47, 47,
                                     48, 48, 48, 49, 49, 49, 50, 50, 50, 51 };
    const int x = clip3(-16, 63, qp - 8 * (BIT_DEPTH - 8) + delta);
    const int q = x < 43 ? x : tab[x - 43];
    return clip3(0, 63 + 8 * (BIT_DEPTH - 8), q + 8 * (BIT_DEPTH - 8));
}

void svac3_dequant_itrans(const int16_t *quant, int M1, int M2, int qp, int16_t *resid)
{
    int32_t coef[64 * 64];
    const int m = ilog2(M1 * M2) / 2;
    const int shift = dq_shift[qp] + m + BIT_DEPTH - 14;
    const int rect = (M1 == 2 * M2) || (M2 == 2 * M1) || (M1 == 8 * M2) || (M2 == 8 * M1);

    /* coefficients past the last non-zero column (nx) and row (ny) are zero
     * and contribute nothing to either pass */
    int nx = 0, ny = 0;
    for (int i = 0; i < M1 * M2; i++) {
        /* flat weights: (q * 64) >> WqmShift(2) == q * 16 */
        int64_t v = ((((int64_t)quant[i] * 16) * dq_tab[qp]) >> 4);
        v = (v + ((int64_t)1 << (shift - 1))) >> shift;
        int c = clip3(-32768, 32767, (int)v);
        if (rect)
            c = (c * 181 + (1 << 7)) >> 8;
        coef[i] = c;
        if (c) {
            if (i % M1 >= nx)
                nx = i % M1 + 1;
            if (i / M1 >= ny)
                ny = i / M1 + 1;
        }
    }

    if (!nx) {
        memset(resid, 0, sizeof(*resid) * (size_t)M1 * M2);
        return;
    }
    const svac3_dsp_t *dsp = svac3_dsp();
    const int8_t *tv = dct2(M2), *th = dct2(M1);
    int16_t cq[64 * 64], a[64 * 64], b[64 * 64], kk[64 * 64];

    /* vertical: K[x][y] = sum_k T_M2[k][y] * C[x][k], then (+16) >> 5, as
     * the product of T_M2 transposed (M2 x ny) and C (ny x cols); columns
     * past nx are zero, so cols may round nx up for the kernel */
    const int cols = M1 == 4 ? 4 : (nx + 7) & ~7;
    for (int y = 0; y < M2; y++)
        for (int k = 0; k < ny; k++)
            a[y * ny + k] = tv[k * M2 + y];
    for (int k = 0; k < ny; k++)
        for (int x = 0; x < cols; x++)
            cq[k * cols + x] = (int16_t)coef[k * M1 + x];
    dsp->matmul(kk, cols, a, ny, cq, cols, M2, ny, cols, 5, -32768, 32767);

    /* horizontal: H[x][y] = sum_k K[k][y] * T_M1[k][x], then rounded
     * >> (20 - BitDepth), as the product of K (M2 x nx) and T_M1 (nx x M1) */
    const int shift1 = 20 - BIT_DEPTH, maxv = (1 << BIT_DEPTH) - 1;
    for (int k = 0; k < nx; k++)
        for (int x = 0; x < M1; x++)
            b[k * M1 + x] = th[k * M1 + x];
    dsp->matmul(resid, M1, kk, cols, b, M1, M2, nx, M1, shift1, -maxv - 1, maxv);
}

void svac3_put_block(svac3_frame_t *f, int c, int x0, int y0, int M, int N,
                     const uint8_t *pred, const int16_t *resid)
{
    uint8_t *dst = f->plane[c];
    const int s = f->stride[c];
    if (x0 + M <= f->width[c] && y0 + N <= f->height[c]) {
        uint8_t *d = dst + (size_t)y0 * s + x0;
        if (resid) {
            svac3_dsp()->add_block(d, s, pred, resid, M, N);
        } else {
            for (int y = 0; y < N; y++)
                memcpy(d + (size_t)y * s, pred + y * M, (size_t)M);
        }
        return;
    }
    /* a block crossing the picture's edge: only its part inside */
    for (int y = 0; y < N && y0 + y < f->height[c]; y++)
        for (int x = 0; x < M && x0 + x < f->width[c]; x++)
            dst[(y0 + y) * s + x0 + x] =
                (uint8_t)clip1(pred[y * M + x] + (resid ? resid[y * M + x] : 0));
}
