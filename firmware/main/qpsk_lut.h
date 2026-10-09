/* QPSK / RRC lookup-table modulator, tables and index maths (portable: also compiled on the PC by host/test_lut.c).
 *
 * Integer samples per symbol (sps = 4, 8, 16; the generic lutg.S loop uses lut_build_t below) and an IF that is a whole multiple of the symbol rate make the RRC filter output
 * a function of the last 12 symbols and of the phase j inside the symbol only; the IF rotation (e^(j 2 pi ifm j / sps)) depends
 * on j alone as well, so it is baked into the tables and costs nothing at run time.
 *
 * History word C: 2 bits per symbol (bit 0 = I level, bit 1 = Q level, 1 = +1), newest symbol in bits 1:0, 24 bits = 12 symbols.
 * Three groups of 4 symbols: group k indexes with 8 bits (C >> 8k) & 255. Entry T[(k * 256 + idx) * sps + j] is a ready word:
 * I field (bits 9:0) + Q field (bits 19:10), offset binary (+512 baked into the last group); the sum of the three entries xor
 * LUT_XOR is the DAC word (two's complement fields). Per sample: 3 loads, 2 adds, 1 xor, 1 store.
 *
 * Needs `float rrc_pulse(float t, float b)` from the includer. */
#ifndef QPSK_LUT_H
#define QPSK_LUT_H

#include <math.h>
#include <stdint.h>

#define LUT_SPAN 12u
#define LUT_XOR  0x80200u

/* log2 of the byte size of one idx row (sps words of 4 bytes) */
#define LUT_ROW_SHIFT(sps) ((sps) == 4u ? 4u : (sps) == 8u ? 5u : 6u)
/* byte offset of the row of group k for history C (compile-time shifts when sps and k are constants) */
#define LUT_OFF(C, k, S) (((k) * 8u >= (S) ? (uint32_t)(C) >> ((k) * 8u - (S)) : (uint32_t)(C) << ((S) - (k) * 8u)) & (255u << (S)))

static inline uint32_t lut_words(uint32_t sps) { return 3u * 256u * sps; }

/* amp: worst-case |I| and |Q| in DAC codes. dci/dcq: DC offset in codes (fractions are fine, they average out in the rounding).
 * g, phi_deg: Q path correction Q' = g (Q cos phi + I sin phi) (image calibration). Returns false for unsupported sps. */
/* ngroups groups of 4 symbols (span 4 * ngroups symbols); the bias +512 goes into the last group, the DC trim into group 0. */
static int lut_build_g(int32_t *T, uint32_t sps, int32_t ifm, float beta, float amp, float dci, float dcq, float g, float phi_deg, uint32_t ngroups) {
    if (sps != 4u && sps != 8u && sps != 16u) return 0;
    if (ngroups < 1u || ngroups > 3u) return 0;
    const uint32_t span = 4u * ngroups;
    const float pi = 3.14159265f;
    const float cq = cosf(phi_deg * pi / 180.0f), sq = sinf(phi_deg * pi / 180.0f);
    const float kq = g * (fabsf(cq) + fabsf(sq));
    float h[LUT_SPAN][16], worst = 0.0f;
    for (uint32_t j = 0; j < sps; ++j) {
        float sum = 0.0f;
        for (uint32_t l = 0; l < span; ++l) {
            h[l][j] = rrc_pulse((float)l + (float)j / (float)sps - (float)span / 2.0f, beta);
            sum += fabsf(h[l][j]);
        }
        const float ph = 2.0f * pi * (float)((ifm * (int32_t)j) % (int32_t)sps) / (float)sps;
        const float w = sum * (fabsf(cosf(ph)) + fabsf(sinf(ph))) * (kq > 1.0f ? kq : 1.0f);
        if (w > worst) worst = w;
    }
    const float sc = amp / worst;
    for (uint32_t k = 0; k < ngroups; ++k)
        for (uint32_t idx = 0; idx < 256u; ++idx)
            for (uint32_t j = 0; j < sps; ++j) {
                float vi = 0.0f, vq = 0.0f;
                for (uint32_t b = 0; b < 4u; ++b) {
                    const float p = h[4u * k + b][j];
                    vi += ((idx >> (2u * b)) & 1u ? p : -p);
                    vq += ((idx >> (2u * b + 1u)) & 1u ? p : -p);
                }
                const float ph = 2.0f * pi * (float)((ifm * (int32_t)j) % (int32_t)sps) / (float)sps;
                const float c = cosf(ph), s = sinf(ph);
                const float ri = vi * c - vq * s, rq = vi * s + vq * c;
                float oi = ri * sc, oq = g * (rq * cq + ri * sq) * sc;
                if (k == 0u) { oi += dci; oq += dcq; }
                if (k == ngroups - 1u) { oi += 512.0f; oq += 512.0f; }
                T[(k * 256u + idx) * sps + j] = (int32_t)lroundf(oi) + ((int32_t)lroundf(oq) << 10);
            }
    return 1;
}

static inline int lut_build(int32_t *T, uint32_t sps, int32_t ifm, float beta, float amp, float dci, float dcq, float g, float phi_deg) {
    return lut_build_g(T, sps, ifm, beta, amp, dci, dcq, g, phi_deg, 3u);
}

/* ---- generic oversampling (lutg.S): transposed tables of 2-symbol groups -------------------------------------------------------------
 * RRC span 8 symbols = 4 groups of 2 symbols; any number of samples per symbol S (16..232 here). Group g indexes with 4 bits of the
 * history, idx = (C >> 4g) & 15 (bits 1:0 = the newer symbol of the pair, bits 3:2 = the older one, bit 0 / 2 = I, bit 1 / 3 = Q).
 * Entry T[(g * S + j) * 16 + idx]: the 16 entries of one (g, j) are 64 bytes, so the row pointer of a group is
 * T + (g * S) * 64 + 4 * idx and the next sample j + 1 is that pointer + 64: the asm loop just adds 64 to four pointers.
 * Same word format as above (I field bits 9:0 + Q field bits 19:10, offset binary, +512 baked into group 3, DC trim into group 0);
 * the sum of the four entries xor LUT_XOR is the DAC word. Size 256 * S bytes. Returns 0 if out of memory. */
#define LUTT_GROUPS 4u
static inline uint32_t lutt_bytes(uint32_t S) { return 256u * S; }

static int lut_build_t(int32_t *T, uint32_t S, int32_t ifm, float beta, float amp, float dci, float dcq, float g, float phi_deg, float *hbuf) {
    const uint32_t span = 2u * LUTT_GROUPS;
    const float pi = 3.14159265f;
    const float cq = cosf(phi_deg * pi / 180.0f), sq = sinf(phi_deg * pi / 180.0f);
    const float kq = g * (fabsf(cq) + fabsf(sq));
    float worst = 0.0f;
    for (uint32_t j = 0; j < S; ++j) {                    /* hbuf: span * S floats, h[l * S + j] for the symbol of age l */
        float sum = 0.0f;
        for (uint32_t l = 0; l < span; ++l) {
            hbuf[l * S + j] = rrc_pulse((float)l + (float)j / (float)S - (float)span / 2.0f, beta);
            sum += fabsf(hbuf[l * S + j]);
        }
        const float ph = 2.0f * pi * (float)((ifm * (int32_t)j) % (int32_t)S) / (float)S;
        const float w = sum * (fabsf(cosf(ph)) + fabsf(sinf(ph))) * (kq > 1.0f ? kq : 1.0f);
        if (w > worst) worst = w;
    }
    const float sc = amp / worst;
    for (uint32_t k = 0; k < LUTT_GROUPS; ++k)
        for (uint32_t j = 0; j < S; ++j) {
            const float ph = 2.0f * pi * (float)((ifm * (int32_t)j) % (int32_t)S) / (float)S;
            const float c = cosf(ph), s = sinf(ph);
            for (uint32_t idx = 0; idx < 16u; ++idx) {
                float vi = 0.0f, vq = 0.0f;
                for (uint32_t b = 0; b < 2u; ++b) {
                    const float p = hbuf[(2u * k + b) * S + j];
                    vi += ((idx >> (2u * b)) & 1u ? p : -p);
                    vq += ((idx >> (2u * b + 1u)) & 1u ? p : -p);
                }
                const float ri = vi * c - vq * s, rq = vi * s + vq * c;
                float oi = ri * sc, oq = g * (rq * cq + ri * sq) * sc;
                if (k == 0u) { oi += dci; oq += dcq; }
                if (k == LUTT_GROUPS - 1u) { oi += 512.0f; oq += 512.0f; }
                T[(k * S + j) * 16u + idx] = (int32_t)lroundf(oi) + ((int32_t)lroundf(oq) << 10);
            }
        }
    return 1;
}

/* ---- 8PSK for the generic loop (lutg_psk8.S): the same four groups of two symbols, but a symbol is a 3 bit angle index k (the point
 * e^(j pi k / 4), k = 0..7) and a group index is 6 bits: idx = k_newer | k_older << 3. Entry T[(g * S + j) * 64 + idx]: the 64 words of one
 * (group, sample) are 256 bytes apart from the next sample (the asm loop adds 256 to the four row pointers). Size 1024 * S bytes. */
static inline uint32_t lutt8_bytes(uint32_t S) { return 1024u * S; }

static int lut_build_p8(int32_t *T, uint32_t S, int32_t ifm, float beta, float amp, float dci, float dcq, float g, float phi_deg, float *hbuf) {
    const uint32_t span = 2u * LUTT_GROUPS;
    const float pi = 3.14159265f;
    const float cq = cosf(phi_deg * pi / 180.0f), sq = sinf(phi_deg * pi / 180.0f);
    const float kq = g * (fabsf(cq) + fabsf(sq));
    float worst = 0.0f, pc[8], ps[8];
    for (uint32_t k = 0; k < 8u; ++k) { pc[k] = cosf(pi * (float)k / 4.0f); ps[k] = sinf(pi * (float)k / 4.0f); }
    for (uint32_t j = 0; j < S; ++j) {
        float sum = 0.0f;
        for (uint32_t l = 0; l < span; ++l) {
            hbuf[l * S + j] = rrc_pulse((float)l + (float)j / (float)S - (float)span / 2.0f, beta);
            sum += fabsf(hbuf[l * S + j]);
        }
        const float ph = 2.0f * pi * (float)((ifm * (int32_t)j) % (int32_t)S) / (float)S;
        const float w = sum * (fabsf(cosf(ph)) + fabsf(sinf(ph))) * (kq > 1.0f ? kq : 1.0f);      /* |I|, |Q| <= 1 for every point */
        if (w > worst) worst = w;
    }
    const float sc = amp / worst;
    for (uint32_t k = 0; k < LUTT_GROUPS; ++k)
        for (uint32_t j = 0; j < S; ++j) {
            const float ph = 2.0f * pi * (float)((ifm * (int32_t)j) % (int32_t)S) / (float)S;
            const float c = cosf(ph), s = sinf(ph);
            const float p0 = hbuf[(2u * k) * S + j], p1 = hbuf[(2u * k + 1u) * S + j];
            for (uint32_t idx = 0; idx < 64u; ++idx) {
                const uint32_t ka = idx & 7u, kb = idx >> 3;
                const float vi = pc[ka] * p0 + pc[kb] * p1, vq = ps[ka] * p0 + ps[kb] * p1;
                const float ri = vi * c - vq * s, rq = vi * s + vq * c;
                float oi = ri * sc, oq = g * (rq * cq + ri * sq) * sc;
                if (k == 0u) { oi += dci; oq += dcq; }
                if (k == LUTT_GROUPS - 1u) { oi += 512.0f; oq += 512.0f; }
                T[(k * S + j) * 64u + idx] = (int32_t)lroundf(oi) + ((int32_t)lroundf(oq) << 10);
            }
        }
    return 1;
}

/* ---- 16APSK for lutg_a16.S: a symbol is the DVB-S2 bit quadruple v = 0..15 (v 0..11 on the outer ring at 45, -45, 135, -135, 15, -15, 165, -165, 75, -75, 105,
 * -105 degrees, v 12..15 on the inner ring at 45, -45, 135, -135 degrees), the radii for a mean power of 1 from gamma = R2 / R1 (gamma100 = 100 * gamma).
 * The RRC filter is truncated to 6 symbols: 3 groups of 2 symbols, 256 rows each, idx = v_newer | v_older << 4. Entry T[(g * 256 + idx) * S + j]: the S words of a
 * row are together (the row pointer is base + idx * 4 * S, the next sample is 4 bytes further). Size 3072 * S bytes. */
#define LUTA16_GROUPS 3u
static inline uint32_t lutt16_bytes(uint32_t S) { return 3072u * S; }

static int lut_build_a16(int32_t *T, uint32_t S, int32_t ifm, float beta, float amp, float dci, float dcq, float g, float phi_deg, float *hbuf, uint32_t gamma100) {
    static const float ang[16] = {45, -45, 135, -135, 15, -15, 165, -165, 75, -75, 105, -105, 45, -45, 135, -135};
    const uint32_t span = 2u * LUTA16_GROUPS;
    const float pi = 3.14159265f;
    const float gm = (float)gamma100 / 100.0f, r1 = 4.0f / sqrtf(4.0f + 12.0f * gm * gm);
    const float cq = cosf(phi_deg * pi / 180.0f), sq = sinf(phi_deg * pi / 180.0f);
    const float kq = g * (fabsf(cq) + fabsf(sq));
    float worst = 0.0f, pc[16], ps[16], pmax = 0.0f;
    for (uint32_t v = 0; v < 16u; ++v) {
        const float r = v < 12u ? gm * r1 : r1;
        pc[v] = r * cosf(ang[v] * pi / 180.0f);
        ps[v] = r * sinf(ang[v] * pi / 180.0f);
        if (fabsf(pc[v]) > pmax) pmax = fabsf(pc[v]);
        if (fabsf(ps[v]) > pmax) pmax = fabsf(ps[v]);
    }
    for (uint32_t j = 0; j < S; ++j) {
        float sum = 0.0f;
        for (uint32_t l = 0; l < span; ++l) {
            hbuf[l * S + j] = rrc_pulse((float)l + (float)j / (float)S - (float)span / 2.0f, beta);
            sum += fabsf(hbuf[l * S + j]);
        }
        const float ph = 2.0f * pi * (float)((ifm * (int32_t)j) % (int32_t)S) / (float)S;
        const float w = sum * pmax * (fabsf(cosf(ph)) + fabsf(sinf(ph))) * (kq > 1.0f ? kq : 1.0f);      /* |I|, |Q| <= pmax for every point */
        if (w > worst) worst = w;
    }
    const float sc = amp / worst;
    for (uint32_t k = 0; k < LUTA16_GROUPS; ++k)
        for (uint32_t j = 0; j < S; ++j) {
            const float ph = 2.0f * pi * (float)((ifm * (int32_t)j) % (int32_t)S) / (float)S;
            const float c = cosf(ph), s = sinf(ph);
            const float p0 = hbuf[(2u * k) * S + j], p1 = hbuf[(2u * k + 1u) * S + j];
            for (uint32_t idx = 0; idx < 256u; ++idx) {
                const uint32_t va = idx & 15u, vb = idx >> 4;
                const float vi = pc[va] * p0 + pc[vb] * p1, vq = ps[va] * p0 + ps[vb] * p1;
                const float ri = vi * c - vq * s, rq = vi * s + vq * c;
                float oi = ri * sc, oq = g * (rq * cq + ri * sq) * sc;
                if (k == 0u) { oi += dci; oq += dcq; }
                if (k == LUTA16_GROUPS - 1u) { oi += 512.0f; oq += 512.0f; }
                T[(k * 256u + idx) * S + j] = (int32_t)lroundf(oi) + ((int32_t)lroundf(oq) << 10);
            }
        }
    return 1;
}

/* ---- 32APSK for the narrowband C loop (tx_a32_c in main.c): a symbol is the DVB-S2 bit quintuple v = 0..31 (rings of 4 + 12 + 16 points, the radii for a mean power of 1 from
 * gamma1 = R2 / R1 and gamma2 = R3 / R1, both x 100). Two symbols of 5 bits would make 1024 rows per table, so the RRC filter (truncated to 6 symbols, as for 16APSK)
 * has one table of 36 rows per tap: T[(tap * 36 + v) * S + j], tap 0 the newest symbol. Rows 32..35 are unit-radius PL points.
 * The row pointer is base + v * S, the next sample is 4 bytes further. Size 864 * S bytes. */
#define LUTA32_TAPS 6u
#define LUTA32_POINTS 36u  /* 32 data points and four optional unit-radius PL points */
static inline uint32_t lutt32_bytes(uint32_t S) { return 4u * LUTA32_POINTS * LUTA32_TAPS * S; }

static int lut_build_a32(int32_t *T, uint32_t S, int32_t ifm, float beta, float amp, float dci, float dcq, float g, float phi_deg, float *hbuf, uint32_t gamma1_100, uint32_t gamma2_100) {
    static const float ang[LUTA32_POINTS] = {45, 75, -45, -75, 135, 105, -135, -105, 22.5f, 67.5f, -45, -90, 135, 90, -157.5f, -112.5f,
                                  15, 45, -15, -45, 165, 135, -165, -135, 0, 45, -22.5f, -67.5f, 157.5f, 112.5f, 180, -135,
                                  45, 135, -135, -45};
    const float pi = 3.14159265f;
    const float g1 = (float)gamma1_100 / 100.0f, g2 = (float)gamma2_100 / 100.0f, r1 = sqrtf(32.0f / (4.0f + 12.0f * g1 * g1 + 16.0f * g2 * g2));
    const float cq = cosf(phi_deg * pi / 180.0f), sq = sinf(phi_deg * pi / 180.0f);
    const float kq = g * (fabsf(cq) + fabsf(sq));
    float worst = 0.0f, pc[LUTA32_POINTS], ps[LUTA32_POINTS], pmax = 0.0f;
    for (uint32_t v = 0; v < LUTA32_POINTS; ++v) {
        const uint32_t ring = v < 8u ? 2u : v < 16u ? 3u : v < 24u ? ((v & 1u) ? 1u : 2u) : 3u;      /* 1 inner (4 points), 2 middle (12), 3 outer (16) */
        const float r = v >= 32u ? 1.0f : ring == 1u ? r1 : ring == 2u ? g1 * r1 : g2 * r1;
        pc[v] = r * cosf(ang[v] * pi / 180.0f);
        ps[v] = r * sinf(ang[v] * pi / 180.0f);
        if (fabsf(pc[v]) > pmax) pmax = fabsf(pc[v]);
        if (fabsf(ps[v]) > pmax) pmax = fabsf(ps[v]);
    }
    for (uint32_t j = 0; j < S; ++j) {
        float sum = 0.0f;
        for (uint32_t l = 0; l < LUTA32_TAPS; ++l) {
            hbuf[l * S + j] = rrc_pulse((float)l + (float)j / (float)S - (float)LUTA32_TAPS / 2.0f, beta);
            sum += fabsf(hbuf[l * S + j]);
        }
        const float ph = 2.0f * pi * (float)((ifm * (int32_t)j) % (int32_t)S) / (float)S;
        const float w = sum * pmax * (fabsf(cosf(ph)) + fabsf(sinf(ph))) * (kq > 1.0f ? kq : 1.0f);      /* |I|, |Q| <= pmax for every point */
        if (w > worst) worst = w;
    }
    const float sc = amp / worst;
    for (uint32_t k = 0; k < LUTA32_TAPS; ++k)
        for (uint32_t j = 0; j < S; ++j) {
            const float ph = 2.0f * pi * (float)((ifm * (int32_t)j) % (int32_t)S) / (float)S;
            const float c = cosf(ph), s = sinf(ph);
            const float p = hbuf[k * S + j];
            for (uint32_t v = 0; v < LUTA32_POINTS; ++v) {
                const float vi = pc[v] * p, vq = ps[v] * p;
                const float ri = vi * c - vq * s, rq = vi * s + vq * c;
                float oi = ri * sc, oq = g * (rq * cq + ri * sq) * sc;
                if (k == 0u) { oi += dci; oq += dcq; }
                if (k == LUTA32_TAPS - 1u) { oi += 512.0f; oq += 512.0f; }
                T[(k * LUTA32_POINTS + v) * S + j] = (int32_t)lroundf(oi) + ((int32_t)lroundf(oq) << 10);
            }
        }
    return 1;
}

#endif
