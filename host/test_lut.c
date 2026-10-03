/* Checks the QPSK lookup-table modulator (firmware/main/qpsk_lut.h) against a direct float RRC convolution.
 *   gcc -O2 -o /tmp/test_lut test_lut.c -lm && /tmp/test_lut [sps [ifm [out.bin]]]
 *   /tmp/test_lut -t S [ifm [out.bin]]      the transposed 4 x 2-symbol tables of the generic loop (lutg.S), any S
 * Prints the worst / rms error in DAC codes; optional dump of the int16 I,Q samples for a spectrum check. */
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>
#include <string.h>

static float rrc_pulse(float t, float b) {
    const float pi = 3.14159265f;
    if (fabsf(t) < 1e-6f) return 1.0f - b + 4.0f * b / pi;
    if (fabsf(fabsf(t) - 1.0f / (4.0f * b)) < 1e-4f)
        return b / 1.41421356f * ((1.0f + 2.0f / pi) * sinf(pi / (4.0f * b)) + (1.0f - 2.0f / pi) * cosf(pi / (4.0f * b)));
    const float num = sinf(pi * t * (1.0f - b)) + 4.0f * b * t * cosf(pi * t * (1.0f + b));
    const float den = pi * t * (1.0f - (4.0f * b * t) * (4.0f * b * t));
    return num / den;
}
#include "../firmware/main/qpsk_lut.h"

/* generic loop: span 8 (4 groups of 2 symbols), transposed layout, the index maths the assembly uses */
static int test_t(int argc, char **argv) {
    const uint32_t S = atoi(argv[2]);
    const int ifm = argc > 3 ? atoi(argv[3]) : 0;
    const float amp = 300, beta = 0.35f, dci = -1.4f, dcq = 2.9f, g = 0.9994f, phi = 0.079f;
    int32_t *T = malloc(lutt_bytes(S));
    float *h = malloc(8 * S * sizeof(float));
    if (!lut_build_t(T, S, ifm, beta, amp, dci, dcq, g, phi, h)) return 2;
    float worst = 0;
    for (uint32_t j = 0; j < S; ++j) {
        float sum = 0;
        for (int l = 0; l < 8; ++l) sum += fabsf(rrc_pulse(l + (float)j / S - 4.0f, beta));
        const float ph = 2 * 3.14159265f * (float)((ifm * (int)j) % (int)S) / S;
        const float kq = g * (fabsf(cosf(phi * 3.14159265f / 180)) + fabsf(sinf(phi * 3.14159265f / 180)));
        const float w = sum * (fabsf(cosf(ph)) + fabsf(sinf(ph))) * (kq > 1 ? kq : 1);
        if (w > worst) worst = w;
    }
    const float sc = amp / worst;
    const int N = 4000;
    int *si = malloc(N * sizeof(int)), *sq = malloc(N * sizeof(int));
    srand(5);
    for (int i = 0; i < N; ++i) { si[i] = rand() & 1; sq[i] = rand() & 1; }
    FILE *fo = argc > 4 ? fopen(argv[4], "wb") : NULL;
    const uint8_t *Tb = (const uint8_t *)T;
    uint32_t C = 0;
    double e2 = 0, emax = 0, p2 = 0;
    long cnt = 0;
    for (int n = 0; n < N; ++n) {
        C = (C << 2) | (uint32_t)(si[n] | sq[n] << 1);               /* history is not masked: the asm only looks at 4 bits per group */
        const uint8_t *q[4];
        for (uint32_t k = 0; k < 4; ++k) q[k] = Tb + (k * S) * 64u + (((C >> (4 * k)) & 15u) << 2);
        for (uint32_t j = 0; j < S; ++j) {
            uint32_t w = 0;
            for (uint32_t k = 0; k < 4; ++k) w += *(const uint32_t *)(q[k] + 64u * j);
            w ^= LUT_XOR;
            int32_t ci = w & 0x3ff, cq = (w >> 10) & 0x3ff;
            if (ci & 0x200) ci -= 1024;
            if (cq & 0x200) cq -= 1024;
            if (n >= 8) {
                double vi = 0, vq = 0;
                for (int l = 0; l < 8; ++l) {
                    const double p = rrc_pulse(l + (float)j / S - 4.0f, beta);
                    vi += (si[n - l] ? p : -p);
                    vq += (sq[n - l] ? p : -p);
                }
                const double ph = 2 * M_PI * (double)((ifm * (int)j) % (int)S) / S;
                const double ri = vi * cos(ph) - vq * sin(ph), rq = vi * sin(ph) + vq * cos(ph);
                const double pr = phi * M_PI / 180;
                const double oi = ri * sc + dci, oq = g * (rq * cos(pr) + ri * sin(pr)) * sc + dcq;
                const double ei = ci - oi, eq = cq - oq;
                e2 += ei * ei + eq * eq; ++cnt;
                p2 += oi * oi + oq * oq;
                if (fabs(ei) > emax) emax = fabs(ei);
                if (fabs(eq) > emax) emax = fabs(eq);
            }
            if (fo) { int16_t o[2] = {(int16_t)ci, (int16_t)cq}; fwrite(o, 2, 2, fo); }
        }
    }
    printf("generic S %u ifm %d (span 8): rms error %.3f codes, worst %.2f codes, signal rms %.1f codes -> SNR %.1f dB, table %u bytes\n", S, ifm,
           sqrt(e2 / cnt / 2), emax, sqrt(p2 / cnt / 2), 10 * log10(p2 / e2), lutt_bytes(S));
    if (fo) fclose(fo);
    return 0;
}

int main(int argc, char **argv) {
    if (argc > 2 && !strcmp(argv[1], "-t")) return test_t(argc, argv);
    const uint32_t sps = argc > 1 ? atoi(argv[1]) : 4;
    const int ifm = argc > 2 ? atoi(argv[2]) : 0;
    const float amp = 300, beta = 0.35f, dci = -1.4f, dcq = 2.9f, g = 0.9994f, phi = 0.079f;
    int32_t *T = malloc(4 * lut_words(sps));
    if (!lut_build(T, sps, ifm, beta, amp, dci, dcq, g, phi)) return 2;
    const uint32_t S = LUT_ROW_SHIFT(sps);
    const int N = 20000;
    int *si = malloc(N * sizeof(int)), *sq = malloc(N * sizeof(int));
    srand(5);
    for (int i = 0; i < N; ++i) { si[i] = rand() & 1; sq[i] = rand() & 1; }
    /* reference scale: same worst-case rule */
    float worst = 0;
    for (uint32_t j = 0; j < sps; ++j) {
        float sum = 0;
        for (int l = 0; l < 12; ++l) sum += fabsf(rrc_pulse(l + (float)j / sps - 6.0f, beta));
        const float ph = 2 * 3.14159265f * (float)((ifm * (int)j) % (int)sps) / sps;
        const float kq = g * (fabsf(cosf(phi * 3.14159265f / 180)) + fabsf(sinf(phi * 3.14159265f / 180)));
        const float w = sum * (fabsf(cosf(ph)) + fabsf(sinf(ph))) * (kq > 1 ? kq : 1);
        if (w > worst) worst = w;
    }
    const float sc = amp / worst;
    uint32_t C = 0;
    double e2 = 0, emax = 0, p2 = 0;
    FILE *fo = argc > 3 ? fopen(argv[3], "wb") : NULL;
    const uint8_t *T0 = (const uint8_t *)T, *T1 = T0 + 256 * sps * 4, *T2 = T1 + 256 * sps * 4;
    long cnt = 0;
    for (int n = 0; n < N; ++n) {
        C = ((C << 2) | (uint32_t)(si[n] | sq[n] << 1)) & 0xFFFFFFu;
        const uint8_t *Q0 = T0 + LUT_OFF(C, 0u, S), *Q1 = T1 + LUT_OFF(C, 1u, S), *Q2 = T2 + LUT_OFF(C, 2u, S);
        if (Q0 != T0 + ((C & 255u) << S) || Q1 != T1 + (((C >> 8) & 255u) << S) || Q2 != T2 + (((C >> 16) & 255u) << S)) { printf("LUT_OFF mismatch\n"); return 3; }
        for (uint32_t j = 0; j < sps; ++j) {
            const uint32_t w = (*(const uint32_t *)(Q0 + 4 * j) + *(const uint32_t *)(Q1 + 4 * j) + *(const uint32_t *)(Q2 + 4 * j)) ^ LUT_XOR;
            int32_t ci = w & 0x3ff, cq = (w >> 10) & 0x3ff;
            if (ci & 0x200) ci -= 1024;
            if (cq & 0x200) cq -= 1024;
            if (n >= 12) {
                double vi = 0, vq = 0;
                for (int l = 0; l < 12; ++l) {
                    const double p = rrc_pulse(l + (float)j / sps - 6.0f, beta);
                    vi += (si[n - l] ? p : -p);
                    vq += (sq[n - l] ? p : -p);
                }
                const double ph = 2 * M_PI * (double)((ifm * (int)j) % (int)sps) / sps;
                const double ri = vi * cos(ph) - vq * sin(ph), rq = vi * sin(ph) + vq * cos(ph);
                const double pr = phi * M_PI / 180;
                const double oi = ri * sc + dci, oq = g * (rq * cos(pr) + ri * sin(pr)) * sc + dcq;
                const double ei = ci - oi, eq = cq - oq;
                e2 += ei * ei + eq * eq; ++cnt;
                p2 += oi * oi + oq * oq;
                if (fabs(ei) > emax) emax = fabs(ei);
                if (fabs(eq) > emax) emax = fabs(eq);
            }
            if (fo) { int16_t o[2] = {(int16_t)ci, (int16_t)cq}; fwrite(o, 2, 2, fo); }
        }
    }
    printf("sps %u ifm %d: rms error %.3f codes, worst %.2f codes, signal rms %.1f codes -> SNR %.1f dB\n", sps, ifm, sqrt(e2 / cnt / 2), emax,
           sqrt(p2 / cnt / 2), 10 * log10(p2 / e2));
    if (fo) fclose(fo);
    return 0;
}
