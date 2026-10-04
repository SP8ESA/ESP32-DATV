/* Checks the DAC words that the generic assembly loop (firmware/main/lutg.S) really produces, against the reference model on the PC.
 *
 *   1. firmware/: python3 tools/gen_lutg.py --rec words > main/lutg.S, "#define LUTG_REC 2" in main.c, build and flash
 *      (the loop then records every DAC word of 12 symbols and prints them; the symbols come from a known pseudo-random ring)
 *   2. python3 host/lutg_record.py "QPSKT 2370.000 125000 64 300 5 0 0" > /tmp/words_64.txt
 *   3. gcc -O2 -o /tmp/check_words host/test_lutg_words.c -lm && /tmp/check_words 64 /tmp/words_64.txt
 * 8PSK (lutg_psk8.S, command PSK8T): generate with --psk8 --rec words, run "PSK8T 2370.000 500000 16 300 5 0 0" and call
 *   /tmp/check_words -p8 16 /tmp/words_p8_16.txt
 * 8PSK at 1 MBd (lutg_p8s8.S, gen_lutg_p8s8.py --rec words, LUTG_REC 2, command "PSK8T 2370.000 1000000 8 300 10 0 0"): the symbols are a bit
 * stream (3 bits each) and the recording covers 7 passes N N N M N N N (56 symbols):  /tmp/check_words -p8s8 8 /tmp/words_p8s8.txt
 * 16APSK (lutg_a16.S, command A16T, gen_lutg_a16.py --rec words): "A16T 2370.000 500000 16 300 5 0 0 0 0 10000 0 315" and
 *   /tmp/check_words -a16 16 /tmp/words_a16_16.txt 300 315      (the last two: amp and gamma x 100)
 * Prints the number of words compared and mismatches (S = the samples per symbol of the QPSKT command; it covers all four code copies
 * of the loop, the history and the table row pointers). Restore the production lutg.S afterwards. */
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
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
int main(int argc, char **argv) {
    int p8 = 0, p8s8 = 0, a16 = 0;
    if (argc > 1 && !strcmp(argv[1], "-p8")) { p8 = 1; ++argv; --argc; }
    if (argc > 1 && !strcmp(argv[1], "-p8s8")) { p8 = p8s8 = 1; ++argv; --argc; }
    if (argc > 1 && !strcmp(argv[1], "-a16")) { a16 = 1; ++argv; --argc; }
    const unsigned gamma100 = a16 && argc > 4 ? (unsigned)atoi(argv[4]) : 315;
    const unsigned nsym = p8s8 ? 64 : 12;
    const uint32_t S = atoi(argv[1]);
    const int amp = argc > 3 ? atoi(argv[3]) : 300;
    FILE *f = fopen(argv[2], "r");
    uint32_t *rec = calloc(nsym * S, 4);
    int *seen = calloc(nsym * S, sizeof(int));
    char line[512], cp;
    unsigned k, j;
    while (fgets(line, sizeof line, f)) {
        char *p = line;
        if (*p == 'W' && sscanf(p + 1, "%u %c %u:", &k, &cp, &j) == 3) {
            p = strchr(p, ':') + 1;
            for (unsigned i = 0; i < 8 && j + i < S; ++i) {
                unsigned v;
                int n;
                if (sscanf(p, " %x%n", &v, &n) != 1) break;
                p += n;
                if (k < nsym) { rec[k * S + j + i] = v; seen[k * S + j + i] = 1; }
            }
        }
    }
    int32_t *T = malloc(a16 ? lutt16_bytes(S) : p8 ? lutt8_bytes(S) : lutt_bytes(S));
    float *h = malloc(8 * S * 4);
    if (a16) lut_build_a16(T, S, 0, 0.35f, (float)amp, 0, 0, 1.0f, 0.0f, h, gamma100);
    else if (p8) lut_build_p8(T, S, 0, 0.35f, (float)amp, 0, 0, 1.0f, 0.0f, h);
    else lut_build_t(T, S, 0, 0.35f, (float)amp, 0, 0, 1.0f, 0.0f, h);
    uint8_t ring[16384];
    uint32_t rng = 0x2545F491u;
    for (int i = 0; i < 16384; ++i) { rng ^= rng << 13; rng ^= rng >> 17; rng ^= rng << 5; ring[i] = (uint8_t)(rng >> 8); }
    long bad = 0, tot = 0;
    uint32_t C = 0;
    for (unsigned n = 0; n < (p8s8 ? 56u : nsym - 1); ++n) {   /* the last symbol is cut short (1 MBd 8PSK: the 8th pass is, only the 7 complete passes count) */
        for (uint32_t j = 0; j < S; ++j) {
            uint32_t w = 0;
            if (a16) for (uint32_t g = 0; g < 3; ++g) w += (uint32_t)T[(g * 256 + ((C >> (8 * g)) & 255)) * S + j];
            else if (p8) for (uint32_t g = 0; g < 4; ++g) w += (uint32_t)T[(g * S + j) * 64 + ((C >> (6 * g)) & 63)];
            else    for (uint32_t g = 0; g < 4; ++g) w += (uint32_t)T[(g * S + j) * 16 + ((C >> (4 * g)) & 15)];
            w ^= LUT_XOR;
            if (!seen[n * S + j]) continue;
            ++tot;
            if ((rec[n * S + j] & 0xFFFFF) != (w & 0xFFFFF)) { if (bad < 8) printf("MISMATCH symbol %u sample %u: got %05x expected %05x\n", n, j, rec[n * S + j] & 0xFFFFF, w & 0xFFFFF); ++bad; }
        }
        if (a16) {
            const uint32_t sym = (ring[(n >> 1) & 8191] >> (4 * (n & 1))) & 15;      /* 16APSK: a nibble per symbol (8 KB ring), the low nibble first */
            C = (C << 4) | sym;
        } else if (p8s8) {
            const uint32_t b = (3 * n) >> 3, w16 = ring[b & 16383] | (uint32_t)ring[(b + 1) & 16383] << 8;
            C = (C << 3) | ((w16 >> ((3 * n) & 7)) & 7);                            /* 1 MBd 8PSK: 3 bits per symbol, a bit stream, the low bits first */
        } else if (p8) {
            const uint32_t sym = (ring[(n >> 1) & 16383] >> (4 * (n & 1))) & 7;     /* 8PSK: a nibble per symbol, the low nibble first */
            C = (C << 3) | sym;
        } else {
            const uint32_t sym = (ring[(n >> 2) & 16383] >> (2 * (n & 3))) & 3;     /* symbol n of the ring (low bits first) */
            C = (C << 2) | sym;
        }
    }
    printf("S=%u: %ld words compared, %ld mismatches\n", S, tot, bad);
    return bad != 0;
}
