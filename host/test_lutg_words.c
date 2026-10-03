/* Checks the DAC words that the generic assembly loop (firmware/main/lutg.S) really produces, against the reference model on the PC.
 *
 *   1. firmware/: python3 tools/gen_lutg.py --rec words > main/lutg.S, "#define LUTG_REC 2" in main.c, build and flash
 *      (the loop then records every DAC word of 12 symbols and prints them; the symbols come from a known pseudo-random ring)
 *   2. python3 host/lutg_record.py "QPSKT 2370.000 125000 64 300 5 0 0" > /tmp/words_64.txt
 *   3. gcc -O2 -o /tmp/check_words host/test_lutg_words.c -lm && /tmp/check_words 64 /tmp/words_64.txt
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
    const uint32_t S = atoi(argv[1]);
    const int amp = argc > 3 ? atoi(argv[3]) : 300;
    FILE *f = fopen(argv[2], "r");
    uint32_t *rec = calloc(12 * S, 4);
    int *seen = calloc(12 * S, sizeof(int));
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
                if (k < 12) { rec[k * S + j + i] = v; seen[k * S + j + i] = 1; }
            }
        }
    }
    int32_t *T = malloc(lutt_bytes(S));
    float *h = malloc(8 * S * 4);
    lut_build_t(T, S, 0, 0.35f, (float)amp, 0, 0, 1.0f, 0.0f, h);
    uint8_t ring[16384];
    uint32_t rng = 0x2545F491u;
    for (int i = 0; i < 16384; ++i) { rng ^= rng << 13; rng ^= rng >> 17; rng ^= rng << 5; ring[i] = (uint8_t)(rng >> 8); }
    long bad = 0, tot = 0;
    uint32_t C = 0;
    for (unsigned n = 0; n < 11; ++n) {                       /* symbol 11 is cut short */
        for (uint32_t j = 0; j < S; ++j) {
            uint32_t w = 0;
            for (uint32_t g = 0; g < 4; ++g) w += (uint32_t)T[(g * S + j) * 16 + ((C >> (4 * g)) & 15)];
            w ^= LUT_XOR;
            if (!seen[n * S + j]) continue;
            ++tot;
            if ((rec[n * S + j] & 0xFFFFF) != (w & 0xFFFFF)) { if (bad < 8) printf("MISMATCH symbol %u sample %u: got %05x expected %05x\n", n, j, rec[n * S + j] & 0xFFFFF, w & 0xFFFFF); ++bad; }
        }
        uint32_t sym = (ring[(n >> 2) & 16383] >> (2 * (n & 3))) & 3;           /* symbol n of the ring (low bits first) */
        C = (C << 2) | sym;
    }
    printf("S=%u: %ld words compared, %ld mismatches\n", S, tot, bad);
    return bad != 0;
}
