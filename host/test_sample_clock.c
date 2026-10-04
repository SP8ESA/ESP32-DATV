/* Run: cc -O2 -Wall -Wextra -Werror host/test_sample_clock.c -o /tmp/test_sample_clock && /tmp/test_sample_clock */
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include "../firmware/main/sample_clock.h"

static void check(uint32_t sample_hz, uint32_t samples) {
    const uint32_t cpu_hz = 160000000u, start = UINT32_MAX - 999u;
    const sample_clock_t clock = sample_clock_init(cpu_hz, sample_hz);
    uint32_t phase = 0, deadline = start;
    uint64_t elapsed = 0;
    for (uint32_t n = 1; n <= samples; ++n) {
        const uint32_t interval = sample_clock_next(clock, &phase);
        assert(interval == clock.period || interval == clock.period + 1u);
        elapsed += interval;
        deadline += interval;
        /* Independent integer-ratio oracle, including symbol/report boundaries and 32-bit wrap. */
        const uint64_t exact = (uint64_t)n * cpu_hz / sample_hz;
        assert(elapsed == exact);
        assert(deadline == (uint32_t)(start + exact));
        assert(phase < sample_hz);
    }
    assert(elapsed * sample_hz == (uint64_t)samples * cpu_hz);
}

int main(void) {
    check(333000u, 333u);               /* symbol clock: 333 symbols in exactly 160000 CPU cycles */
    check(333000u, 333000u);            /* 480/481 cycles per symbol over one nominal second */
    check(33000u, 33000u);              /* generic QPSK symbol clock */
    check(66000u, 66000u);
    check(444444u, 444444u);            /* 16APSK assembly, 18 samples per symbol */
    check(125000u * 16u, 125000u * 16u); /* 16APSK C loop: 80 cycles */
    check(33000u * 24u, 33000u * 24u);   /* 16APSK at 33 kBd: 202/203 cycles */
    check(33000u * 64u, 33000u * 64u);   /* 33 kBd: 75/76 cycles, exactly one nominal second */
    check(66000u * 32u, 66000u * 32u);   /* 66 kBd, same DAC rate */
    check(33000u * 48u, 33000u * 48u);   /* explicitly selected SPS: 101/102 cycles */
    check(66000u * 24u, 66000u * 24u);
    check(10000u * 64u, 10000u * 64u);   /* exact integer interval */
    check(123456u * 17u, 123456u * 17u); /* irregular rate, still exact over a nominal second */
    puts("sample clock: exact average deadlines and counter wrap passed");
    return 0;
}
