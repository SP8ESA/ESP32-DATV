/* Rational sample deadlines, in CPU cycles. No division in the sample loop. */
#pragma once
#include <stdint.h>

typedef struct {
    uint32_t period, remainder, divisor;
} sample_clock_t;

static inline sample_clock_t sample_clock_init(uint32_t cpu_hz, uint32_t sample_hz) {
    return (sample_clock_t){cpu_hz / sample_hz, cpu_hz % sample_hz, sample_hz};
}

/* Keep phase across symbols and USB reports. After n calls, the total interval
 * is floor(n * cpu_hz / sample_hz), with less than one cycle of accumulated error. */
static inline __attribute__((always_inline)) uint32_t sample_clock_next(sample_clock_t clock, uint32_t *phase) {
    uint32_t interval = clock.period;
    uint32_t next = *phase + clock.remainder;
    if (next >= clock.divisor) {
        next -= clock.divisor;
        ++interval;
    }
    *phase = next;
    return interval;
}
