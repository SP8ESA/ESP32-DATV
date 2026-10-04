/* ESP32-DATV: DVB-S / QPSK transmitter firmware for the ESP32-C3 (13 cm amateur band).
 *
 * The chip's Wi-Fi transmitter is used as an I/Q modulator. A 10-bit I/Q DAC word is written to the RF block's DAC replay
 * engine, which is run as a held one-word register that the CPU updates with a plain store. Three loops do that:
 *   lut8.S   1 MBd, 8 samples per symbol, one store every 20 CPU cycles (8 MS/s), hand-scheduled assembly
 *   lutg_psk8.S  the same for 8PSK (DVB-S2), 16..64 samples per symbol at 8 MS/s
 *   lutg.S   any symbol rate that gives 16..232 samples per symbol at a store every 20 or 24 CPU cycles (8 or 6.67 MS/s),
 *            generated hand-scheduled assembly with a run-time samples-per-symbol (500k, 333k, 250k, 125k, 66k, 33k Bd ...)
 *   C loops  every other rate, 4 / 8 / 16 samples per symbol at up to 4 MS/s
 * The raised-cosine (RRC, roll-off 0.35) filter is evaluated through lookup tables: one output sample costs a few table loads, adds,
 * one xor and one store. The PC sends raw QPSK symbols over the native USB Serial/JTAG port (4 symbols per byte).
 *
 * Commands (text, one per line, 115200 is irrelevant: USB CDC):
 *   INFO
 *   HEAP
 *   QPSKT f_MHz baud sps [amp [seconds [ifm [target [dcI dcQ [g phi]]]]]]   see qpsk_lut.h and README.md
 *   PSK8T ...same arguments   8PSK from the generic loop (3 bit symbols, 16..64 samples per symbol at 8 MS/s)
 *
 * Transmit only. Licensed amateur use only: the firmware refuses anything outside 2300..2450 MHz.
 */
#include <inttypes.h>
#include <stddef.h>
#include <math.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "driver/usb_serial_jtag.h"
#include "esp_attr.h"
#include "esp_cpu.h"
#include "esp_event.h"
#include "esp_heap_caps.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "hal/usb_serial_jtag_ll.h"
#include "nvs_flash.h"
#include "soc/usb_serial_jtag_struct.h"
#include "heap_memory_layout.h"
#include "soc/soc.h"
#include "sample_clock.h"

/* RF dump bank (ADC capture / DAC playback, librftest adctrig / dactrig):
 * data at 0x3FCB0000. Handing the bank to the RF block (0x600C1020) takes
 * the whole 128 KiB 0x3FCA0000..0x3FCBFFFF, so none of it may hold heap or
 * static data (keep .bss below 0x3FCA0000). */
SOC_RESERVE_MEMORY_REGION(0x3fca0000, 0x3fcc0000, c3_rf_dump);

#define CPU_HZ   160000000u

extern void stop_tx_tone(unsigned);
extern void rom_pbus_workmode(void);
extern void rom_pbus_xpd_rx_on(unsigned);
extern void rom_pbus_xpd_tx_off(void);
extern void rom_set_rxclk_en(unsigned);
extern void set_chanfreq(unsigned, unsigned);
extern void phy_set_freq(unsigned, int);
extern void force_rx_gain(unsigned, unsigned, unsigned);
extern void **g_phyFuns;
extern void txcal_work_mode(void);

static inline uint32_t rd(uint32_t a) { return *(volatile uint32_t *)a; }
static inline void wr(uint32_t a, uint32_t v) { *(volatile uint32_t *)a = v; }
static inline uint32_t ccount(void) { return (uint32_t)esp_cpu_get_cycle_count(); }

/* ------------------------------------------------------------ USB text I/O */

static void usb_write(const void *data, size_t n) {
    const uint8_t *p = data;
    int64_t deadline = esp_timer_get_time() + 500000;
    while (n) {
        if (usb_serial_jtag_ll_txfifo_writable()) {
            int w = usb_serial_jtag_ll_write_txfifo(p, n > 64 ? 64 : n);
            usb_serial_jtag_ll_txfifo_flush();
            p += w;
            n -= w;
        } else if (esp_timer_get_time() > deadline) {
            return;   /* nobody reads */
        }
    }
}

static void say(const char *fmt, ...) {
    char b[300];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(b, sizeof(b), fmt, ap);
    va_end(ap);
    if (n > 0) usb_write(b, n < (int)sizeof(b) ? (size_t)n : sizeof(b) - 1);
}

static void usb_drain_rx(void) {
    uint8_t c;
    while (usb_serial_jtag_ll_read_rxfifo(&c, 1)) {}
}

/* ------------------------------------------------------------ radio */

static double lo_hz;               /* exact LO after the last tune */


static unsigned chan_mhz(unsigned ch) { return ch == 14 ? 2484 : 2407 + 5 * ch; }

/* Same integer arithmetic as libphy rfpll_set_freq, 40 MHz crystal. */
static double pll_hz(unsigned mhz, int khz) {
    const int32_t d = 120000;
    int32_t a = 4 * (int32_t)(1000 * mhz + khz) - 32 * d;
    int32_t o0 = a / d;
    a -= o0 * d;
    a <<= 8;
    int32_t o1 = a / d;
    a -= o1 * d;
    a <<= 8;
    int32_t o2 = a / d;
    return 30e6 * ((o0 & 255) + 32 + ((o1 & 255) * 256 + (o2 & 255)) / 65536.0);
}


/* Nearest channel 1..13 for the calibration, then the PLL to MHz + kHz. */
static bool tune(uint32_t fkhz) {
    if (fkhz < 2200000 || fkhz > 2800000) return false;
    int ch = ((int)fkhz - 2407000 + 2500) / 5000;
    ch = ch < 1 ? 1 : ch > 13 ? 13 : ch;
    unsigned mhz = fkhz / 1000;
    int khz = (int)(fkhz % 1000);
    set_chanfreq(chan_mhz(ch), 0);
    if (fkhz != chan_mhz(ch) * 1000u) phy_set_freq(mhz, khz);
    stop_tx_tone(1);
    rom_pbus_workmode();
    rom_pbus_xpd_tx_off();
    rom_pbus_xpd_rx_on(1);
    rom_set_rxclk_en(1);
    force_rx_gain(0, 40, 0);                 /* hardware AGC; the receiver is not used here */
    lo_hz = pll_hz(mhz, khz);
    say("LO %" PRIu32 " Hz (channel %d, offset %d kHz)\r\n", (uint32_t)llround(lo_hz), ch, (int)fkhz - chan_mhz(ch) * 1000);
    return true;
}


/* ------------------------------------------------------------ TX */

/* DAC replay engine of the RF block (librftest dactrig): control register, data bank, SRAM owner register. */
#define DAC_CTRL   0x60033D64u
#define DAC_BUF    ((volatile uint32_t *)0x3fcb0000)
#define SRAM_OWNER 0x600c1020u
#define TX_FMIN_KHZ 2300000u
#define TX_FMAX_KHZ 2450000u

static portMUX_TYPE stream_mux = portMUX_INITIALIZER_UNLOCKED;
static void phy_txforce(int on) { ((void (*)(int))g_phyFuns[50])(on); }

/* Leave TX: the same tail as tune(). */
static void tx_leave(void) {
    txcal_work_mode();
    rom_pbus_workmode();
    rom_pbus_xpd_tx_off();
    rom_pbus_xpd_rx_on(1);
    rom_set_rxclk_en(1);
    force_rx_gain(0, 40, 0);
}

/* "7" = channel 7, "2400.25" = MHz with up to three decimals -> kHz */
static bool parse_freq(const char *s, uint32_t *khz) {
    char *e;
    unsigned long whole = strtoul(s, &e, 10);
    if (e == s) return false;
    if (*e != '.' && whole >= 1 && whole <= 14) { *khz = chan_mhz(whole) * 1000; return true; }
    uint32_t frac = 0, scale = 100;
    if (*e == '.') {
        for (++e; *e >= '0' && *e <= '9'; ++e) {
            frac += (*e - '0') * scale;
            scale /= 10;
            if (!scale && e[1] >= '0' && e[1] <= '9') return false;
        }
    }
    if (*e) return false;
    *khz = whole * 1000 + frac;
    return true;
}


static float rrc_pulse(float t, float b) {
    const float pi = 3.14159265f;
    if (fabsf(t) < 1e-6f) return 1.0f - b + 4.0f * b / pi;
    if (fabsf(fabsf(t) - 1.0f / (4.0f * b)) < 1e-4f)
        return b / 1.41421356f * ((1.0f + 2.0f / pi) * sinf(pi / (4.0f * b)) + (1.0f - 2.0f / pi) * cosf(pi / (4.0f * b)));
    const float num = sinf(pi * t * (1.0f - b)) + 4.0f * b * t * cosf(pi * t * (1.0f + b));
    const float den = pi * t * (1.0f - (4.0f * b * t) * (4.0f * b * t));
    return num / den;
}

#include "qpsk_lut.h"

/* ------------------------------------------------------------ QPSK lookup-table modulator (QPSKT), see qpsk_lut.h
 * One output sample = 3 table loads + 2 adds + xor + one store into the held DAC word. Two symbols (A, B) per loop pass; the work
 * of a symbol is spread over its first slots so that no slot overruns its period, and every slot touches the slow USB peripheral
 * at most once:
 *   slot 0   decode the next symbol, table row 0          (A: pointers N, B: pointers P - no register moves)
 *   slot 1   rows 1-2, is a USB byte waiting?
 *   slot 2   A with an empty symbol buffer: refill it (PRBS or ring, 1 in 8 symbols, reports the fill); otherwise read the byte
 *            straight into the ring
 *   slot 3   B: time / stop checks every 2048 symbols
 * The stream is RAW symbol bytes (4 per byte, bit 0 = I, bit 1 = Q level, 1 = +1, first symbol in the low bits): no frames and no
 * parser, so there is nothing to resynchronise. USB is reliable and the ring is only read when it has room; a host that goes quiet
 * for 0.5 s (3 s before the first byte) switches the transmitter off. Up to one byte per symbol can be read (the stream needs 0.25).
 * Define LUT_STATS for lateness statistics (they cost registers and cycles, so they change what is measured). */
#define LUT_STATS 1      /* development: lateness statistics (they cost cycles themselves) */
typedef struct {
    uint32_t qw, qr;               /* ring byte counters, free running (qr stays even: the loop takes 2 bytes = 8 symbols at a time) */
    uint32_t under, pops, lateness, lag_or;
    bool stopped;
} lut_io_t;

#define LUT_RING_BYTES 16384u        /* symbol ring, 64 Ki symbols, from the heap (.bss must stay below the RF dump bank) */

static inline void IRAM_ATTR lut_report(uint32_t fill_pairs, uint32_t under) {
    if (!USB_SERIAL_JTAG.ep1_conf.serial_in_ep_data_free) return;
    USB_SERIAL_JTAG.ep1.val = 0xB7;
    USB_SERIAL_JTAG.ep1.val = fill_pairs & 255;
    USB_SERIAL_JTAG.ep1.val = fill_pairs >> 8;
    USB_SERIAL_JTAG.ep1.val = under > 255 ? 255 : under;
    usb_serial_jtag_ll_txfifo_flush();
}

static inline __attribute__((always_inline)) uint32_t IRAM_ATTR lut_run(lut_io_t *io, uint8_t *qb, const uint8_t *T, uint32_t period, uint32_t max_sym,
                                                                        bool prbs, const uint32_t SPS) {
    const uint32_t S = LUT_ROW_SHIFT(SPS);
    const uint8_t *T1 = T + 256u * SPS * 4u, *T2 = T1 + 256u * SPS * 4u;
    uint32_t C = 0, bits = 0, nleft = 1, rng = 0x2545F491u, tn = ccount() + 8000u, chk = 1024, passes = 0, qw = 0, qr = 0, qw_seen = 0, under = 0, pops = 0;
    uint32_t t_rx = ccount();
#ifdef LUT_STATS
    uint32_t lt = 0, mx = 0;
#endif
    bool have = false;
    const uint8_t *P0 = T + LUT_OFF(C, 0u, S), *P1 = T1 + LUT_OFF(C, 1u, S), *P2 = T2 + LUT_OFF(C, 2u, S);
    const uint8_t *N0 = P0, *N1 = P1, *N2 = P2;
#define LUT_WORD(a, b, c, j) ((*(const uint32_t *)((a) + 4u * (j)) + *(const uint32_t *)((b) + 4u * (j)) + *(const uint32_t *)((c) + 4u * (j))) ^ LUT_XOR)
    uint32_t word = LUT_WORD(P0, P1, P2, 0u);
    for (;;) {
#pragma GCC unroll 32
        for (uint32_t u = 0; u < 2u * SPS; ++u) {
            const uint32_t j = u % SPS;
            const bool odd = u >= SPS;
            uint32_t now;
            do { now = ccount(); } while ((int32_t)(now - tn) < 0);
            DAC_BUF[0] = word;
#ifdef LUT_STATS
            lt += (now - tn) > 6u;
            mx |= now - tn;
#endif
            tn += period;
            if (j == 0u) {
                const uint32_t sym = bits & 3u;
                bits >>= 2;
                --nleft;
                C = ((C << 2) | sym) & 0xFFFFFFu;
                if (!odd) { N0 = T + LUT_OFF(C, 0u, S); __asm__ volatile("" : "+r"(N0), "+r"(C), "+r"(bits), "+r"(nleft)); }
                else      { P0 = T + LUT_OFF(C, 0u, S); __asm__ volatile("" : "+r"(P0), "+r"(C), "+r"(bits), "+r"(nleft)); }
            } else if (j == 1u) {
                if (!odd) { N1 = T1 + LUT_OFF(C, 1u, S); N2 = T2 + LUT_OFF(C, 2u, S); __asm__ volatile("" : "+r"(N1), "+r"(N2)); }
                else      { P1 = T1 + LUT_OFF(C, 1u, S); P2 = T2 + LUT_OFF(C, 2u, S); __asm__ volatile("" : "+r"(P1), "+r"(P2)); }
                have = !prbs && USB_SERIAL_JTAG.ep1_conf.serial_out_ep_data_avail;
                __asm__ volatile("" : "+r"(have) : : "memory");
            } else if (j == 2u) {
                if (!nleft) {                          /* only after an A decode: nleft is a multiple of 8 there */
                    if (prbs) {
                        rng ^= rng << 13; rng ^= rng >> 17; rng ^= rng << 5;
                        bits = rng;
                        nleft = 16;
                    } else if (qw - qr >= 2u) {
                        bits = *(const uint16_t *)(qb + (qr & (LUT_RING_BYTES - 1u)));
                        qr += 2u;
                        nleft = 8;
                        if (((++pops) & 255u) == 0) lut_report((qw - qr) >> 1, under);
                    } else {
                        ++under;
                        rng ^= rng << 13; rng ^= rng >> 17; rng ^= rng << 5;
                        bits = rng;
                        nleft = 8;
                    }
                    __asm__ volatile("" : "+r"(bits), "+r"(nleft), "+r"(rng) : : "memory");
                } else if (have && qw - qr < LUT_RING_BYTES - 64u) {      /* ring full: USB holds the PC back */
                    qb[qw & (LUT_RING_BYTES - 1u)] = (uint8_t)USB_SERIAL_JTAG.ep1.val;
                    ++qw;
                    __asm__ volatile("" : "+r"(qw) : : "memory");
                }
            } else if (j == 3u) {
                if (odd && --chk == 0) {
                    chk = 1024;
                    passes += 1;
                    if (prbs) {
                        if (USB_SERIAL_JTAG.ep1_conf.serial_out_ep_data_avail) { io->stopped = true; goto lut_out; }
                    } else {
                        const uint32_t t = ccount();
                        if (qw != qw_seen) { qw_seen = qw; t_rx = t; }
                        else if (t - t_rx > (qw ? CPU_HZ / 2 : 3 * CPU_HZ)) goto lut_out;
                    }
                    if (passes * 2048u > max_sym) goto lut_out;
                }
                __asm__ volatile("" : : : "memory");
            }
            if (!odd) word = j + 1u < SPS ? LUT_WORD(P0, P1, P2, j + 1u) : LUT_WORD(N0, N1, N2, 0u);
            else      word = j + 1u < SPS ? LUT_WORD(N0, N1, N2, j + 1u) : LUT_WORD(P0, P1, P2, 0u);
        }
    }
lut_out:
#undef LUT_WORD
    io->qw = qw; io->qr = qr; io->under = under; io->pops = pops;
#ifdef LUT_STATS
    io->lateness = lt; io->lag_or = mx;      /* lag_or: OR of all slot lags, its top bit is the order of the worst one */
#endif
    return passes * 2048u;
}

#define LUT_RUN_FN(N) static uint32_t IRAM_ATTR __attribute__((noinline)) tx_sym_lut_##N(lut_io_t *io, uint8_t *qb, const uint8_t *T, uint32_t period, uint32_t max_sym, bool prbs) { \
    return lut_run(io, qb, T, period, max_sym, prbs, N); }
LUT_RUN_FN(4)
LUT_RUN_FN(8)
LUT_RUN_FN(16)

/* ------------------------------------------------------------ 8 MS/s modulator (lut8.S, generated by gen_lut8.py)
 * 1 MBd with 8 samples per symbol leaves 20 CPU cycles per sample: the loop is hand-scheduled assembly with a fixed cycle
 * count per slot and one cycle-counter sync per symbol, so every DAC store lands exactly 20 cycles after the previous one.
 * RRC span 8 symbols (2 tables); the zero-order-hold images move from +-4 MHz to +-8 MHz. Ring mode only (USB stream). */
typedef struct {
    const int32_t *t0, *t1;    /*  0,  4 */
    uint8_t *ring;             /*  8 */
    uint32_t qw, qr;           /* 12, 16 */
    uint32_t tn;               /* 20: start of the first symbol (cycle counter) */
    uint32_t nsp;              /* 24: superpasses (8 symbols) to run */
    uint32_t under;            /* 28 */
    uint32_t mper, mcnt;       /* 32, 36: maintenance period / countdown (superpasses) */
    uint32_t trx, qwseen;      /* 40, 44: last time a USB byte arrived */
    uint32_t lim, lim1;        /* 48, 52: silence limit now / after the first byte (cycles) */
    uint32_t exitc;            /* 56 */
    uint32_t late;             /* 60 */
    uint32_t rec[128];         /* 64: timing records (timing build) */
} lut8_ctx_t;
_Static_assert(offsetof(lut8_ctx_t, rec) == 64, "lut8_ctx_t layout must match lut8.S");
extern uint32_t lut8_run(lut8_ctx_t *c);

static uint32_t tx_lut8(lut8_ctx_t *c, uint8_t *ring, const int32_t *T, uint32_t max_sym) {
    memset(c, 0, sizeof(*c));
    c->t0 = T;
    c->t1 = T + 256 * 8;
    c->ring = ring;
    c->nsp = max_sym / 8 + 1;
    c->mper = c->mcnt = 256;                   /* report every 2048 symbols */
    c->lim = 3 * CPU_HZ;
    c->lim1 = CPU_HZ / 2;
    c->tn = ccount() + 20000;
    c->trx = c->tn;
    const uint32_t nsp0 = c->nsp;
    lut8_run(c);
    return (nsp0 - c->nsp) * 8;
}

/* ------------------------------------------------------------ generic 8 / 6.67 MS/s modulator (lutg.S, generated by gen_lutg.py)
 * Any samples per symbol S in LUTG_MIN_S..LUTG_MAX_S at 20 or 24 CPU cycles per DAC store (8 or 6.67 MS/s), RRC span 8 symbols in four
 * 2-symbol table groups (qpsk_lut.h, lut_build_t). Ring mode only (USB stream). Define LUTG_REC 1 for the timing build (the loop records
 * the cycle counter of every store instead of driving the DAC) or 2 to record the DAC words (gen_lutg.py --rec timing|words). */
#define LUTG_MIN_S 16
#define LUTG_MAX_S 232           /* tables take 256 * S bytes plus a temporary 32 * S: the heap after Wi-Fi init holds about 78 KB with the 16 KB ring */
/* #define LUTG_REC 1 */
typedef struct {
    const int32_t *t[4];       /*   0 table group bases */
    uint8_t *ring;             /*  16 */
    uint32_t qw, qr;           /*  20, 24: ring bytes written, symbols read (free running) */
    uint32_t tn;               /*  28: start of the first symbol (cycle counter) */
    uint32_t nsym;             /*  32: symbols left */
    uint32_t under;            /*  36 */
    uint32_t mper, mcnt;       /*  40, 44: maintenance period / countdown (symbols) */
    uint32_t trx, qwseen;      /*  48, 52: last time a USB byte arrived */
    uint32_t lim, lim1;        /*  56, 60: silence limit now / after the first byte (cycles) */
    uint32_t exitc;            /*  64 */
    uint32_t late;             /*  68: sync points that were not on schedule */
    uint32_t s64m, sper, rlim; /*  72, 76, 80: 64 * (S - 1), S * P, ring room limit in symbols */
    uint32_t ns[4];            /*  84: sled ends of the code copies (filled by the asm) */
    uint32_t sledrec;          /* 100 */
    uint32_t *rec, *recp;      /* 104, 108 */
} lutg_ctx_t;
_Static_assert(offsetof(lutg_ctx_t, ring) == 16 && offsetof(lutg_ctx_t, nsym) == 32 && offsetof(lutg_ctx_t, mper) == 40 && offsetof(lutg_ctx_t, trx) == 48 &&
               offsetof(lutg_ctx_t, lim) == 56 && offsetof(lutg_ctx_t, exitc) == 64 && offsetof(lutg_ctx_t, s64m) == 72 && offsetof(lutg_ctx_t, ns) == 84 &&
               offsetof(lutg_ctx_t, sledrec) == 100 && offsetof(lutg_ctx_t, rec) == 104 && sizeof(lutg_ctx_t) == 112, "lutg_ctx_t layout must match lutg.S");
extern uint32_t lutg_run_p20(lutg_ctx_t *c);
extern uint32_t lutg_run_p24(lutg_ctx_t *c);
extern uint32_t lutg_psk8_run_p20(lutg_ctx_t *c);                 /* 8PSK: 3 bit symbols, one per nibble of the ring */
extern uint32_t lutg_p8s8_run_p20(lutg_ctx_t *c);                 /* 8PSK at 1 MBd (8 samples per symbol): 3 bits per symbol packed, ring in bytes, one pass = 8 symbols = 3 bytes */
extern uint32_t lutg_a16_run_p20(lutg_ctx_t *c);                  /* 16APSK: 4 bit symbols, one per nibble of an 8 KB ring, RRC span 6 */
#define LUTG8_MAX_S 64                                              /* 8PSK tables take 1024 * S bytes: 64 KB at S = 64 (125 kBd) next to the 16 KB ring is what the heap holds */
#define LUTA16_MIN_S 16                                             /* 16APSK tables take 3072 * S bytes: 74 KB at S = 24 is all the heap holds */
#define LUTA16_MAX_S 24
#define LUT_RING_A16 8192u
#define LUTG8_S8 8
#define LUTG_P8S8_PHASE 0       /* start phase of the 1 MBd 8PSK loop in cycles (mod 10), tools/lutg_tune_p8s8.py --secs N tunes the pads for phase N mod 10 */

#ifdef LUTG_REC
/* Recording builds: a fault inside the loop (interrupts are off, the IDF panic output never gets out) prints the trap CSRs over USB instead. */
static void IRAM_ATTR trap_put(char ch) {
    while (!USB_SERIAL_JTAG.ep1_conf.serial_in_ep_data_free) usb_serial_jtag_ll_txfifo_flush();
    USB_SERIAL_JTAG.ep1.val = (uint8_t)ch;
}
static void IRAM_ATTR trap_hex(const char *tag, uint32_t v) {
    for (; *tag; ++tag) trap_put(*tag);
    for (int i = 28; i >= 0; i -= 4) trap_put("0123456789abcdef"[(v >> i) & 15]);
    trap_put(' ');
}
static void IRAM_ATTR __attribute__((aligned(256))) trap_dump(void) {
    uint32_t mepc, mcause, mtval;
    __asm__ volatile("csrr %0, mepc" : "=r"(mepc));
    __asm__ volatile("csrr %0, mcause" : "=r"(mcause));
    __asm__ volatile("csrr %0, mtval" : "=r"(mtval));
    trap_hex("\r\nTRAP mepc=", mepc); trap_hex("mcause=", mcause); trap_hex("mtval=", mtval);
    trap_put('\r'); trap_put('\n');
    usb_serial_jtag_ll_txfifo_flush();
    for (volatile uint32_t i = 0; i < 4000000; ++i) {}
    esp_rom_software_reset_cpu(0);
    for (;;) {}
}
#endif

#ifdef LUTG_REC
static uint32_t rec_phase;
#endif
static uint32_t tx_lutg(lutg_ctx_t *c, uint8_t *ring, const int32_t *T, uint32_t S, uint32_t period, uint32_t max_sym, uint32_t mper, uint32_t *rec, bool psk8, bool a16) {
    const bool p8s8 = psk8 && S == LUTG8_S8;                         /* 1 MBd 8PSK: own loop, bit-packed symbols, row pointers biased by +1024 bytes */
    const uint32_t ring_bytes = a16 ? LUT_RING_A16 : LUT_RING_BYTES;
    memset(c, 0, sizeof(*c));
    for (uint32_t g = 0; g < 4; ++g) c->t[g] = a16 ? T + g * 256 * S : T + g * S * (psk8 ? 64 : 16) + (p8s8 ? 256 : 0);       /* words per group: rows (16, 8PSK 64, 16APSK 256) * S */
    c->ring = ring;
    c->nsym = max_sym;
    c->mper = c->mcnt = mper;                                        /* symbols between maintenance passes (silence check, fill report) */
    c->lim = 3 * CPU_HZ;
    c->lim1 = CPU_HZ / 2;
    c->s64m = a16 ? 4u * (S - 1) : (psk8 ? 256u : 64u) * (S - 1);   /* bytes from the first to the last sample of a table row set (16APSK: of a row) */
    c->sper = S * period;
    c->rlim = p8s8 ? (LUT_RING_BYTES - 64) : ((psk8 || a16) ? 2u : 4u) * (ring_bytes - 64);     /* ring room in symbols (4 per byte, 2 for 8PSK and 16APSK), in bytes for the 1 MBd 8PSK loop */
    c->rec = rec;
#ifdef LUTG_REC
    uint32_t rng = 0x2545F491u;                                       /* known symbols in a full ring: no USB needed */
    for (uint32_t i = 0; i < ring_bytes; ++i) { rng ^= rng << 13; rng ^= rng >> 17; rng ^= rng << 5; ring[i] = (uint8_t)(rng >> 8); }
    c->qw = ring_bytes - 128;
#endif
#ifdef LUTG_REC
    c->tn = ccount() + 20000 + rec_phase;                            /* recording builds: the 'seconds' argument shifts the start phase against the DAC / USB clocks */
#else
    c->tn = ccount() + 20000;
#endif
    if (p8s8) {                                                      /* USB accesses complete on the 48 MHz clock: a fixed start phase (mod 10 cycles) makes every run, recording or production, see the same timing */
#ifdef LUTG_REC
        const uint32_t ph = rec_phase % 10u;
#else
        const uint32_t ph = LUTG_P8S8_PHASE;
#endif
        c->tn = (ccount() + 20000) / 10u * 10u + ph;
    }
    c->trx = c->tn;
    const uint32_t n0 = c->nsym;
#ifdef LUTG_REC
    uint32_t mtvec0;
    __asm__ volatile("csrr %0, mtvec" : "=r"(mtvec0));
    __asm__ volatile("csrw mtvec, %0" :: "r"((uintptr_t)trap_dump));
#endif
    if (a16) lutg_a16_run_p20(c);
    else if (p8s8) lutg_p8s8_run_p20(c);
    else if (psk8) lutg_psk8_run_p20(c);
    else if (period == 20) lutg_run_p20(c);
    else lutg_run_p24(c);
#ifdef LUTG_REC
    __asm__ volatile("csrw mtvec, %0" :: "r"(mtvec0));
#endif
    return n0 - c->nsym;
}

/* ------------------------------------------------------------ narrowband 8PSK (C loop with rational cycle-counter deadlines, at least 75 cycles per sample)
 * The assembly loops hold S table samples of every row, 1024 * S bytes of tables: 66 and 33 kBd would need about 121 and 242 KiB near 8 MS/s. Here the DAC is slower (2.112 MS/s,
 * S = 32 at 66 kBd or 64 at 33 kBd), so the tables fit. A persistent remainder accumulator alternates floor/ceil cycle intervals;
 * the average rate is the requested baud, with less than one cycle of deadline error. A minimum interval of 75 cycles leaves room
 * for the accumulator, symbol decode, and USB work. The 8 MS/s assembly loops retain their fixed timing.
 * Tables as for the generic 8PSK loop (lut_build_p8, row pointer of group g = T + g * S * 64 + index, the next sample 64 words further); the ring holds one symbol per
 * nibble, the first symbol of a byte in the low nibble. In a recording build (LUTG_REC) the words of the first symbols go to rec. */
static uint32_t IRAM_ATTR __attribute__((noinline)) tx_p8_c(lut_io_t *io, uint8_t *qb, const int32_t *T, uint32_t S, sample_clock_t clock, uint32_t max_sym, uint32_t *rec) {
    const uint32_t gs = 64u * S;                                     /* words from one group's tables to the next */
    const int32_t *r0 = T, *r1 = T + gs, *r2 = T + 2u * gs, *r3 = T + 3u * gs, *n0 = r0, *n1 = r1, *n2 = r2, *n3 = r3;
    uint32_t C = 0, qw = 0, qr = 0, under = 0, late = 0, nsym = 0, tn = ccount() + 20000u, qw_seen = 0, t_rx = ccount(), pops = 0;
    uint32_t phase = 0;                                             /* continuous across symbols, maintenance, and cycle-counter wrap */
    bool have = false;
    uint32_t repw = 0, repn = 0;                                     /* report: four bytes plus a flush, each in a separate slot from USB RX */
#ifdef LUTG_REC
    qw = LUT_RING_BYTES - 128;                                       /* the recording build fills the ring with known symbols */
#endif
    uint32_t word = ((uint32_t)*r0 + (uint32_t)*r1 + (uint32_t)*r2 + (uint32_t)*r3) ^ LUT_XOR;
    for (;;) {
        const bool maintenance = (nsym & 2047u) == 2047u;
        for (uint32_t j = 0; j < S; ++j) {
            uint32_t now;
            do { now = ccount(); } while ((int32_t)(now - tn) < 0);
            DAC_BUF[0] = word;
            late += (now - tn) > 8u;
            tn += sample_clock_next(clock, &phase);
#ifdef LUTG_REC
            if (rec && nsym < 12) rec[nsym * S + j] = word;
#endif
            if (j == 0) {                                            /* the symbol that the next one's history needs */
                uint32_t sym = 0;
                if ((int32_t)(2u * qw - qr) > 0) { sym = (qb[(qr >> 1) & (LUT_RING_BYTES - 1u)] >> ((qr & 1u) * 4u)) & 7u; ++qr; }
                else ++under;
                C = ((C << 3) | sym) & 0xFFFFFFu;
            } else if (j == 1) {
                n0 = T + (C & 63u);
                n1 = T + gs + ((C >> 6) & 63u);
            } else if (j == 2) {
                n2 = T + 2u * gs + ((C >> 12) & 63u);
                n3 = T + 3u * gs + ((C >> 18) & 63u);
            }
            if ((j & 7u) == 3u) have = USB_SERIAL_JTAG.ep1_conf.serial_out_ep_data_avail;          /* the USB registers are slow: look in one slot, read in another */
            else if ((j & 7u) == 5u && have && qw - (qr >> 1) < LUT_RING_BYTES - 64u) {            /* ring full: USB holds the PC back */
                qb[qw & (LUT_RING_BYTES - 1u)] = (uint8_t)USB_SERIAL_JTAG.ep1.val;
                ++qw;
                have = false;
            }
            if (j == 3u && max_sym && nsym >= max_sym) goto out;
            if (maintenance && j >= 4u) {
                if (j == 4u) {
                    repn = USB_SERIAL_JTAG.ep1_conf.serial_in_ep_data_free ? 5u : 0u;
                    ++pops;
                } else if (j == 6u && repn) {
                    const uint32_t fill = (qw - (qr >> 1)) >> 1;
                    repw = 0xB7u | (fill & 255u) << 8 | (fill >> 8) << 16 | (under > 255u ? 255u : under) << 24;
                } else if (j >= 7u && j <= 10u && repn > 1u) {
                    USB_SERIAL_JTAG.ep1.val = (uint8_t)repw;
                    repw >>= 8;
                    --repn;
                } else if (j == 12u && repn == 1u) {
                    usb_serial_jtag_ll_txfifo_flush();
                    repn = 0;
                } else if (j == 14u) {
#ifndef LUTG_REC
                    const uint32_t t = ccount();
                    if (qw != qw_seen) { qw_seen = qw; t_rx = t; }
                    else if (t - t_rx > (qw ? CPU_HZ / 2 : 3 * CPU_HZ)) goto out;
#endif
                }
            }
            if (j + 1u < S) { r0 += 64; r1 += 64; r2 += 64; r3 += 64; }
            else { r0 = n0; r1 = n1; r2 = n2; r3 = n3; }
            word = ((uint32_t)*r0 + (uint32_t)*r1 + (uint32_t)*r2 + (uint32_t)*r3) ^ LUT_XOR;
        }
        ++nsym;
    }
out:
    io->qw = qw; io->qr = qr >> 1; io->under = under; io->pops = pops; io->lateness = late; io->stopped = false;
    (void)qw_seen; (void)t_rx;
    return nsym;
}

static void tx_qpsk_lut(uint32_t fkhz, uint32_t baud_req, uint32_t sps, int32_t amp, uint32_t secs, int32_t ifm, int32_t target, int32_t dc4i, int32_t dc4q,
                        int32_t gq, int32_t phim, bool psk8, bool a16, uint32_t gamma100) {
    const uint32_t period = (CPU_HZ + baud_req * sps / 2) / (baud_req * sps);
    const sample_clock_t clock = sample_clock_init(CPU_HZ, baud_req * sps);
    const bool fractional = psk8 && period != 20 && sps >= LUTG_MIN_S && sps <= LUTG8_MAX_S && clock.period >= 75;
    const double baud = fractional ? baud_req : (double)CPU_HZ / ((double)period * sps), fso = baud * sps;
    const double half_bw = baud * 1.35 / 2.0, lo_nom = fkhz * 1000.0, ifh = ifm * baud;
    const double sig_lo = lo_nom + (ifh < 0 ? ifh : 0) - half_bw, sig_hi = lo_nom + (ifh > 0 ? ifh : 0) + half_bw;
    if (period < 16) { say("ERR QPSKT period %lu cycles < 16 (baud * sps too high)\r\n", (unsigned long)period); return; }
    if (sig_lo < TX_FMIN_KHZ * 1000.0 || sig_hi > TX_FMAX_KHZ * 1000.0) { say("ERR QPSKT only in the 13 cm band (2300..2450 MHz)\r\n"); return; }
    if (fabs(ifh) + half_bw >= fso / 2.0) { say("ERR QPSKT IF too high for the output rate (%.0f Hz)\r\n", fso); return; }
    static uint8_t *ring;
    free(ring);                                                      /* every run starts from a clean heap: the tables (up to 74 KB) take the big block first, the ring goes into what is left */
    ring = NULL;
    const bool fast8 = !psk8 && !a16 && sps == 8 && period == 20;    /* 8 MS/s at 1 MBd: hand-scheduled loop, 2 table groups */
    const bool fastg = a16 ? (sps >= LUTA16_MIN_S && sps <= LUTA16_MAX_S && period == 20)                       /* 16APSK: 8 MS/s with 16..24 samples per symbol */
                    : psk8 ? ((sps == LUTG8_S8 || (sps >= LUTG_MIN_S && sps <= LUTG8_MAX_S)) && period == 20)   /* 8PSK: 1 MBd loop or the generic loop, at 8 MS/s only */
                            : (!fast8 && sps >= LUTG_MIN_S && sps <= LUTG_MAX_S && (period == 20 || period == 24));   /* generic loop, 8 or 6.67 MS/s */
    if (a16 && !fastg) {
        say("ERR A16T 16APSK needs 8 MS/s with %d..%d samples per symbol (baud * sps = 8 MHz, i.e. %lu .. %lu Bd)\r\n", LUTA16_MIN_S, LUTA16_MAX_S, (unsigned long)(8000000 / LUTA16_MAX_S), (unsigned long)(8000000 / LUTA16_MIN_S));
        return;
    }
    const bool p8c = fractional;                                    /* narrowband 8PSK: minimum floor interval includes accumulator overhead */
    if (psk8 && !fastg && !p8c) {
        say("ERR PSK8T 8PSK needs 8 MS/s with 8 or %d..%d samples per symbol, or %d..%d samples per symbol with a minimum interval of 75 cycles (baud * sps <= 2.13 MHz)\r\n", LUTG_MIN_S, LUTG8_MAX_S, LUTG_MIN_S, LUTG8_MAX_S);
        return;
    }
    if (!fast8 && !fastg && !p8c && sps != 4 && sps != 8 && sps != 16) {
        say("ERR QPSKT %lu samples per symbol need 20 or 24 CPU cycles per sample (baud * sps = 8 or 6.67 MHz, sps %d..%d)\r\n", (unsigned long)sps, LUTG_MIN_S, LUTG_MAX_S);
        return;
    }
#ifdef LUTG_REC
    if (fast8) { say("ERR QPSKT the recording build has no 1 MBd loop\r\n"); return; }
    if (fastg || p8c) target = 1;
    rec_phase = secs;
#endif
    if ((fast8 || fastg || p8c) && target == 0) { say("ERR QPSKT the 8 / 6.67 MS/s loops need the USB stream (target > 0)\r\n"); return; }
    int32_t *T = malloc(fast8 ? 4 * 2 * 256 * 8 : (fastg || p8c) ? (a16 ? lutt16_bytes(sps) : psk8 ? lutt8_bytes(sps) : lutt_bytes(sps)) : 4 * lut_words(sps));
    if (T) ring = malloc(a16 ? LUT_RING_A16 : LUT_RING_BYTES);          /* 16APSK: an 8 KB ring, the RAM below the RF dump bank has to hold the loop's code beside it */
    if (!ring || !T) { free(T); free(ring); ring = NULL; say("ERR out of memory\r\n"); return; }
    if (fastg || p8c) {
        float *hb = malloc(8 * sps * sizeof(float));
        if (!hb) { free(T); say("ERR out of memory\r\n"); return; }
        if (a16) lut_build_a16(T, sps, ifm, 0.35f, (float)amp, dc4i / 16.0f, dc4q / 16.0f, gq / 10000.0f, phim / 1000.0f, hb, gamma100);
        else if (psk8) lut_build_p8(T, sps, ifm, 0.35f, (float)amp, dc4i / 16.0f, dc4q / 16.0f, gq / 10000.0f, phim / 1000.0f, hb);
        else lut_build_t(T, sps, ifm, 0.35f, (float)amp, dc4i / 16.0f, dc4q / 16.0f, gq / 10000.0f, phim / 1000.0f, hb);
        free(hb);
    } else {
        lut_build_g(T, sps, ifm, 0.35f, (float)amp, dc4i / 16.0f, dc4q / 16.0f, gq / 10000.0f, phim / 1000.0f, fast8 ? 2u : 3u);
    }
    static lut8_ctx_t c8;
    static lutg_ctx_t cg;
    uint32_t *rec = NULL;
#ifdef LUTG_REC
    rec = malloc(4 * ((psk8 && sps == LUTG8_S8 ? 64 : 12) * sps + 16));
    if ((fastg || p8c) && !rec) { free(T); say("ERR out of memory\r\n"); return; }
#endif
    if (!tune(fkhz)) { free(T); say("ERR TUNE\r\n"); return; }
    const uint32_t owner0 = rd(SRAM_OWNER);
    phy_txforce(1);
    esp_rom_delay_us(3000);
    DAC_BUF[0] = 0;
    usb_drain_rx();
    say("OK QPSKT LO %.0f BAUD %.3f OUT %.0f AMP %ld SPS %lu PERIOD %lu IF %.0f TARGET %ld MOD %s REM %lu DIV %lu\r\n", lo_hz, baud, fso, (long)amp, (unsigned long)sps, (unsigned long)(p8c ? clock.period : period), ifh, (long)target, a16 ? "16APSK" : psk8 ? "8PSK" : "QPSK", (unsigned long)(p8c ? clock.remainder : 0), (unsigned long)(p8c ? clock.divisor : 1));
    lut_io_t io = {0};
#ifdef LUTG_REC
    esp_rom_delay_us(150000);                                        /* time for the host to put bytes into the USB FIFO (exercises the read slots) */
#endif
#ifdef LUTG_REC
    const uint32_t max_sym = psk8 && sps == LUTG8_S8 ? 64 : 12, mper = 3;        /* the 1 MBd 8PSK loop counts passes of 8 symbols: 8 passes = N N N M N N N and a cut one */
#else
    const uint32_t max_sym = (uint32_t)((double)secs * baud) & ~7u, mper = psk8 && sps == LUTG8_S8 ? 256 : 2048;
#endif
#ifdef LUTG_REC
    if (p8c) {
        uint32_t rng = 0x2545F491u;                                  /* known symbols in a full ring (the same generator as tx_lutg) */
        for (uint32_t i = 0; i < LUT_RING_BYTES; ++i) { rng ^= rng << 13; rng ^= rng >> 17; rng ^= rng << 5; ring[i] = (uint8_t)(rng >> 8); }
    }
#endif
    taskENTER_CRITICAL(&stream_mux);
    wr(SRAM_OWNER, (owner0 & ~7u) | 2u | 8u);
    __asm__ volatile("fence rw,rw" ::: "memory");
    wr(DAC_CTRL, rd(DAC_CTRL) & ~(1u << 31));
    wr(DAC_CTRL, 0x80000u | 1u);
    wr(DAC_CTRL, 0x80000u | 1u | (1u << 31));
    const uint32_t n = fast8 ? tx_lut8(&c8, ring, T, max_sym)
               : p8c ? tx_p8_c(&io, ring, T, sps, clock, max_sym, rec)
               : fastg ? tx_lutg(&cg, ring, T, sps, period, max_sym, mper, rec, psk8, a16)
               : sps == 4 ? tx_sym_lut_4(&io, ring, (const uint8_t *)T, period, max_sym, target == 0)
                     : sps == 8 ? tx_sym_lut_8(&io, ring, (const uint8_t *)T, period, max_sym, target == 0)
                                : tx_sym_lut_16(&io, ring, (const uint8_t *)T, period, max_sym, target == 0);
    DAC_BUF[0] = 0;
    wr(DAC_CTRL, rd(DAC_CTRL) & ~(1u << 31));
    wr(SRAM_OWNER, owner0);
    taskEXIT_CRITICAL(&stream_mux);
    phy_txforce(0);
    tx_leave();
    free(T);
    vTaskDelay(pdMS_TO_TICKS(30));
    usb_drain_rx();
    if (fast8) { io.qw = c8.qw; io.qr = c8.qr; io.under = c8.under; io.lateness = c8.late; io.stopped = false; }
    if (fastg) { io.qw = cg.qw; io.qr = psk8 && sps == LUTG8_S8 ? cg.qr : cg.qr / ((psk8 || a16) ? 2 : 4); io.under = cg.under; io.lateness = cg.late; io.stopped = false; }
#ifdef LUTG_REC
    if (p8c) {                                                       /* the narrowband loop records the words of its first 12 symbols (LUTG_REC 2); the ring symbols are the known ones */
        for (uint32_t k = 0; k < n && k < 12; ++k)
            for (uint32_t j = 0; j < sps; j += 8) {
                say("W%lu N %lu:", (unsigned long)k, (unsigned long)j);
                for (uint32_t i = j; i < j + 8 && i < sps; ++i) say(" %05lx", (unsigned long)rec[k * sps + i]);
                say("\r\n");
            }
        say("SLED 0 late %lu exit 0\r\n", (unsigned long)io.lateness);
    }
    if (fastg) {
        const bool p8s8 = psk8 && sps == LUTG8_S8;
        const uint32_t mp = cg.mper + (p8s8 ? 1 : 3), pass = p8s8 ? 8 : 1;             /* the 1 MBd 8PSK loop: one maintenance copy M, the copies change every pass of 8 symbols */
        for (uint32_t k = 0; k < n; ++k) {
            const uint32_t *w = cg.rec + k * sps;
            const char cp = ((k / pass) % mp) < cg.mper ? 'N' : p8s8 ? 'M' : "ABC"[((k / pass) % mp) - cg.mper];
#if LUTG_REC == 1
            say("T%lu %c:", (unsigned long)k, cp);
            int shown = 0;
            for (uint32_t j = 0; j + 1 < sps; ++j) {
                const long d = (long)(w[j + 1] - w[j]);
                if (d != (long)period && shown++ < 60) say(" %lu:%ld", (unsigned long)j, d);
            }
            say(" | E %ld\r\n", k + 1 < n ? (long)(cg.rec[(k + 1) * sps] - w[sps - 1]) : -1L);
#else
            for (uint32_t j = 0; j < sps; j += 8) {
                say("W%lu %c %lu:", (unsigned long)k, cp, (unsigned long)j);
                for (uint32_t i = j; i < j + 8 && i < sps; ++i) say(" %05lx", (unsigned long)w[i]);
                say("\r\n");
            }
#endif
        }
        say("SLED %lu late %lu exit %lu\r\n", (unsigned long)(cg.sledrec / 4), (unsigned long)cg.late, (unsigned long)cg.exitc);
    }
    free(rec);
#endif
#ifdef LUT8_TIMING
    if (fast8) {
        for (int cp = 0; cp < 2; ++cp) {
            say("\r\nT8 copy %c slot spacing:", cp ? 'M' : 'N');
            for (int k = 1; k < 64; ++k) say(" %ld", (long)(c8.rec[cp * 64 + k] - c8.rec[cp * 64 + k - 1]));
        }
        say("\r\nT8 exit %lu late %lu\r\n", (unsigned long)c8.exitc, (unsigned long)c8.late);
    }
#endif
    say("\r\nTX END symbols=%" PRIu32 " bytes_in=%" PRIu32 " underruns=%" PRIu32 " late_slots=%" PRIu32 " (lag bits 0x%" PRIx32 ") buffer=%" PRIu32 " pairs (%s)\r\n",
        n * sps, io.qw, io.under, io.lateness, io.lag_or, (io.qw - io.qr) / 2, io.stopped ? "stop byte" : "host silent or time over");
}


/* ------------------------------------------------------------ commands */

static void handle(char *line) {
    if (!strcmp(line, "INFO")) {
        say("ESP32DATV 1\r\n");
    } else if (!strcmp(line, "HEAP")) {
        say("HEAP free %u largest %u\r\n", (unsigned)heap_caps_get_free_size(MALLOC_CAP_8BIT), (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));
    } else if (!strncmp(line, "QPSKT ", 6) || !strncmp(line, "PSK8T ", 6) || !strncmp(line, "A16T ", 5)) {
        const bool a16 = line[0] == 'A';                                   /* A16T: the same arguments plus gamma x 100 (R2 / R1 of the rings), 4 bit symbols (nibbles in the ring) */
        const bool psk8 = line[0] == 'P';                                  /* PSK8T: the same arguments, 3 bit symbols (nibbles in the ring) */
        char fs_[32];
        uint32_t fk = 0;
        long baud = 0, sps = 4, amp = 300, secs = 1800, ifm = 0, tgt = 0, dc4i = 0, dc4q = 0, gq = 10000, phim = 0, gam = 315;
        const bool parsed = sscanf(line + (a16 ? 5 : 6), "%31s %ld %ld %ld %ld %ld %ld %ld %ld %ld %ld %ld", fs_, &baud, &sps, &amp, &secs, &ifm, &tgt, &dc4i, &dc4q, &gq, &phim, &gam) >= 3 &&
                            parse_freq(fs_, &fk);
        if (!parsed || baud < 2000 || baud > 1500000 || sps < 4 || sps > LUTG_MAX_S || amp < 1 || amp > 480 || secs < 1 || secs > 86400 || labs(ifm) > 6 ||
            tgt < 0 || tgt > 6000 || (tgt > 0 && tgt < 64) || gq < 7000 || gq > 13000 || labs(phim) > 40000 || gam < 200 || gam > 400) {
            say("ERR QPSKT|PSK8T|A16T f_MHz baud sps(4|8|16, or 16..232 when baud * sps = 8 or 6.67 MHz; PSK8T: 16..64 at 8 MHz) [amp 1..480 [seconds [if in multiples of baud, +-6 [target 0 = PRBS in the ESP, >0 = stream from USB [dcI dcQ in 1/16 code [g in 1e-4 [phase in 1e-3 deg [gamma x 100: 16APSK ring ratio 200..400]]]]]]]]]\r\n");
            return;
        }
        tx_qpsk_lut(fk, (uint32_t)baud, (uint32_t)sps, (int32_t)amp, (uint32_t)secs, (int32_t)ifm, (int32_t)tgt, (int32_t)dc4i, (int32_t)dc4q, (int32_t)gq, (int32_t)phim, psk8, a16, (uint32_t)gam);
    } else {
        say("ERR ? (commands: INFO, HEAP, QPSKT, PSK8T, A16T)\r\n");
    }
}

void app_main(void) {
    esp_err_t e = nvs_flash_init();
    if (e == ESP_ERR_NVS_NO_FREE_PAGES || e == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        e = nvs_flash_init();
    }
    ESP_ERROR_CHECK(e);
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_NULL));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
    ESP_ERROR_CHECK(esp_wifi_set_promiscuous(true));
    ESP_ERROR_CHECK(esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE));
    tune(2412000);                               /* the PHY needs a valid channel before the first TX */

    char line[128];
    size_t used = 0;
    for (;;) {
        uint8_t c;
        if (!usb_serial_jtag_ll_read_rxfifo(&c, 1)) { vTaskDelay(1); continue; }
        if (c == '\r') continue;
        if (c != '\n') {
            if (used < sizeof(line) - 1) line[used++] = (char)c;
            continue;
        }
        line[used] = 0;
        used = 0;
        if (line[0]) handle(line);
    }
}
