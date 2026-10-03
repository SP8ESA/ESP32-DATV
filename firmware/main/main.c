/* ESP32-DATV: DVB-S / QPSK transmitter firmware for the ESP32-C3 (13 cm amateur band).
 *
 * The chip's Wi-Fi transmitter is used as an I/Q modulator. A 10-bit I/Q DAC word is written to the RF block's DAC replay
 * engine, which is run as a held one-word register, at 4 MS/s (one write every 40 CPU cycles). A raised-cosine (RRC,
 * roll-off 0.35, 12 symbols) filter is evaluated through lookup tables, so one output sample costs three table loads, two adds,
 * one xor and one store. The PC sends raw QPSK symbols over the native USB Serial/JTAG port (4 symbols per byte).
 *
 * Commands (text, one per line, 115200 is irrelevant: USB CDC):
 *   INFO
 *   QPSKT f_MHz baud sps [amp [seconds [ifm [target [dcI dcQ [g phi]]]]]]   see qpsk_lut.h and README.md
 *
 * Transmit only. Licensed amateur use only: the firmware refuses anything outside 2300..2450 MHz.
 */
#include <inttypes.h>
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

static void tx_qpsk_lut(uint32_t fkhz, uint32_t baud_req, uint32_t sps, int32_t amp, uint32_t secs, int32_t ifm, int32_t target, int32_t dc4i, int32_t dc4q,
                        int32_t gq, int32_t phim) {
    const uint32_t period = (CPU_HZ + baud_req * sps / 2) / (baud_req * sps);
    const double baud = (double)CPU_HZ / ((double)period * sps), fso = (double)CPU_HZ / period;
    const double half_bw = baud * 1.35 / 2.0, lo_nom = fkhz * 1000.0, ifh = ifm * baud;
    const double sig_lo = lo_nom + (ifh < 0 ? ifh : 0) - half_bw, sig_hi = lo_nom + (ifh > 0 ? ifh : 0) + half_bw;
    if (period < 16) { say("ERR QPSKT period %lu cycles < 16 (baud * sps too high)\r\n", (unsigned long)period); return; }
    if (sig_lo < TX_FMIN_KHZ * 1000.0 || sig_hi > TX_FMAX_KHZ * 1000.0) { say("ERR QPSKT only in the 13 cm band (2300..2450 MHz)\r\n"); return; }
    if (fabs(ifh) + half_bw >= fso / 2.0) { say("ERR QPSKT IF too high for the output rate (%.0f Hz)\r\n", fso); return; }
    static uint8_t *ring;
    if (!ring) ring = malloc(LUT_RING_BYTES);
    int32_t *T = malloc(4 * lut_words(sps));
    if (!ring || !T) { free(T); say("ERR out of memory\r\n"); return; }
    lut_build(T, sps, ifm, 0.35f, (float)amp, dc4i / 16.0f, dc4q / 16.0f, gq / 10000.0f, phim / 1000.0f);
    if (!tune(fkhz)) { free(T); say("ERR TUNE\r\n"); return; }
    const uint32_t owner0 = rd(SRAM_OWNER);
    phy_txforce(1);
    esp_rom_delay_us(3000);
    DAC_BUF[0] = 0;
    usb_drain_rx();
    say("OK QPSKT LO %.0f BAUD %.3f OUT %.0f AMP %ld SPS %lu PERIOD %lu IF %.0f TARGET %ld\r\n", lo_hz, baud, fso, (long)amp, (unsigned long)sps, (unsigned long)period, ifh, (long)target);
    lut_io_t io = {0};
    const uint32_t max_sym = (uint32_t)((double)secs * baud);
    taskENTER_CRITICAL(&stream_mux);
    wr(SRAM_OWNER, (owner0 & ~7u) | 2u | 8u);
    __asm__ volatile("fence rw,rw" ::: "memory");
    wr(DAC_CTRL, rd(DAC_CTRL) & ~(1u << 31));
    wr(DAC_CTRL, 0x80000u | 1u);
    wr(DAC_CTRL, 0x80000u | 1u | (1u << 31));
    const uint32_t n = sps == 4 ? tx_sym_lut_4(&io, ring, (const uint8_t *)T, period, max_sym, target == 0)
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
    say("\r\nTX END symbols=%" PRIu32 " bytes_in=%" PRIu32 " underruns=%" PRIu32 " late_slots=%" PRIu32 " (lag bits 0x%" PRIx32 ") buffer=%" PRIu32 " pairs (%s)\r\n",
        n * sps, io.qw, io.under, io.lateness, io.lag_or, (io.qw - io.qr) / 2, io.stopped ? "stop byte" : "host silent or time over");
}


/* ------------------------------------------------------------ commands */

static void handle(char *line) {
    if (!strcmp(line, "INFO")) {
        say("ESP32DATV 1\r\n");
    } else if (!strncmp(line, "QPSKT ", 6)) {
        char fs_[32];
        uint32_t fk = 0;
        long baud = 0, sps = 4, amp = 300, secs = 1800, ifm = 0, tgt = 0, dc4i = 0, dc4q = 0, gq = 10000, phim = 0;
        const bool parsed = sscanf(line, "QPSKT %31s %ld %ld %ld %ld %ld %ld %ld %ld %ld %ld", fs_, &baud, &sps, &amp, &secs, &ifm, &tgt, &dc4i, &dc4q, &gq, &phim) >= 3 &&
                            parse_freq(fs_, &fk);
        if (!parsed || baud < 2000 || baud > 1500000 || (sps != 4 && sps != 8 && sps != 16) || amp < 1 || amp > 480 || secs < 1 || secs > 86400 || labs(ifm) > 6 ||
            tgt < 0 || tgt > 6000 || (tgt > 0 && tgt < 64) || gq < 7000 || gq > 13000 || labs(phim) > 40000) {
            say("ERR QPSKT f_MHz baud sps(4|8|16) [amp 1..480 [seconds [if in multiples of baud, +-6 [target 0 = PRBS in the ESP, >0 = stream from USB [dcI dcQ in 1/16 code [g in 1e-4 [phase in 1e-3 deg]]]]]]]]\r\n");
            return;
        }
        tx_qpsk_lut(fk, (uint32_t)baud, (uint32_t)sps, (int32_t)amp, (uint32_t)secs, (int32_t)ifm, (int32_t)tgt, (int32_t)dc4i, (int32_t)dc4q, (int32_t)gq, (int32_t)phim);
    } else {
        say("ERR ? (commands: INFO, QPSKT)\r\n");
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
