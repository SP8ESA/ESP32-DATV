<p align="center"><img src="docs/meme.jpg" alt="Is this a 2.4GHz DATV transmitter? (an ESP32-C3 SuperMini)" width="620"></p>

# ESP32-DATV

**A DVB-S digital amateur TV transmitter in a bare ESP32-C3: QPSK from 1 Msymbol/s down to 33 ksymbol/s in the 13 cm band, no RF hardware added.**

![Spectrum of the transmitted DVB-S signal](docs/spectrum.png)

*The 1 MBd DVB-S signal around 2402 MHz, seen in SDR++ on a HackRF One (the author's screenshot, taken with the earlier
4 MS/s output). The faint humps a few MHz to the right are the images of that 4 MS/s zero-order-hold output. The firmware now
updates the DAC at 8 MS/s (6.67 MS/s for the lowest rates): the images moved from +-4 MHz out to +-8 MHz and the strongest
one is about 6 dB lower, and much lower still at the narrower symbol rates (see [Spectra](#spectra)).*

The ESP32-C3's Wi-Fi transmitter has an I/Q modulator and a 10-bit I/Q DAC feeding it. This project drives that DAC directly
from the CPU at 8 mega-samples per second (6.67 MS/s for the two lowest symbol rates), so the chip itself produces a
root-raised-cosine QPSK signal on 2.4 GHz. A PC
encodes an MPEG transport stream to DVB-S (energy dispersal, Reed-Solomon, interleaver, convolutional code) and streams the
symbols over USB; the ESP does the pulse shaping and the output. The demo film in `media/` plays on a normal DVB-S receiver.

**For licensed radio amateurs only.** See [Legal](#legal-and-safety) before you transmit anything.

## What is in the box

| Path | What |
|---|---|
| `firmware/` | ESP-IDF project, transmit only (`main/main.c`, `main/qpsk_lut.h`, `main/lut8.S`, `main/lutg.S`) |
| `firmware/tools/gen_lut8.py` | Generates `main/lut8.S`, the hand-scheduled 8 MS/s loop for 1 MBd, from `lut8_pads.json` (per-slot padding) |
| `firmware/tools/gen_lutg.py` | Generates `main/lutg.S`, the hand-scheduled loop for all the other rates (any samples per symbol, 8 or 6.67 MS/s), from `lutg_pads.json` |
| `host/dvbs.py` | DVB-S encoder, TS to QPSK symbols (EN 300 421, all code rates 1/2 ... 7/8), with a self-test |
| `host/tx_dvbs.py` | The transmitter script: transport stream source (demo film, any video, test pattern, stdin) or an unmodulated carrier, encoder, USB streaming |
| `host/tx_qpsk_test.py` | Random-symbol QPSK test for looking at the spectrum |
| `host/esp_link.py` | USB link to the firmware |
| `host/test_lut.c` | Checks the firmware's lookup tables against a direct floating-point RRC filter, on the PC |
| `host/test_lutg_words.c`, `host/lutg_record.py` | Check the DAC words the generic assembly loop really produces against the reference model (recording build) |
| `host/cal.json` | Example DC / I-Q trim (measured on the author's board) |
| `docs/` | Pictures for this README, including the measured spectra of all symbol rates |
| `media/` | The demo film (Sintel trailer, CC BY 3.0) |

## Quick start

You need an ESP32-C3 board with the native USB port (USB Serial/JTAG, shows up as `/dev/ttyACM0`; any DevKit or SuperMini
will do), ESP-IDF, Python 3 with `numpy` and `pyserial`, and `ffmpeg`.

```sh
# 1. firmware (tested with ESP-IDF 6.2.0)
cd firmware
idf.py set-target esp32c3
idf.py build flash

# 2. transmit the demo film in a loop on 2402.000 MHz, 1 MBd, FEC 1/2
cd ../host
pip install -r requirements.txt
python3 tx_dvbs.py --freq 2402.000
```

Tune a DVB-S receiver to the centre frequency printed by the script, **symbol rate 1000 kS/s, FEC 1/2** (or auto), roll-off
0.35 (for another `--baud`, the symbol rate the script prints). If it does not lock, try `--invert` or `--swap-iq`. Stop with
Ctrl-C; the ESP also switches itself off 0.5 s after the PC goes quiet.

Symbol rates. The DAC is updated every 20 CPU cycles (8 MS/s) or 24 cycles (6.67 MS/s); the script picks the number of samples per
symbol (`--sps` overrides it):

| `--baud` | samples per symbol | cycles per DAC store | DAC rate | actual symbol rate |
|---|---|---|---|---|
| 1 000 000 | 8 | 20 | 8 MS/s | 1 000 000 (`lut8.S`) |
| 500 000 | 16 | 20 | 8 MS/s | 500 000 |
| 333 000 | 24 | 20 | 8 MS/s | 333 333 (+0.10 %) |
| 250 000 | 32 | 20 | 8 MS/s | 250 000 |
| 125 000 | 64 | 20 | 8 MS/s | 125 000 |
| 66 000 | 101 | 24 | 6.67 MS/s | 66 007 (+0.01 %) |
| 33 000 | 202 | 24 | 6.67 MS/s | 33 003 (+0.01 %) |

Any other `--baud` that gives 16..232 samples per symbol within 1 % of the requested rate at one of the two periods (100 000 Bd = 80
samples at 20 cycles, say) uses the same loop; the rest runs on the C loops at up to 4 MS/s. The script prints the actual symbol rate.

Other sources:

```sh
python3 tx_dvbs.py --film my_video.mp4                  # any video; ffmpeg encodes it to fit the channel (H.264 + MP2)
python3 tx_dvbs.py --test                               # ffmpeg test pattern
python3 tx_dvbs.py --null                               # empty multiplex: the receiver should lock but show nothing
python3 tx_dvbs.py --cw                                 # no DVB-S: an unmodulated carrier on the centre frequency (tuning, level checks)
ffmpeg -re -i in.mp4 ... -f mpegts -muxrate 880000 - | python3 tx_dvbs.py --ts -       # your own transport stream
python3 tx_qpsk_test.py --seconds 60                    # plain random QPSK, for spectrum checks
```

Useful options: `--baud` (2000 ... 1000000, e.g. 33000 for narrow-band DATV: picture size, frame rate and audio shrink with
the channel), `--fec 1/2|2/3|3/4|5/6|7/8`, `--amp` (peak DAC code, default 300, max 480; lower
it if the output is compressed), `--ifm N` (centre = LO + N x baud, moves the LO leakage out of the signal), `--sps` (samples per
symbol, normally chosen for you), `--cw` (carrier only), `--ppm`.

**Set `--ppm` for your board.** The PLL assumes an exact 40 MHz crystal, and real crystals are off by some ppm (about +12 ppm
on the author's board, which is 29 kHz at 2.4 GHz). The crystal also drifts several ppm while the chip warms up (about
250 Hz/s over the first minutes, 10 ppm in total on the author's board), so use a receiver with AFC; normal DATV receivers have it.

## How it works

* **The DAC.** The RF block has a replay engine that plays RAM at 0x3FCB0000 into the I/Q DAC at 40 MS/s (the
  `dactrig` of Espressif's RF test library). Running it as a loop of **one** word makes it a held 10-bit I/Q register that the
  CPU updates with a plain store (I in bits 9:0, Q in bits 19:10). Longer rings cannot be written live (the CPU access to the
  bank is garbled while it plays); a one-word loop works because the DMA pointer never moves.
* **Pulse shaping with tables.** With an integer number of samples per symbol the RRC filter output depends only
  on the last few symbols and the phase inside the symbol, so tables hold ready DAC words (I field + Q field): one sample is a few
  loads, adds, an xor and a store. The C loops (rates without a fast loop) use three tables of 256 rows (4 symbols of I and Q each,
  RRC span 12) and need about 14 of the 40 cycles available at 1 MBd and 4 MS/s; the two assembly loops below trade the span for a
  higher DAC rate (span 8). An IF that is a whole multiple of the symbol rate is baked into the tables for free, and so are the
  DC and I/Q-imbalance trims. `host/test_lut.c` checks the tables (error under 1.5 DAC codes, SNR about 50 dB).
* **8 MS/s at 1 MBd** (`main/lut8.S`). 8 samples per symbol leave 20 CPU cycles per sample, too few for compiled code: the
  loop is hand-scheduled RISC-V assembly in which every slot (store, next word, a share of the symbol's work, padding) takes
  exactly 20 cycles, measured with a timing build that records every store. One cycle-counter sync per symbol (a jump into a
  `nop` sled) absorbs what cannot be made exact: access to the USB peripheral crosses into its 48 MHz clock domain and varies
  by a cycle. RRC span 8 symbols there (2 tables), the fill report runs in a balanced maintenance pass every 2 ms. Measured
  instruction costs on this core: cycle counter read 1, USB register read 6, USB register write 8, DAC store 1, RAM load 1 (+1 when the next
  instruction uses it), taken branch 3.
* **Every other symbol rate at 8 / 6.67 MS/s** (`main/lutg.S`, generated by `tools/gen_lutg.py`). The same method for any samples per
  symbol S from 16 to 232, chosen at run time. The RRC span is 8 symbols, from four tables of two symbols (16 rows each) laid out so that
  the next sample of a row is 64 bytes further: a sample is four loads, three adds, an xor and a store, and the loop just adds 64 to the
  four row pointers. A symbol is S slots of exactly 20 (or 24) cycles: 13 unrolled slots carry the per-symbol work (the next symbol out
  of the ring, the row pointers, one USB byte), a one-slot loop of 18 cycles produces the other samples, and the last two slots read
  the cycle counter, compare it with the absolute symbol schedule and jump into a 24-`nop` sled in front of the next symbol's code, which
  absorbs whatever deviates, so the symbol rate is exact and no slot but that one has a variable length. There are four copies of the
  symbol code: normal, and three maintenance symbols after each other every 2048 symbols (silence check, fill report written to the
  USB FIFO). The timing was tuned with a recording build that stamps the cycle counter after every DAC store (the stores are spaced
  exactly 20 or 24 cycles apart, apart from a cycle where a slot touches the USB registers), and the DAC words it produces were compared
  with the reference model on the PC (`host/test_lutg_words.c`). Tables take 256 x S bytes (52 KB at 33 kBd).
* **Scheduling (C loops).** Two symbols per loop pass, with the per-symbol work (decode, table row pointers, USB read, ring refill)
  spread over the slots so that no sample slot overruns its period. USB (a 64-byte FIFO, each register read costs ~12
  cycles from compiled code) is touched at most once per slot.
* **USB protocol.** Text command `QPSKT f_MHz baud sps [amp [seconds [ifm [target [dcI dcQ [g phi]]]]]]`, then a **raw
  symbol stream**, 4 symbols per byte (bit 0 = I level, bit 1 = Q level, 1 = +1, first symbol in the low bits), no framing.
  `target` = 0 makes the ESP generate random symbols itself. The ESP returns 4-byte fill reports (`B7`, fill lo/hi, underruns)
  every 256 pairs; the PC keeps the ring about 3000 pairs (about 24 ms) full. A silent host for 0.5 s switches the
  transmitter off.
* **DVB-S on the PC** (`host/dvbs.py`): scrambler (1+x^14+x^15, restarted every 8 packets), RS(204,188), Forney interleaver
  (I=12), K=7 convolutional code (171, 133 octal) with puncturing, QPSK Gray mapping. Null packets pad the stream whenever
  the source is slower than the channel.

## Measured

Everything below is on one board (ESP32-C3 rev 0.4, 40 MHz crystal), with a HackRF One a short distance away.

* The signal decoded with **leandvb**, an independent DVB-S decoder, bit for bit, in simulation for every code rate and over
  the air at 1 MBd, FEC 1/2: 6003 transport stream packets in 10 s with no residual errors, H.264 640x360 video and audio
  intact. leandvb reported MER 16-18 dB. With the 8 / 6.67 MS/s loops at 1 MBd, 500, 333, 250, 66 and 33 kBd it locked with VBER 0
  and MER 20-26 dB; at 125 kBd a 10 s capture decoded offline to 705 transport stream packets and 79 video frames.
* 1 MBd, 4 MS/s (C loop) against 8 MS/s (assembly loop), random symbols, same setup; level per bin relative to the level in
  the signal band, both sides averaged, measurement floor about -47 dB:

  | offset from centre | 4 MS/s | 8 MS/s |
  |---|---|---|
  | 1.0-1.5 MHz | -33.3 dB | -36.4 dB |
  | 2.0-2.5 MHz | -35.2 dB | -39.4 dB |
  | 2.5-3.0 MHz | -35.5 dB | -41.2 dB |
  | 3.5-4.0 MHz (images of 4 MS/s) | -24.4 dB | -43.4 dB |
  | 7.5-8.0 MHz (images of 8 MS/s) | -32.5 dB | -31.3 dB |

  DVB-S decode in the same conditions: 4 MS/s 6082 packets in 10 s, MER 17.1 dB; 8 MS/s 6062 packets, MER 16.7 dB.
* All symbol rates, tinySA Ultra+ with the transmitter connected through an attenuator, 30 passes averaged in power, 2370 MHz.
  The strongest image of the zero-order-hold DAC output sits at +-f(DAC) (8 MHz, 6.67 MHz for 66 and 33 kBd). Level in one 30 kHz
  bin relative to the same bin at the top of the signal, both sides:

  | symbol rate | DAC rate | image at +-f(DAC) |
  |---|---|---|
  | 1 MBd | 8 MS/s | -27 dB (the 4 MS/s output it replaced: -21 dB at +-4 MHz) |
  | 500 kBd | 8 MS/s | -34 dB |
  | 333 kBd | 8 MS/s | -37 dB |
  | 250 kBd | 8 MS/s | -41 dB |
  | 125 kBd | 8 MS/s | -49 dB |
  | 66 kBd | 6.67 MS/s | -52 dB |
  | 33 kBd | 6.67 MS/s | -59 dB (near the measurement floor) |

  The image carries the same power at every rate, but it is as wide as the signal, so it sinks below a fixed RBW bin as the signal
  gets narrower. The spectra themselves are in [Spectra](#spectra).
* The timing of the generic loop: the DAC stores are exactly 20 or 24 CPU cycles apart in all slots of a symbol (a recording build
  stamps the cycle counter after every store), apart from the 1-3 slots per symbol that read or write the USB registers, which
  vary by a cycle with the state of the USB FIFO; the sync sled makes up for it, so the symbol rate stays exact. `late_slots` was 0
  in every run of these loops, minutes long, at every rate.
* What is left near the signal is mostly the noise of the carrier itself: an unmodulated carrier shows the same skirt
  (-75 dBc/Hz at 100 kHz, -91 dBc/Hz at 1 MHz offset, receiver included).

## Spectra

Conducted measurements with a tinySA Ultra+ (transmitter - attenuator - analyzer), DVB-S, FEC 1/2, centre 2370 MHz, 30 passes
averaged in power and lightly smoothed (5 bins). Levels are in dB relative to the top of the signal, the scan step is never
larger than the RBW. Left: 30 MHz span at RBW 30 kHz, with the images of the DAC output marked "alias". Right: a span of about six
symbol rates. "MS/s" and "kS/s" in the titles are the DVB-S symbol rate, not the DAC rate (8 MS/s, except 6.67 MS/s at 66 and 33 kBd).
The shoulder about 33-40 dB down next to the signal edge is the RRC filter truncated to 8 symbols; further out the skirt is the noise
of the carrier itself.

### 1 MBd (8 samples per symbol, `lut8.S`): alias -27 dB

![QPSK 1 MS/s](docs/spectrum_1MBd.png)

### 500 kBd (16 samples per symbol): alias -34 dB

![QPSK 500 kS/s](docs/spectrum_500kBd.png)

### 333 kBd (24 samples per symbol, 333 333 Bd): alias -37 dB

![QPSK 333 kS/s](docs/spectrum_333kBd.png)

### 250 kBd (32 samples per symbol): alias -41 dB

![QPSK 250 kS/s](docs/spectrum_250kBd.png)

### 125 kBd (64 samples per symbol): alias -49 dB

![QPSK 125 kS/s](docs/spectrum_125kBd.png)

### 66 kBd (101 samples per symbol at 24 cycles, 66 007 Bd): alias -52 dB at +-6.67 MHz

![QPSK 66 kS/s](docs/spectrum_66kBd.png)

### 33 kBd (202 samples per symbol at 24 cycles, 33 003 Bd): alias -59 dB at +-6.67 MHz

![QPSK 33 kS/s](docs/spectrum_33kBd.png)

## Limitations

* Tested on the air with a HackRF One and the independent leandvb decoder (lock, VBER 0, MER about 20-25 dB): **1 MBd, 500, 333,
  250, 125, 66 and 33 kBd** at 8 / 6.67 MS/s, FEC 1/2 (other FEC rates bit-exact in simulation); the host scripts on Linux. The 1 MBd
  loop needs the USB stream (no on-chip PRBS there) and so do the generic 8 / 6.67 MS/s loops; other symbol rates use the C loops at
  up to 4 MS/s (not re-tested after the new loops were added, they are unchanged).
* **The output is not a clean transmitter.** There is no filter and no amplifier: zero-order-hold images remain at +-8 MHz
  (+-6.67 MHz at 66 and 33 kBd), 27 dB (1 MBd) to 59 dB (33 kBd) below the signal in a 30 kHz bin, and the carrier's own noise forms
  a skirt about 35-40 dB below the signal level per bin. Add a band-pass filter and/or attenuation as your licence and
  local rules require.
* Transmit only, and only in the 13 cm band: the firmware refuses to transmit outside 2300..2450 MHz.
* The firmware calls undocumented PHY ROM functions and writes undocumented registers of the ESP32-C3 (found by reading the
  ROM and `libphy`). It was built and tested with ESP-IDF 6.2.0; other IDF or chip revisions may behave differently.
  `.bss` has to stay below 0x3FCA0000, because the RF block takes the 128 KiB 0x3FCA0000..0x3FCBFFFF while transmitting.

## Legal and safety

You need an amateur radio licence that permits digital ATV on the frequency you pick, and you are responsible for obeying it:
frequency, bandwidth, power, identification, and spurious emissions. This transmitter has no output filter. The author
runs it at very low power. The software comes as is, without any warranty (see the licence).

## Credits

* **Espressif**: the ESP32-C3 and ESP-IDF (Apache-2.0). The register-level work came from reading the chip's ROM and the
  precompiled PHY library `libphy.a` that ships with ESP-IDF.
* **leansdr / leandvb** by pabr, https://github.com/pabr/leansdr: the independent DVB-S decoder used to verify the signal.
  Not included in this repository.
* **ETSI EN 300 421**: the DVB-S standard.
* **FFmpeg** and **x264**: encoding of the demo and test streams. **NumPy**, **pySerial** and (for the optional RS cross-check)
  **reedsolo** on the PC side.
* **Sintel** trailer (c) Blender Foundation, https://durian.blender.org, CC BY 3.0 (see `media/README.md`).

## License

[PolyForm Noncommercial License 1.0.0](LICENSE): you may use, modify and share this software for any noncommercial purpose,
including hobby, amateur radio, research and education. Commercial use is not permitted. The demo film has its own licence (CC BY 3.0).

Copyright (c) 2026 SP8ESA
