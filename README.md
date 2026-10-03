<p align="center"><img src="docs/meme.jpg" alt="Is this a 2.4GHz DATV transmitter? (an ESP32-C3 SuperMini)" width="620"></p>

# ESP32-DATV

**A DVB-S digital amateur TV transmitter in a bare ESP32-C3: 1 Msymbol/s QPSK in the 13 cm band, no RF hardware added.**

![Spectrum of the transmitted DVB-S signal](docs/spectrum.png)

*The 1 MBd DVB-S signal around 2402 MHz, seen in SDR++ on a HackRF One (the author's screenshot, taken with the earlier
4 MS/s output). The faint humps a few MHz to the right are the images of that 4 MS/s zero-order-hold output. At 1 MBd the
firmware now updates the DAC at 8 MS/s: those images are gone (19 dB lower, see [Measured](#measured)).*

The ESP32-C3's Wi-Fi transmitter has an I/Q modulator and a 10-bit I/Q DAC feeding it. This project drives that DAC directly
from the CPU at 8 mega-samples per second (1 MBd; other symbol rates at up to 4 MS/s), so the chip itself produces a
root-raised-cosine QPSK signal on 2.4 GHz. A PC
encodes an MPEG transport stream to DVB-S (energy dispersal, Reed-Solomon, interleaver, convolutional code) and streams the
symbols over USB; the ESP does the pulse shaping and the output. The demo film in `media/` plays on a normal DVB-S receiver.

**For licensed radio amateurs only.** See [Legal](#legal-and-safety) before you transmit anything.

## What is in the box

| Path | What |
|---|---|
| `firmware/` | ESP-IDF project, transmit only (`main/main.c`, `main/qpsk_lut.h`, `main/lut8.S`) |
| `firmware/tools/gen_lut8.py` | Generates `main/lut8.S`, the hand-scheduled 8 MS/s loop, from `lut8_pads.json` (per-slot padding) |
| `host/dvbs.py` | DVB-S encoder, TS to QPSK symbols (EN 300 421, all code rates 1/2 ... 7/8), with a self-test |
| `host/tx_dvbs.py` | The transmitter script: transport stream source (demo film, any video, test pattern, stdin), encoder, USB streaming |
| `host/tx_qpsk_test.py` | Random-symbol QPSK test for looking at the spectrum |
| `host/esp_link.py` | USB link to the firmware |
| `host/test_lut.c` | Checks the firmware's lookup tables against a direct floating-point RRC filter, on the PC |
| `host/cal.json` | Example DC / I-Q trim (measured on the author's board) |
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
0.35. If it does not lock, try `--invert` or `--swap-iq`. Stop with Ctrl-C; the ESP also switches itself off 0.5 s after the PC
goes quiet.

Other sources:

```sh
python3 tx_dvbs.py --film my_video.mp4                  # any video; ffmpeg encodes it to fit the channel (H.264 + MP2)
python3 tx_dvbs.py --test                               # ffmpeg test pattern
python3 tx_dvbs.py --null                               # empty multiplex: the receiver should lock but show nothing
ffmpeg -re -i in.mp4 ... -f mpegts -muxrate 880000 - | python3 tx_dvbs.py --ts -       # your own transport stream
python3 tx_qpsk_test.py --seconds 60                    # plain random QPSK, for spectrum checks
```

Useful options: `--baud` (2000 ... 1000000, e.g. 33000 for narrow-band DATV: picture size, frame rate and audio shrink with
the channel), `--fec 1/2|2/3|3/4|5/6|7/8`, `--amp` (peak DAC code, default 300, max 480; lower
it if the output is compressed), `--ifm N` (centre = LO + N x baud, moves the LO leakage out of the signal), `--ppm`.

**Set `--ppm` for your board.** The PLL assumes an exact 40 MHz crystal, and real crystals are off by some ppm (about +12 ppm
on the author's board, which is 29 kHz at 2.4 GHz). The crystal also drifts several ppm while the chip warms up (about
250 Hz/s over the first minutes, 10 ppm in total on the author's board), so use a receiver with AFC; normal DATV receivers have it.

## How it works

* **The DAC.** The RF block has a replay engine that plays RAM at 0x3FCB0000 into the I/Q DAC at 40 MS/s (the
  `dactrig` of Espressif's RF test library). Running it as a loop of **one** word makes it a held 10-bit I/Q register that the
  CPU updates with a plain store (I in bits 9:0, Q in bits 19:10). Longer rings cannot be written live (the CPU access to the
  bank is garbled while it plays); a one-word loop works because the DMA pointer never moves.
* **Pulse shaping with tables.** With an integer number of samples per symbol the RRC filter output depends only
  on the last 12 symbols and the phase inside the symbol. Three tables of 256 rows (4 symbols of I and Q each, 8 bits of
  history) hold ready DAC words, so one sample is three loads, two adds, an xor and a store: about 14 of the 40 cycles
  available at 1 MBd. An IF that is a whole multiple of the symbol rate is baked into the tables for free, and so are the
  DC and I/Q-imbalance trims. `host/test_lut.c` checks the tables (error under 1.5 DAC codes, SNR about 50 dB).
* **8 MS/s at 1 MBd** (`main/lut8.S`). 8 samples per symbol leave 20 CPU cycles per sample, too few for compiled code: the
  loop is hand-scheduled RISC-V assembly in which every slot (store, next word, a share of the symbol's work, padding) takes
  exactly 20 cycles, measured with a timing build that records every store. One cycle-counter sync per symbol (a jump into a
  `nop` sled) absorbs what cannot be made exact: access to the USB peripheral crosses into its 48 MHz clock domain and varies
  by a cycle. RRC span 8 symbols there (2 tables), the fill report runs in a balanced maintenance pass every 2 ms. Measured
  instruction costs on this core: cycle counter read 1, USB register read 6, DAC store 1, RAM load ~1.25, taken branch 3.
* **Scheduling.** Two symbols per loop pass, with the per-symbol work (decode, table row pointers, USB read, ring refill)
  spread over the slots so that no sample slot overruns its period. USB (a 64-byte FIFO, each register read costs ~12
  cycles) is touched at most once per slot.
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
  intact. leandvb reported MER 16-18 dB.
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
* What is left near the signal is mostly the noise of the carrier itself: an unmodulated carrier shows the same skirt
  (-75 dBc/Hz at 100 kHz, -91 dBc/Hz at 1 MHz offset, receiver included).

## Limitations

* Tested on the air: **1 MBd at 8 MS/s and at 4 MS/s, 500 kBd at 4 MS/s, 33 kBd**, FEC 1/2 (other FEC rates bit-exact in
  simulation); the host scripts on Linux. The 8 MS/s loop exists for exactly 1 MBd and needs the USB stream (no on-chip PRBS
  there); other symbol rates use the C loop at up to 4 MS/s.
* **The output is not a clean transmitter.** There is no filter and no amplifier: zero-order-hold images remain at +-8 MHz
  (at 1 MBd; at +-4 MHz for the other rates), and the carrier's own noise forms a skirt about 35-40 dB below the signal level
  per bin. Add a band-pass filter and/or attenuation as your licence and
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
