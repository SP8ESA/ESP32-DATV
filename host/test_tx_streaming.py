"""Host streaming regressions; run with python3 -B host/test_tx_streaming.py."""
import queue
import threading
import unittest

from tx_dvbs import CPU_HZ, P8_MIN_PERIOD, auto_sps_8psk, feed_symbols, output_baud


class ObservedQueue(queue.Queue):
    def __init__(self):
        super().__init__(maxsize=1)
        self.full_timeout = threading.Event()

    def put(self, item, block=True, timeout=None):
        try:
            return super().put(item, block, timeout)
        except queue.Full:
            self.full_timeout.set()
            raise


class Source:
    def __init__(self):
        self.calls = 0

    def take(self, count):
        self.calls += 1
        return self.calls


class Encoder:
    def __init__(self):
        self.blocks = []

    def encode(self, block):
        self.blocks.append(block)
        return bytes([block % 256])


class StreamingTests(unittest.TestCase):
    def start_feeder(self, ready, src, enc, cw_byte=None):
        stop = threading.Event()
        worker = threading.Thread(target=feed_symbols, args=(ready, stop, src, enc, cw_byte), daemon=True)
        worker.start()

        def cleanup():
            stop.set()
            worker.join(timeout=1)
            self.assertFalse(worker.is_alive(), "encoder did not stop while the queue was full")

        self.addCleanup(cleanup)
        return stop, worker

    def test_full_queue_retains_encoded_block_and_encoder_state(self):
        ready, src, enc = ObservedQueue(), Source(), Encoder()
        ready.put(b"already queued")
        self.start_feeder(ready, src, enc)
        self.assertTrue(ready.full_timeout.wait(timeout=1))
        self.assertEqual(src.calls, 1)
        self.assertEqual(enc.blocks, [1])
        self.assertEqual(ready.get(timeout=1), b"already queued")
        self.assertEqual(ready.get(timeout=1), b"\x01")
        self.assertEqual(ready.get(timeout=1), b"\x02")

    def test_stop_while_queue_is_full(self):
        ready = ObservedQueue()
        ready.put(b"already queued")
        stop, worker = self.start_feeder(ready, Source(), Encoder())
        self.assertTrue(ready.full_timeout.wait(timeout=1))
        stop.set()
        worker.join(timeout=1)
        self.assertFalse(worker.is_alive())
        self.assertEqual(ready.get(timeout=1), b"already queued")

    def test_cw_keeps_zero_symbol_without_consuming_ts(self):
        ready, src, enc = ObservedQueue(), Source(), Encoder()
        self.start_feeder(ready, src, enc, cw_byte=0)
        self.assertEqual(ready.get(timeout=1), b"\x00" * 4096)
        self.assertEqual(src.calls, 0)
        self.assertEqual(enc.blocks, [])

    def test_low_8psk_rates_select_supported_firmware_periods(self):
        for baud, expected_sps in [(10000, 64), (33000, 64), (66000, 32), (125000, 64), (500000, 16), (1000000, 8)]:
            with self.subTest(baud=baud):
                sps = auto_sps_8psk(baud)
                self.assertEqual(sps, expected_sps)
                period = (CPU_HZ + baud * sps // 2) // (baud * sps)
                self.assertEqual(output_baud(baud, sps, "8psk"), baud)
                self.assertTrue(period == 20 or CPU_HZ // (baud * sps) >= P8_MIN_PERIOD)

    def test_fixed_assembly_rates_keep_their_existing_rounding(self):
        self.assertEqual(output_baud(333000, 24, "8psk"), 1_000_000 / 3)
        self.assertAlmostEqual(output_baud(33000, 202, "qpsk"), 33003.300330)

    def test_fractional_rate_matches_explicit_samples_per_symbol(self):
        self.assertEqual(output_baud(33000, 48, "8psk"), 33000)
        self.assertEqual(output_baud(66000, 24, "8psk"), 66000)


if __name__ == "__main__":
    unittest.main()
