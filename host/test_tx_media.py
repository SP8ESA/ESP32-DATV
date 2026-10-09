"""Real FFmpeg input/looping and TS-reader shutdown, without opening the RF device."""
from pathlib import Path
import tempfile
import time
import unittest

import dvbs
from tx_dvbs import TsSource


def wait_for(check, seconds=7):
    start = time.monotonic()
    while time.monotonic() - start < seconds:
        if check():return True
        time.sleep(.05)
    return False


class Sources(unittest.TestCase):
    def test_ffmpeg_test_source_produces_transport_stream_and_stops(self):
        source = TsSource('test', '', 905000)
        try:
            self.assertTrue(wait_for(lambda: source.q.qsize() >= 24))
            source.check()
            data = source.take(24)
            self.assertEqual(len(data), 24 * 188)
            self.assertTrue(all(data[i] == 0x47 for i in range(0, len(data), 188)))
            self.assertEqual(source.pkts, 24)
            self.assertTrue(wait_for(lambda: source.q.full()))
        finally:
            source.close()
        self.assertIsNotNone(source.proc.poll())
        self.assertFalse(source.reader.is_alive())

    def test_ts_file_loops_and_resynchronizes_without_ffmpeg(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / 'stream with spaces.ts'
            path.write_bytes(b'junk' + dvbs.NULL_PACKET * 12)
            source = TsSource('ts', str(path), 905000)
            try:
                self.assertTrue(wait_for(source.q.full))
                self.assertEqual(source.take(30), dvbs.NULL_PACKET * 30)
                self.assertIsNone(source.proc)
            finally:
                source.close()
            self.assertFalse(source.reader.is_alive())

    def test_failed_ffmpeg_input_is_reported(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / 'broken.mp4'
            path.write_bytes(b'not a video')
            source = TsSource('film', str(path), 905000)
            try:
                self.assertTrue(wait_for(lambda: source.proc.poll() is not None))
                with self.assertRaises(RuntimeError):source.check()
            finally:
                source.close()


if __name__ == '__main__':
    unittest.main()
