"""Configuration validation, GUI choices, and asynchronous transmitter lifecycle."""
import json
import os
os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
from pathlib import Path
import sys
import tempfile
import time
import unittest
from unittest.mock import patch

from PyQt5.QtWidgets import QApplication, QTabWidget, QMessageBox
from PyQt5.QtTest import QTest
from PyQt5.QtCore import Qt
from qo100_bandplan import CHANNELS, BEACON_UPLINK, channel_warning
from tx_gui import MainWindow, Transmitter
from tx_gui_model import Settings, fec_choices, process_matches, amp_from_db, db_from_amp, level_limits, minimum_level_db
from tx_dvbs import build_parser, resolve_transmission
from tx_media import build_media_command, encoding_settings

APP = QApplication.instance() or QApplication([])


def wait_for(check, timeout=5):
    start = time.monotonic()
    while time.monotonic() - start < timeout:
        APP.processEvents()
        if check():
            return True
        time.sleep(.02)
    return False


class ConfigurationTests(unittest.TestCase):
    def test_pa_enable_round_trip_and_legacy_default(self):
        self.assertFalse(Settings().pa_enable)
        settings = Settings(pa_enable=True)
        self.assertIn('--pa-enable', settings.argv())
        self.assertEqual(Settings.from_profile(settings.profile()), settings)
        self.assertTrue(Settings.from_command([sys.executable, 'host/tx_dvbs.py'] + settings.argv()).pa_enable)
        old = Settings().profile()
        old['settings'].pop('pa_enable')
        self.assertFalse(Settings.from_profile(old).pa_enable)
        old['settings']['pa_enable'] = 'false'
        with self.assertRaises(ValueError):Settings.from_profile(old)

    def test_defaults_have_no_frequency_or_iq_correction(self):
        settings = Settings()
        self.assertEqual(settings.ppm, 0)
        mode, args = settings.resolve(check_files=True)
        calibration = json.loads(Path(args.cal).read_text())
        self.assertEqual(calibration['dc'], [0, 0])
        self.assertEqual(calibration['iq_gain'], 1)
        self.assertEqual(calibration['iq_phase_deg'], 0)
        self.assertEqual(args.ppm, 0)

    def test_existing_live_parameters_round_trip(self):
        command = [sys.executable, '-u', '-B', 'host/tx_dvbs.py', '--freq', '2370', '--baud', '250000', '--dvbs2', '--mod', '32apsk', '--fec', '3/4', '--apsk-pl', 'unit', '--pilots', '--ppm', '12']
        settings = Settings.from_command(command)
        mode, a = settings.resolve(check_files=True)
        self.assertEqual((a.sps, a.amp, a.target), (32, 400, 3500))
        self.assertEqual(mode.sample_hz, 8000000)
        self.assertEqual(Settings.from_profile(settings.profile()), settings)

    def test_short_frames_and_modulation_restrict_fec(self):
        self.assertNotIn('9/10', fec_choices('DVB-S2', '32apsk', 'short'))
        self.assertEqual(fec_choices('DVB-S2', '32apsk', 'normal')[0], '3/4')
        with self.assertRaises(ValueError):
            Settings(fec='1/2').resolve()
        with self.assertRaises(SystemExit):
            Settings(mod='32apsk', baud=500000).resolve()
        with self.assertRaises(ValueError):
            Settings(standard='DVB-S', mod='32apsk').resolve()

    def test_missing_source_rejected_before_usb(self):
        with self.assertRaises(ValueError):
            Settings(film='/no/such/input.mp4').resolve(check_files=True)
        with self.assertRaises(ValueError):
            Settings(source='camera', camera='/no/such/video0').resolve(check_files=True)
        with tempfile.TemporaryDirectory() as tmp:
            p = Path(tmp) / 'invalid.ts'
            p.write_bytes(b'not a transport stream')
            with self.assertRaises(ValueError):
                Settings(source='ts', ts=str(p)).resolve(check_files=True)

    def test_calibration_file_and_camera_arguments_survive_spaces(self):
        s = Settings(source='camera', camera='/dev/video0', camera_size='1280x720', camera_fps=30, camera_format='mjpeg', audio_source='pulse', audio_device='Mic with spaces')
        _, a = s.resolve()
        self.assertEqual(a.audio_device, 'Mic with spaces')
        self.assertFalse(a.no_cal)
        self.assertEqual(a.cal, s.cal)
        self.assertIsNone(a.dc_i)
        cmd = build_media_command('camera', a.camera, 905000, camera_size=a.camera_size, camera_fps=a.camera_fps, camera_format=a.camera_format, audio_source=a.audio_source, audio_device=a.audio_device)
        self.assertIn('Mic with spaces', cmd)
        self.assertIn('mjpeg', cmd)
        self.assertIn('1:a:0', cmd)

    def test_ready_ts_is_not_reencoded(self):
        cmd = build_media_command('ts', 'udp://127.0.0.1:8888', 905000)
        self.assertEqual(cmd[cmd.index('-c') + 1], 'copy')
        self.assertNotIn('libx264', cmd)

    def test_bitrate_cannot_overfill_mux(self):
        with self.assertRaises(ValueError):
            encoding_settings(100000, video_k=500)
        with self.assertRaises(ValueError):
            Settings(level_db=10).resolve()

    def test_removed_controls_cannot_override_backend_defaults(self):
        legacy = Settings(mod='32apsk', baud=250000).profile()
        legacy['version'] = 1
        legacy['settings'].update(sps=16, target=100, ifm=2, invert=True, swap_iq=True,
                                  calibration='manual', dc_i=10, dc_q=20, iq_gain=.8, iq_phase=30)
        settings = Settings.from_profile(legacy)
        _, a = settings.resolve()
        self.assertEqual((a.sps, a.target, a.ifm), (32, 3500, 0))
        self.assertFalse(a.invert or a.swap_iq or a.no_cal)
        self.assertIsNone(a.iq_gain)
        for flag in ('--sps', '--target', '--ifm', '--invert', '--swap-iq', '--dc-i', '--iq-gain'):
            self.assertNotIn(flag, settings.argv())
        self.assertEqual(settings.profile()['version'], 3)

    def test_db_level_conversion_and_legacy_amplitude_profiles(self):
        for mod, base in [('qpsk', 300), ('8psk', 420), ('16apsk', 420), ('32apsk', 400)]:
            self.assertEqual(amp_from_db(0, mod), base)
            self.assertEqual(amp_from_db(-20, mod), base // 10)
            low, high = level_limits(mod)
            self.assertEqual(low, -27)
            self.assertGreaterEqual(db_from_amp(amp_from_db(low, mod), mod), -27 - 1e-9)
            self.assertLessEqual(amp_from_db(high, mod), 480)
            with self.assertRaises(ValueError):amp_from_db(10, mod)
            for amp in (150, 300, 480):
                self.assertEqual(amp_from_db(db_from_amp(amp, mod), mod), amp)
        legacy = Settings(mod='32apsk', baud=250000).profile()
        legacy['version'] = 2
        legacy['settings'].pop('level_db')
        legacy['settings']['amp'] = 200
        settings = Settings.from_profile(legacy)
        _, args = settings.resolve()
        self.assertEqual(args.amp, 200)
        self.assertAlmostEqual(settings.level_db, -6.020599913, places=6)

    def test_configured_minimum_is_enforced_for_profiles_and_arguments(self):
        self.assertEqual(minimum_level_db(), -27)
        with self.assertRaises(ValueError):Settings(level_db=-27.1).argv()
        profile = Settings().profile()
        profile['settings']['level_db'] = -40
        with self.assertRaises(ValueError):Settings.from_profile(profile)
        with patch('tx_gui_model.minimum_level_db', return_value=-12):
            self.assertEqual(level_limits('32apsk')[0], -12)
            with self.assertRaises(ValueError):Settings(level_db=-12.1).resolve()
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / 'config.json'
            path.write_text('{"min_level_db": -19.5}')
            self.assertEqual(minimum_level_db(path), -19.5)
            for value in (1, -100, True, 'wrong'):
                path.write_text(json.dumps({'min_level_db': value}))
                with self.assertRaises(ValueError):minimum_level_db(path)

    def test_pid_identity_never_accepts_unrelated_process(self):
        self.assertFalse(process_matches(dict(pid=os.getpid(), command=['other command'], log='/tmp/x')))
        self.assertFalse(process_matches(dict(pid=1, command=['tx_dvbs.py'], log='/tmp/x')))

    def test_bandplan_warnings_match_rate_and_physical_frequency(self):
        self.assertIsNone(channel_warning(2370, 1000000))
        self.assertIsNone(channel_warning(2403.75, 1000000))  # Also N2/V3, but a valid wide center.
        self.assertIsNone(channel_warning(2408.25, 250000))
        self.assertIsNone(channel_warning(2408.5, 66000))
        self.assertIsNone(channel_warning(2404.25, 500000))
        self.assertIsNotNone(channel_warning(2408.25, 1000000))
        self.assertIsNotNone(channel_warning(2408.25, 500000))
        self.assertIsNotNone(channel_warning(2404, 1000000))
        self.assertIsNotNone(channel_warning(2402, 125000))
        self.assertIsNotNone(channel_warning(2409.9, 333000))


class GuiTests(unittest.TestCase):
    def test_pa_checkbox_controls_command_and_profile(self):
        box = self.window.widgets['pa_enable']
        self.assertEqual(box.text(), 'PA enable')
        self.assertFalse(box.isChecked())
        box.setChecked(True)
        self.assertIn('--pa-enable', self.window.settings().argv())
        self.window.populate(Settings(pa_enable=True))
        self.assertTrue(box.isChecked())
        box.setChecked(False)
        self.assertNotIn('--pa-enable', self.window.settings().argv())

    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.window = MainWindow(adopt=False, settings_store=False, state_path=Path(self.tmp.name) / 'state.json')
        self.addCleanup(self.tmp.cleanup)
        self.addCleanup(self.close_window)

    def close_window(self):
        self.window.controller.stop()
        wait_for(lambda: not self.window.controller.running())
        self.window.close()
        APP.processEvents()

    def test_fec_updates_when_frame_and_standard_change(self):
        self.window.populate(Settings(frame='short', fec='9/10'))
        self.assertNotEqual(self.window.settings().fec, '9/10')
        self.window.set_value('standard', 'DVB-S')
        self.assertEqual(self.window.settings().mod, 'qpsk')
        self.assertFalse(self.window.widgets['mod'].isEnabled())
        self.assertTrue(self.window.estimate.text().startswith('Setup:'))

    def test_source_switch_and_custom_rate(self):
        self.window.set_value('source', 'camera')
        self.assertEqual(self.window.source_pages.currentIndex(), 3)
        self.window.widgets['baud'].setEditText('12,5')
        self.assertEqual(self.window.settings().baud, 12500)
        self.window.set_value('source', 'ts')
        self.assertTrue(self.window.encoding_controls.isHidden())

    def test_single_view_and_channel_click_sets_uplink_only(self):
        self.assertFalse(self.window.findChildren(QTabWidget))
        forbidden = {'sps', 'target', 'ifm', 'invert', 'swap_iq', 'calibration', 'dc_i', 'dc_q', 'iq_gain', 'iq_phase'}
        self.assertFalse(forbidden & set(self.window.widgets))
        self.window.show()
        APP.processEvents()
        channel = next(c for c in CHANNELS if c.name == 'N11')
        self.assertEqual((channel.uplink, channel.downlink), (2408.25, 10497.75))
        point = self.window.bandplan.channel_rect(channel).center().toPoint()
        QTest.mouseClick(self.window.bandplan, Qt.LeftButton, pos=point)
        self.assertEqual(self.window.settings().freq, 2408.25)
        self.assertEqual(self.window.settings().baud, 1000000)
        self.assertEqual(self.window.controller.state, 'idle')
        old = self.window.settings().freq
        point.setX(round(self.window.bandplan.x(BEACON_UPLINK)))
        QTest.mouseClick(self.window.bandplan, Qt.LeftButton, pos=point)
        self.assertEqual(self.window.settings().freq, old)

    def test_db_slider_replaces_amp_and_updates_for_modulation(self):
        self.assertNotIn('amp', self.window.widgets)
        level = self.window.widgets['level_db']
        self.assertEqual(level.value(), 0)
        self.assertEqual(level.label.text(), '0.0 dB')
        level.setValue(-20)
        _, args = self.window.settings().resolve()
        self.assertEqual(args.amp, 42)
        self.window.set_value('mod', 'qpsk')
        _, args = self.window.settings().resolve()
        self.assertEqual(args.amp, 30)
        self.assertEqual(level.slider.maximum(), 40)

    def test_slider_cannot_cross_configured_floor(self):
        level = self.window.widgets['level_db']
        self.assertEqual(level.slider.minimum(), -270)
        level.setValue(-50)
        self.assertEqual(level.value(), -27)
        self.window.set_value('mod', 'qpsk')
        self.assertEqual(level.value(), -27)
        self.assertEqual(self.window.settings().resolve()[1].amp, 14)
        with patch('tx_gui_model.minimum_level_db', return_value=-15):
            level.set_modulation('qpsk')
            self.assertEqual(level.slider.minimum(), -150)
            self.assertEqual(level.value(), -15)

    def test_async_stop_restart_and_log_stream(self):
        helper = Path(self.tmp.name) / 'backend.py'
        helper.write_text("import signal,time\nprint('OK QPSKT LO 2370000000 BAUD 250000 OUT 8000000 AMP 400 SPS 32',flush=True)\ntry:\n while True:time.sleep(.1)\nexcept KeyboardInterrupt:print('TX END',flush=True)\n")
        c = self.window.controller
        cmd = [sys.executable, '-u', str(helper)]
        c.start(cmd)
        self.assertTrue(wait_for(lambda: c.state == 'running'))
        first = c.live['pid']
        self.assertIn('DAC 8 MS/s', self.window.actual.text())
        c.start(cmd)
        self.assertTrue(wait_for(lambda: c.state == 'running' and c.live['pid'] != first))
        self.assertTrue(Path(self.tmp.name, 'state.json').is_file())
        c.stop()
        self.assertTrue(wait_for(lambda: c.state == 'idle'))

    def test_stop_cancels_pending_restart(self):
        helper = Path(self.tmp.name) / 'backend.py'
        helper.write_text("import time\nprint('ready',flush=True)\ntime.sleep(20)\n")
        c = self.window.controller
        cmd = [sys.executable, '-u', str(helper)]
        c.start(cmd)
        c.start(cmd)
        c.stop()
        self.assertTrue(wait_for(lambda: c.state == 'idle'))
        self.assertIsNone(c.pending)

    def test_unusual_wb_start_requires_explicit_yes(self):
        self.window.populate(Settings(freq=2408.25, baud=1000000, mod='8psk'))
        with patch('tx_gui.W.QMessageBox.question', return_value=QMessageBox.No) as question, patch.object(self.window.controller, 'start') as start:
            self.window.start_transmission()
            self.assertIn('Start anyway?', question.call_args.args[2])
            self.assertEqual(question.call_args.args[4], QMessageBox.No)
            start.assert_not_called()
        with patch('tx_gui.W.QMessageBox.question', return_value=QMessageBox.Yes), patch.object(self.window.controller, 'start') as start:
            self.window.start_transmission()
            start.assert_called_once()

    def test_valid_wb_start_does_not_prompt(self):
        self.window.populate(Settings(freq=2403.75, baud=1000000, mod='8psk'))
        with patch('tx_gui.W.QMessageBox.question') as question, patch.object(self.window.controller, 'start') as start:
            self.window.start_transmission()
            question.assert_not_called()
            start.assert_called_once()


if __name__ == '__main__':
    unittest.main()
