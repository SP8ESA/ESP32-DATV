#!/usr/bin/env python3
"""Desktop controls for ESP32-DATV. Launch from the repository with uruchom_nadajnik.sh."""
import argparse
from dataclasses import asdict
import json
import os
from pathlib import Path
import re
import shutil
import signal
import subprocess
import sys
import tempfile
import time

try:
    from PyQt5 import QtCore, QtGui, QtWidgets as W
except ImportError:
    raise SystemExit("GUI requires PyQt5: python3 -m pip install -r host/requirements-gui.txt")

from serial.tools import list_ports
from tx_gui_model import REPO, SCRIPT, TX_STATE, Settings, camera_devices, fec_choices, load_live_state, process_matches, level_limits
from tx_media import encoding_settings
from tx_bandplan_widget import BandplanWidget
from qo100_bandplan import SOURCE_URL, channel_warning


class Transmitter(QtCore.QObject):
    line = QtCore.pyqtSignal(str)
    state_changed = QtCore.pyqtSignal(str)
    finished = QtCore.pyqtSignal()

    def __init__(self, parent=None, state_path=TX_STATE):
        super().__init__(parent)
        self.state_path = Path(state_path)
        self.live = None
        self.process = None
        self.pending = None
        self.state = "idle"
        self.offset = 0
        self.fragment = ""
        self.stop_deadline = 0
        self.forced = False
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.poll)
        self.timer.start(150)

    def running(self):
        if self.process is not None:
            return self.process.poll() is None
        return self.live is not None and process_matches(self.live)

    def set_state(self, value):
        if value != self.state:
            self.state = value
            self.state_changed.emit(value)

    def adopt(self, state):
        if not process_matches(state):
            return False
        self.live = state
        self.process = None
        self.offset = 0
        self.fragment = ""
        self.set_state("running")
        self.read_log(initial=True)
        return True

    def start(self, command):
        if self.running():
            self.pending = list(command)
            self.stop(cancel_restart=False)
        else:
            self.launch(command)

    def launch(self, command):
        folder = Path(tempfile.mkdtemp(prefix="esp32_datv_gui_"))
        log = folder / "transmitter.log"
        with log.open("wb") as stream:
            self.process = subprocess.Popen(command, cwd=REPO, stdin=subprocess.DEVNULL,
                                            stdout=stream, stderr=subprocess.STDOUT, start_new_session=True)
        self.live = dict(pid=self.process.pid, command=list(command), log=str(log))
        temp = self.state_path.with_suffix(".tmp")
        temp.write_text(json.dumps(self.live, indent=2) + "\n")
        temp.replace(self.state_path)
        self.offset, self.fragment = 0, ""
        self.pending = None
        self.set_state("starting")
        self.line.emit("Starting…")

    def stop(self, cancel_restart=True):
        if cancel_restart:
            self.pending = None
        if not self.running():
            self.complete()
            return
        if self.state == "stopping":
            return
        self.set_state("stopping")
        self.stop_deadline = time.monotonic() + 25
        self.forced = False
        try:
            os.kill(self.live["pid"], signal.SIGINT)
        except ProcessLookupError:
            self.complete()
            return
        self.line.emit("Stopping…")

    def read_log(self, initial=False):
        if self.live is None:
            return
        try:
            path = Path(self.live["log"])
            if not path.exists():
                return
            size = path.stat().st_size
            if initial:
                # Show the mode acknowledgement and recent activity, without flooding the UI.
                with path.open("rb") as f:
                    head = f.read(8192).decode(errors="replace").splitlines()
                    f.seek(max(0, size - 6000))
                    tail = f.read().decode(errors="replace").splitlines()
                for text in head[:8] + tail[-12:]:
                    self.line.emit(text)
                self.offset = size
                return
            if size < self.offset:
                self.offset, self.fragment = 0, ""
            with path.open("rb") as f:
                f.seek(self.offset)
                data = f.read(65536)
                self.offset = f.tell()
            text = self.fragment + data.decode(errors="replace")
            parts = text.split("\n")
            self.fragment = parts.pop()
            for part in parts:
                self.line.emit(part.rstrip("\r"))
                if part.startswith("OK QPSKT") and self.state == "starting":
                    self.set_state("running")
        except OSError as e:
            self.line.emit(str(e))

    def complete(self):
        self.read_log()
        if self.process is not None:
            status = self.process.poll()
            if status is not None:
                self.line.emit(f"TX stopped (exit {status}).")
        self.process = None
        self.live = None
        self.set_state("idle")
        pending, self.pending = self.pending, None
        if pending is not None:
            try:
                self.launch(pending)
            except OSError as e:
                self.line.emit(f"Start failed: {e}")
        else:
            self.finished.emit()

    def poll(self):
        if self.live is None:
            return
        self.read_log()
        if not self.running():
            self.complete()
        elif self.state == "stopping" and time.monotonic() > self.stop_deadline:
            # Identity is rechecked by running() for adopted processes.
            try:
                os.kill(self.live["pid"], signal.SIGKILL if self.forced else signal.SIGTERM)
            except ProcessLookupError:
                self.complete()
                return
            self.forced = True
            self.stop_deadline = time.monotonic() + 5
            self.line.emit("TX unresponsive; stopping.")


class SourceStack(W.QStackedWidget):
    def sizeHint(self):
        return self.currentWidget().sizeHint() if self.currentWidget() else super().sizeHint()

    def minimumSizeHint(self):
        return self.currentWidget().minimumSizeHint() if self.currentWidget() else super().minimumSizeHint()


class LevelSlider(W.QWidget):
    valueChanged = QtCore.pyqtSignal(float)

    def __init__(self, parent=None):
        super().__init__(parent)
        layout = W.QHBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        self.slider = W.QSlider(QtCore.Qt.Horizontal)
        self.slider.setSingleStep(1)
        self.slider.setPageStep(10)
        self.label = W.QLabel("0.0 dB")
        self.label.setMinimumWidth(64)
        self.label.setAlignment(QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter)
        reset = W.QPushButton("0 dB")
        reset.clicked.connect(lambda: self.setValue(0))
        layout.addWidget(self.slider, 1)
        layout.addWidget(self.label)
        layout.addWidget(reset)
        self.setToolTip("Relative to the default level for this modulation (0 dB).")
        self.slider.valueChanged.connect(self.changed)
        self.set_modulation("32apsk")
        self.setValue(0)

    def set_modulation(self, mod):
        low, high = level_limits(mod)
        self.slider.setRange(round(low * 10), round(high * 10))

    def value(self):
        return self.slider.value() / 10

    def setValue(self, value):
        self.slider.setValue(round(value * 10))

    def changed(self, value):
        db = value / 10
        self.label.setText(f"{db:+.1f} dB" if db else "0.0 dB")
        self.valueChanged.emit(db)


class MainWindow(W.QMainWindow):
    def __init__(self, adopt=True, settings_store=None, state_path=TX_STATE):
        super().__init__()
        self.setWindowTitle("ESP32-DATV — TX")
        screen = W.QApplication.primaryScreen().availableGeometry()
        self.resize(min(1080, screen.width() - 40), min(880, screen.height() - 50))
        self.store = QtCore.QSettings("SP8ESA", "ESP32-DATV") if settings_store is None else settings_store
        self._loading = False
        self._closing = False
        self.widgets = {}
        self.controller = Transmitter(self, state_path)
        self.controller.line.connect(self.log_line)
        self.controller.state_changed.connect(self.update_state)
        self.controller.finished.connect(self.transmission_finished)
        self.build_ui()
        defaults = Settings()
        saved = defaults
        if self.store:
            try:
                text = self.store.value("profile", "")
                if text:
                    saved = Settings.from_profile(json.loads(text))
            except (ValueError, OSError, TypeError, SystemExit):
                pass
        # Cold starts begin at W1 / 0 dB; explicit profiles remain loadable.
        saved.freq, saved.level_db, saved.baud = defaults.freq, defaults.level_db, defaults.baud
        if saved.mod == "32apsk":
            saved.mod = defaults.mod  # 32APSK currently cannot transmit at 1 MS/s.
        live = load_live_state() if adopt else None
        if live:
            try:
                saved = Settings.from_command(live["command"])
            except (ValueError, StopIteration, SystemExit):
                live = None
        self.populate(saved)
        if live:
            self.controller.adopt(live)
        self.update_state(self.controller.state)

    def spin(self, key, low, high, default=0, decimals=None, auto=False):
        w = W.QDoubleSpinBox() if decimals is not None else W.QSpinBox()
        if decimals is not None:
            w.setDecimals(decimals)
        w.setRange(low, high)
        w.setValue(default)
        if auto:
            w.setSpecialValueText("Auto" if key != "seconds" else "Until Stop")
        w.valueChanged.connect(self.settings_changed)
        self.widgets[key] = w
        return w

    def combo(self, key, items, editable=False):
        w = W.QComboBox()
        w.setEditable(editable)
        for label, value in items:
            w.addItem(label, value)
        w.currentIndexChanged.connect(self.settings_changed)
        if editable:
            w.editTextChanged.connect(self.settings_changed)
        self.widgets[key] = w
        return w

    def check(self, key, label):
        w = W.QCheckBox(label)
        w.toggled.connect(self.settings_changed)
        self.widgets[key] = w
        return w

    def edit(self, key, placeholder=""):
        w = W.QLineEdit()
        w.setPlaceholderText(placeholder)
        w.textChanged.connect(self.settings_changed)
        self.widgets[key] = w
        return w

    def file_row(self, key, filters):
        holder = W.QWidget()
        layout = W.QHBoxLayout(holder)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.addWidget(self.edit(key))
        browse = W.QPushButton("Browse")
        browse.clicked.connect(lambda: self.choose_file(key, filters))
        layout.addWidget(browse)
        return holder

    def pair(self, layout, row, left_label, left, right_label=None, right=None):
        layout.addWidget(W.QLabel(left_label), row, 0)
        layout.addWidget(left, row, 1)
        if right is not None:
            layout.addWidget(W.QLabel(right_label), row, 2)
            layout.addWidget(right, row, 3)
        else:
            layout.addWidget(left, row, 1, 1, 3)

    def build_ui(self):
        central = W.QWidget()
        outer = W.QVBoxLayout(central)
        outer.setContentsMargins(12, 10, 12, 10)
        outer.setSpacing(6)
        heading = W.QHBoxLayout()
        title = W.QLabel("ESP32-DATV")
        title.setObjectName("title")
        heading.addWidget(title)
        heading.addStretch()
        self.status = W.QLabel("Stopped")
        self.status.setObjectName("status")
        heading.addWidget(self.status)
        outer.addLayout(heading)
        usb = W.QHBoxLayout()
        usb.addWidget(W.QLabel("USB"))
        usb.addWidget(self.combo("port", [("Auto · Espressif USB", "")], editable=True), 1)
        scan = W.QPushButton("Refresh")
        scan.clicked.connect(self.refresh_devices)
        usb.addWidget(scan)
        source_link = W.QLabel(f'<a href="{SOURCE_URL}">BATC bandplan</a>')
        source_link.setOpenExternalLinks(True)
        usb.addWidget(source_link)
        outer.addLayout(usb)
        self.bandplan = BandplanWidget()
        self.bandplan.frequency_selected.connect(self.select_channel)
        outer.addWidget(self.bandplan)

        controls = W.QWidget()
        controls_layout = W.QVBoxLayout(controls)
        controls_layout.setContentsMargins(0, 0, 0, 0)
        body = W.QHBoxLayout()
        rf_group = W.QGroupBox("TX")
        rf = W.QGridLayout(rf_group)
        rf.setHorizontalSpacing(8)
        rf.setVerticalSpacing(7)
        self.pair(rf, 0, "TX MHz", self.spin("freq", 2300, 2450, Settings().freq, 6), "Mod", self.combo("mod", [(v.upper(), v) for v in ("qpsk", "8psk", "16apsk", "32apsk")]))
        self.pair(rf, 1, "Standard", self.combo("standard", [("DVB-S", "DVB-S"), ("DVB-S2", "DVB-S2")]), "FEC", self.combo("fec", [("3/4", "3/4")]))
        self.pair(rf, 2, "SR kS/s", self.combo("baud", [(f"{v}", v * 1000) for v in (33, 66, 125, 250, 333, 500, 1000)], editable=True), "Frame", self.combo("frame", [("Normal", "normal"), ("Short", "short")]))
        self.widgets["baud"].setToolTip("Symbol rate in kS/s")
        rf.addWidget(self.check("pilots", "Pilots"), 3, 0, 1, 2)
        rf.addWidget(W.QLabel("PL"), 3, 2)
        rf.addWidget(self.combo("apsk_pl", [("Unit (SDRangel)", "unit"), ("Outer ring", "outer")]), 3, 3)
        level = LevelSlider()
        level.valueChanged.connect(self.settings_changed)
        self.widgets["level_db"] = level
        self.pair(rf, 4, "Level", level)
        self.pair(rf, 5, "PPM", self.spin("ppm", -1000, 1000, 0, 3), "Time s", self.spin("seconds", 0, 86400, 0, 1, auto=True))
        pa = self.check("pa_enable", "PA enable")
        pa.setToolTip("GPIO3 · high during TX · Apply/Restart")
        rf.addWidget(pa, 6, 0, 1, 4)
        for key in ("standard", "mod", "frame"):
            self.widgets[key].currentIndexChanged.connect(self.update_choices)
        body.addWidget(rf_group, 1)
        source_group = W.QGroupBox("Source")
        source_layout = W.QVBoxLayout(source_group)
        source_layout.setSpacing(6)
        source_layout.addWidget(self.combo("source", [("Video file", "film"), ("TS file", "ts"), ("TS URL", "ts_url"), ("Camera", "camera"), ("Test", "test"), ("Null TS", "null"), ("Carrier", "cw")]))
        self.source_pages = SourceStack()
        self.source_pages.setSizePolicy(W.QSizePolicy.Preferred, W.QSizePolicy.Maximum)
        for kind in ("film", "ts", "ts_url", "camera", "test", "null", "cw"):
            page = W.QWidget()
            f = W.QGridLayout(page)
            f.setContentsMargins(0, 0, 0, 0)
            f.setVerticalSpacing(6)
            if kind == "film":
                f.addWidget(self.file_row("film", "Video (*.mp4 *.mkv *.avi *.mov *.webm *.mpeg *.mpg);;All files (*)"), 0, 0, 1, 4)
            elif kind == "ts":
                f.addWidget(self.file_row("ts", "TS (*.ts);;All files (*)"), 0, 0, 1, 4)
            elif kind == "ts_url":
                f.addWidget(self.edit("ts_url", "udp://0.0.0.0:8888"), 0, 0, 1, 4)
            elif kind == "camera":
                self.pair(f, 0, "Device", self.combo("camera", [("/dev/video0", "/dev/video0")], editable=True))
                self.pair(f, 1, "Size", self.combo("camera_size", [(v, v) for v in ("640x480", "1280x720", "1920x1080")], editable=True), "Cam FPS", self.spin("camera_fps", 1, 120, 30, 1))
                self.pair(f, 2, "Format", self.combo("camera_format", [("Auto", "auto"), ("MJPEG", "mjpeg"), ("YUYV422", "yuyv422"), ("NV12", "nv12"), ("H.264", "h264")]), "Audio", self.combo("audio_source", [("Silence", "none"), ("Pulse / PipeWire", "pulse"), ("ALSA", "alsa")]))
                self.pair(f, 3, "Audio device", self.edit("audio_device", "default or hw:0,0"))
            self.source_pages.addWidget(page)
        source_layout.addWidget(self.source_pages)
        self.service_controls = W.QWidget()
        service = W.QGridLayout(self.service_controls)
        service.setContentsMargins(0, 0, 0, 0)
        self.pair(service, 0, "Service name", self.edit("service_name"))
        self.pair(service, 1, "Service provider", self.edit("service_provider"))
        source_layout.addWidget(self.service_controls)
        self.encoding_controls = W.QWidget()
        enc = W.QGridLayout(self.encoding_controls)
        enc.setContentsMargins(0, 0, 0, 0)
        self.pair(enc, 0, "Width px", self.spin("width", 32, 4096, 640), "Video kb/s", self.spin("video_k", 0, 10000, auto=True))
        self.widgets["width"].setSingleStep(2)
        self.pair(enc, 1, "FPS", self.spin("fps", 0, 60, 0, 1, auto=True))
        source_layout.addWidget(self.encoding_controls)
        self.video_hint = W.QLabel()
        self.video_hint.setWordWrap(True)
        source_layout.addWidget(self.video_hint)
        source_layout.addStretch()
        body.addWidget(source_group, 1)
        controls_layout.addLayout(body)
        scroll = W.QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(W.QFrame.NoFrame)
        scroll.setWidget(controls)
        scroll.setMinimumHeight(140)
        outer.addWidget(scroll, 2)
        cal = W.QHBoxLayout()
        cal.addWidget(W.QLabel("Cal file"))
        cal.addWidget(self.file_row("cal", "Calibration (*.json);;All files (*)"), 1)
        outer.addLayout(cal)
        self.estimate = W.QLabel()
        self.estimate.setObjectName("estimate")
        self.estimate.setWordWrap(True)
        outer.addWidget(self.estimate)
        self.error = W.QLabel()
        self.error.setWordWrap(True)
        self.error.setStyleSheet("color: #b23b35;")
        self.error.hide()
        outer.addWidget(self.error)
        actions = W.QHBoxLayout()
        self.start_button = W.QPushButton("Start")
        self.start_button.setObjectName("start")
        self.start_button.clicked.connect(self.start_transmission)
        self.stop_button = W.QPushButton("Stop")
        self.stop_button.clicked.connect(self.stop_transmission)
        actions.addWidget(self.start_button)
        actions.addWidget(self.stop_button)
        actions.addStretch()
        for label, handler in (("Load profile", self.load_profile), ("Save profile", self.save_profile)):
            button = W.QPushButton(label)
            button.clicked.connect(handler)
            actions.addWidget(button)
        outer.addLayout(actions)
        self.actual = W.QLabel("TX: stopped")
        self.buffer_label = W.QLabel("Buffer: —")
        info = W.QHBoxLayout()
        info.addWidget(self.actual)
        info.addStretch()
        info.addWidget(self.buffer_label)
        outer.addLayout(info)
        self.log = W.QPlainTextEdit()
        self.log.setReadOnly(True)
        self.log.setMaximumBlockCount(1800)
        self.log.setMinimumHeight(60)
        self.log.setFont(QtGui.QFont("Monospace", 9))
        outer.addWidget(self.log, 1)
        footer = W.QHBoxLayout()
        self.stop_on_close = W.QCheckBox("Stop TX on close")
        self.stop_on_close.setChecked(True)
        footer.addWidget(self.stop_on_close)
        footer.addStretch()
        clear = W.QPushButton("Clear log")
        clear.clicked.connect(self.log.clear)
        footer.addWidget(clear)
        export = W.QPushButton("Save log")
        export.clicked.connect(self.export_log)
        footer.addWidget(export)
        outer.addLayout(footer)
        self.setCentralWidget(central)
        self.refresh_devices()

    def value(self, key):
        w = self.widgets[key]
        if isinstance(w, W.QCheckBox):
            return w.isChecked()
        if isinstance(w, (W.QSpinBox, W.QDoubleSpinBox, LevelSlider)):
            return w.value()
        if isinstance(w, W.QLineEdit):
            return w.text().strip()
        if w.isEditable():
            if key == "baud":
                text = w.currentText().replace("kS/s", "").strip().replace(",", ".")
                rate = float(text) * 1000
                if not rate.is_integer():
                    raise ValueError("Use a whole number of symbols/s")
                return int(rate)
            if w.currentIndex() >= 0 and w.currentText() == w.itemText(w.currentIndex()):
                return w.currentData()
            return w.currentText().strip()
        return w.currentData()

    def settings(self):
        return Settings(**{key: self.value(key) for key in self.widgets})

    def populate(self, settings):
        self._loading = True
        self.widgets["level_db"].set_modulation(settings.mod)
        for key, value in asdict(settings).items():
            if key not in ("fec", "mod", "frame", "standard"):
                self.set_value(key, value)
        for key in ("standard", "mod", "frame"):
            self.set_value(key, getattr(settings, key))
        self._loading = False
        self.update_choices()
        self.set_value("fec", settings.fec)
        self.settings_changed()

    def set_value(self, key, value):
        w = self.widgets[key]
        if isinstance(w, W.QCheckBox):
            w.setChecked(bool(value))
        elif isinstance(w, (W.QSpinBox, W.QDoubleSpinBox, LevelSlider)):
            w.setValue(value)
        elif isinstance(w, W.QLineEdit):
            w.setText(str(value))
        else:
            index = w.findData(value)
            if index >= 0:
                w.setCurrentIndex(index)
            elif w.isEditable():
                w.setEditText(f"{value / 1000:g}" if key == "baud" else str(value))

    def select_channel(self, frequency):
        self.widgets["freq"].setValue(frequency)

    def update_choices(self):
        if self._loading:
            return
        self._loading = True
        standard = self.value("standard")
        s2 = standard == "DVB-S2"
        if not s2:
            self.set_value("mod", "qpsk")
        mod, frame = self.value("mod"), self.value("frame")
        self.widgets["level_db"].set_modulation(mod)
        self.widgets["mod"].setEnabled(s2)
        for key in ("frame", "pilots"):
            self.widgets[key].setEnabled(s2)
        self.widgets["apsk_pl"].setEnabled(mod == "32apsk")
        fec = self.value("fec")
        self.widgets["fec"].clear()
        for rate in fec_choices(standard, mod, frame):
            self.widgets["fec"].addItem(rate, rate)
        self.set_value("fec", fec)
        rate_text = self.widgets["baud"].currentText()
        rates = (33, 66, 125, 250) if mod == "32apsk" else (33, 66, 125, 250, 333, 500, 1000)
        self.widgets["baud"].clear()
        for rate in rates:
            self.widgets["baud"].addItem(f"{rate}", rate * 1000)
        self.widgets["baud"].setEditText(rate_text)
        self._loading = False
        self.settings_changed()

    def show_error(self, text):
        self.error.setText(text)
        self.error.setVisible(bool(text))

    def settings_changed(self):
        if self._loading or not hasattr(self, "estimate"):
            return
        try:
            settings = self.settings()
            source = settings.source
            self.source_pages.setCurrentIndex(("film", "ts", "ts_url", "camera", "test", "null", "cw").index(source))
            self.source_pages.updateGeometry()
            self.encoding_controls.setVisible(source in ("film", "test", "camera"))
            self.service_controls.setEnabled(source in ("film", "test", "camera", "ts_url"))
            self.service_controls.setToolTip("TS files keep their service metadata." if source == "ts" else "")
            self.widgets["audio_device"].setEnabled(settings.audio_source != "none")
            self.bandplan.set_selection(settings.freq, settings.baud)
            mode, a = settings.resolve()
            self.estimate.setText(f"Setup: {settings.mod.upper()} · {mode.baud / 1000:g} kS/s · DAC {mode.sample_hz / 1e6:g} MS/s · TS {mode.capacity / 1000:.1f} kb/s")
            if source in ("film", "test", "camera"):
                v = encoding_settings(mode.capacity, settings.width, settings.video_k, settings.fps)
                self.video_hint.setText(f"Output: {v.width} px · {v.fps:g} fps · {v.video_bps / 1000:.0f} kb/s video · {v.audio_k} kb/s audio")
            elif source in ("ts", "ts_url"):
                self.video_hint.setText(f"TS limit: {mode.capacity / 1000:.1f} kb/s · no re-encoding")
            else:
                self.video_hint.setText("")
            self.show_error("")
        except (ValueError, TypeError, KeyError, SystemExit) as e:
            self.estimate.setText("Check TX settings")
            self.show_error(str(e))

    def choose_file(self, key, filters):
        path, _ = W.QFileDialog.getOpenFileName(self, "Select file", self.value(key) or str(REPO), filters, options=W.QFileDialog.DontUseNativeDialog)
        if path:
            self.widgets[key].setText(path)

    def refresh_devices(self):
        self._loading = True
        port = self.value("port") if "port" in self.widgets else ""
        w = self.widgets["port"]
        w.clear()
        w.addItem("Auto · Espressif USB", "")
        for p in list_ports.comports():
            if p.vid == 0x303a or "espressif" in p.description.lower():
                w.addItem(f"{p.device} — {p.description}", p.device)
        self.set_value("port", port)
        if "camera" in self.widgets:
            camera = self.value("camera")
            w = self.widgets["camera"]
            w.clear()
            for value, label in camera_devices():
                w.addItem(label, value)
            self.set_value("camera", camera or "/dev/video0")
        self._loading = False
        self.settings_changed()

    def start_transmission(self):
        try:
            settings = self.settings()
            settings.resolve(check_files=True)
            if shutil.which("ffmpeg") is None and settings.source in ("film", "test", "camera", "ts_url"):
                raise ValueError("FFmpeg not found")
            warning = channel_warning(settings.freq, settings.baud)
            if warning:
                answer = W.QMessageBox.question(
                    self, "Check WB channel", warning + "\n\nStart anyway?",
                    W.QMessageBox.Yes | W.QMessageBox.No, W.QMessageBox.No)
                if answer != W.QMessageBox.Yes:
                    return
            command = [sys.executable, "-u", "-B", str(SCRIPT)] + settings.argv()
            self.controller.start(command)
            self.show_error("")
            self.save_preferences()
        except (ValueError, OSError, KeyError, SystemExit) as e:
            self.show_error(str(e))

    def stop_transmission(self):
        self.controller.stop()

    def update_state(self, state):
        labels = {"idle": "Stopped", "starting": "Starting…", "running": "On air", "stopping": "Stopping…"}
        self.status.setText(labels[state])
        self.status.setStyleSheet("color: #147d58;" if state == "running" else "color: #596572;")
        self.start_button.setEnabled(state not in ("starting", "stopping"))
        self.start_button.setText("Apply / Restart" if state == "running" else "Start")
        self.stop_button.setEnabled(state != "idle")

    def log_line(self, line):
        self.log.appendPlainText(line)
        if line.startswith("OK QPSKT"):
            parts = line.split()
            values = dict(zip(parts[2::2], parts[3::2]))
            try:
                frequency = Settings.from_command(self.controller.live["command"]).freq
                prefix = f"TX {frequency:.3f} MHz"
            except (ValueError, TypeError, KeyError, StopIteration, SystemExit):
                prefix = "TX"
            self.actual.setText(f"{prefix}: {values.get('MOD', '')} · {float(values['BAUD']) / 1000:g} kS/s · DAC {float(values['OUT']) / 1e6:g} MS/s")
        match = re.search(r"ESP buffer (\d+)\.\.(\d+) pairs, TS packets: (\d+) from the source, (\d+) null", line)
        if match:
            lo, hi, packets, nulls = match.groups()
            self.buffer_label.setText(f"Buffer: {lo}–{hi} · TS: {packets}")
        if "Traceback" in line or line.startswith("ERR") or "does not answer" in line:
            self.show_error("TX error · see log")

    def load_profile(self):
        path, _ = W.QFileDialog.getOpenFileName(self, "Load profile", str(REPO), "TX profile (*.json)", options=W.QFileDialog.DontUseNativeDialog)
        if path:
            try:
                self.populate(Settings.from_profile(json.loads(Path(path).read_text())))
            except (ValueError, OSError, TypeError, KeyError, SystemExit) as e:
                self.show_error(str(e))

    def save_profile(self):
        try:
            settings = self.settings()
            settings.resolve()
            path, _ = W.QFileDialog.getSaveFileName(self, "Save profile", "tx.json", "TX profile (*.json)", options=W.QFileDialog.DontUseNativeDialog)
            if path:
                Path(path).write_text(json.dumps(settings.profile(), ensure_ascii=False, indent=2) + "\n")
        except (ValueError, OSError, SystemExit) as e:
            self.show_error(str(e))

    def save_preferences(self):
        if self.store:
            try:
                self.store.setValue("profile", json.dumps(self.settings().profile()))
                self.store.sync()
            except (ValueError, TypeError):
                pass

    def export_log(self):
        path, _ = W.QFileDialog.getSaveFileName(self, "Save log", "tx.log", "Log (*.log *.txt)", options=W.QFileDialog.DontUseNativeDialog)
        if path:
            try:
                Path(path).write_text(self.log.toPlainText() + "\n")
            except OSError as e:
                self.show_error(str(e))

    def transmission_finished(self):
        if self._closing:
            self.close()

    def closeEvent(self, event):
        self.save_preferences()
        if self.stop_on_close.isChecked() and self.controller.running():
            self._closing = True
            self.controller.stop()
            event.ignore()
        else:
            event.accept()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--no-adopt", action="store_true", help="open without attaching to an existing transmitter")
    parser.add_argument("--profile", type=Path, help="open a saved profile without starting TX")
    args = parser.parse_args()
    initial = Settings.from_profile(json.loads(args.profile.read_text())) if args.profile else None
    QtCore.QLocale.setDefault(QtCore.QLocale(QtCore.QLocale.English, QtCore.QLocale.UnitedKingdom))
    app = W.QApplication(sys.argv[:1])
    app.setStyle("Fusion")
    app.setStyleSheet("""
        QMainWindow { background: #f4f6f8; }
        QLabel#title { font-size: 22px; font-weight: 700; color: #172d40; }
        QLabel#status { font-size: 14px; font-weight: 600; }
        QLabel#estimate { background: #e6edf4; padding: 7px; border-radius: 4px; color: #18374d; }
        QPushButton { padding: 4px 9px; }
        QPushButton#start { background: #167552; color: white; font-weight: 600; }
        QGroupBox { border: 1px solid #d6dee5; border-radius: 4px; margin-top: 8px; padding-top: 8px; }
        QPlainTextEdit { background: #152431; color: #dbe7f1; }
    """)
    window = MainWindow(adopt=not args.no_adopt)
    if initial is not None:
        window.populate(initial)
    window.show()
    return app.exec_()


if __name__ == "__main__":
    raise SystemExit(main())
