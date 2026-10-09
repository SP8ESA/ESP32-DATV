"""GUI settings and process identity checks; no GUI or hardware side effects."""
from dataclasses import asdict, dataclass, fields
import glob
import json
import math
import os
import fcntl
import struct
from pathlib import Path

import dvbs
import dvbs2
from tx_dvbs import DEMO_FILM, HERE, build_parser, resolve_transmission, source_kind
from tx_media import (encoding_settings, is_ts_url, validate_source,
                      DEFAULT_SERVICE_NAME, DEFAULT_SERVICE_PROVIDER, validate_service_metadata)
from qo100_bandplan import CHANNELS

REPO = Path(HERE).parent
SCRIPT = REPO / "host/tx_dvbs.py"
TX_STATE = Path("/tmp/esp32_datv_tx_current.json")
GUI_CONFIG = Path(HERE) / "tx_gui_config.json"


def minimum_level_db(config_path=GUI_CONFIG):
    data = json.loads(Path(config_path).read_text()) if Path(config_path).exists() else {}
    value = data.get("min_level_db", -27.)
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or not -60 <= value <= 0:
        raise ValueError("min_level_db must be between -60 and 0 dB")
    return float(value)


def default_amp(mod):
    return 400 if mod == "32apsk" else 420 if mod in ("8psk", "16apsk") else 300


def level_limits(mod):
    base = default_amp(mod)
    low = max(20 * math.log10(1 / base), minimum_level_db())
    return math.ceil(low * 10) / 10, math.floor(20 * math.log10(480 / base) * 10) / 10


def amp_from_db(level_db, mod):
    base = default_amp(mod)
    minimum = minimum_level_db()
    if not math.isfinite(level_db) or level_db < minimum - 1e-9:
        raise ValueError(f"Minimum TX level: {minimum:g} dB")
    if not math.isfinite(level_db) or level_db < 20 * math.log10(1 / base) - 1e-9 or level_db > 20 * math.log10(480 / base) + 1e-9:
        raise ValueError("TX level exceeds the DAC range")
    minimum_amp = max(1, math.ceil(base * 10 ** (minimum / 20) - 1e-9))
    return max(minimum_amp, min(480, round(base * 10 ** (level_db / 20))))


def db_from_amp(amp, mod):
    if amp == 0:
        return 0.
    if not 1 <= amp <= 480:
        raise ValueError("Invalid DAC amplitude")
    return 20 * math.log10(amp / default_amp(mod))


@dataclass
class Settings:
    freq: float = next(c.uplink for c in CHANNELS if c.name == "W1")
    baud: int = 1000000
    standard: str = "DVB-S2"
    mod: str = "8psk"
    fec: str = "3/4"
    frame: str = "normal"
    pilots: bool = True
    apsk_pl: str = "unit"
    level_db: float = 0.
    pa_enable: bool = False
    ppm: float = 0.
    seconds: float = 0.
    port: str = ""
    source: str = "film"
    film: str = str(Path(DEMO_FILM).resolve())
    ts: str = ""
    ts_url: str = "udp://0.0.0.0:8888?fifo_size=1000000&overrun_nonfatal=1"
    camera: str = "/dev/video0"
    camera_size: str = "640x480"
    camera_fps: float = 30.
    camera_format: str = "auto"
    audio_source: str = "none"
    audio_device: str = "default"
    service_name: str = DEFAULT_SERVICE_NAME
    service_provider: str = DEFAULT_SERVICE_PROVIDER
    width: int = 640
    video_k: int = 0
    fps: float = 0.
    cal: str = str(Path(HERE) / "cal.json")

    def argv(self):
        validate_service_metadata(self.service_name, self.service_provider)
        if not isinstance(self.pa_enable, bool):
            raise ValueError("PA enable must be true or false")
        if self.standard not in ("DVB-S", "DVB-S2"):
            raise ValueError("Unknown standard")
        if self.standard == "DVB-S" and self.mod != "qpsk":
            raise ValueError("DVB-S requires QPSK")
        values = ("freq", "baud", "fec", "mod", "ppm", "seconds", "width", "video_k", "fps")
        args = [part for key in values for part in ("--" + key.replace("_", "-"), str(getattr(self, key)))]
        args += ["--amp", str(amp_from_db(self.level_db, self.mod))]
        if self.pa_enable:
            args.append("--pa-enable")
        if self.standard == "DVB-S2":
            args += ["--dvbs2", "--frame", self.frame]
            if self.pilots:
                args.append("--pilots")
        if self.mod == "32apsk":
            args += ["--apsk-pl", self.apsk_pl]
        if self.port:
            args += ["--port", self.port]
        args += ["--cal", self.cal]
        args += [f"--service-name={self.service_name}", f"--service-provider={self.service_provider}"]
        if self.source in ("film", "ts", "camera"):
            args += ["--" + self.source, getattr(self, self.source)]
        elif self.source == "ts_url":
            if not is_ts_url(self.ts_url):
                raise ValueError("TS URL: udp://, rtp://, tcp://, http:// or https://")
            args += ["--ts", self.ts_url]
        elif self.source in ("test", "null", "cw"):
            args += ["--" + self.source]
        else:
            raise ValueError("Unknown source")
        if self.source == "camera":
            for key in ("camera_size", "camera_fps", "camera_format", "audio_source", "audio_device"):
                args += ["--" + key.replace("_", "-"), str(getattr(self, key))]
        return args

    def resolve(self, check_files=False):
        a = build_parser().parse_args(self.argv())
        mode = resolve_transmission(a)
        if self.source in ("film", "camera", "test"):
            encoding_settings(mode.capacity, self.width, self.video_k, self.fps)
        if check_files:
            kind = "ts" if self.source == "ts_url" else self.source
            value = self.ts_url if self.source == "ts_url" else getattr(self, self.source, "")
            validate_source(kind, value, self.camera_size, self.camera_fps, self.camera_format, self.audio_source)
            calibration = json.loads(Path(self.cal).read_text())
            values = (*calibration["dc"], calibration["iq_gain"], calibration["iq_phase_deg"])
            if len(values) != 4 or not all(math.isfinite(v) for v in values) or not .7 <= values[2] <= 1.3 or not -40 <= values[3] <= 40:
                raise ValueError("Invalid calibration file")
        return mode, a

    def profile(self):
        return {"version": 3, "settings": asdict(self)}

    @classmethod
    def from_profile(cls, data):
        if data.get("version") not in (1, 2, 3) or not isinstance(data.get("settings"), dict):
            raise ValueError("Invalid ESP32-DATV profile")
        values = dict(data["settings"])
        if data["version"] == 1:
            for key in ("sps", "target", "ifm", "invert", "swap_iq", "calibration", "dc_i", "dc_q", "iq_gain", "iq_phase"):
                values.pop(key, None)
        if data["version"] in (1, 2):
            values["level_db"] = db_from_amp(values.pop("amp", 0), values.get("mod", "32apsk"))
        if set(values) - {f.name for f in fields(cls)}:
            raise ValueError("Unknown profile parameters")
        result = cls(**values)
        result.resolve()
        return result

    @classmethod
    def from_command(cls, command):
        i = next(i for i, arg in enumerate(command) if Path(arg).name == "tx_dvbs.py")
        a = build_parser().parse_args(command[i + 1:])
        names = {f.name for f in fields(cls)}
        result = cls(**{key: value for key, value in vars(a).items() if key in names and value is not None})
        result.standard = "DVB-S2" if a.dvbs2 else "DVB-S"
        result.source = "cw" if a.cw else source_kind(a)
        if a.ts and is_ts_url(a.ts):
            result.source, result.ts_url = "ts_url", a.ts
        result.port = a.port or ""
        result.level_db = db_from_amp(a.amp, a.mod)
        return result


def fec_choices(standard, mod, frame):
    return list(dvbs.PUNCT) if standard == "DVB-S" else dvbs2.rates(mod, frame)


def process_matches(state):
    """Only signal the exact transmitter command in this repository, never a reused PID."""
    try:
        pid = int(state["pid"])
        if pid <= 1:
            return False
        base = Path(f"/proc/{pid}")
        if base.joinpath("stat").read_text().rsplit(")", 1)[1].split()[0] == "Z":
            return False
        actual = base.joinpath("cmdline").read_bytes().split(b"\0")[:-1]
        command = state["command"]
        if actual != [os.fsencode(arg) for arg in command]:
            return False
        cwd = base.joinpath("cwd").resolve()
        return cwd == REPO and any((cwd / arg).resolve() == SCRIPT for arg in command if Path(arg).name == "tx_dvbs.py")
    except (OSError, KeyError, TypeError, ValueError):
        return False


def load_live_state():
    try:
        state = json.loads(TX_STATE.read_text())
        return state if process_matches(state) else None
    except (OSError, ValueError, TypeError):
        return None


def camera_devices():
    result = []
    for path in sorted(glob.glob("/dev/video*")):
        try:
            # VIDIOC_QUERYCAP only reads capabilities; it does not start capture.
            fd = os.open(path, os.O_RDONLY | os.O_NONBLOCK)
            try:
                capabilities = bytearray(104)
                fcntl.ioctl(fd, 0x80685600, capabilities)
                caps, node_caps = struct.unpack_from("=II", capabilities, 84)
                if caps & 0x80000000:
                    caps = node_caps
                if not caps & (0x1 | 0x1000):
                    continue
            finally:
                os.close(fd)
        except OSError:
            pass
        name = Path("/sys/class/video4linux") / Path(path).name / "name"
        try:
            label = name.read_text().strip()
        except OSError:
            label = "Camera"
        result.append((path, f"{label} — {path}"))
    return result
