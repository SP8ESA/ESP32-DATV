"""FFmpeg inputs and channel-sized video encoding shared by the CLI and GUI."""
from dataclasses import dataclass
from pathlib import Path
import re
from urllib.parse import urlsplit

DEFAULT_SERVICE_NAME = "ESP32-C3 DATV"
DEFAULT_SERVICE_PROVIDER = "ESP32-DATV"


def validate_service_metadata(service_name, service_provider):
    for label, value in (("Service name", service_name), ("Service provider", service_provider)):
        if not isinstance(value, str) or "\0" in value:
            raise ValueError(f"{label} must be text without NUL characters")


@dataclass(frozen=True)
class VideoEncoding:
    mux: int
    width: int
    fps: float
    video_bps: int
    audio_k: int
    channels: int
    sample_rate: int
    pat_period: float


def encoding_settings(cap, width=640, video_k=0, output_fps=0):
    mux = int(cap * .965)
    if mux >= 600_000:
        aud, channels, rate, fps, pat, w = 96, 2, 48000, 25, .2, width
    elif mux >= 200_000:
        aud, channels, rate, fps, pat, w = 32, 1, 24000, 15, .5, min(width, 320)
    else:
        aud, channels, rate, fps, pat, w = (8 if mux < 40_000 else 16), 1, 16000, 10, 1., min(width, 160)
    psi = int(3 * 188 * 8 / pat)
    budget = mux - aud * 1000 - psi
    vb = video_k * 1000 if video_k else int(budget * .88)
    if vb < 6000:
        raise ValueError(f"Channel capacity {cap / 1000:.1f} kb/s is too small for video")
    if vb > budget:
        raise ValueError(f"Video bitrate exceeds channel capacity: maximum {max(0, budget) // 1000} kb/s")
    if not 32 <= width <= 4096 or width % 2:
        raise ValueError("Video width must be an even number from 32 to 4096")
    if output_fps and not 1 <= output_fps <= 60:
        raise ValueError("Output FPS must be 0 (automatic) or 1..60")
    return VideoEncoding(mux, w, output_fps or fps, vb, aud, channels, rate, pat)


def is_ts_url(value):
    return urlsplit(value).scheme.lower() in ("udp", "rtp", "tcp", "http", "https")


def validate_source(kind, value, camera_size="640x480", camera_fps=30, camera_format="auto", audio_source="none"):
    if kind in ("film", "ts") and value != "-" and not (kind == "ts" and is_ts_url(value)):
        p = Path(value)
        if not p.is_file() or p.stat().st_size == 0:
            raise ValueError(f"Source file does not exist or is empty: {value}")
        if kind == "ts":
            with p.open("rb") as f:
                head = f.read(65536)
            if not any(head[i] == 0x47 and head[i + 188] == 0x47
                       for i in range(max(0, len(head) - 188))):
                raise ValueError("TS input must contain 188-byte MPEG transport packets")
    if kind == "camera":
        if not re.fullmatch(r"\d{2,4}x\d{2,4}", camera_size):
            raise ValueError("Camera size must have the form 640x480")
        if not 1 <= camera_fps <= 120:
            raise ValueError("Camera FPS must be 1..120")
        if camera_format not in ("auto", "mjpeg", "yuyv422", "nv12", "h264"):
            raise ValueError("Unsupported camera input format")
        if audio_source not in ("none", "pulse", "alsa"):
            raise ValueError("Camera audio source must be none, pulse or alsa")
        if not Path(value).exists():
            raise ValueError(f"Camera device does not exist: {value}")


def build_media_command(kind, value, cap, width=640, video_k=0, output_fps=0,
                        camera_size="640x480", camera_fps=30, camera_format="auto",
                        audio_source="none", audio_device="default",
                        service_name=DEFAULT_SERVICE_NAME, service_provider=DEFAULT_SERVICE_PROVIDER):
    validate_service_metadata(service_name, service_provider)
    metadata = ["-metadata", f"service_provider={service_provider}", "-metadata", f"service_name={service_name}"]
    prefix = ["ffmpeg", "-nostdin", "-hide_banner", "-loglevel", "error"]
    if kind == "ts" and is_ts_url(value):
        return prefix + ["-i", value, "-map", "0:v?", "-map", "0:a?", "-c", "copy", "-f", "mpegts"] + metadata + ["-"]
    if kind not in ("film", "test", "camera"):
        return None
    v = encoding_settings(cap, width, video_k, output_fps)
    if kind == "film":
        inputs = ["-re", "-stream_loop", "-1", "-i", value]
        maps = ["-map", "0:v:0", "-map", "0:a:0?"]
    elif kind == "test":
        inputs = ["-re", "-f", "lavfi", "-i", f"testsrc2=size={v.width}x{(v.width * 9 // 16) // 2 * 2}:rate={v.fps}",
                  "-re", "-f", "lavfi", "-i", f"sine=frequency=800:sample_rate={v.sample_rate}"]
        maps = ["-map", "0:v:0", "-map", "1:a:0"]
    else:
        inputs = ["-thread_queue_size", "512", "-f", "v4l2", "-framerate", str(camera_fps), "-video_size", camera_size]
        if camera_format != "auto":
            inputs += ["-input_format", camera_format]
        inputs += ["-i", value]
        if audio_source == "none":
            layout = "stereo" if v.channels == 2 else "mono"
            inputs += ["-re", "-f", "lavfi", "-i", f"anullsrc=r={v.sample_rate}:cl={layout}"]
        else:
            inputs += ["-thread_queue_size", "512", "-f", audio_source, "-i", audio_device]
        maps = ["-map", "0:v:0", "-map", "1:a:0"]
    encode = ["-vf", f"scale={v.width}:-2,fps={v.fps}",
              "-c:v", "libx264", "-preset", "veryfast", "-profile:v", "main", "-g", str(round(2 * v.fps)), "-bf", "2",
              "-b:v", str(v.video_bps), "-maxrate", str(v.video_bps), "-bufsize", str(v.video_bps // 2),
              "-x264-params", "nal-hrd=cbr:force-cfr=1", "-pix_fmt", "yuv420p",
              "-c:a", "mp2", "-b:a", f"{v.audio_k}k", "-ac", str(v.channels), "-ar", str(v.sample_rate),
              "-f", "mpegts", "-muxrate", str(v.mux), "-pcr_period", "40" if v.audio_k > 50 else "100",
              "-pat_period", str(v.pat_period), "-mpegts_flags", "+resend_headers"]
    return prefix + inputs + maps + encode + metadata + ["-"]
