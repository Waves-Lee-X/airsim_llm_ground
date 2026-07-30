"""Supervise the D435i RGB to EasyDarwin RTSP pipeline."""

import os
import signal
import subprocess
import time
from pathlib import Path


RTSP_URL = os.environ.get("AEROMIND_RTSP_URL", "rtsp://127.0.0.1:15544/cam")
CAMERA_DEVICE = Path(os.environ.get("AEROMIND_RGB_DEVICE", "/dev/video4"))
EASYDARWIN = os.environ.get(
    "AEROMIND_EASYDARWIN",
    "/home/lenovo3/stream/EasyDarwin/easydarwin",
)
RESTART_DELAY_S = 2.0

stopping = False
easy_proc = None
ffmpeg_proc = None


def log(message):
    print(message, flush=True)


def request_stop(_signum, _frame):
    global stopping
    stopping = True


def terminate(process, name):
    if process is None or process.poll() is not None:
        return
    log("[INFO] stopping {}".format(name))
    process.terminate()
    try:
        process.wait(timeout=5)
    except subprocess.TimeoutExpired:
        process.kill()
        process.wait(timeout=2)


def ffmpeg_command():
    return [
        "ffmpeg",
        "-hide_banner",
        "-loglevel",
        "warning",
        "-f",
        "v4l2",
        "-rtbufsize",
        "2M",
        "-fflags",
        "nobuffer",
        "-framerate",
        "15",
        "-video_size",
        "424x240",
        "-i",
        str(CAMERA_DEVICE),
        "-an",
        "-vcodec",
        "libx264",
        "-preset",
        "ultrafast",
        "-tune",
        "zerolatency",
        "-bf",
        "0",
        "-b:v",
        "800k",
        "-maxrate",
        "800k",
        "-bufsize",
        "400k",
        "-g",
        "15",
        "-keyint_min",
        "15",
        "-sc_threshold",
        "0",
        "-pix_fmt",
        "yuv420p",
        "-rtsp_transport",
        "tcp",
        "-flush_packets",
        "1",
        "-f",
        "rtsp",
        RTSP_URL,
    ]


def main():
    global easy_proc, ffmpeg_proc
    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)

    log("[INFO] starting EasyDarwin")
    easy_proc = subprocess.Popen([EASYDARWIN])
    for _ in range(25):
        if stopping or easy_proc.poll() is not None:
            break
        time.sleep(0.2)

    try:
        while not stopping:
            if easy_proc.poll() is not None:
                raise RuntimeError(
                    "EasyDarwin exited with code {}".format(easy_proc.returncode)
                )
            if not CAMERA_DEVICE.exists():
                log("[WARN] waiting for {}".format(CAMERA_DEVICE))
                time.sleep(RESTART_DELAY_S)
                continue

            log("[INFO] publishing {} to {}".format(CAMERA_DEVICE, RTSP_URL))
            ffmpeg_proc = subprocess.Popen(ffmpeg_command())
            while not stopping and ffmpeg_proc.poll() is None:
                if easy_proc.poll() is not None:
                    raise RuntimeError(
                        "EasyDarwin exited with code {}".format(easy_proc.returncode)
                    )
                time.sleep(0.5)
            if stopping:
                break
            log(
                "[WARN] FFmpeg exited with code {}; retrying".format(
                    ffmpeg_proc.returncode
                )
            )
            ffmpeg_proc = None
            time.sleep(RESTART_DELAY_S)
    finally:
        terminate(ffmpeg_proc, "FFmpeg")
        terminate(easy_proc, "EasyDarwin")


if __name__ == "__main__":
    main()
