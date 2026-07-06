"""Video recording — writes the rear and front feeds to MP4.

Runs on its own daemon thread while telemetry.recording.active is true; the
frames come from the same TelemetryStore buffers the MJPEG streams serve.
"""
import logging
import time

from agrobot_dashboard.services.events import log_event

log = logging.getLogger("dashboard")

RECORD_FPS = 15.0   # rear.mp4 + front.mp4 saved at 15 fps (lighter on the Jetson)


def recording_loop(telemetry):
    """Write rear and front cameras to MP4 at RECORD_FPS."""
    try:
        import cv2
        import numpy as np
    except ImportError:
        log.error("[record] cv2 not available — cannot record video")
        log_event("ERROR", "System",
                  "Video recording failed — OpenCV (cv2) is not installed",
                  "Install OpenCV: pip3 install opencv-python",
                  _key="record-cv2", _debounce_s=3600)
        with telemetry.recording.lock:
            telemetry.recording.active = False
        return

    interval     = 1.0 / max(RECORD_FPS, 0.1)
    fourcc       = cv2.VideoWriter_fourcc(*'mp4v')
    rear_writer  = None
    front_writer = None
    rear_frames  = 0
    front_frames = 0

    with telemetry.recording.lock:
        rec_dir = telemetry.recording.dir
        rec_ts  = telemetry.recording.ts

    if rec_dir is None:
        log.error("[record] rec_dir is None at loop start — aborting")
        with telemetry.recording.lock:
            telemetry.recording.active = False
        return

    rear_path  = rec_dir / "rear.mp4"
    front_path = rec_dir / "front.mp4"

    while True:
        t0 = time.monotonic()
        with telemetry.recording.lock:
            if not telemetry.recording.active:
                break
        with telemetry.rear_cam.lock:
            rear_jpeg = telemetry.rear_cam.jpeg
        with telemetry.front_zed.lock:
            front_jpeg = telemetry.front_zed.jpeg

        for jpeg, which in ((rear_jpeg, 'rear'), (front_jpeg, 'front')):
            if jpeg is None:
                continue
            arr = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
            if arr is None:
                continue
            h, w = arr.shape[:2]
            if which == 'rear':
                if rear_writer is None:
                    rear_writer = cv2.VideoWriter(str(rear_path), fourcc, RECORD_FPS, (w, h))
                rear_writer.write(arr)
                rear_frames += 1
            else:
                if front_writer is None:
                    front_writer = cv2.VideoWriter(str(front_path), fourcc, RECORD_FPS, (w, h))
                front_writer.write(arr)
                front_frames += 1

        elapsed = time.monotonic() - t0
        sleep_t = interval - elapsed
        if sleep_t > 0:
            time.sleep(sleep_t)

    if rear_writer:
        rear_writer.release()
    if front_writer:
        front_writer.release()
    log.info("[record] Saved — rear=%d frames, front=%d frames", rear_frames, front_frames)
