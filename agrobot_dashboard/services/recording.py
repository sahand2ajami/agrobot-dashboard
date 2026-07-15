"""Video recording — writes the rear and front feeds to segmented MP4.

Runs on its own daemon thread while telemetry.recording.active is true; the
frames come from the same TelemetryStore buffers the MJPEG streams serve.

Crash safety: an MP4's index (the moov atom) is only written when the writer
is released, so a single continuous file is unplayable if the process dies
mid-recording. Instead each feed is written in fixed-length segments
(front_000.mp4, front_001.mp4, …); every segment is finalized when it rolls
over, so a crash or power loss loses at most the current in-progress segment
(≤ SEGMENT_SECONDS) rather than the whole session.
"""
import logging
import time

from agrobot_dashboard.services.events import log_event

log = logging.getLogger("dashboard")

RECORD_FPS = 15.0        # rear + front saved at 15 fps (lighter on the Jetson)
SEGMENT_SECONDS = 30.0   # finalize each MP4 chunk on this cadence (crash-safety buffer)


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

    interval = 1.0 / max(RECORD_FPS, 0.1)
    fourcc   = cv2.VideoWriter_fourcc(*'mp4v')

    with telemetry.recording.lock:
        rec_dir = telemetry.recording.dir
        rec_ts  = telemetry.recording.ts

    if rec_dir is None:
        log.error("[record] rec_dir is None at loop start — aborting")
        with telemetry.recording.lock:
            telemetry.recording.active = False
        return

    # One rolling writer per feed; a writer opens lazily on the first frame of
    # each segment (so it can size itself to that frame) and is released when
    # the segment rolls over or recording stops.
    cams = {
        'rear':  {'writer': None, 'frames': 0},
        'front': {'writer': None, 'frames': 0},
    }
    seg_index = 0
    seg_start = time.monotonic()

    def _release_all():
        for c in cams.values():
            if c['writer'] is not None:
                c['writer'].release()
                c['writer'] = None

    while True:
        t0 = time.monotonic()
        with telemetry.recording.lock:
            if not telemetry.recording.active:
                break

        # Roll to a new segment on cadence — this finalizes (indexes) the chunk
        # just written, so everything up to the last rollover survives a crash.
        if t0 - seg_start >= SEGMENT_SECONDS:
            _release_all()
            seg_index += 1
            seg_start = t0

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
            c = cams[which]
            if c['writer'] is None:
                seg_path = rec_dir / f"{which}_{seg_index:03d}.mp4"
                c['writer'] = cv2.VideoWriter(str(seg_path), fourcc, RECORD_FPS, (w, h))
            c['writer'].write(arr)
            c['frames'] += 1

        elapsed = time.monotonic() - t0
        sleep_t = interval - elapsed
        if sleep_t > 0:
            time.sleep(sleep_t)

    _release_all()
    log.info("[record] Saved — rear=%d frames, front=%d frames across %d segment(s) of %.0fs",
             cams['rear']['frames'], cams['front']['frames'], seg_index + 1, SEGMENT_SECONDS)
