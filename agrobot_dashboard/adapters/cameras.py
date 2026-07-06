"""Camera capture — ZED 2i (pyzed SDK) front/rear and generic USB webcams.

Each start_* function spawns a daemon capture thread that writes encoded
frames into the given TelemetryStore and queues frames for the YOLO workers
(services.detection) while a client is viewing detections.
"""
import contextlib
import json
import logging
import os
import threading
import time
from pathlib import Path

from agrobot_dashboard.services.detection import (
    YOLO_CONFIDENCE, YOLO_IMGSZ, YOLO_MAX_DET, YOLO_PERSON_CLASS,
    YOLO_THROTTLE_HZ, YOLO_INFER_LOCK, depth_at_bbox, get_shared_yolo)
from agrobot_dashboard.services.events import log_event

log = logging.getLogger("dashboard")

DETECTIONS_FILE      = "/tmp/object_detections.json"
ZED_DEVICE           = "/dev/zed2i"    # symlink used by the OpenCV grayscale fallback only
ZED_FRONT_INDEX      = 0              # pyzed camera index — front ZED 2i
ZED_REAR_INDEX       = 1              # pyzed camera index — rear ZED 2i
# Front-ZED capture mode. --wide switches to HD2K: the full sensor (~110° FOV,
# 2208×1242) at its 15 fps hardware cap; the UI letterboxes instead of cropping.
ZED_FRONT_RESOLUTION = "HD720"
ZED_FRONT_FPS        = 30
WEBCAM_DEVICE_DEFAULT = "/dev/video0"  # generic USB UVC webcam (e.g. Logitech)
RS_DISPLAY_FPS       = 20.0   # live camera capture / display (20 fps: smoother + far lighter than 30 on the Jetson)
RECORD_FPS           = 15.0   # rear.mp4 + front.mp4 saved at 15 fps (lighter on the Jetson)
STREAM_FPS           = 20.0   # live MJPEG stream to the browser at 20 fps

# ZED SDK is the only color path; the OpenCV fallback is grayscale on this Jetson.
# The SDK open can fail transiently ("CAMERA NOT DETECTED") if the camera is still
# releasing from a prior (unclean) shutdown or a V4L2 process briefly holds it, so
# retry before giving up to grayscale instead of falling back on the first failure.
ZED_SDK_OPEN_RETRIES     = 6
ZED_SDK_OPEN_RETRY_DELAY = 2.5   # seconds between attempts (~15 s total)
ZED_GRAB_LOST_LIMIT      = 15    # consecutive grab failures → treat camera as lost, re-open


@contextlib.contextmanager
def _quiet_stderr():
    """Redirect C-level stderr to /dev/null to suppress OpenCV V4L2 WARN spam."""
    null_fd = os.open(os.devnull, os.O_WRONLY)
    saved   = os.dup(2)
    os.dup2(null_fd, 2)
    try:
        yield
    finally:
        os.dup2(saved, 2)
        os.close(null_fd)
        os.close(saved)



def is_capture_webcam(dev):
    """True if `dev` is a generic USB UVC capture camera (not a RealSense or ZED).
    The ZED and RealSense are driven elsewhere (ZED SDK / RealSense node), so the
    rear webcam must never grab them — doing so yields a grayscale, split stereo
    frame and blocks the ZED SDK's color path."""
    import subprocess
    try:
        info = subprocess.run(
            ['v4l2-ctl', f'--device={dev}', '--info', '--list-formats'],
            capture_output=True, text=True, timeout=2,
        ).stdout
    except Exception:
        return False
    if 'RealSense' in info or 'ZED' in info:
        return False
    return ('Video Capture' in info or 'MJPG' in info or 'YUYV' in info)


def find_webcam_device(preferred=None):
    """Resolve the rear USB webcam to a STABLE device path so it can never be
    confused with the ZED/RealSense and survives unplug/replug or node renumbering.

    Priority: explicit `preferred` > a /dev/v4l/by-id/* symlink (keyed to the
    camera's USB serial — does not change when devices are re-enumerated) > a bare
    /dev/videoN scan. RealSense and ZED nodes are always skipped."""
    if preferred is not None:
        return preferred
    import os
    # Prefer the serial-keyed by-id symlinks: stable across replug/renumber.
    byid_dir = "/dev/v4l/by-id"
    try:
        names = sorted(n for n in os.listdir(byid_dir) if n.endswith("-video-index0"))
    except OSError:
        names = []
    for name in names:
        if "RealSense" in name or "ZED" in name:   # skip by name without opening
            continue
        path = os.path.join(byid_dir, name)
        if is_capture_webcam(path):
            return path                              # stable serial-keyed path
    # Fallback: scan bare nodes (older kernels / no by-id) and skip ZED/RealSense.
    for i in range(10):
        dev = f"/dev/video{i}"
        if os.path.exists(dev) and is_capture_webcam(dev):
            return dev
    return WEBCAM_DEVICE_DEFAULT



def start_rear_camera_thread(telemetry, source="zed", device=None):
    """Capture the rear camera directly via V4L2 — no ROS driver needed.

    source: 'zed' (ZED 2i via pyzed SDK, index ZED_REAR_INDEX) | 'webcam' (generic USB
    UVC, opened MJPG for full frame-rate at 720p). `device` optionally pins the V4L2
    device path/index for the webcam source; auto-detected when None.

    Writes to telemetry.rear_cam.jpeg / _cam_frame_count so /api/camera works even when no
    ROS camera node is running.  If the ROS subscriber is also active (realsense
    source), both paths write the same buffer; last write wins — harmless, same camera.
    """
    if source == "zed":
        start_zed_rear_thread(telemetry)
        return

    try:
        import cv2
        import numpy as np
    except ImportError:
        log.warning("[rear-cam] cv2 not available — rear camera direct capture disabled")
        return

    if source == "webcam":
        use_mjpg = True
        label    = "webcam (USB UVC)"
        # Re-resolved on every (re)connect so an unplug/replug re-binds to the
        # correct stable by-id path rather than whatever node it lands on.
        _resolve_dev = lambda: find_webcam_device(device)
    else:
        use_mjpg = False
        label    = "webcam (V4L2)"
        _resolve_dev = lambda: (device if device is not None else WEBCAM_DEVICE_DEFAULT)

    def _capture_loop():
        _interval    = 1.0 / max(RS_DISPLAY_FPS, 0.1)
        last_display = 0.0
        retry_sleep  = 1.0   # exponential back-off on repeated failures
        cap = None
        dev = _resolve_dev()
        while True:
            if cap is None or not cap.isOpened():
                dev = _resolve_dev()              # re-bind on each reconnect (replug-safe)
                with _quiet_stderr():
                    cap = cv2.VideoCapture(dev, cv2.CAP_V4L2)
                if cap.isOpened():
                    if use_mjpg:   # webcams need MJPG for 30 fps at 720p
                        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
                    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  1280)
                    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
                    cap.set(cv2.CAP_PROP_FPS, 30)
                    # Verify the device can actually deliver a frame before announcing
                    with _quiet_stderr():
                        ok_test, _ = cap.read()
                    if not ok_test:
                        cap.release()
                        cap = None
                        with telemetry.rear_cam.lock:
                            telemetry.rear_cam.connected  = False
                            telemetry.rear_cam.last_error = f"{dev} opened but no frames"
                        time.sleep(retry_sleep)
                        retry_sleep = min(retry_sleep * 2, 5.0)
                        continue
                    retry_sleep = 1.0
                    log.info("[rear-cam] Opened %s (%s)", dev, label)
                else:
                    with telemetry.rear_cam.lock:
                        telemetry.rear_cam.connected  = False
                        telemetry.rear_cam.last_error = f"{dev} not available"
                    time.sleep(retry_sleep)
                    retry_sleep = min(retry_sleep * 2, 5.0)
                    continue
            with _quiet_stderr():
                ret, frame = cap.read()
            if not ret:
                with telemetry.rear_cam.lock:
                    telemetry.rear_cam.connected  = False
                    telemetry.rear_cam.last_error = "Frame read failed — reconnecting"
                cap.release()
                cap = None
                time.sleep(retry_sleep)
                retry_sleep = min(retry_sleep * 2, 5.0)
                continue

            now = time.monotonic()
            if now - last_display >= _interval:
                last_display = now
                ok, enc = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 75])
                if ok:
                    with telemetry.rear_cam.lock:
                        telemetry.rear_cam.jpeg            = enc.tobytes()
                        telemetry.rear_cam.frame_count     += 1
                        telemetry.rear_cam.connected       = True
                        telemetry.rear_cam.last_error      = None
                        telemetry.rear_cam.last_frame_time = time.monotonic()
                        n = telemetry.rear_cam.frame_count
                    if n == 1 or n % 300 == 0:
                        log.debug("[rear-cam] %d frames captured (%dx%d)",
                                  n, frame.shape[1], frame.shape[0])

    threading.Thread(target=_capture_loop, daemon=True, name="rear-cam").start()
    log.info("[rear-cam] Direct capture thread started (%s, %s)", _resolve_dev(), label)


def start_zed_rear_thread(telemetry):
    """Capture the ZED 2i rear camera (pyzed SDK index ZED_REAR_INDEX) for color.

    Mirrors start_zed_thread: runs YOLO when detection is requested on the rear view.
    Mirrors start_zed_thread including depth (DEPTH_MODE.PERFORMANCE) so detections
    carry both confidence and distance_m. Uses the shared YOLO singleton +
    YOLO_INFER_LOCK so front and rear inferences are serialized and don't fight over the GPU.
    """
    try:
        import cv2
    except ImportError:
        log.warning("[rear-cam] cv2 not available — rear ZED capture disabled")
        return

    _min_infer_interval = 1.0 / max(YOLO_THROTTLE_HZ, 0.1)

    def _capture_loop(zed, image_mat, depth_mat):
        import pyzed.sl as sl
        yolo, yolo_dev, yolo_half = get_shared_yolo()

        _inf_lock  = threading.Lock()
        _inf_frame = [None]   # (color_bgr, depth_float32 | None)
        _inf_event = threading.Event()

        def _rear_yolo_worker():
            while True:
                _inf_event.wait()
                _inf_event.clear()
                with _inf_lock:
                    data = _inf_frame[0]
                    _inf_frame[0] = None
                if data is None:
                    continue
                color, depth = data
                try:
                    with YOLO_INFER_LOCK:
                        results = yolo(color, classes=[YOLO_PERSON_CLASS],
                                       conf=YOLO_CONFIDENCE, imgsz=YOLO_IMGSZ,
                                       device=yolo_dev, half=yolo_half,
                                       max_det=YOLO_MAX_DET, verbose=False)
                    boxes_half = []
                    detections = []
                    for result in results:
                        for box in result.boxes:
                            x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
                            conf = float(box.conf[0])
                            dist = depth_at_bbox(depth, x1, y1, x2, y2) if depth is not None else None
                            label = f'person {conf:.0%}'
                            if dist is not None:
                                label += f'  {dist:.1f} m'
                            boxes_half.append((x1 // 2, y1 // 2, x2 // 2, y2 // 2, label))
                            detections.append({
                                'label':      'person',
                                'confidence': round(conf, 3),
                                'distance_m': round(dist, 2) if dist is not None else None,
                                'bbox':       [x1, y1, x2, y2],
                            })
                    det_payload = {
                        'ts':         time.time(),
                        'count':      len(detections),
                        'detections': detections,
                    }
                    with telemetry.rear_det.lock:
                        telemetry.rear_det.boxes   = boxes_half
                        telemetry.rear_det.payload = det_payload
                except Exception as exc:
                    log.error("[rear-cam] YOLO error: %s", exc)

        if yolo is not None:
            threading.Thread(target=_rear_yolo_worker, daemon=True,
                             name="yolo-rear").start()

        runtime_params    = sl.RuntimeParameters()
        last_display      = 0.0
        last_queue        = 0.0
        _display_interval = 1.0 / STREAM_FPS
        retry_sleep       = 1.0
        lost              = 0

        while True:
            err = zed.grab(runtime_params)
            if err == sl.ERROR_CODE.SUCCESS:
                zed.retrieve_image(image_mat, sl.VIEW.LEFT)
                frame = image_mat.get_data()[:, :, :3]   # BGRA → BGR
                retry_sleep = 1.0
                lost        = 0
                now = time.monotonic()
                if now - last_display >= _display_interval:
                    last_display = now
                    small = cv2.resize(frame, (frame.shape[1] // 2, frame.shape[0] // 2))
                    ok, enc = cv2.imencode('.jpg', small, [cv2.IMWRITE_JPEG_QUALITY, 60])
                    if ok:
                        with telemetry.rear_cam.lock:
                            telemetry.rear_cam.jpeg            = enc.tobytes()
                            telemetry.rear_cam.frame           = small   # raw frame for rear detection
                            telemetry.rear_cam.frame_count    += 1
                            telemetry.rear_cam.connected       = True
                            telemetry.rear_cam.last_error      = None
                            telemetry.rear_cam.last_frame_time = time.monotonic()

                if yolo is not None and telemetry.rear_det.wanted():
                    if now - last_queue >= _min_infer_interval:
                        last_queue = now
                        zed.retrieve_measure(depth_mat, sl.MEASURE.DEPTH)
                        depth_arr = depth_mat.get_data().copy()
                        with _inf_lock:
                            _inf_frame[0] = (frame.copy(), depth_arr)
                        _inf_event.set()

            elif err == sl.ERROR_CODE.END_OF_SVOFILE_REACHED:
                zed.set_svo_position(0)
            else:
                with telemetry.rear_cam.lock:
                    telemetry.rear_cam.connected  = False
                    telemetry.rear_cam.last_error = f"rear ZED grab: {err}"
                lost += 1
                if lost >= ZED_GRAB_LOST_LIMIT:
                    log.warning("[rear-cam] rear ZED grab failing (%s) — releasing to re-open", err)
                    log_event("WARN", "Camera",
                              f"Rear ZED camera grab failed ({err}) — reconnecting",
                              "Unplug and replug the rear ZED 2i USB-C cable. "
                              "Check: lsusb | grep STEREOLABS",
                              _key="rear-zed-grab", _debounce_s=30)
                    return
                time.sleep(retry_sleep)
                retry_sleep = min(retry_sleep * 2, 5.0)

    def _start():
        try:
            import ctypes
            ctypes.CDLL('/lib/aarch64-linux-gnu/libusb-1.0.so.0').libusb_init(None)
            import pyzed.sl as sl
        except ImportError:
            log.warning("[rear-cam] pyzed not installed — rear ZED capture disabled")
            return
        except Exception as exc:
            log.warning("[rear-cam] libusb/pyzed init failed (%s) — rear ZED disabled", exc)
            return

        attempt = 0
        delay   = ZED_SDK_OPEN_RETRY_DELAY
        while True:
            attempt += 1
            zed = None
            try:
                zed         = sl.Camera()
                init_params = sl.InitParameters()
                init_params.camera_resolution = sl.RESOLUTION.HD720
                init_params.camera_fps        = 30
                init_params.depth_mode        = sl.DEPTH_MODE.PERFORMANCE
                init_params.coordinate_units  = sl.UNIT.METER
                # Select the second camera (pyzed 4.x API; 3.x uses camera_linux_id)
                try:
                    init_params.input.set_from_camera_index(ZED_REAR_INDEX)
                except AttributeError:
                    try:
                        init_params.camera_linux_id = ZED_REAR_INDEX
                    except AttributeError:
                        pass
                err = zed.open(init_params)
                if err == sl.ERROR_CODE.SUCCESS:
                    image_mat = sl.Mat()
                    depth_mat = sl.Mat()
                    log.info("[rear-cam] ZED 2i rear (SDK index %d) opened", ZED_REAR_INDEX)
                    delay = ZED_SDK_OPEN_RETRY_DELAY
                    _capture_loop(zed, image_mat, depth_mat)
                    err = "camera lost"
                else:
                    with telemetry.rear_cam.lock:
                        telemetry.rear_cam.last_error = f"rear ZED SDK open: {err}"
            except Exception as exc:
                err = exc
                with telemetry.rear_cam.lock:
                    telemetry.rear_cam.last_error = f"rear ZED SDK: {exc}"

            with telemetry.rear_cam.lock:
                telemetry.rear_cam.connected = False
            try:
                if zed is not None:
                    zed.close()
            except Exception:
                pass

            if attempt <= ZED_SDK_OPEN_RETRIES or attempt % 10 == 0:
                log.warning("[rear-cam] rear ZED open failed (%s) — retrying in %.1fs",
                            err, delay)
                log_event("WARN", "Camera",
                          f"Rear ZED camera unavailable (attempt {attempt}): {err}",
                          "Check USB-C cable on the rear ZED 2i. "
                          "Run: lsusb | grep STEREOLABS — two entries expected.",
                          _key="rear-zed-open", _debounce_s=60)
            time.sleep(delay)
            delay = min(delay * 1.5, 5.0)

    threading.Thread(target=_start, daemon=True, name="rear-cam").start()
    log.info("[rear-cam] ZED 2i rear capture thread started (SDK, index %d)", ZED_REAR_INDEX)


def start_zed_thread(telemetry):
    """Capture the ZED 2i front feed in COLOR via the pyzed SDK, which identifies the
    camera by USB serial (deterministic) and yields a single rectified left-eye image
    (no split, no mirror). The SDK open is retried until the camera is available, so
    the feed is always color — the grayscale OpenCV fallback runs ONLY when the pyzed
    SDK is not installed at all."""
    try:
        import cv2
        import numpy as np
    except ImportError:
        log.warning("[zed] cv2 not available — /api/zed will return 503")
        return

    _min_infer_interval = 1.0 / max(YOLO_THROTTLE_HZ, 0.1)

    # ── pyzed SDK capture (full color + depth for YOLO distance) ─────────────
    def _capture_loop_sdk(zed, image_mat):
        import pyzed.sl as sl
        import numpy as np
        yolo, yolo_dev, yolo_half = get_shared_yolo()
        depth_mat = sl.Mat()   # reused each grab; retrieve_measure fills it in-place

        # ── YOLO worker thread ────────────────────────────────────────────────
        # Inference can take >1 s (NMS on large scenes), which would stall grab()
        # and trigger ZED grab-lost errors if done inline. The worker runs at its
        # own pace: the grab loop writes the latest frame and signals; the worker
        # always picks up the most recent frame (last-write-wins, no queue build-up).
        _inf_lock  = threading.Lock()
        _inf_frame = [None]        # (color_bgr_copy, depth_float32_copy) or None
        _inf_event = threading.Event()

        def _yolo_worker():
            while True:
                _inf_event.wait()
                _inf_event.clear()
                with _inf_lock:
                    data = _inf_frame[0]
                    _inf_frame[0] = None
                if data is None:
                    continue
                color, depth = data
                try:
                    with YOLO_INFER_LOCK:
                        results = yolo(color, classes=[YOLO_PERSON_CLASS],
                                       conf=YOLO_CONFIDENCE, imgsz=YOLO_IMGSZ,
                                       device=yolo_dev, half=yolo_half,
                                       max_det=YOLO_MAX_DET, verbose=False)
                    # Store boxes at stream-res (÷2) — the detection stream composites
                    # them onto the live camera frame so video is always real-time.
                    boxes_half = []
                    detections = []
                    for result in results:
                        for box in result.boxes:
                            x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
                            conf = float(box.conf[0])
                            dist = depth_at_bbox(depth, x1, y1, x2, y2)
                            label = f'person {conf:.0%}'
                            if dist is not None:
                                label += f'  {dist:.1f} m'
                            boxes_half.append((x1 // 2, y1 // 2, x2 // 2, y2 // 2, label))
                            detections.append({
                                'label':      'person',
                                'confidence': round(conf, 3),
                                'distance_m': round(dist, 2) if dist is not None else None,
                                'bbox':       [x1, y1, x2, y2],
                            })
                    det_payload = {
                        'ts':         time.time(),
                        'count':      len(detections),
                        'detections': detections,
                    }
                    with telemetry.front_det.lock:
                        telemetry.front_det.boxes   = boxes_half
                        telemetry.front_det.payload = det_payload
                    try:
                        Path(DETECTIONS_FILE).write_text(json.dumps(det_payload))
                    except Exception:
                        pass
                except Exception as exc:
                    log.error("[zed] YOLO error: %s", exc)

        if yolo is not None:
            threading.Thread(target=_yolo_worker, daemon=True, name="yolo-infer").start()

        # ── ZED grab loop — must call grab() continuously at camera rate ──────
        runtime_params    = sl.RuntimeParameters()
        last_queue        = 0.0    # last time we queued a frame for the YOLO worker
        last_display      = 0.0
        _display_interval = 1.0 / STREAM_FPS
        retry_sleep       = 1.0
        lost              = 0

        while True:
            err = zed.grab(runtime_params)
            if err == sl.ERROR_CODE.SUCCESS:
                zed.retrieve_image(image_mat, sl.VIEW.LEFT)
                # get_data() returns BGRA (4-channel); drop alpha
                left = image_mat.get_data()[:, :, :3]

                telemetry.front_zed.set_status(True)
                retry_sleep = 1.0
                lost        = 0

                now = time.monotonic()
                if now - last_display >= _display_interval:
                    last_display = now
                    # Halve resolution for the stream — cuts per-frame size ~4× and
                    # prevents the OS/browser from buffering stale frames.
                    small = cv2.resize(left, (left.shape[1] // 2, left.shape[0] // 2))
                    ok, enc = cv2.imencode('.jpg', small, [cv2.IMWRITE_JPEG_QUALITY, 60])
                    if ok:
                        with telemetry.front_zed.lock:
                            telemetry.front_zed.jpeg            = enc.tobytes()
                            telemetry.front_zed.frame           = small   # raw frame for detection compositing
                            telemetry.front_zed.frame_count    += 1
                            telemetry.front_zed.last_frame_time = time.monotonic()

                # Queue a frame for the YOLO worker (non-blocking — never stalls grab).
                # retrieve_measure is fast (buffer copy, computed at grab time).
                if yolo is not None and telemetry.front_det.wanted():
                    if now - last_queue >= _min_infer_interval:
                        last_queue = now
                        zed.retrieve_measure(depth_mat, sl.MEASURE.DEPTH)
                        with _inf_lock:
                            _inf_frame[0] = (left.copy(), depth_mat.get_data().copy())
                        _inf_event.set()

            elif err == sl.ERROR_CODE.END_OF_SVOFILE_REACHED:
                zed.set_svo_position(0)
            else:
                telemetry.front_zed.set_status(False, f"grab error: {err}")
                lost += 1
                # Camera unplugged / hung: stop grabbing and return so the caller
                # closes this handle and re-opens (by serial) when it reconnects.
                if lost >= ZED_GRAB_LOST_LIMIT:
                    log.warning("[zed] grab failing (%s) — releasing to re-open on reconnect", err)
                    log_event("WARN", "Camera",
                              f"Front ZED camera grab failed ({err}) — reconnecting",
                              "Unplug and replug the front ZED 2i USB-C cable. "
                              "Check: lsusb | grep STEREOLABS",
                              _key="front-zed-grab", _debounce_s=30)
                    return
                time.sleep(retry_sleep)
                retry_sleep = min(retry_sleep * 2, 5.0)

    # ── OpenCV fallback (grayscale — YUYV color bug on Jetson pip OpenCV) ─────
    def _capture_loop_opencv():
        yolo, yolo_dev, yolo_half = get_shared_yolo()

        # Same worker-thread pattern as the SDK path so capture isn't stalled.
        _inf_lock  = threading.Lock()
        _inf_frame = [None]
        _inf_event = threading.Event()

        def _yolo_worker_cv():
            while True:
                _inf_event.wait()
                _inf_event.clear()
                with _inf_lock:
                    color = _inf_frame[0]
                    _inf_frame[0] = None
                if color is None:
                    continue
                try:
                    with YOLO_INFER_LOCK:
                        results = yolo(color, classes=[YOLO_PERSON_CLASS],
                                       conf=YOLO_CONFIDENCE, imgsz=YOLO_IMGSZ,
                                       device=yolo_dev, half=yolo_half,
                                       max_det=YOLO_MAX_DET, verbose=False)
                    boxes_half = []
                    detections = []
                    for result in results:
                        for box in result.boxes:
                            x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
                            conf = float(box.conf[0])
                            boxes_half.append((x1 // 2, y1 // 2, x2 // 2, y2 // 2,
                                               f'person {conf:.0%}'))
                            detections.append({
                                'label':      'person',
                                'confidence': round(conf, 3),
                                'distance_m': None,
                                'bbox':       [x1, y1, x2, y2],
                            })
                    det_payload = {
                        'ts': time.time(), 'count': len(detections), 'detections': detections,
                    }
                    with telemetry.front_det.lock:
                        telemetry.front_det.boxes   = boxes_half
                        telemetry.front_det.payload = det_payload
                    try:
                        Path(DETECTIONS_FILE).write_text(json.dumps(det_payload))
                    except Exception:
                        pass
                except Exception as exc:
                    log.error("[zed] YOLO error: %s", exc)

        if yolo is not None:
            threading.Thread(target=_yolo_worker_cv, daemon=True, name="yolo-infer").start()

        def _device_index():
            try:
                target = os.readlink(ZED_DEVICE)
                return int(target.replace("video", ""))
            except Exception:
                return ZED_DEVICE

        def _fix_yuyv(frame):
            # pip OpenCV NEON YUYV→BGR zeros UV on Jetson: G channel = Y (luminance only).
            gray = frame[:, :, 1]
            return cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)

        cap          = None
        last_queue   = 0.0
        last_display = 0.0
        _display_interval = 1.0 / STREAM_FPS
        retry_sleep  = 1.0

        while True:
            if cap is None or not cap.isOpened():
                with _quiet_stderr():
                    cap = cv2.VideoCapture(_device_index())
                if cap.isOpened():
                    telemetry.front_zed.set_status(True)
                    retry_sleep = 1.0
                    log.info("[zed] Opened %s via OpenCV (grayscale)", ZED_DEVICE)
                else:
                    telemetry.front_zed.set_status(False, f"{ZED_DEVICE} not available")
                    time.sleep(retry_sleep)
                    retry_sleep = min(retry_sleep * 2, 5.0)
                    continue

            with _quiet_stderr():
                ret, frame = cap.read()
            if not ret:
                telemetry.front_zed.set_status(False, "Frame read failed — reconnecting")
                cap.release()
                cap = None
                time.sleep(retry_sleep)
                retry_sleep = min(retry_sleep * 2, 5.0)
                continue

            h, w = frame.shape[:2]
            left = _fix_yuyv(frame[:, :w // 2])

            now = time.monotonic()
            if now - last_display >= _display_interval:
                last_display = now
                small = cv2.resize(left, (left.shape[1] // 2, left.shape[0] // 2))
                ok, enc = cv2.imencode('.jpg', small, [cv2.IMWRITE_JPEG_QUALITY, 60])
                if ok:
                    with telemetry.front_zed.lock:
                        telemetry.front_zed.jpeg            = enc.tobytes()
                        telemetry.front_zed.frame           = small
                        telemetry.front_zed.frame_count    += 1
                        telemetry.front_zed.last_frame_time = time.monotonic()

            if yolo is not None and telemetry.front_det.wanted():
                if now - last_queue >= _min_infer_interval:
                    last_queue = now
                    with _inf_lock:
                        _inf_frame[0] = left.copy()
                    _inf_event.set()

    # ── ZED front feed: SDK only (color), retried forever. The SDK identifies the
    #    camera by its USB serial, so the front view is deterministic — it is always
    #    the ZED, never another camera, and never the grayscale V4L2 fallback. If the
    #    ZED is absent/busy the front reports "unavailable" and auto-recovers on
    #    reconnect. The grayscale OpenCV fallback runs ONLY when pyzed is not installed.
    def _start():
        try:
            # libusb 1.0.25 (Ubuntu 22.04) doesn't auto-init the default context;
            # ZED SDK calls libusb_get_device_list(NULL) which segfaults without it.
            import ctypes
            ctypes.CDLL('/lib/aarch64-linux-gnu/libusb-1.0.so.0').libusb_init(None)
            import pyzed.sl as sl
        except ImportError:
            log.warning("[zed] pyzed SDK not installed — cannot guarantee color; "
                        "using OpenCV grayscale fallback")
            _capture_loop_opencv()
            return
        except Exception as exc:
            log.warning("[zed] libusb/pyzed init failed (%s) — OpenCV grayscale fallback", exc)
            _capture_loop_opencv()
            return

        attempt = 0
        delay   = ZED_SDK_OPEN_RETRY_DELAY
        while True:                       # retry forever — color or nothing, never grayscale
            attempt += 1
            zed = None
            try:
                zed         = sl.Camera()
                init_params = sl.InitParameters()
                init_params.camera_resolution = getattr(sl.RESOLUTION, ZED_FRONT_RESOLUTION)
                init_params.camera_fps        = ZED_FRONT_FPS
                # PERFORMANCE depth enables distance readout in YOLO detection.
                init_params.depth_mode = sl.DEPTH_MODE.PERFORMANCE
                try:
                    init_params.coordinate_units = sl.UNIT.METER
                except AttributeError:
                    pass
                # Front camera is always the default (index 0); don't call
                # set_from_camera_index here — it doesn't exist in pyzed 3.x
                # and would crash the retry loop on that SDK version.

                err = zed.open(init_params)
                if err == sl.ERROR_CODE.SUCCESS:
                    image_mat = sl.Mat()
                    log.info("[zed] ZED 2i front SDK opened — color+depth active")
                    delay = ZED_SDK_OPEN_RETRY_DELAY
                    _capture_loop_sdk(zed, image_mat)   # returns if the camera is lost
                    # capture loop exited (camera unplugged) — release and re-open below
                    err = "camera lost"
                else:
                    telemetry.front_zed.set_status(False, f"SDK open: {err}")
            except Exception as exc:
                err = exc
                telemetry.front_zed.set_status(False, f"SDK open: {exc}")

            with telemetry.front_zed.lock:
                telemetry.front_zed.connected = False   # keep the last error message for /api/zed/status
            try:
                if zed is not None:
                    zed.close()   # release before retrying so the next open can detect it
            except Exception:
                pass

            # Stay quiet after the first few failures to avoid log spam while idle.
            if attempt <= ZED_SDK_OPEN_RETRIES or attempt % 10 == 0:
                log.warning("[zed] SDK open failed (%s) — retrying in %.1fs (color-only, "
                            "no grayscale)", err, delay)
                log_event("WARN", "Camera",
                          f"Front ZED camera unavailable (attempt {attempt}): {err}",
                          "Check USB-C cable on the front ZED 2i. "
                          "Run: lsusb | grep STEREOLABS — two entries expected (one per camera).",
                          _key="front-zed-open", _debounce_s=60)
            time.sleep(delay)
            delay = min(delay * 1.5, 5.0)

    threading.Thread(target=_start, daemon=True, name="zed-capture").start()
    log.info("[zed] Capture thread started (%s)", ZED_DEVICE)
