#!/usr/bin/env python3
"""Agrobot Robot dashboard server — wide-angle camera variant.

Two changes vs serve.py:
  1. index_wide.html is served (uses object-fit:contain so frames are never cropped).
  2. ZED 2i is opened at HD2K (2208×1242) instead of HD720, exposing the full sensor
     and the widest available FOV (~110°).  Max framerate drops from 30 → 15 fps.

Nothing else is changed.  All robot control, GNSS, ROS, and recording logic is
inherited from serve.py by module-level monkey-patching.
"""
import os
import sys
import json
import time
import threading
from http.server import SimpleHTTPRequestHandler
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
import serve as _serve  # noqa: E402

# ── 1. Serve index_wide.html instead of index.html ────────────────────────────

_orig_do_GET = _serve.Handler.do_GET


def _wide_do_GET(self):
    p = self.path.split('?')[0].split('#')[0]
    if p in ('/', '/index.html', '/index_wide.html'):
        self.path = '/index_wide.html'
        SimpleHTTPRequestHandler.do_GET(self)
    else:
        _orig_do_GET(self)


_serve.Handler.do_GET = _wide_do_GET


# ── 2. ZED 2i at HD2K (full sensor FOV) ───────────────────────────────────────
# _start_zed_thread() is called by name from inside serve.main(), so replacing
# it in the serve module's namespace here means serve.main() picks up our version.

def _start_zed_thread():
    """Capture ZED 2i frames at HD2K (2208×1242, full sensor) via pyzed or OpenCV."""
    try:
        import cv2
        import numpy as np
    except ImportError:
        _serve.log.warning("[zed] cv2 not available — /api/zed will return 503")
        return

    Handler = _serve.Handler
    log     = _serve.log
    _min_infer_interval = 1.0 / max(_serve.YOLO_THROTTLE_HZ, 0.1)

    def _draw_box(img, x1, y1, x2, y2, label):
        cv2.rectangle(img, (x1, y1), (x2, y2), (0, 0, 220), 2)
        (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.55, 2)
        cv2.rectangle(img, (x1, y1 - th - 8), (x1 + tw + 6, y1), (0, 0, 220), -1)
        cv2.putText(img, label, (x1 + 3, y1 - 4),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)

    # ── pyzed SDK capture ──────────────────────────────────────────────────────
    def _capture_loop_sdk(zed, image_mat):
        import pyzed.sl as sl
        yolo = None
        try:
            from ultralytics import YOLO
            yolo = YOLO(_serve.YOLO_MODEL)
            yolo(np.zeros((480, 640, 3), dtype=np.uint8),
                 classes=[_serve.YOLO_PERSON_CLASS], verbose=False)
            log.info("[zed] YOLO model '%s' ready (SDK path)", _serve.YOLO_MODEL)
        except Exception as exc:
            log.warning("[zed] YOLO unavailable — %s", exc)

        runtime_params    = sl.RuntimeParameters()
        last_infer        = 0.0
        last_display      = 0.0
        _display_interval = 1.0 / 15.0  # HD2K hardware cap is 15 fps
        retry_sleep       = 1.0

        while True:
            err = zed.grab(runtime_params)
            if err == sl.ERROR_CODE.SUCCESS:
                zed.retrieve_image(image_mat, sl.VIEW.LEFT_UNRECTIFIED)
                left = image_mat.get_data()[:, :, :3]  # BGRA → BGR

                Handler._zed_connected  = True
                Handler._zed_last_error = None
                retry_sleep = 1.0

                now_disp = time.monotonic()
                if now_disp - last_display >= _display_interval:
                    last_display = now_disp
                    ok, enc = cv2.imencode('.jpg', left, [cv2.IMWRITE_JPEG_QUALITY, 75])
                    if ok:
                        with Handler._zed_lock:
                            Handler._zed_jpeg            = enc.tobytes()
                            Handler._zed_frame_count     += 1
                            Handler._zed_last_frame_time = time.monotonic()

                if yolo is None:
                    continue
                now = time.monotonic()
                if now - last_infer < _min_infer_interval:
                    continue
                last_infer = now

                try:
                    results   = yolo(left, classes=[_serve.YOLO_PERSON_CLASS],
                                     conf=_serve.YOLO_CONFIDENCE, verbose=False)
                    annotated = left.copy()
                    detections = []
                    for result in results:
                        for box in result.boxes:
                            x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
                            conf = float(box.conf[0])
                            _draw_box(annotated, x1, y1, x2, y2, f'person {conf:.0%}')
                            detections.append({
                                'label':      'person',
                                'confidence': round(conf, 3),
                                'distance_m': None,
                                'bbox':       [x1, y1, x2, y2],
                            })
                    ok2, enc2 = cv2.imencode('.jpg', annotated,
                                             [cv2.IMWRITE_JPEG_QUALITY, 80])
                    if ok2:
                        with Handler._det_lock:
                            Handler._det_jpeg = enc2.tobytes()
                    payload = json.dumps({
                        'ts':         time.time(),
                        'count':      len(detections),
                        'detections': detections,
                    })
                    try:
                        Path(_serve.DETECTIONS_FILE).write_text(payload)
                    except Exception:
                        pass
                except Exception as exc:
                    log.error("[zed] YOLO error: %s", exc)

            elif err == sl.ERROR_CODE.END_OF_SVOFILE_REACHED:
                zed.set_svo_position(0)
            else:
                Handler._zed_connected  = False
                Handler._zed_last_error = f"grab error: {err}"
                time.sleep(retry_sleep)
                retry_sleep = min(retry_sleep * 2, 5.0)

    # ── OpenCV fallback (grayscale, Jetson YUYV bug) ───────────────────────────
    def _capture_loop_opencv():
        yolo = None
        try:
            from ultralytics import YOLO
            yolo = YOLO(_serve.YOLO_MODEL)
            yolo(np.zeros((480, 640, 3), dtype=np.uint8),
                 classes=[_serve.YOLO_PERSON_CLASS], verbose=False)
            log.info("[zed] YOLO model '%s' ready (OpenCV fallback)", _serve.YOLO_MODEL)
        except Exception as exc:
            log.warning("[zed] YOLO unavailable — %s", exc)

        def _device_index():
            try:
                target = os.readlink(_serve.ZED_DEVICE)
                return int(target.replace("video", ""))
            except Exception:
                return _serve.ZED_DEVICE

        def _fix_yuyv(frame):
            # pip OpenCV NEON YUYV→BGR zeros UV on Jetson: G channel = luminance only.
            gray = frame[:, :, 1]
            return cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)

        cap = None
        last_infer        = 0.0
        last_display      = 0.0
        _display_interval = 1.0 / 15.0
        retry_sleep       = 1.0

        while True:
            if cap is None or not cap.isOpened():
                with _serve._quiet_stderr():
                    cap = cv2.VideoCapture(_device_index())
                if cap.isOpened():
                    # Request HD2K side-by-side stereo (4416 = 2×2208 wide)
                    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  4416)
                    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1242)
                    Handler._zed_connected  = True
                    Handler._zed_last_error = None
                    retry_sleep = 1.0
                    log.info("[zed] Opened %s via OpenCV (HD2K grayscale)",
                             _serve.ZED_DEVICE)
                else:
                    Handler._zed_connected  = False
                    Handler._zed_last_error = f"{_serve.ZED_DEVICE} not available"
                    time.sleep(retry_sleep)
                    retry_sleep = min(retry_sleep * 2, 5.0)
                    continue

            with _serve._quiet_stderr():
                ret, frame = cap.read()
            if not ret:
                Handler._zed_connected  = False
                Handler._zed_last_error = "Frame read failed — reconnecting"
                cap.release()
                cap = None
                time.sleep(retry_sleep)
                retry_sleep = min(retry_sleep * 2, 5.0)
                continue

            h, w = frame.shape[:2]
            left = _fix_yuyv(frame[:, :w // 2])

            now_disp = time.monotonic()
            if now_disp - last_display >= _display_interval:
                last_display = now_disp
                ok, enc = cv2.imencode('.jpg', left, [cv2.IMWRITE_JPEG_QUALITY, 75])
                if ok:
                    with Handler._zed_lock:
                        Handler._zed_jpeg            = enc.tobytes()
                        Handler._zed_frame_count     += 1
                        Handler._zed_last_frame_time = time.monotonic()

            if yolo is None:
                continue
            now = time.monotonic()
            if now - last_infer < _min_infer_interval:
                continue
            last_infer = now

            try:
                results   = yolo(left, classes=[_serve.YOLO_PERSON_CLASS],
                                 conf=_serve.YOLO_CONFIDENCE, verbose=False)
                annotated = left.copy()
                detections = []
                for result in results:
                    for box in result.boxes:
                        x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
                        conf = float(box.conf[0])
                        _draw_box(annotated, x1, y1, x2, y2, f'person {conf:.0%}')
                        detections.append({
                            'label':      'person',
                            'confidence': round(conf, 3),
                            'distance_m': None,
                            'bbox':       [x1, y1, x2, y2],
                        })
                ok2, enc2 = cv2.imencode('.jpg', annotated,
                                         [cv2.IMWRITE_JPEG_QUALITY, 80])
                if ok2:
                    with Handler._det_lock:
                        Handler._det_jpeg = enc2.tobytes()
                payload = json.dumps({
                    'ts':         time.time(),
                    'count':      len(detections),
                    'detections': detections,
                })
                try:
                    Path(_serve.DETECTIONS_FILE).write_text(payload)
                except Exception:
                    pass
            except Exception as exc:
                log.error("[zed] YOLO error: %s", exc)

    def _start():
        try:
            import ctypes
            ctypes.CDLL('/lib/aarch64-linux-gnu/libusb-1.0.so.0').libusb_init(None)

            import pyzed.sl as sl
            zed         = sl.Camera()
            init_params = sl.InitParameters()
            init_params.camera_resolution = sl.RESOLUTION.HD2K  # full sensor, ~110° FOV
            init_params.camera_fps        = 15                   # HD2K hardware max
            init_params.depth_mode        = sl.DEPTH_MODE.NONE

            err = zed.open(init_params)
            if err != sl.ERROR_CODE.SUCCESS:
                raise RuntimeError(f"ZED SDK open failed: {err}")

            image_mat = sl.Mat()
            log.info("[zed] ZED SDK opened — HD2K unrectified color feed active (15 fps, full FOV)")
            _capture_loop_sdk(zed, image_mat)

        except ImportError:
            log.info("[zed] pyzed not found — using OpenCV HD2K grayscale fallback")
            _capture_loop_opencv()
        except Exception as exc:
            log.warning("[zed] SDK error (%s) — falling back to OpenCV", exc)
            _capture_loop_opencv()

    threading.Thread(target=_start, daemon=True, name="zed-capture").start()
    log.info("[zed] Capture thread started (HD2K) (%s)", _serve.ZED_DEVICE)


# Replace the function in serve's module namespace so serve.main() picks it up.
_serve._start_zed_thread = _start_zed_thread

if __name__ == '__main__':
    _serve.main()
