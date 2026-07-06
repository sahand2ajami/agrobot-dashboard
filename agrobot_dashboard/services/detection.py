"""YOLOv8 person detection — shared GPU model, tuning constants and helpers.

One yolov8n model is loaded once onto the Jetson GPU (CUDA FP16) and shared
by the front and rear camera workers; inference is serialized under
YOLO_INFER_LOCK so the two feeds never fight over the GPU.
"""
import logging
import threading
from pathlib import Path

from agrobot_dashboard.services.events import log_event

log = logging.getLogger("dashboard")

REPO_ROOT = Path(__file__).resolve().parents[2]

# Person-detection (front/ZED feed), tuned for the Jetson Orin GPU.
# The CPU cost of detection is dominated by fixed per-call overhead (~18 ms/call),
# Detection runs entirely on the GPU (CUDA FP16). CPU is only used for the
# thin Python glue: queuing frames (numpy copy) and writing the JPEG + JSON.
# Model: yolov8n (nano) — 3-4× faster than small on the same GPU with
# negligible accuracy loss for nearby persons.
# imgsz=320: 4× fewer NMS candidates than 640, which eliminates the
# "NMS time limit exceeded" warning seen with the larger model+resolution.
# max_det=20: further caps NMS work (unlikely to see >20 people at once).
# At 10 Hz throttle and ~30-80 ms GPU inference, detection lag ≈ 100-200 ms.
YOLO_MODEL        = str(REPO_ROOT / 'models' / 'yolov8n.pt')   # nano: 3-4× faster than small, GPU FP16
YOLO_CONFIDENCE   = 0.5
YOLO_PERSON_CLASS = 0
YOLO_THROTTLE_HZ  = 10.0           # 10 Hz — achievable now that inference is fast
YOLO_IMGSZ        = 320            # 320: fast NMS, good enough for nearby persons
YOLO_MAX_DET      = 20             # cap NMS candidates
YOLO_DEVICE       = 0              # CUDA device index (GPU-only; warns loudly if unavailable)
YOLO_HALF         = True           # FP16 — halves GPU memory and speeds matmul


def load_yolo(label=""):
    """Load YOLO on the GPU (CUDA FP16), limit CPU threads, and warm up.

    Returns (model, device, half) on success, (None, 'cpu', False) on failure.
    torch.set_num_threads(1): all heavy math runs on the GPU; spawning one
    intra-op thread per CPU core only causes cache thrashing with no benefit.
    """
    try:
        import numpy as np
        import torch
        from ultralytics import YOLO
        torch.set_num_threads(1)
        if not torch.cuda.is_available():
            log.error("[yolo] CUDA unavailable — detection will NOT run. "
                      "Check that the Jetson's CUDA drivers are installed.")
            log_event("ERROR", "Camera",
                      "YOLO person detection disabled — CUDA GPU not available",
                      "Check Jetson GPU driver: nvidia-smi. "
                      "If unavailable, reinstall the Jetson CUDA toolkit. "
                      "Detection will not run until CUDA is available.",
                      _key="yolo-cuda", _debounce_s=3600)
            return None, "cpu", False
        dev, half = YOLO_DEVICE, YOLO_HALF
        model = YOLO(YOLO_MODEL)
        model.to(f'cuda:{dev}')   # pin model to GPU before warm-up
        model(np.zeros((YOLO_IMGSZ, YOLO_IMGSZ, 3), dtype=np.uint8),
              classes=[YOLO_PERSON_CLASS], imgsz=YOLO_IMGSZ, device=dev,
              half=half, max_det=YOLO_MAX_DET, verbose=False)
        gpu_name = torch.cuda.get_device_name(dev)
        log.info("[yolo] %s'%s' on GPU:%d (%s) FP16=%s imgsz=%d %.0f Hz — GPU-only confirmed",
                 f"{label} " if label else "", YOLO_MODEL, dev, gpu_name,
                 half, YOLO_IMGSZ, YOLO_THROTTLE_HZ)
        return model, dev, half
    except Exception as exc:
        log.warning("[yolo] model unavailable — %s", exc)
        log_event("WARN", "Camera",
                  f"YOLO model failed to load: {exc}",
                  f"Check that '{YOLO_MODEL}' exists in the project root. "
                  f"Download with: python3 -c \"from ultralytics import YOLO; YOLO('{YOLO_MODEL}')\"",
                  _key="yolo-model", _debounce_s=3600)
        return None, "cpu", False


# ---------------------------------------------------------------------------
# Shared YOLO singleton — loaded once; both front and rear ZED threads share it.
# Inference is serialized under YOLO_INFER_LOCK so neither camera stalls the
# GPU with concurrent calls (yolov8n is too small to benefit from parallelism).
# ---------------------------------------------------------------------------
YOLO_MODEL_OBJ      = None
YOLO_DEV        = 'cpu'
YOLO_HALF_ACTIVE       = False
YOLO_LOAD_LOCK  = threading.Lock()
YOLO_INFER_LOCK = threading.Lock()


def get_shared_yolo():
    """Return (model, dev, half), loading the singleton on first call."""
    global YOLO_MODEL_OBJ, YOLO_DEV, YOLO_HALF_ACTIVE
    with YOLO_LOAD_LOCK:
        if YOLO_MODEL_OBJ is None:
            YOLO_MODEL_OBJ, YOLO_DEV, YOLO_HALF_ACTIVE = load_yolo("(shared)")
    return YOLO_MODEL_OBJ, YOLO_DEV, YOLO_HALF_ACTIVE


def draw_box(img, x1, y1, x2, y2, label):
    """Draw a red bounding box + label on img in-place (used by YOLO worker and detection stream)."""
    import cv2
    cv2.rectangle(img, (x1, y1), (x2, y2), (0, 0, 220), 2)
    (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.55, 2)
    cv2.rectangle(img, (x1, y1 - th - 8), (x1 + tw + 6, y1), (0, 0, 220), -1)
    cv2.putText(img, label, (x1 + 3, y1 - 4),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)




def depth_at_bbox(depth_data, x1, y1, x2, y2, r=5):
    """Median depth (metres) in a small patch at the bounding-box centre.

    depth_data: float32 numpy array from ZED MEASURE.DEPTH (metres, NaN/inf for invalid).
    Returns None if no valid pixels are found.
    """
    import numpy as np
    cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
    h, w   = depth_data.shape[:2]
    patch  = depth_data[max(0, cy - r):min(h, cy + r + 1),
                        max(0, cx - r):min(w, cx + r + 1)]
    valid  = patch[np.isfinite(patch) & (patch > 0.1) & (patch < 40.0)]
    return float(np.median(valid)) if len(valid) > 0 else None
