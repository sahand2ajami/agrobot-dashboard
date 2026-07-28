# Person Detection — Setup & Reference

Real-time person detection for the Agrobot dashboard. Both the front and rear ZED 2i cameras detect persons and report their distance from the camera. Detection is **on-demand only** — the GPU runs inference only while the Det button is active in the browser; it idles completely when nobody is watching.

---

## What it does

- Detects people in both camera feeds simultaneously using **YOLOv8 nano (FP16)** on the Jetson GPU
- Overlays red bounding boxes on the video stream with confidence and distance (e.g. `person 87%  1.3 m`)
- Shows a live summary bar at the bottom of the camera view: **Front: 1 person  1.3 m | Rear: 1 person  2.0 m**
- Distance is measured using the ZED's built-in stereo depth sensor — no extra hardware required

---

## Hardware requirements

| Item | Details |
|------|---------|
| Jetson (any Orin/Xavier) | Runs Ubuntu 22.04 + JetPack |
| Front ZED 2i | USB 3.0 → Jetson, SDK index **0** |
| Rear ZED 2i  | USB 3.0 → Jetson, SDK index **1** |

Both cameras must be plugged in before starting the dashboard. The detection pipeline talks to the cameras directly through the ZED SDK — no ROS nodes are involved.

---

## Software requirements

### 1 — ZED SDK

Download and install from Stereolabs: https://www.stereolabs.com/developers/release

Choose the version that matches your JetPack. After installation verify:

```bash
python3 -c "import pyzed.sl as sl; print('ZED SDK ok')"
```

### 2 — Python packages

Install from the project root:

```bash
pip3 install -r requirements.txt
```

The key packages for detection are:

| Package | Version | What it does |
|---------|---------|--------------|
| `ultralytics` | ≥ 8.4 | YOLOv8 model loading and inference |
| `torch` | JetPack-bundled | GPU tensor compute (comes with JetPack, do **not** reinstall via pip) |
| `opencv-python` | ≥ 4.5 | Frame resize and JPEG encode |
| `numpy` | ≥ 1.21 | Array operations |

> **Important:** Do not `pip install torch` on a Jetson. The GPU-capable PyTorch comes pre-installed with JetPack. Installing from PyPI will replace it with a CPU-only build and detection will fail.

Verify CUDA is visible to PyTorch:

```bash
python3 -c "import torch; print(torch.cuda.is_available())"
# Expected output: True
```

### 3 — YOLO model file

The model file `yolov8n.pt` must be in the project root directory (the repo root, e.g. `/home/jetson/agrobot/`). It is already present in this repo. If it is missing, download it:

```bash
cd "$(git rev-parse --show-toplevel)"   # the project root
python3 -c "from ultralytics import YOLO; YOLO('yolov8n.pt')"
```

This downloads ~6 MB from Ultralytics automatically on first run. A higher-accuracy (but slower) alternative, `yolov8s.pt`, is also available in the repo — see [Tuning](#tuning) below.

---

## How to run

Detection is built into the dashboard server — there is no separate process to start. Just launch the dashboard normally:

```bash
./launch_dashboard_plc.sh
```

or

```bash
./launch_dashboard.sh
```

Then open the dashboard in a browser and click the **Det** button in the camera panel. The button lights up green when active. Detection starts streaming within 1–2 seconds (the model takes a moment to warm up on first use after boot).

---

## What happens under the hood (step by step)

1. **Dashboard starts** — `serve.py` launches two background threads, one per ZED camera, that continuously grab frames from the ZED SDK.

2. **Det button clicked** — the browser opens a streaming connection to `/api/detection/stream` (front) and `/api/detection/rear_stream` (rear). The server sees these requests and marks detection as "wanted."

3. **YOLO model loads** — on the first detection request after boot, the shared YOLOv8 model is loaded onto the GPU and warmed up with a dummy frame. This happens once and takes ~2–5 seconds. Subsequent clicks are instant.

4. **Inference loop** — each camera thread queues the latest frame (up to 10 times/second) for its YOLO worker thread. The two workers share a single GPU lock (`_YOLO_INFER_LOCK`) so they take turns — front infers, then rear, then front again. This prevents GPU memory contention.

5. **Depth estimation** — for each detected person, the ZED depth map is sampled at the bounding box centre. A 5×5 pixel patch is taken and the median of valid (non-NaN, 0.1–40 m) depth values is reported as the distance.

6. **Stream compositing** — the camera thread composites the latest YOLO boxes onto the live video frame at stream rate (20 fps). Inference and streaming are decoupled: the video is always smooth even if inference is slower.

7. **Det button clicked again** — the browser closes its streaming connections. After 3 seconds of no requests the server stops queuing frames for inference. The GPU goes back to idle.

---

## API endpoints

These are served by the dashboard on port 8769 (plc launcher) or 8766 (standard launcher).

| Endpoint | Description |
|----------|-------------|
| `GET /api/detection/stream` | Front camera MJPEG stream with bounding boxes |
| `GET /api/detection/data` | Front camera JSON: `{ts, count, detections:[{label, confidence, distance_m, bbox}]}` |
| `GET /api/detection/rear_stream` | Rear camera MJPEG stream with bounding boxes |
| `GET /api/detection/rear_data` | Rear camera JSON (same schema as above) |

You can open either stream URL directly in a browser tab to see the raw annotated feed without the dashboard UI.

---

## Tuning

All detection constants are at the top of `dashboard/serve.py` around line 1531:

```python
YOLO_MODEL        = 'yolov8n.pt'   # swap to 'yolov8s.pt' for better accuracy (slower)
YOLO_CONFIDENCE   = 0.5            # minimum score to show a box (0–1); raise to reduce false positives
YOLO_THROTTLE_HZ  = 10.0          # max inference rate per camera; lower saves GPU load
YOLO_IMGSZ        = 320            # input resolution; 640 is more accurate but ~4× slower
YOLO_MAX_DET      = 20             # max persons per frame
YOLO_HALF         = True           # FP16 — keep True on Jetson for best performance
YOLO_DEVICE       = 0              # GPU index; 0 is the only GPU on Jetson
```

**Common adjustments:**

- **Too many false positives** → raise `YOLO_CONFIDENCE` (e.g. 0.6 or 0.7)
- **Missing people at a distance** → lower `YOLO_CONFIDENCE` (e.g. 0.35) or increase `YOLO_IMGSZ` to 640
- **Better accuracy needed** → change `YOLO_MODEL` to `'yolov8s.pt'` (small; already in repo)
- **Too much GPU load** → lower `YOLO_THROTTLE_HZ` (e.g. 5.0)

After editing, restart the dashboard for changes to take effect.

---

## Troubleshooting

**Det button does nothing / stream shows blank**

Check the dashboard terminal log for `[yolo]` lines. Common causes:

- *`CUDA unavailable`* — PyTorch was installed from PyPI and has no GPU support. Reinstall the JetPack-bundled version.
- *`model unavailable`* — `yolov8n.pt` is missing from the project root. Download it (see above).
- *`pyzed not installed`* — ZED SDK Python bindings are missing. Reinstall the ZED SDK.

**Boxes appear but no distance shown**

The depth map returned `NaN` or out-of-range values for that person. This happens when:
- The person is closer than ~0.3 m or farther than 40 m
- The person is at the edge of the camera's stereo baseline (very wide angle)
- The surface has low texture (depth stereo needs contrast to match patches)

This is normal — the label shows confidence only when depth is unavailable.

**Rear camera shows boxes but no distance**

The rear ZED is opened with `DEPTH_MODE.PERFORMANCE`. If pyzed reports an error opening the rear camera in performance mode, check the ZED SDK version — versions older than 4.0 may require `DEPTH_MODE.ULTRA` or a different API. The log line `[rear-cam] ZED 2i rear (SDK index 1) opened` confirms a successful open.

**Camera feed goes blank after toggling Det off**

Refresh the browser tab. If it happens consistently, check whether another process has grabbed exclusive access to the ZED camera (e.g. a stray `python3 object_detector.py` process). Kill it:

```bash
pkill -f object_detector.py
```

**High CPU or GPU temperature**

Lower `YOLO_THROTTLE_HZ`. At 10 Hz each camera does up to 10 inferences/second — the two cameras are serialized so the GPU never runs two at once, but 10 Hz is already near the thermal limit on an uncooled Jetson. 5 Hz is a safe default for field use.
