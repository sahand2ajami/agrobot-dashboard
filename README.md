# Dual-Robot Teleoperation Dashboard

One web dashboard, **two robot chassis**. The same teleop UI, cameras, GNSS map,
YOLOv8 person detection, planting tools, and session recording work with either
robot — only the chassis communication layer changes, selected by configuration.

| Chassis | Robot | How the dashboard reaches the base |
|---------|-------|------------------------------------|
| **`agrobot`**   | Agrobot differential-drive tree-planting robot | **Modbus RTU** over serial (`/dev/ttyUSB0`, 38400 baud). The dashboard publishes wheel speeds (`Int16MultiArray`) on `/avatar_robot/speed_cmd`; the `robot_base_node` ROS node performs the Modbus writes and reports wheel odometry / battery / oil / error. |
| **`jackal`** | Clearpath Jackal UGV | **ROS 2 (DDS) over a LAN cable**. The dashboard publishes `geometry_msgs/Twist` straight to `/jackal1/cmd_vel`; the Jackal's onboard stack drives the motors and publishes its camera. |

This project merges the former `agrobot_robot` and `jackal_teleop` codebases into one.
The original repositories are left untouched; read-only clones live under
[reference/](reference/) for traceability and are **not** part of the running system.

> **Runtime target:** Ubuntu 22.04 + ROS 2 Humble (typically a Jetson on the robot).
> Editing on Windows/OneDrive is fine, but there is no Windows runtime — the robot
> services (serial, DDS, cameras) run on the Linux machine.

---

## Table of contents

1. [Repository layout](#repository-layout)
2. [Install & build (one-time)](#install--build-one-time)
3. [Configuration](#configuration) — how chassis and options are selected
4. **[Tutorial 1 — Bring up the dashboard](#tutorial-1--bring-up-the-dashboard)** (agrobot, jackal, wide, rear camera, headless / remote access)
5. **[Tutorial 2 — Using the dashboard](#tutorial-2--using-the-dashboard)** (driving, cameras, GPS, recording, planting, status)
6. [Configuration cookbook](#configuration-cookbook) — copy-paste commands per scenario
7. [HTTP API](#http-api)
8. [Architecture](#architecture)
9. [Adding a third chassis](#adding-a-third-chassis)
10. [Tests](#tests)
11. [Logs](#logs)
12. [Troubleshooting](#troubleshooting)

---

## Repository layout

```
agrobot_dual_robot/
├── config/
│   ├── active_chassis.yaml         # persisted default chassis  ( chassis: agrobot )
│   └── chassis/
│       ├── agrobot.yaml               # Modbus chassis: limits, scaling, topics, serial, battery, features
│       └── jackal.yaml             # ROS-Twist chassis: limits, topics, LAN network, features
├── dashboard/
│   ├── serve.py                    # robot-agnostic HTTP server (port 8766) + ROS bridge
│   ├── chassis.py                  # loads the active chassis, builds its ROS publisher/subscriptions
│   ├── index.html                  # the single-page dashboard UI (adapts via /api/config)
│   └── index_wide.html             # wide-angle UI variant (no crop, ZED HD2K)
├── scripts/
│   ├── gnss_p9_read.py             # Columbus P-9 Race GNSS reader → /tmp/gnss_coords.json
│   └── object_detector.py          # standalone YOLOv8 detector (optional; detection now runs in the dashboard, on-demand)
├── src/avatar_robot_base/          # agrobot ROS 2 package: Modbus driver + odometry (T3/T13/T17E)
├── launch_dashboard.sh             # unified launcher (chassis-aware)
├── launch_dashboard_wide.sh        # wide-UI launcher
├── start_all.sh / start.sh / teleop.sh   # agrobot-only ROS dev helpers
├── tests/                          # pytest suite (no ROS / no hardware required)
├── requirements.txt                # Python deps (ROS msgs come from apt, not pip)
├── README.md                       # this file
└── DEVELOPMENT.md                       # developer / implementation guide
```

---

## Install & build (one-time)

On the robot's Ubuntu 22.04 + ROS 2 Humble machine:

```bash
# 1. ROS 2 Humble must already be installed (ros-humble-desktop or ros-base).
source /opt/ros/humble/setup.bash

# 2. Python dependencies for the dashboard, GNSS, and detector.
#    rclpy and ROS message packages come from apt (ros-humble-*), NEVER pip.
pip3 install -r requirements.txt

# 3. Build the agrobot ROS workspace (needed only for the agrobot chassis).
colcon build --symlink-install
source install/setup.bash
```

Hardware/driver prerequisites by chassis:

- **agrobot** — RealSense SDK + `ros-humble-realsense2-camera`, `ros-humble-rosbridge-suite`,
  a USB-serial adapter at `/dev/ttyUSB0` to the chassis, and (optional) the Columbus
  P-9 GNSS receiver. The YOLOv8 detector needs `ultralytics`.
- **jackal** — a LAN cable to the Jackal and permission to add an IP on the wired
  interface (`eno1`). No local base driver or RealSense is required.

You can run and test the dashboard **without** any of this hardware — the UI loads,
the API responds, and missing devices degrade gracefully (panels show "No data").

---

## Configuration

**Every run option at a glance.** All are optional — running `./launch_dashboard.sh`
with no flags uses every default (agrobot chassis, RealSense rear camera present, a local
browser on the Jetson, port 8766):

| Option | Values | Default | How to set it (highest priority first) |
|--------|--------|---------|----------------------------------------|
| **Chassis** | `agrobot` \| `jackal` | **`agrobot`** | `--chassis <name>` → `ROBOT_CHASSIS` env → `chassis:` in `config/active_chassis.yaml` |
| **Rear camera** | `realsense` \| `webcam` \| `none` | **`realsense`** (present) | `--rear-camera <v>` → `REAR_CAMERA` env → `rear_camera:` in the chassis YAML |
| **Headless** (don't open a browser on the Jetson) | enabled \| disabled | **disabled** (opens a local browser) | `--headless` / `--no-headless` → `DASHBOARD_HEADLESS=1` env |
| **Port** | any free TCP port | **`8766`** | `--port <n>` |

Each option is explained in detail below.

### Selecting the chassis

The active chassis is resolved in this order (highest priority first):

1. `--chassis <name>` passed to `launch_dashboard.sh` or `dashboard/serve.py`
2. the `ROBOT_CHASSIS` environment variable
3. the `chassis:` field in [config/active_chassis.yaml](config/active_chassis.yaml)
4. the built-in default (`agrobot`)

```bash
# Persisted default — edit config/active_chassis.yaml:
chassis: agrobot

# Per-launch override (does not edit the file):
./launch_dashboard.sh --chassis jackal
```

### Per-chassis config files

Everything chassis-specific lives in `config/chassis/<name>.yaml`. The dashboard and
UI never hard-code robot details — they read these files (the server) and `/api/config`
(the browser). Key fields:

| Field | Applies to | Meaning |
|-------|-----------|---------|
| `comms` | both | `modbus_speed` (agrobot) or `ros_twist` (jackal) — selects the publisher type. |
| `max_linear`, `max_angular` | both | Velocity ceilings accepted by the server (m/s, rad/s). |
| `speed_cmd_topic` | both | ROS topic the velocity command is published on. |
| `linear_scale`, `angular_scale`, `speed_max`, `pulse_per_m` | agrobot | Twist → wheel-speed scaling and encoder calibration. |
| `wheel_odom_topic` | agrobot | Subscribed for mileage / Chassis-Link status. |
| `battery_topic` | agrobot | `Float32` pack voltage from `robot_base_node`. |
| `battery_min_v`, `battery_max_v` | agrobot | Voltage range mapped to the 0–100 % battery gauge (default 42–58 V). |
| `camera_topic` | both | ROS camera image topic. |
| `rear_camera` | both | `realsense` or `webcam` — the rear camera source (see below). |
| `rear_camera_device` | both | Optional explicit V4L2 device/index for the webcam. |
| `serial_port`, `baud`, `slave_id` | agrobot | Modbus serial settings (recorded for reference). |
| `chassis_variant` | agrobot | `T3` \| `T13` \| `T17E` — wheel radius / axle / PPR for odometry (exported as `CAR_TYPE`). |
| `host_iface`, `host_ip`, `robot_ip`, `ros_domain_id` | jackal | LAN setup applied by the launcher. |
| `features` | both | Booleans that show/hide UI panels: `battery`, `oil`, `wheel_odom`, `fwd2m`, `modbus_slider`, `actuators`. |

### Rear camera (realsense | webcam | none)

The dashboard shows a rear camera in a picture-in-picture corner. You choose its
**source** — the RealSense D435 or a generic USB webcam (e.g. a Logitech) — or turn the
rear view **off** entirely. Resolution order: `--rear-camera` flag → `REAR_CAMERA` env
→ `rear_camera:` in the chassis YAML → `realsense`.

```bash
./launch_dashboard.sh --chassis agrobot --rear-camera webcam     # use a USB webcam
./launch_dashboard.sh --chassis agrobot --rear-camera none       # no rear view (front fills the frame)
REAR_CAMERA=webcam ./launch_dashboard.sh                       # same, via env
```

- **`realsense`** (default) — the RealSense D435 rear feed.
- **`webcam`** — opens a local USB UVC camera and **skips the ROS camera topic** so the
  two feeds don't fight over the buffer. Pin a specific device with
  `rear_camera_device: /dev/video2` if auto-detection picks the wrong one.
- **`none`** (aliases: `off`, `disabled`) — disables the rear view and its capture
  entirely; the front (ZED) camera fills the frame. Saves CPU when you only need the
  front view.

The front (ZED) camera is unaffected by this setting.

### Headless (don't open a browser on the Jetson)

By default the launcher opens a browser **on the Jetson** at `http://localhost:8766`.
In **headless** mode it skips that and only serves — you open the dashboard from your
laptop's browser instead. This removes the Jetson's browser-rendering cost (and, if you
use a remote desktop like NoMachine, its screen re-encoding too), leaving more CPU for
the camera and control loops.

```bash
./launch_dashboard.sh --headless          # serve only; don't open a local browser
./launch_dashboard.sh --no-headless       # force a local browser (the default)
export DASHBOARD_HEADLESS=1               # make headless the default (e.g. in ~/.bashrc)
```

Precedence: `--headless`/`--no-headless` flag → `DASHBOARD_HEADLESS` env → default
(open a browser). In headless mode the launcher prints every address the Jetson is
reachable on — open whichever shares a network with your laptop. See Tutorial 1 →
*Access from another device* for the full workflow.

---

## Tutorial 1 — Bring up the dashboard

The launcher starts exactly the services the chosen chassis needs, waits for the HTTP
server to accept connections, then (unless `--headless`) opens a browser at
`http://localhost:8766`.

### A. agrobot robot (Modbus)

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
./launch_dashboard.sh --chassis agrobot
```

This brings up, in order: RealSense D435i (color) → GNSS reader → `robot_base_node`
(Modbus RTU on `/dev/ttyUSB0`) → rosbridge → dashboard server. Person detection runs
**inside the dashboard** on the ZED front feed, on demand (see Tutorial 2 → Cameras).
You should see the **Chassis Link** card go green, **Chassis Battery** show a voltage,
and the rear camera appear once the RealSense initialises. Drive with **W A S D**.

> If the serial port needs permission, the launcher attempts `sudo chmod a+rw
> /dev/ttyUSB0`. If the chassis is on a different port, update `serial_port` in
> `agrobot.yaml` (and the `udev`/launch device list).

### B. Jackal (LAN)

1. Connect the LAN cable between the Jetson and the Jackal.
2. Launch:

```bash
./launch_dashboard.sh --chassis jackal
```

The launcher sets `ROS_DOMAIN_ID=0`, adds the host IP `192.168.1.100/24` on `eno1`
(so the Jetson can reach the Jackal at `192.168.1.200`), starts the GNSS reader, and
launches the dashboard. The Jackal supplies its own motors and camera over ROS, so
the agrobot-only panels (battery, oil, wheel odometry, auto-forward, Modbus slider) are
hidden automatically. Drive with **W A S D**; commands publish `Twist` to
`/jackal1/cmd_vel`.

> Confirm connectivity with `ping 192.168.1.200` and `ros2 topic list` before driving.
> `config/fastdds_jackal.xml` is available if you need unicast DDS on a field network.

### C. Wide-angle UI

Same services, but a no-crop layout and ZED HD2K resolution:

```bash
./launch_dashboard_wide.sh --chassis agrobot
./launch_dashboard_wide.sh --chassis jackal
```

### D. Choosing a port / running two at once

```bash
./launch_dashboard.sh --chassis agrobot  --port 8766
./launch_dashboard.sh --chassis jackal --port 8767   # second robot, second port
```

### E. Access from another device (and headless mode)

You can drive entirely from a laptop/tablet browser — the Jetson doesn't need to run a
browser at all. This is the recommended setup: it removes the Jetson's browser
rendering (and a remote desktop's screen re-encoding), leaving more CPU for the camera
and control loops.

```bash
./launch_dashboard.sh --chassis agrobot --headless
```

In headless mode the launcher lists every address the Jetson is reachable on, e.g.:

```
Open the dashboard from your laptop's browser at one of these:
  →  http://192.168.1.100:8766      (wired LAN)
  →  http://10.111.38.27:8766       (WiFi)
  →  http://100.x.x.x:8766          (Tailscale, if configured — reachable anywhere)
```

Open whichever address shares a network with your laptop; the full UI (driving,
cameras, map) works in any modern browser. Without `--headless`, the launcher still
prints a `Network → …` URL you can use the same way while a local browser is also open.

### F. Stopping

Press **Ctrl-C** in the launcher terminal. It tears down every service it started
(camera node, GNSS, detector, `robot_base_node`, rosbridge, server) cleanly.

### agrobot-only developer helpers

These exit immediately if the active chassis is `jackal`:

| Script | Purpose |
|--------|---------|
| `./start_all.sh` | agrobot ROS chassis stack + RViz (no web UI) |
| `./start.sh T3\|T13\|T17E` | chassis + RViz for a specific URDF variant |
| `./teleop.sh` | Modbus driver + terminal WASD teleop |

---

## Tutorial 2 — Using the dashboard

The screen has a **live camera view** (centre), a **GPS map** (right), a **drive/keys
panel** with action buttons, and a **settings** gear (top-right). What appears depends
on the active chassis — the UI hides panels a chassis doesn't support.

### Driving

- **W / A / S / D** drive forward / left / back / right. Releasing all keys stops the
  robot. The on-screen key tiles also work with mouse/touch (press and hold).
- A 50 ms command loop streams velocity while a key is held; the server enforces a
  0.5 s deadman stop if the browser goes quiet.
- **Speed presets** — `Slow`, `Normal` (default), `Fast` — set the maximum speed the
  keys command. Pick a preset before driving in tight spaces.
- Commanded velocity is clamped to the chassis's `max_linear`/`max_angular`, so you
  can never exceed what the server will accept.

### Cameras

- The view is a **split**: the **front** camera (ZED) fills the frame and the **rear**
  camera (RealSense or webcam) sits in a picture-in-picture corner. **Click the PiP**
  to swap which feed is large. (With `--rear-camera none` there is no PiP — the front
  fills the frame.)
- **Detection** button — overlays YOLOv8 person detection on the front (ZED) feed.
  Detection runs **on demand**: YOLO only does work while this view is open, so it
  costs nothing when off. Toggling it never touches the chassis link, GPS, or driving —
  WASD stays fully responsive.
- The live feed streams at 20 fps (recordings are saved at 15 fps).

### GPS map & recording

- The map follows the robot. **Center on robot** re-centres, **+ / −** zoom, and the
  compass button toggles 2D/3D.
- **REC** starts a recording session:
  - Front + rear camera are saved at **15 fps** (the live view stays 20 fps).
  - A blue marker stays at the screen centre and the map pans under it, drawing the
    GPS trace as the robot moves.
  - Press **STOP** to end: recording stops, the live trace is released, and the camera
    videos + the GPS path are written to `logs/recordings/<YYYY-MM-DD_HH-MM-SS>/`.

### Planting tools (agrobot, and Jackal if `actuators` is enabled)

- **Planter** — momentary: it releases itself on press, increments the **Planted**
  counter, drops a **red pin** at the current GPS fix, and appends the geo-location to
  `logs/planted_seedlings/seedlings.jsonl` (a GPS fix is required).
- **Auger** — latches on/off (press again to stop).
- **Both** — momentary, fires planter + auger together.

### Auto-forward 2 m (agrobot only)

The **2 m** button drives the robot forward exactly two metres using a server-side
encoder loop (the stop decision is on the server, not the browser, so speed doesn't
affect accuracy). Hidden on chassis without the `fwd2m` feature.

### Status & telemetry

- **System Status** cards: RealSense (Rear), ZED 2i (Front), GPS Receiver, **Chassis
  Link** (wheel-odom heartbeat), and **Chassis Battery** (pack voltage → % over the
  configured `battery_min_v`/`battery_max_v` range). Cards a chassis doesn't provide
  are hidden.
- **Satellite Navigation**: fix type, satellites (used / in view), and HDOP.
- **Distance Traveled**: mileage from wheel encoders (agrobot).

### Settings (gear icon)

Adjust the max linear/angular speed and preset mapping. On agrobot, the **Modbus master
slider** lets you command raw Modbus speed units directly; its scale mirrors
`linear_scale` in `agrobot.yaml`. Click **Apply** to persist.

---

## Configuration cookbook

| I want to… | Command |
|------------|---------|
| Run the persisted default chassis | `./launch_dashboard.sh` |
| Run agrobot explicitly | `./launch_dashboard.sh --chassis agrobot` |
| Run the Jackal | `./launch_dashboard.sh --chassis jackal` |
| Use a USB webcam as the rear camera | `./launch_dashboard.sh --chassis agrobot --rear-camera webcam` |
| Turn off the rear camera (front only) | `./launch_dashboard.sh --rear-camera none` |
| Pin the webcam device | set `rear_camera_device: /dev/video2` in `agrobot.yaml` |
| Change the default chassis permanently | edit `chassis:` in `config/active_chassis.yaml` |
| Run headless (drive from your laptop's browser) | `./launch_dashboard.sh --headless` |
| Make headless the default | `export DASHBOARD_HEADLESS=1` (e.g. in `~/.bashrc`) |
| Run on a different port | `./launch_dashboard.sh --chassis agrobot --port 8080` |
| Run two robots side by side | launch each chassis on its own `--port` |
| Use the wide-angle UI | `./launch_dashboard_wide.sh --chassis agrobot` |
| Pick the agrobot chassis URDF variant | set `chassis_variant: T13` (or `T3`/`T17E`) in `agrobot.yaml` |
| Adjust the Jackal's network/domain | edit `host_ip` / `robot_ip` / `ros_domain_id` in `jackal.yaml` |
| Re-calibrate the battery gauge | set `battery_min_v` / `battery_max_v` in `agrobot.yaml` |
| Hide/show a UI panel | flip the matching flag under `features:` in the chassis YAML |

---

## HTTP API

Server: `dashboard/serve.py`, default port **8766**.

| Method | Path | Notes |
|--------|------|-------|
| GET | `/api/config` | Active chassis name, comms, feature flags, velocity limits, scaling, rear-camera source, battery gauge range. The UI adapts to this. |
| POST | `/api/cmd_vel` | `{linear_x, angular_z}`. 400 if it exceeds the active chassis's `max_linear`/`max_angular`. |
| GET | `/api/chassis_battery` | `{voltage_v, connected}` — median-smoothed chassis pack voltage (agrobot). |
| GET | `/api/wheel_odom` | `{left, right, mileage, connected}` encoder data (agrobot). |
| GET | `/api/gnss` | Current GNSS fix / heartbeat (connection + fix state, satellites, HDOP). |
| POST | `/api/fwd2m` | Server-side 2 m auto-drive. 503 on chassis without the `fwd2m` feature. |
| POST | `/api/plant` | Logs a planting event + geo-location to `logs/planted_seedlings/`. |
| POST | `/api/record/start`, `/api/record/stop` | Start/stop a camera + GPS-track recording session. |
| GET | `/api/camera*`, `/api/zed*`, `/api/detection*` | MJPEG streams / status / detection data. |
| GET/POST | `/api/settings` | Read / write speed and Modbus settings. |

---

## Architecture

```
                         Browser (dashboard/index.html)
                         │  HTTP :8766  (cmd_vel, camera, gnss, detection, config, settings)
                         ▼
                 dashboard/serve.py            ← robot-agnostic HTTP server
                         │  delegates ROS wiring to:
                         ▼
                 dashboard/chassis.py          ← reads config/chassis/<active>.yaml
                  ┌──────┴───────────────────────────────┐
        comms = modbus_speed                      comms = ros_twist
                  │                                       │
   Int16MultiArray│ /avatar_robot/speed_cmd      Twist    │ /jackal1/cmd_vel
                  ▼                                       ▼
        robot_base_node (Modbus RTU)            Jackal onboard stack (LAN / DDS)
        + wheel_odom / battery / oil
```

The browser fetches `GET /api/config` on load, hides any element tagged
`data-chassis-feature="X"` when `features.X` is false, and clamps commanded velocity
to the chassis limits. See [DEVELOPMENT.md](DEVELOPMENT.md) for the full developer guide.

---

## Adding a third chassis

1. Copy an existing `config/chassis/<name>.yaml`; set `comms`, limits, topics, and
   `features`.
2. If it needs a new comms type, extend `Chassis.setup_ros` in
   [dashboard/chassis.py](dashboard/chassis.py).
3. Add a branch in [launch_dashboard.sh](launch_dashboard.sh) for its service set.

No `index.html` changes are needed unless the chassis introduces a brand-new UI panel.

---

## Tests

```bash
pytest tests/
```

All tests run **without ROS or hardware**: motor math, encoder sign-extension, NMEA
parsing, HTTP endpoint behaviour, the dual-chassis config/limits/feature logic, the
rear-camera resolution, and the chassis-battery endpoint.

---

## Logs

| Path | Contents |
|------|----------|
| `logs/dashboard/{ts}_{chassis}_dashboard.log` | Combined stdout/stderr from a launch |
| `logs/gnss/{ts}_gnss.jsonl` | One GPS fix per line |
| `logs/planted_seedlings/seedlings.jsonl` | One planting event per line (index, ts, lat/lon, fix, sats) |
| `logs/recordings/{ts}/` | Front/rear `.mp4` + `gnss.jsonl` for a recording session |
| `/tmp/gnss_coords.json` | Latest GPS fix (volatile, polled by the server) |
| `/tmp/object_detections.json` | Latest detections (volatile) |

---

## Troubleshooting

| Symptom | Check |
|---------|-------|
| Dashboard loads but driving does nothing | Is ROS sourced? For agrobot, is `robot_base_node` running and `/dev/ttyUSB0` accessible? For jackal, can you `ping 192.168.1.200` and see `/jackal1/cmd_vel` in `ros2 topic list`? |
| **Chassis Link** stays red (agrobot) | `robot_base_node` not publishing `/avatar_robot/wheel_odom` — check the serial port and the launch log. |
| **Chassis Battery** shows "No data" | No `Float32` on `/avatar_robot/battery` yet, or voltage outside the 30–70 V validity window. Confirm the chassis is powered and `battery_topic` is set in `agrobot.yaml`. |
| Rear camera blank | Wrong source/device — try `--rear-camera webcam` or set `rear_camera_device`. RealSense needs the camera node up first. |
| GPS map shows "No Data" | Plug in the Columbus P-9 receiver; the GNSS reader auto-detects `/dev/ttyACM*` / `/dev/ttyUSB1`. |
| Port already in use | Launch with a different `--port`. |
| agrobot helper script "refuses to run" | `start_all.sh` / `start.sh` / `teleop.sh` are agrobot-only; the active chassis is `jackal`. |

For implementation details (the chassis abstraction, Modbus register map, deadman
timeouts, encoder sign-extension, recording internals), see **[DEVELOPMENT.md](DEVELOPMENT.md)**.
