# DEVELOPMENT.md — Dual-Robot Dashboard Developer Guide

## What this project is

A single ROS 2 (Humble) teleoperation platform that drives **two different
chassis** from one web dashboard, selected by configuration:

- **`agrobot`** — Agrobot differential-drive tree-planting robot, **Modbus RTU**
  over serial (`/dev/ttyUSB0`, 38400 baud) via a separate `robot_base_node`.
- **`jackal`** — Clearpath Jackal UGV, **ROS 2 (DDS) over a LAN cable**; the
  dashboard publishes `geometry_msgs/Twist` to `/jackal1/cmd_vel`.

It merges the former `agrobot_robot` and `jackal_teleop` repos. Those repos are left
untouched; read-only clones live under `reference/` (gitignored) for traceability.
The agrobot codebase was the superset and became the base; jackal is the
feature-reduced configuration.

Runs on Ubuntu 22.04 + ROS 2 Humble (Jetson). Windows/OneDrive is only used for
editing — there is no Windows runtime.

---

## Build & test

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install     # builds src/avatar_robot_base (agrobot chassis)
source install/setup.bash

pytest tests/                      # no hardware / no ROS required
```

---

## The chassis abstraction (the heart of the merge)

`dashboard/serve.py` and `dashboard/index.html` are robot-agnostic. Everything
chassis-specific is isolated in:

| File | Role |
|------|------|
| `config/active_chassis.yaml` | Selects the default chassis (`chassis: agrobot`). |
| `config/chassis/<name>.yaml` | Per-chassis params: comms type, velocity limits, Twist→wheel scaling, ROS topics, network (jackal), serial/variant (agrobot), and UI feature flags. |
| `dashboard/chassis.py` | Loads the active chassis and builds its ROS velocity publisher / feedback subscriptions. Pure-Python and import-safe without ROS (ROS msgs are imported lazily inside `setup_ros`). |

**Selection order:** `--chassis` flag → `$ROBOT_CHASSIS` → `active_chassis.yaml`
→ default `agrobot`. Implemented by `chassis.resolve_name()`.

**How `serve.py` uses it** (in `main()`):

```python
CHASSIS = chassis.load_active(args.chassis)   # --chassis > env > active file
Handler.chassis = CHASSIS                      # HTTP handlers read limits/features here
publish_velocity = CHASSIS.setup_ros(_node, Handler)   # builds publisher (+ wheel_odom sub)
# the 20 Hz velocity timer just calls publish_velocity(lx, az)
# the camera subscription uses CHASSIS.camera_topic
```

`setup_ros` returns a `publish_velocity(linear_x, angular_z)` closure:
- `comms: ros_twist` → publishes `geometry_msgs/Twist`.
- `comms: modbus_speed` → converts via `twist_to_wheel_speeds` → `Int16MultiArray`.

`twist_to_wheel_speeds` also still exists as a module-level function in `serve.py`
(unchanged agrobot defaults: `LINEAR_SCALE=3000`, `ANGULAR_SCALE=1000`,
`SPEED_MAX=32767`) because `tests/test_robot_base.py` imports it directly. The
agrobot `Chassis` carries an identical copy sourced from `agrobot.yaml`.

### Adding a third chassis

1. Add `config/chassis/<name>.yaml` (copy an existing one; set `comms`, limits,
   topics, `features`).
2. If it needs a new comms type, extend `Chassis.setup_ros` in `chassis.py`.
3. Add a branch in `launch_dashboard.sh` for its service set.
No changes to `index.html` are needed unless it introduces a new UI panel.

---

## HTTP API (dashboard/serve.py, port 8766)

Unchanged from the agrobot dashboard, plus one new endpoint:

| Method | Path | Notes |
|--------|------|-------|
| GET | `/api/config` | **New.** Active chassis name, comms, feature flags, limits, scaling, rear-camera source, and the battery gauge range (`battery.minV`/`maxV`) — the UI adapts to this. |
| GET | `/api/chassis_battery` | **New.** `{voltage_v, connected}` — median-smoothed chassis pack voltage. Reports `0 / false` on chassis without the `battery` feature (jackal). |
| POST | `/api/cmd_vel` | `{linear_x, angular_z}`. Rejected with 400 if it exceeds the **active chassis's** `max_linear`/`max_angular` (falls back to module `MAX_*` when `Handler.chassis` is unset, e.g. in tests). |
| POST | `/api/fwd2m` | Server-side 2 m auto-drive. Returns 503 on a chassis whose `fwd2m` feature is false (jackal). |
| GET | `/api/wheel_odom`, `/api/gnss`, `/api/settings` | As before. |
| GET | `/api/camera/stream` | Rear camera MJPEG (raw). |
| GET | `/api/zed/stream` | Front camera MJPEG (raw). |
| GET | `/api/detection/stream`, `/api/detection/data` | Front camera with YOLO boxes / front detection JSON `{ts, count, detections:[{label, confidence, distance_m, bbox}]}`. |
| GET | `/api/detection/rear_stream`, `/api/detection/rear_data` | Rear camera with YOLO boxes / rear detection JSON (same schema). |
| POST | `/api/plc/{auger,planter,both}` | **New (PLC).** `{command: START\|STOP}` → Modbus pulse of the auger/planter pushbutton word(s). |
| POST | `/api/plc/machine` | **New (PLC).** `{command}` (SET_AUTO, ENABLE_*, HOME_ALL, FAULT_RESET…) → `MachineCommand`. |
| POST | `/api/plc/robot` | **New (PLC).** `{command}` (HOME, START, STOP, PAUSE, MOTORS_ON…) → `ControlRobot`. |
| GET | `/api/plc/{status,sequence,auger_motor}` | **New (PLC).** `GetMachineStatus` / `GetSequenceDetail` / `GetAugerMotorStatus`. |
| GET | `/api/plc/tags` | **New (PLC).** Static tag/register reference for the UI's **PLC Reference** panel: the curated read/write/reserved tag map (`plc_client.PLC_TAG_MAP`) + the full PLC symbol table (read from `docs/plc/GTS_Tree_Planter_symbols.csv`), each symbol annotated with its integration role. Makes no gateway call (works gateway-down); 503 on a chassis without a PLC. |

> All `/api/plc/*` routes return **503** on a chassis without `plc.enabled` (jackal). They
> never 5xx on a downed gateway — the response is a normal 200 with `connected:false`, so
> the UI shows "Gateway offline". Write commands are validated against an allow-list
> (`plc_client.{SEQUENCE,MACHINE,ROBOT}_COMMANDS`) → unknown commands are **400**.

### Adaptive UI

`index.html` calls `GET /api/config` on load (`_applyChassisConfig`) and:
- hides any element tagged `data-chassis-feature="X"` when `features.X === false`;
- clamps commanded velocity to `limits.maxLinear`/`maxAngular` in `publishCmdVel`.

Gated elements: Modbus master slider (`modbus_slider`), auto-forward-2 m button
(`fwd2m`), Distance-Traveled + Chassis-Link cards (`wheel_odom`), the Chassis-Battery
status card (`battery`), the planter/auger buttons + Planted counter (`actuators`), and
the PLC panels — machine-status strip, Machine-Setup + Robot-Arm sections, and the
header **PLC Reference** button + panel (`plc`).

> `plc` is a **derived** feature flag: `to_browser_config()` sets `features.plc` from the
> chassis's `plc.enabled`, so the same hide mechanism gates the PLC UI. It is independent
> of `actuators` — jackal keeps its cosmetic planter/auger buttons (`actuators:true`) but
> has no PLC (`plc.enabled:false`), so the buttons stay local-only there.

---

## agrobot chassis specifics (preserved from agrobot_robot)

`robot_base_node` is kept essentially **as-is**; the one change adopted from the
upstream `battery-2` branch is the corrected sensor register layout (see below).

### Modbus driver — `src/avatar_robot_base/avatar_robot_base/robot_base_node.py`
Subscribes `/avatar_robot/speed_cmd` (`Int16MultiArray`), writes speeds and reads
sensors over Modbus RTU, publishes `/avatar_robot/{battery,wheel_odom,error,oil}`.
1.5 s deadman timeout. Waits 50 ms before transmitting to avoid colliding with the
chassis's internal 100 ms Modbus cycle — **do not remove this sleep**.

**Sensor register layout** (`0x0019..0x001F`, corrected from `battery-2` by passive
bus capture): `[0] odom_L hi, [1] odom_L lo, [2] odom_R hi, [3] odom_R lo,
[4] battery×100 (V), [5] fault code, [6] oil %`. Battery is therefore `regs[4]/100.0`
(not the old `regs[5]/10.0`), and the odom registers shifted by one — so this fix
also corrects mileage. A missed speed-ACK now flushes the input buffer and proceeds
to the sensor read instead of `continue`-ing, so battery/odom is never starved.

### Encoder sign-extension
Two unsigned 16-bit registers → unsigned 32-bit → sign-extended to int32:
`count = raw if raw < 0x80000000 else raw - 0x100000000`. Tested in
`tests/test_robot_base.py::TestOdomSignExtension` (formula-only; independent of the
register indices above).

### Chassis battery
`robot_base_node` publishes pack voltage on `/avatar_robot/battery` (`Float32`, V).
The dashboard subscribes (in `chassis.setup_ros`, gated by the `battery` feature +
`battery_topic`), smooths it — readings accumulate in a 15 s window, outliers
(≤0 V or outside 30–70 V) are dropped, and the **median is recomputed every 10 s** —
and exposes it at `GET /api/chassis_battery`. The UI's "Chassis Battery" status card
polls it (2 s) and maps voltage to a 0–100 % gauge over `battery_min_v`/`battery_max_v`
from `agrobot.yaml` (defaults 42–58 V, a ~48 V / 14S pack), surfaced via `/api/config`.
Odom accumulation also gained outlier rejection (`MAX_ODOM_DELTA = 3000`) and a
mileage reset on >5 s reconnect, matching `battery-2`. Jackal hides the card (no
`battery_topic`, `battery: false`).

> The upstream `battery-2` branch also carries a standalone ANT-BMS-V2 reader
> (`scripts/battery_bms_read.py`) writing `/tmp/battery_bms.json`. It is **not** wired
> into the dashboard there, so it was not ported; wire it to a new endpoint if richer
> BMS telemetry (per-cell, current, SoC) is needed later.

### Speed scaling
`motor_units = linear_x * 3000`, `differential = angular_z * 1000`,
`left = units - diff`, `right = units + diff`, clamped to ±32767. The Modbus
master slider (`MODBUS_LINEAR_SCALE = 3000` in `index.html`) must match
`linear_scale` in `agrobot.yaml`.

### Auto-forward 2 m
`/api/fwd2m` runs a 50 Hz server-side encoder loop (`PULSE_PER_M = 3211`,
two-phase with a slow final 0.5 m). The stop decision is on the server, never the
browser.

### Deadman timeouts
`robot_base_node`: 1.5 s. `serve.py` velocity timer: 0.5 s (`VEL_TIMEOUT`). Both
independent.

### Chassis variants (T3 / T13 / T17E)
`odom_calculation.py` holds the per-variant wheel radius / axle width / encoder
PPR, selected by `CAR_TYPE`. Set via `chassis_variant` in `agrobot.yaml`
(the launcher exports `CAR_TYPE`). Only relevant when running odometry/RViz; the
dashboard path uses `robot_base_node` directly.

---

## jackal chassis specifics

- Publishes `geometry_msgs/Twist` to `/jackal1/cmd_vel`; camera subscribed from
  `/jackal1/sensors/camera_0/color/image`.
- `launch_dashboard.sh` sets `ROS_DOMAIN_ID` (0) and adds the Jetson's IP
  (`192.168.1.100/24`) on `eno1` to reach the Jackal at `192.168.1.200`.
- No `robot_base_node`, no wheel odometry / battery to the dashboard — those
  panels are hidden via feature flags.
- `config/fastdds_jackal.xml` is available for unicast DDS on field networks.

---

## Shared subsystems (both chassis)

- **GNSS** — `scripts/gnss_rtu608bt_read.py` (GeoAstra RTU608BT — GPS+BeiDou+GLONASS+Galileo,
  38400 baud, USB or Bluetooth). Auto-detects port (`/dev/gnss` symlink preferred, then
  `/dev/ttyUSB*`, `/dev/ttyACM*`, `/dev/rfcomm*`). For BT: pair device (name "Geoastra",
  PIN 1234) then `sudo rfcomm bind 0 <MAC>`. Writes `/tmp/gnss_coords.json` atomically;
  polled by `serve.py`.
- **Cameras** — `serve.py` opens both ZED 2i cameras directly via the pyzed SDK
  (front index 0, rear index 1) with `DEPTH_MODE.PERFORMANCE` on both. MJPEG is
  streamed to the UI at 20 fps. The chassis ROS camera topic is only used as a
  fallback when pyzed is unavailable.
- **YOLOv8 person detection** — runs **inside `serve.py`**, on demand, on **both**
  ZED feeds simultaneously. A shared `yolov8n.pt` singleton is loaded once onto the
  GPU (FP16) and serialised under `_YOLO_INFER_LOCK` so front and rear workers take
  turns without GPU contention. Detection is active only while the browser's Det view
  is open (`DET_IDLE_TIMEOUT = 3.0 s`). Each detected person gets a confidence score
  and a depth-derived distance from the ZED stereo depth map. The status bar shows
  **Front: N persons  X m | Rear: N persons  Y m**. See [detection.md](detection.md)
  for full setup and tuning. `scripts/object_detector.py` is a legacy ROS node that
  is no longer launched — detection now lives entirely in the dashboard.
- **Recording / track / plant logging** — under `logs/`.

---

## PLC integration (agrobot tree-planter)

The auger, planter, and robot manipulator are owned by an **LS Electric PLC**. The dashboard
talks to it **directly over Modbus TCP** — `serve.py` is the Modbus client and relays the
browser's button presses as REST:

```
Browser ──REST──► serve.py ──Modbus TCP──► LS Electric PLC
        :8766              :502 (192.168.1.2, CPU Ethernet port on the LAN)
```

The PLC is reached over the LAN cable: the Jetson's `eno1` carries an address on the PLC's
`192.168.1.0/24` subnet (`192.168.1.100/24`, set in `agrobot.yaml`'s `host_ip` + persisted on
`eno1`); the PLC CPU's Ethernet port at `192.168.1.2` serves Modbus TCP on 502 (the FEnet
card at `.1` speaks LS's own protocol — 502 is NOT served there). The dashboard connects
lazily and degrades gracefully ("PLC offline", a normal 200 with `connected:false`) when the
PLC is unreachable — startup never blocks on it.

> **History — the gRPC gateway was removed.** This used to route through a separate gRPC
> gateway process (`~/plc_gateway/gRPC-Gateway-Agrobot`, port 50051): `serve.py ─gRPC─► gateway
> ─Modbus─► PLC`. The gateway ran on the same Jetson serving only this dashboard, so the gRPC
> hop was pure overhead — plus its stubs needed protobuf 6.x and wouldn't boot under the
> Jetson's 4.25.x. `plc_client.py` now embeds the gateway's register map + command bit-values
> + pulse pattern and speaks Modbus itself. The old gateway repo, `dashboard/plc/` (vendored
> gRPC stubs), and `scripts/mock_plc_gateway.py` are **no longer used** by the dashboard.

**Testing without the real PLC:** point `plc.host` in `agrobot.yaml` at any Modbus-TCP server
that exposes the `%MW`/`%MX` registers in `plc_client._REG` — e.g. a `pymodbus` simulator on
the Jetson, or LS Electric's **XG5000** simulator (Windows, highest fidelity — runs the real
ladder). There is no longer a Python gRPC emulator; the old `mock_plc_gateway.py` is obsolete.

| File | Role |
|------|------|
| `dashboard/plc_client.py` | `PlcClient` — thread-safe **Modbus TCP** client (`pymodbus`, lazy-imported so it's import-safe / `pytest`-safe without it). One method per operation, returns plain dicts with `connected`/`success`/`message`, short socket timeout, auto-reconnect on error. Holds the ported register map (`_REG`) + command bit tables (`_MACHINE_CMD_MAP`/`_ROBOT_CMD_MAP`) + pulse helper. Also exports the command allow-lists and `PLC_TAG_MAP` + `symbol_roles()` (the read/write/reserved tag→register reference served at `/api/plc/tags`). |
| `dashboard/serve.py` | 8 `/api/plc/*` routes → `Handler.plc.*` (built in `main()` when `chassis.plc_enabled`). |
| `config/chassis/*.yaml` | `plc: {enabled, host, port}` — host/port = the PLC's **Modbus endpoint** (agrobot: `192.168.1.2:502`). jackal disabled. |
| `dashboard/index.html` | `_toggleActuatorPlc` + status/sequence pollers; Machine-Setup & Robot-Arm panels (Settings); PLC status strip; **PLC Reference panel** (`openPlcPanel`/`_renderPlcRef`) — documents every PLC tag (read/write/reserved · struct · `%MW`) with live values polled while open, plus the full symbol table from `/api/plc/tags`. |

**Key semantics**
- `success:true` means the **Modbus write landed, not that the machine moved** — the PLC
  ladder gates real motion on Auto-mode + subsystem-enabled + safety. So the UI confirms real
  completion by polling `get_sequence_detail` and watching `*_in_cycle` go `true → false`
  (then toasts "complete" and, for the planter, logs the seedling pin).
- The actuator buttons branch on `_plcEnabled`: real PLC on agrobot, original cosmetic pin-drop
  on jackal. Sequence-start buttons are disabled outside Auto mode.
- **Pushbuttons are pulsed**, mirroring the HMI: write the bit value → hold 100 ms → write 0.
  Auger uses `%MW6500` (START=1/STOP=2); the planter has no MotorPB word so it's driven via
  the machine PB word `%MW5000` (START=8192/STOP=16384); robot via `%MW6200`; machine commands
  via `%MW5000`/`%MW5001` bits. M-area Modbus offset is 0 (so `%MW6500` → holding register 6500,
  `%MX43204` → coil 43204) — verify this base matches the PLC's Modbus server if reads look off.

---

## Startup scripts

| Script | When to use |
|--------|-------------|
| `./launch_dashboard.sh [--chassis X] [--port N] [--headless]` | Full dashboard for chassis X. `--headless` (or `DASHBOARD_HEADLESS=1`) serves only and skips opening a local browser — view it from another device at the printed `http://<jetson-ip>:<port>`. Default still opens a browser on the Jetson. |
| `./launch_dashboard_plc.sh [--chassis X] [--port N] [--headless]` | Same as above but serves `plc_combined.html` (4-tab UI: Camera · GPS · Connectivity · PLC Handshake) via `serve_plc.py`. Default port **8769**. All chassis flags forwarded. |
| `./launch_dashboard_wide.sh [--chassis X]` | Wide-angle UI (`serve_wide.py` monkey-patches `serve.py`, so chassis logic is inherited) |
| `./start_all.sh`, `./start.sh <variant>`, `./teleop.sh` | agrobot-only ROS dev helpers; they exit early if the active chassis is not `agrobot` |

---

## Development notes

- `rclpy` and ROS message packages come from `ros-humble-*` apt packages, never pip.
- The ROS executor is `MultiThreadedExecutor(num_threads=4)`. It handles wheel-odom, battery, and ROS camera-topic callbacks concurrently. The velocity publisher runs on its **own dedicated thread** (not in the executor) so a busy callback queue can never jitter teleop commands.
- `dashboard/chassis.py` prefers PyYAML but has a minimal built-in YAML parser
  fallback, so the dashboard and tests work even without PyYAML installed.
- The static-file whitelist in `serve.py` allows only `/`, `/index.html`,
  `/index_wide.html` (wide), and `/logo/`. Add new static assets explicitly.
- `dashboard/logo/` and `reference/` are gitignored.
- `serve_wide.py` inherits all chassis behaviour from `serve.py` by importing it
  and monkey-patching only the HTML served and the ZED resolution.
- `serve_plc.py` inherits from `serve.py` the same way and adds `/api/amr/*` routes
  (AMR ↔ PLC handshake registers) plus auto-writes `%MW5112` on moving-state changes.
  It serves `plc_combined.html` instead of `index.html`.

---

## Logs

| Path | Contents |
|------|----------|
| `logs/dashboard/{ts}_{chassis}_dashboard.log` | Combined stdout/stderr from a launch |
| `logs/gnss/{ts}_gnss.jsonl` | One GPS fix per line |
| `logs/plants.jsonl`, `logs/recordings/{ts}/` | Planting events / recording sessions |
| `/tmp/gnss_coords.json`, `/tmp/object_detections.json` | Current fix / detections (volatile) |
