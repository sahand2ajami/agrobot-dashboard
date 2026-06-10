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
| GET | `/api/wheel_odom`, `/api/camera*`, `/api/zed*`, `/api/detection*`, `/api/gnss`, `/api/settings` | As before. |

### Adaptive UI

`index.html` calls `GET /api/config` on load (`_applyChassisConfig`) and:
- hides any element tagged `data-chassis-feature="X"` when `features.X === false`;
- clamps commanded velocity to `limits.maxLinear`/`maxAngular` in `publishCmdVel`.

Gated elements: Modbus master slider (`modbus_slider`), auto-forward-2 m button
(`fwd2m`), Distance-Traveled + Chassis-Link cards (`wheel_odom`), the Chassis-Battery
status card (`battery`), and the planter/auger buttons + Planted counter (`actuators`).

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

- **GNSS** — `scripts/gnss_p9_read.py` (Columbus P-9 Race). Auto-detects port,
  writes `/tmp/gnss_coords.json` atomically; polled by `serve.py`.
- **Cameras** — `serve.py` runs best-effort local RealSense/ZED capture threads
  **and** subscribes to the chassis ROS camera topic; MJPEG streamed to the UI.
- **YOLOv8** — `scripts/object_detector.py` (persons only). Launched for agrobot;
  skipped for jackal (no local RealSense topics).
- **Recording / track / plant logging** — under `logs/`.

---

## Startup scripts

| Script | When to use |
|--------|-------------|
| `./launch_dashboard.sh [--chassis X] [--port N] [--headless]` | Full dashboard for chassis X. `--headless` (or `DASHBOARD_HEADLESS=1`) serves only and skips opening a local browser — view it from another device at the printed `http://<jetson-ip>:<port>`. Default still opens a browser on the Jetson. |
| `./launch_dashboard_wide.sh [--chassis X]` | Wide-angle UI (`serve_wide.py` monkey-patches `serve.py`, so chassis logic is inherited) |
| `./start_all.sh`, `./start.sh <variant>`, `./teleop.sh` | agrobot-only ROS dev helpers; they exit early if the active chassis is not `agrobot` |

---

## Development notes

- `rclpy` and ROS message packages come from `ros-humble-*` apt packages, never pip.
- `dashboard/chassis.py` prefers PyYAML but has a minimal built-in YAML parser
  fallback, so the dashboard and tests work even without PyYAML installed.
- The static-file whitelist in `serve.py` allows only `/`, `/index.html`,
  `/index_wide.html` (wide), and `/logo/`. Add new static assets explicitly.
- `dashboard/logo/` and `reference/` are gitignored.
- `serve_wide.py` inherits all chassis behaviour from `serve.py` by importing it
  and monkey-patching only the HTML served and the ZED resolution.

---

## Logs

| Path | Contents |
|------|----------|
| `logs/dashboard/{ts}_{chassis}_dashboard.log` | Combined stdout/stderr from a launch |
| `logs/gnss/{ts}_gnss.jsonl` | One GPS fix per line |
| `logs/plants.jsonl`, `logs/recordings/{ts}/` | Planting events / recording sessions |
| `/tmp/gnss_coords.json`, `/tmp/object_detections.json` | Current fix / detections (volatile) |
