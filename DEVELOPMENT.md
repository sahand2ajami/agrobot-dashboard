# Developer Guide — Agrobot Dashboard

## What this project is

A single ROS 2 (Humble) teleoperation platform that drives **two different
chassis** from one web dashboard, selected by configuration:

- **`agrobot`** — Agrobot differential-drive tree-planting robot, **Modbus RTU**
  over serial (`/dev/ttyUSB0`, 38400 baud) via a separate `robot_base_node`.
- **`jackal`** — Clearpath Jackal UGV, **ROS 2 (DDS) over a LAN cable**; the
  dashboard publishes `geometry_msgs/Twist` to `/jackal1/cmd_vel`.

Runs on Ubuntu 22.04 + ROS 2 Humble (Jetson). Read
[docs/architecture.md](docs/architecture.md) first — it is the map of the
codebase (layering, dependency rule, how to extend safely, remaining risks).
`reference/` holds read-only clones of the pre-merge repos (gitignored).

**Docs map:** [architecture.md](docs/architecture.md) (codebase map) ·
[jetson.md](docs/jetson.md) (the AGX Orin unit: specs, network, serial devices) ·
[plc.md](docs/plc.md) (PLC integration) + [plc/manufacturer-docs.md](docs/plc/manufacturer-docs.md)
(LS Electric manuals) · [ui-guide.md](docs/ui-guide.md) (every button/panel) ·
[hmi.md](docs/hmi.md) (HMI mirror) · [detection.md](docs/detection.md) (YOLO).

---

## Build & test

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install     # builds src/avatar_robot_base (agrobot chassis)
source install/setup.bash

pytest tests/                      # no hardware / no ROS required
```

---

## Layout (dependency flow: domain ← services ← adapters ← web)

| Path | Role |
|------|------|
| `agrobot_dashboard/domain/` | Pure business logic (kinematics, odometry, battery filter, fwd2m planner, geo). No I/O, no threads, no ROS. |
| `agrobot_dashboard/services/` | `telemetry.py` (**TelemetryStore** — ALL mutable shared state lives here, one locked object per subsystem), `events.py` (browser event log), `detection.py` (shared YOLO singleton + GPU lock), `recording.py`. |
| `agrobot_dashboard/adapters/` | `cameras.py` (ZED front/rear + webcam capture threads; take the store as an argument), `cloud.py` (HTTP ingest upload, key from `$AGROBOT_INGEST_KEY`). |
| `dashboard/serve.py` | HTTP layer + composition root (`main()`); ~1,270 lines. Declarative routing: `GET_EXACT`/`GET_PREFIX`/`POST_EXACT` (exact first, then longest prefix — order can't shadow routes). Extensions register via `Handler.add_route`; **monkey-patching is forbidden**. |
| `dashboard/chassis.py` | Chassis abstraction (see below). Transitional home; carries the `sys.path` shim that makes `agrobot_dashboard` importable from a checkout. |
| `dashboard/plc_client.py` | The ONE Modbus TCP client to the PLC (register map `_REG`, command bit tables, AMR handshake block). |
| `dashboard/serve_plc.py` | Adds `/api/amr/*` + `/api/hmi/*` (HMI mirror: read screens + control-page writes) routes + the %MW5112 auto-write and jog-deadman threads via `add_route` (no patching); serves `plc_combined.html`. |
| `dashboard/serve_battery.py` | Adds `/api/battery_test/*` routes (and reuses `serve_plc`'s `/api/amr/{poll,write}` + `%MW5112` auto-writer, so the page's manual auger/planter/both buttons work) + serves `battery_test.html` via `add_route` (no patching). Owns the `BatteryTest` controller: a background thread that repeats **forward 2 m → auger → planter → backward 2 m → auger → planter** (drives via `serve.drive_distance`; auger/planter via `Handler.plc`, marking the AMR stationary/moving on `%MW5112`) until the pack voltage hits the cutoff. Strictly sequential — no drive/actuator overlap. Each plant fires a **momentary** start pulse (`amr_write(pulse=True)`, so no latched bit → no free-run) and waits for the cycle via the **Clear-of-Ground** handshake bit (`%MW5100`/`5101` bit1: 1 home → 0 working → 1 done — see [[agrobot-auger-planter-done-signal]]), with a `_PLANT_START_TIMEOUT` (never left home → cycle didn't start) and the `plant_timeout` hard cap so a machine not in AMR-gated cycle mode can never hang the test; no PLC → plant steps no-op and it degrades to fwd/back only. Tracks fwd/back/cycle + auger/planter/plant-timeout counters. |
| `dashboard/index.html`, `plc_combined.html`, `battery_test.html`, `dashboard/js/teleop.js` | The three pages + the shared teleop transport (the only JS allowed to POST `/api/cmd_vel`; clamps to chassis ceilings; pages hook in via `window.TELEOP_HOOKS`). |
| `src/avatar_robot_base/` | ROS package: `robot_base_node` (Modbus RTU driver), `protocol.py` (pure frame build/parse — tested without rclpy), `odom_calculation` (`car_type` ROS param: T3/T13/T17E), one parameterized `robot_launch.py`. |
| `tests/` | Domain units, in-process HTTP endpoint tests, PLC register-map integrity, protocol frames. |

Rules that keep this intact: one source of truth per fact (scaling in the
chassis YAML, PLC registers in `_REG`); new shared state goes on
`TelemetryStore`, never module globals or Handler attributes; secrets only
from the environment; PyYAML is a hard dependency (the old hand-rolled
fallback parser is gone).

---

## The chassis abstraction

`serve.py` and the UI are robot-agnostic. Everything chassis-specific is in:

| File | Role |
|------|------|
| `config/active_chassis.yaml` | Selects the default chassis (`chassis: agrobot`). |
| `config/chassis/<name>.yaml` | Per-chassis params: comms type, velocity limits, Twist→wheel scaling, ROS topics, network, serial/variant, `plc`, UI feature flags. **Single source of truth** — server and browser both consume these values. |
| `dashboard/chassis.py` | Loads the active chassis; `setup_ros()` builds the velocity publisher and feeds odometry/battery callbacks into the TelemetryStore (the math lives in `domain/`). |

**Selection order:** `--chassis` flag → `$ROBOT_CHASSIS` → `active_chassis.yaml`
→ default `agrobot` (`chassis.resolve_name()`).

`setup_ros` returns `publish_velocity(linear_x, angular_z)`:
`comms: ros_twist` → publishes Twist; `comms: modbus_speed` → converts via
`domain.kinematics.twist_to_wheel_speeds` (agrobot: `linear_scale` 3000,
`angular_scale` 1000, clamp ±32767) → `Int16MultiArray`.

**Adding a third chassis:** add `config/chassis/<name>.yaml`; a new comms
type = one new branch in `Chassis.setup_ros`; a new service set = one branch
in `launch_dashboard.sh`. No UI changes needed.

### Adaptive UI

`index.html` calls `GET /api/config` on load and: hides elements tagged
`data-chassis-feature="X"` when `features.X === false`; clamps commanded
velocity to `limits`; takes `scaling` (linearScale, pulsePerM…) from the
response — **never hardcode those in JS**; applies `ui.wide` (letterboxed
video) when the server runs with `--wide`. `features.plc` is derived from
the chassis's `plc.enabled` and gates all PLC panels; `actuators` stays
separate (jackal keeps cosmetic planter/auger buttons with no PLC).

---

## HTTP API (port 8766)

| Method | Path | Notes |
|--------|------|-------|
| GET | `/api/config` | Chassis name, comms, features, limits, scaling, battery gauge range, `ui.wide`. |
| GET | `/api/chassis_battery` | `{voltage_v, connected}` — median-smoothed pack voltage (agrobot). |
| POST | `/api/cmd_vel` | `{linear_x, angular_z}`; 400 if beyond the active chassis's limits. |
| POST | `/api/fwd2m` | Server-side 2 m auto-drive (50 Hz encoder loop, `pulse_per_m` from the chassis YAML, two-phase crawl finish; stop decision on the server). Body `{speed, direction: forward\|backward}`. 503 when `features.fwd2m` is false. Shares `serve.drive_distance` with the battery test. |
| GET/POST | `/api/battery_test/{status,start,stop}` | Registered by `serve_battery.py`. `start {speed, stop_v?, plant_timeout?}` runs the server-side **forward 2 m → auger → planter → backward 2 m → auger → planter** endurance loop until the pack voltage ≤ `stop_v` (default `battery_min_v`); `stop` kills the loop, stops the robot, and releases the auger/planter command bits; `status` → `{running, phase (idle\|forward\|plant\|backward), plant_which, forward, backward, cycles, augers, planters, plant_timeouts, speed}`. 503 when `features.fwd2m` is false. |
| GET | `/api/wheel_odom`, `/api/gnss`, `/api/settings`, `/api/events` | Telemetry / settings / event log. |
| GET | `/api/camera/*`, `/api/zed/*`, `/api/detection/*` | Rear/front MJPEG (raw or with YOLO boxes) + detection JSON `{ts, count, detections:[{label, confidence, distance_m, bbox}]}`; detection runs on demand only (3 s idle timeout). |
| POST | `/api/plc/{auger,planter,both}` | `{command: START\|STOP}` → handshake words %MW5110/%MW5111. |
| POST | `/api/plc/machine`, `/api/plc/robot` | Pulsed pushbutton words (allow-listed commands, unknown → 400). |
| GET | `/api/plc/{status,sequence,auger_motor,banner,tags}` | Machine status / sequence detail / auger motor / HMI banner / tag+symbol reference (works PLC-down). |
| GET/POST | `/api/amr/{poll,ping,write}` | AMR↔PLC handshake block %MW5100–5112 (registered by `serve_plc.py`). `write {reg,value,pulse?}` — `pulse:true` writes `value` then self-clears to 0 (momentary start for %MW5110/5111; a latched bit makes the machine free-run). |
| GET | `/api/hmi/{screens,read}` | Live mirror of the machine HMI screens (registered by `serve_plc.py`; see [hmi.md](docs/hmi.md)). `read?screen=<id>` returns each screen's panels with a live value per row; numeric fields carry the C-more fractional-digit formatting. |
| POST | `/api/hmi/press`, `/api/hmi/jog` | Control-page writes: `press {block,button}` pulses a momentary axis/motor pushbutton; `jog {block,button,action}` holds a jog bit with a server deadman. Allow-listed PB words (%MW5400–6500, FC06); writes below %MW5000 refused. 503 off-chassis, 400 unknown button. |

All `/api/plc/*` and `/api/amr/*` return **503** on a chassis without
`plc.enabled`; a downed PLC is a normal 200 with `connected:false`.
Static whitelist: `/`, `/index.html`, the active `INDEX_FILE`, `/logo/`,
`/js/`. Paths are unquoted + normalized **before** matching (path-traversal
guard — keep it that way).

---

## agrobot chassis specifics

- **`robot_base_node`** subscribes `/avatar_robot/speed_cmd`, writes speeds and
  reads sensors over Modbus RTU, publishes `/avatar_robot/{battery,wheel_odom,error,oil}`.
  1.5 s deadman. Waits 50 ms before transmitting to avoid colliding with the
  chassis's internal 100 ms Modbus cycle — **do not remove this sleep**. A
  missed speed-ACK flushes the input buffer and still does the sensor read.
  The wire format lives in `avatar_robot_base/protocol.py` (pure, tested
  without rclpy): sensor block `[0..3]` odom hi/lo pairs (sign-extended
  int32), `[4]` battery×100 (V), `[5]` fault code, `[6]` oil % — layout
  confirmed by passive bus capture.
- **Chassis battery**: raw readings → `domain.battery.MedianVoltageFilter`
  (15 s window, 30–70 V validity, 10 s recompute) → `/api/chassis_battery`;
  gauge range `battery_min_v/max_v` from `agrobot.yaml` (42–58 V, ~48 V 14S pack).
- **Odometry**: `domain.odometry.OdometryAccumulator` — outlier deltas
  (≥3000 pulses) rejected, mileage resets after a >5 s gap (reconnect).
- **Deadman timeouts**: `robot_base_node` 1.5 s; dashboard velocity thread
  0.5 s. Independent.
- **Variants**: `odom_calculation` takes a `car_type` ROS parameter
  (T3 | T13 | T17E; geometry in its `VARIANTS` table). One parameterized
  launch: `ros2 launch avatar_robot_base robot_launch.py car_type:=T13`.
  Note: T3 expects `/avatar_robot/vel_raw`, which nothing in this repo
  publishes — only relevant with T3 firmware that provides it.

## jackal chassis specifics

Publishes Twist to `/jackal1/cmd_vel`; camera from
`/jackal1/sensors/camera_0/color/image`. `launch_dashboard.sh` sets
`ROS_DOMAIN_ID` (0) and adds `192.168.1.100/24` on `eno1` (Jackal at
`192.168.1.200`). No wheel odom / battery / PLC — hidden via feature flags.
`config/fastdds_jackal.xml` is available for unicast DDS on field networks.

---

## Shared subsystems

- **GNSS** — `scripts/gnss_rtu608bt_read.py` (GeoAstra RTU608BT, 38400 baud,
  USB or Bluetooth; BT: pair "Geoastra" PIN 1234, then
  `sudo rfcomm bind 0 <MAC>`). Auto-detects the port (`/dev/gnss` symlink
  preferred). Writes `/tmp/gnss_coords.json` atomically; `serve.py` polls it
  and derives freshness from the `ts` field.
- **Cameras** — both ZED 2i opened directly via pyzed
  (`agrobot_dashboard/adapters/cameras.py`; front index 0, rear index 1,
  PERFORMANCE depth); MJPEG to the UI at 20 fps. `--wide` = front at
  HD2K/15 fps. The chassis ROS camera topic is only a fallback when pyzed is
  unavailable.
- **YOLOv8 person detection** — `agrobot_dashboard/services/detection.py`: one
  `yolov8n.pt` loaded once onto the GPU (FP16), front/rear inference
  serialized under `YOLO_INFER_LOCK`, runs only while a Det view is open
  (3 s idle timeout). Detections carry confidence + ZED-depth distance.
  See [detection.md](docs/detection.md).
- **Recording / plant logging** — `agrobot_dashboard/services/recording.py` →
  MP4s under `logs/recordings/`; seedling records to
  `logs/planted_seedlings/seedlings.jsonl` and (when `AGROBOT_INGEST_KEY` is
  set) to the configured ingest endpoint via `agrobot_dashboard/adapters/cloud.py`.

---

## PLC integration (agrobot tree-planter)

```
Browser ──REST──► serve.py ──Modbus TCP──► LS Electric PLC (192.168.1.2:502)
```

The PLC CPU's Ethernet port serves Modbus TCP on 502 (the FEnet card at `.1`
speaks LS's own protocol — 502 is NOT served there). The dashboard connects
lazily and degrades gracefully; startup never blocks on the PLC. The gRPC
gateway era is over — `plc_client.py` speaks Modbus itself.

**Register truth** (bench-confirmed): the AMR↔PLC handshake block is
**%MW5100–5112** — PLC→AMR status at %MW5100 (auger) / %MW5101 (planter);
AMR→PLC commands at %MW5110 (auger start), %MW5111 (planter start), %MW5112
(AMR state: 1 stationary / 2 moving, auto-written by `serve_plc.py` on
movement transitions). An older map used %MW100/101 — those sit below the
FEnet write base and can never be written over Modbus; a test in
`tests/test_plc_client.py` rejects any write target below %MW5000, so the
wrong map cannot silently come back.

**Semantics**: `success:true` means the **Modbus write landed, not that the
machine moved** — the ladder gates real motion on Auto mode + subsystem
enables + safety. **Auger/planter completion is tracked via the Clear-of-Ground
handshake bit** (`%MW5100`/`5101` bit1: 1 home → 0 working → 1 done), *not*
`*_in_cycle`/`*_complete` — on this machine those only arm in the automated
AMR cycle and `%MW5100/5101` bit2 "Complete" is latched high, so it gives no
usable edge (bench-confirmed; see [[agrobot-auger-planter-done-signal]]). The
auger/planter buttons and the battery test fire a **momentary** start pulse
(`amr_write(pulse=True)` — write 1 → hold ≥1 scan → write 0; a latched
%MW5110/5111 bit makes the machine free-run) and show "Working" until
Clear-of-Ground returns to 1. Pushbuttons are
pulsed (write bit value → hold 100 ms → write 0): machine via
%MW5000/%MW5001, robot via %MW6200, auger motor via %MW6500. FEnet offsets:
FC04 reads = addr−1000, FC06 writes = addr−5000, bit reads FC02 at offset 0;
reads use **FC04, never FC03** (FC03 returns zeros on this PLC).

**Testing without the real PLC**: point `plc.host` in `agrobot.yaml` at any
Modbus-TCP server exposing the `_REG` registers — a `pymodbus` simulator, or
LS Electric's XG5000 simulator (Windows; runs the real ladder).

---

## Startup scripts

| Script | When to use |
|--------|-------------|
| `./launch_dashboard.sh [--chassis X] [--port N] [--headless] [--rear-camera src]` | Full dashboard for chassis X. `--headless` (or `DASHBOARD_HEADLESS=1`) serves only — view from another device at `http://<jetson-ip>:<port>`. |
| `./launch_dashboard_wide.sh` | Same, with `serve.py --wide` (HD2K front ZED, letterboxed UI). No separate server or HTML fork. |
| `./launch_dashboard_plc.sh` | Same + `/api/amr/*` and the 4-tab `plc_combined.html` (default port 8769) via `serve_plc.py`. |
| `./launch_dashboard_battery_test.sh` | Battery drain-test page (`battery_test.html`, default port 8770) via `serve_battery.py`: 2 m Fwd/Bwd, **manual auger/planter/both buttons** (momentary fire + Clear-of-Ground tracking, drives locked out while an actuator runs), WASD, battery %, speed, Auto endurance loop (forward → auger → planter → backward → auger → planter), big always-live STOP + fwd/back/cycle/auger/planter counters. Every cycle step runs manually or automatically. |
| `./start_all.sh`, `./teleop.sh` | agrobot-only ROS dev helpers (chassis stack + RViz / keyboard teleop); they exit early if the active chassis is not `agrobot`. |

---

## Development notes

- `rclpy` and ROS message packages come from `ros-humble-*` apt packages,
  never pip. PyYAML **is required** (pyproject.toml / requirements.txt).
- The ROS executor is `MultiThreadedExecutor(num_threads=4)`. The 20 Hz
  velocity publisher runs on its **own dedicated thread** so a busy callback
  queue can never jitter teleop commands.
- Secrets: `AGROBOT_INGEST_KEY` and `AGROBOT_INGEST_URL` (both required; no default endpoint) come from
  the environment. Never commit keys.
- New static assets go under a whitelisted prefix (`/logo/`, `/js/`) or an
  explicit addition to `Handler.STATIC_PREFIXES`.
- `reference/` is gitignored.

## Logs

| Path | Contents |
|------|----------|
| `logs/dashboard/{ts}_{chassis}_dashboard.log` | Combined stdout/stderr from a launch |
| `logs/gnss/{ts}_gnss.jsonl` | One GPS fix per line |
| `logs/plants.jsonl`, `logs/planted_seedlings/seedlings.jsonl`, `logs/recordings/{ts}/` | Planting events / seedling records / recording sessions |
| `/tmp/gnss_coords.json`, `/tmp/object_detections.json` | Current fix / detections (volatile) |
