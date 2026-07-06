# Architecture

This document is the map of the codebase after the 2026-07 refactor: what
lives where, why, how to find your way around, and how to extend it without
recreating the problems the refactor removed. The remaining risks are listed
at the end.

## 1. High-level overview

One HTTP server (`dashboard/serve.py`, stdlib `http.server`, port 8766)
drives two robot chassis — the Agrobot tree-planter (Modbus RTU via a ROS node)
and the Clearpath Jackal (ROS 2 Twist over LAN) — selected by configuration,
plus the LS Electric PLC (Modbus TCP) that owns the planter hardware.

```
Browser ── REST/MJPEG ──► serve.py ──┬─ ROS 2 ─► robot_base_node ─ Modbus RTU ─► agrobot chassis
   ▲  index.html /                   ├─ ROS 2 ─► /jackal1/cmd_vel (Twist) ─────► Jackal
   │  plc_combined.html              ├─ Modbus TCP ────────────────────────────► LS Electric PLC (:502)
   └─ /js/teleop.js (shared)         ├─ pyzed SDK ─────────────────────────────► ZED 2i front + rear
                                     └─ /tmp/gnss_coords.json ◄─ gnss_rtu608bt_read.py
```

### Layers and the dependency rule

Dependencies point strictly downward. Nothing in a lower layer imports from
a higher one.

| Layer | Where | Contents | May import |
|---|---|---|---|
| **domain** | `agrobot_dashboard/domain/` | Pure logic: kinematics, odometry accumulation, battery median filter, fwd2m speed planner, DMS formatting. No I/O, no threads, no ROS. | stdlib only |
| **services** | `agrobot_dashboard/services/` | Stateful runtime: `TelemetryStore` (ALL mutable shared state), event log, YOLO singleton, recording loop. Own their locks/threads. | domain |
| **adapters** | `agrobot_dashboard/adapters/` | Hardware/network I/O: camera capture threads (`cameras.py`), Supabase upload (`cloud.py`). `dashboard/plc_client.py` and `dashboard/chassis.py` belong to this layer (they move into the package in a future step). | domain, services |
| **web / app** | `dashboard/serve.py` | HTTP routing (declarative tables), request handlers, `main()` composition root. | everything above |

Key objects:

- **`TelemetryStore`** (`services/telemetry.py`) — the single home for shared
  mutable state: camera feeds, detection results + on-demand gates, the
  velocity command + deadman, odometry, battery, recording, settings, PLC
  link state. One small object per subsystem, each owning its lock. The old
  design (~40 class attributes on the HTTP handler mutated from every
  thread) is gone; **never put shared state anywhere else**.
- **`Chassis`** (`dashboard/chassis.py`) — loads `config/chassis/<name>.yaml`
  and builds the chassis's ROS publisher/subscriptions. The YAML is the
  single source of truth for velocity limits, Twist→wheel scaling
  (`linear_scale` etc.) and feature flags; the server *and* the browser
  (via `GET /api/config`) consume those values — no hardcoded copies.
- **`PlcClient`** (`dashboard/plc_client.py`) — the ONE Modbus TCP client to
  the PLC. Register map `_REG`, command bit tables, and the AMR↔PLC
  handshake block (**%MW5100–5112** — bench-confirmed; the older %MW100/101
  map was wrong and is intentionally rejected by a test) all live here.
  One socket, serialized by one lock.
- **Routing** — `Handler.GET_EXACT` / `GET_PREFIX` / `POST_EXACT` tables:
  exact match first, then longest prefix. Extensions register with
  `Handler.add_route(...)` (see `serve_plc.py`); monkey-patching `do_GET`
  is forbidden.

### Frontend

Two pages, one brain:

- `dashboard/index.html` — the main dashboard.
- `dashboard/plc_combined.html` — the 4-tab PLC-handshake variant
  (served when launched via `launch_dashboard_plc.sh`, port 8769).
- `dashboard/js/teleop.js` — shared by both: `_fetchWithTimeout` and the
  **only** code allowed to POST `/api/cmd_vel` (`publishCmdVel`, clamped to
  the chassis ceilings from `/api/config`). Page-specific behaviour hooks in
  via `window.TELEOP_HOOKS`.
- "Wide" is not a fork: `serve.py --wide` opens the front ZED at HD2K and
  `/api/config` reports `ui.wide`, which the page maps to letterboxed video.

### Configuration flow

```
config/chassis/<name>.yaml ─► chassis.Chassis ─► serve.py (limits, scaling, features)
                                            └──► GET /api/config ─► browser (clamp, scaling, UI flags)
```

Selection precedence: `--chassis` flag → `$ROBOT_CHASSIS` →
`config/active_chassis.yaml` → `agrobot`. Secrets come only from the
environment (`AGROBOT_SUPABASE_KEY`).

## 2. How to understand the project

Read in this order:

1. `config/chassis/agrobot.yaml` — what a chassis *is* to this system.
2. `agrobot_dashboard/domain/` — the business rules, each file ~50 lines, fully
   unit-tested in `tests/test_domain.py`.
3. `agrobot_dashboard/services/telemetry.py` — the shape of runtime state.
4. `dashboard/serve.py` top-to-bottom: constants → routing tables → handler
   methods (thin: parse → store/service → JSON) → `main()` (composition:
   chassis, PLC client, ROS wiring, camera threads, HTTP server).
5. `agrobot_dashboard/adapters/cameras.py` — the capture threads, only if you
   need to touch video/detection.

Trace one command end-to-end to internalize the flow: browser key press →
`js/teleop.js publishCmdVel` (clamped) → `POST /api/cmd_vel` → validation
against `chassis.max_linear` → `TELEM.vel.set_command()` → the dedicated
20 Hz `_vel_loop` thread → `publish_velocity()` (built by
`Chassis.setup_ros`) → `Int16MultiArray` on `/avatar_robot/speed_cmd` →
`robot_base_node` → Modbus RTU write. Safety: a 0.5 s deadman in
`VelocityState.consume_for_publish` and a 1.5 s deadman in
`robot_base_node` are independent.

Tests (`pytest tests/` — no ROS or hardware needed): domain unit tests,
in-process HTTP endpoint tests (security, degradation, routing, traversal),
PLC register-map integrity tests, real-protocol frame tests.

## 3. How to extend the project safely

**A new chassis** — add `config/chassis/<name>.yaml`; a new comms type means
one new branch in `Chassis.setup_ros`. Nothing else changes; the UI adapts
via `/api/config`.

**A new HTTP endpoint** — add a row to the routing table (or call
`Handler.add_route` from an extension module), write a thin handler that
delegates to a service, add an endpoint test. Never parse business data or
loop inside the handler beyond what fwd2m already (regrettably) does.

**New shared state** — a new field-set on `TelemetryStore` with its own
lock; never a module global, never a Handler attribute.

**New hardware** — a new module in `agrobot_dashboard/adapters/`; it receives
the `TelemetryStore` (and config values) as explicit arguments. It must
degrade gracefully when the device is absent — startup never blocks on
hardware.

**New PLC registers** — extend `plc_client._REG` (+ `PLC_TAG_MAP` so the UI
reference panel documents it) and add a register-map test. Never hardcode a
register number anywhere else. Write targets must sit at/above the FEnet
write base (%MW5000) — there is a test asserting this.

**New business rule** — a pure function/class in `agrobot_dashboard/domain/`
with unit tests, then wire it from a service.

**Frontend** — shared behaviour goes into `dashboard/js/` (whitelist the
path in `Handler.STATIC_PREFIXES` if you add a new directory); constants the
server knows must arrive via `/api/config`, never be retyped in JS.

Rules that keep the architecture intact:
- No monkey-patching — extend via `add_route` / config / composition.
- One source of truth per fact (scaling in YAML, registers in `_REG`).
- `domain/` stays free of I/O and threads.
- All secrets from the environment.
- Run `pytest tests/` before shipping; add tests with every change.

## 4. Remaining risks / future improvements

1. **No clean shutdown.** Capture/YOLO/velocity threads are daemons that die
   with the process; `zed.close()` / `rclpy.shutdown()` are not called on
   exit. The launcher's `pkill` cleanup compensates. Fix: give services
   `start()/stop()` and a signal handler in `main()`.
2. **fwd2m runs inside an HTTP request thread** (up to 30 s). Correct
   (server-side stop decision) but it pins a request thread and cancellation
   is via velocity-override polling. Fix: move to a service thread with an
   explicit cancel endpoint.
3. **`dashboard/*.py` not yet inside the package.** `chassis.py`,
   `plc_client.py`, `serve.py` still live in `dashboard/` with a transitional
   `sys.path` shim in `chassis.py`. Finishing the move (and `pip install -e .`)
   removes the last path hacks.
4. **Untested by machine:** camera capture loops, YOLO workers, PLC client
   against a live socket (validation and register maps are tested; the
   pymodbus paths are not — a `pymodbus` simulator harness would close this),
   and everything in the browser. The HTML pages still hold large inline
   scripts; further extraction into `js/` modules is worthwhile.
5. **Polling UI.** The browser polls 6–8 endpoints on timers; several keep
   polling while hidden. A push channel (SSE) would cut Jetson load.
6. **`plc_combined.html` still duplicates some page scaffolding** with
   `index.html` (~200 lines of camera/settings helpers). Fold into shared
   `js/` modules as they stabilize.
7. **The leaked Supabase key** must be rotated server-side; it remains in
   git history.
8. **`scripts/plc_read.py` / `plc_test.py`** are standalone bench tools that
   re-declare the FEnet offset formulas; acceptable for diagnostics, but
   check them against `plc_client._REG` before trusting output.
9. **T3 odometry** expects `/avatar_robot/vel_raw`, which nothing in this
   repo publishes (it now warns loudly). Confirm against real T3 firmware or
   remove the variant.
