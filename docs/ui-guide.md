# Dashboard UI Guide — every button, panel, and feature

A first-time-reader catalog of the three web pages that make up the Agrobot
dual-robot dashboard. For each page it lists every visible control, what the
control does, the HTTP endpoint or JavaScript hook it calls, and any
preconditions. Screenshots are referenced at the top of each section; the image
files are captured by hand (see [`images/README.md`](images/README.md)) and may
not exist yet.

Related reading:
- [`../README.md`](../README.md) — project overview and quick start.
- [`../DEVELOPMENT.md`](../DEVELOPMENT.md) — authoritative HTTP API table and page summaries.
- [`plc.md`](plc.md) — PLC register map and Modbus details.
- [`hmi.md`](hmi.md) — the machine HMI screens mirrored by the PLC page's HMI tab.
- [`architecture.md`](architecture.md) — how the codebase is layered.

> **Accuracy note.** Every claim below is traceable to `dashboard/index.html`,
> `dashboard/plc_combined.html`, `dashboard/battery_test.html`,
> `dashboard/js/teleop.js`, or the DEVELOPMENT.md API table. Where the code does not
> make a behavior fully clear, that is called out explicitly rather than guessed.

---

## 1. The three pages at a glance

| Page | Served by | Default port | Purpose |
|------|-----------|--------------|---------|
| `index.html` | `./launch_dashboard.sh` (and `./launch_dashboard_wide.sh` for letterboxed HD2K video) | **8766** | Main teleoperation dashboard — cameras, driving, map, telemetry, settings. Works for both the `agrobot` and `jackal` chassis; PLC panels appear only on `agrobot`. |
| `plc_combined.html` | `./launch_dashboard_plc.sh` | **8769** | A 7-item sidebar app centered on the AMR↔PLC handshake: Cameras · GPS · PLC · HMI · Connectivity · Settings · Event Logs, plus a persistent WASD/actuator sidebar. |
| `battery_test.html` | `./launch_dashboard_battery_test.sh` | **8770** | Endurance / battery-drain test rig: 2 m drives, manual actuators, WASD, and an automated forward→auger→planter→backward→auger→planter loop. |

All three load the shared transport `dashboard/js/teleop.js`, which is the only
code allowed to POST `/api/cmd_vel` and which clamps every drive command to the
active chassis's ceilings (see [§4 Keyboard controls](#4-keyboard-controls) and
[§3 Adaptive UI](#3-adaptive-ui-behavior)).

Ports are defaults; each launch script accepts `--port N` and prints the
resolved LAN URL on startup.

---

## 2. Main dashboard — `index.html` (port 8766)

![Main dashboard overview](images/index-overview.png)
*Full main dashboard: header, split camera feed, drive controls, GPS map, and telemetry strip.*

Layout: a top **header**, a two-column **main** area (left = cameras + drive
controls, right = map + telemetry), and three slide-in side panels (Event Log,
PLC Reference, Advanced Settings). A draggable vertical splitter between the two
columns resizes them (drag the thin bar; clamped to 20–80 % width).

### 2.1 Header

Three icon buttons sit at the right of the header:

| Button (icon) | Element id | Action | Endpoint / hook | Preconditions |
|---------------|-----------|--------|-----------------|---------------|
| Bell — **Event Log** | `logBtn` | Opens the Event Log side panel. A red badge shows the unread warn/error count while the panel is closed. | `openLog()`; log entries come from `GET /api/events` (polled every 3 s) plus client-generated entries. | Always visible. |
| CPU — **PLC Reference & Diagnostics** | `plcRefBtn` | Opens the PLC Reference side panel. | `openPlcPanel()`; fetches `GET /api/plc/tags` once, then polls `GET /api/plc/sequence` and `/api/plc/auger_motor`. | `data-chassis-feature="plc"` — hidden when `features.plc` is false (jackal). |
| Gear — **Advanced Settings** | `gearBtn` | Opens the Advanced Settings side panel. | `openSettings()`; loads `GET /api/settings`, polls `GET /api/*` status endpoints while open. | Always visible. |

The Agrobot logo sits at the left; if the SVG fails to load it falls back to the
text "AGROBOT ROBOTICS".

### 2.2 Camera area

![Camera area](images/index-camera.png)
*Front camera fills the frame; the rear camera rides in a draggable picture-in-picture; a red LIVE badge sits top-right.*

- **LIVE badge** (`liveBadge`) — static red "● LIVE" indicator, top-right of the video.
- **Main video pane** — the front ZED feed by default (`/api/zed/stream`), or
  the YOLO detection stream (`/api/detection/stream`) when Detection is on.
- **Picture-in-picture (PiP) pane** — the rear camera (`/api/camera/stream`) by
  default. It is interactive:
  - **Click / tap** the PiP to **swap** which camera is main and which is PiP
    (`_camSwapped` toggles; a "Swap" overlay appears on hover).
  - **Drag** the PiP to reposition it; a 3×3 zone grid highlights and snaps it to
    one of nine corners/edges.
  - The PiP is hidden entirely when the chassis config sets `rear_camera: none`
    (front camera then fills the frame, no swap target).
- **Detection info bar** (`detBar`) — hidden until Detection is on; then shows
  per-camera person counts and distances, polling `GET /api/detection/data`
  (front) and `GET /api/detection/rear_data` (rear) every 500 ms. Turns red text
  when a person is detected.
- **"Camera not found"** overlay appears per-pane if a stream delivers no frame
  within ~6 s; a watchdog force-reconnects any stream stalled >12 s.

The Detection toggle itself is a button in the action cluster (§2.3), not on the
video.

### 2.3 Drive controls (bottom-left)

![Drive controls](images/index-drive-controls.png)
*WASD keypad, live speed readout with Slow/Normal/Fast presets, and the action cluster (actuators, 2 m auto-drive, REC, Detection) with the PLC status strip.*

Three groups sit side by side: the WASD keypad, the speed readout, and the
action cluster.

**WASD keypad** — four on-screen keys (`kw`/`ka`/`ks`/`kd`) labeled W/A/S/D.
Hold (mouse/touch) or press the physical keys to drive; details in
[§4](#4-keyboard-controls). Keys light blue while active.

**Speed readout**

| Element | What it shows / does |
|---------|---------------------|
| `speedVal` | Current linear speed magnitude (m/s), large numerals. |
| `speedBar` | Horizontal bar scaled to `|linear| / maxLinear`; green forward, amber reverse. |
| `angVal` | Current angular rate magnitude (rad/s). |
| **Slow** (`spSlow`) | Sets the speed multiplier to 1.0 (`setSpeedPreset(1.0)`). |
| **Normal** (`spMed`) | Sets the multiplier to 2.0 — the default active preset (`setSpeedPreset(2.0)`). |
| **Fast** (`spFast`) | Sets the multiplier to 4.0 (`setSpeedPreset(4.0)`). |

The multiplier scales the WASD linear command (`CFG.maxLinear * multiplier`) and
the 2 m auto-drive speed. The commanded value is still hard-clamped to the
chassis ceiling by `teleop.js`.

**Action cluster** — two rows of buttons plus the PLC status strip.

Row 1 — actuators (wrapped in `data-chassis-feature="actuators"`; shown on both
chassis, but behavior differs — see below):

| Button | Element id | Action |
|--------|-----------|--------|
| **Planter** | `planterBtn` | `toggleActuator('planter')` |
| **Auger** | `augerBtn` | `toggleActuator('auger')` |
| **Both** | `bothBtn` | `toggleActuator('both')` |

`toggleActuator` dispatches on whether the PLC is enabled (`_plcEnabled`, set
from `features.plc`):

- **On agrobot (PLC enabled)** the buttons drive the real machine via REST:
  - Planter → `POST /api/plc/planter {command:"START"}` (momentary; then polls
    `/api/plc/sequence`).
  - Auger → `POST /api/plc/auger` with `START` or `STOP` depending on
    `auger_in_cycle` (it latches; the button shows an "on" glow while in cycle).
  - Both → `POST /api/plc/both {command:"START"}`.
  - A completed planter cycle (edge on `planter_in_cycle` clearing) logs a
    seedling via `POST /api/plant` and drops a red pin on the map.
  - "success" only means the Modbus write landed; the ladder still gates real
    motion on Auto mode + subsystem enable + safety (see [`../DEVELOPMENT.md`](../DEVELOPMENT.md)).
- **On jackal (no PLC)** the same buttons are purely cosmetic
  (`_toggleActuatorLocal`): Auger latches its highlight; Planter/Both flash
  momentarily and drop a local seedling pin/`POST /api/plant` if a GPS fix
  exists. No machine motion.

**PLC status strip** (`plcStrip`, `data-chassis-feature="plc"` — agrobot only) — a
row of live chips fed by `GET /api/plc/status` (polled every 1 s):

| Chip | Meaning |
|------|---------|
| `plcGw` — "● Gateway" | Green when the PLC/Modbus link is up, red "Gateway offline" when down. |
| `plcMode` — "AUTO" / "MANUAL" / "MODE —" | PLC mode (green Auto, amber Manual, `—` offline). |
| `plcEstop` — "E-stop" | Green when `estop_ok`, red otherwise. |
| `plcGate` — "Gate" | Green when `gate_ok`, red otherwise. |
| `plcFault` — "FAULT" | Shown only when the PLC reports `faulted`. |
| `plcAugerCyc` — "Auger ⟳" | Shown while `auger_in_cycle`. |
| `plcPlanterCyc` — "Planter ⟳" | Shown while `planter_in_cycle`. |

Row 2 — auto-drive, recording, detection:

| Button | Element id | Action | Endpoint / hook | Preconditions |
|--------|-----------|--------|-----------------|---------------|
| **→ 2 m** (with live speed sub-label) | `fwd2mBtn` | Server-side 2 m auto-drive. The button disables and shows live progress ("0.00 / 2.00 m") while running. | `POST /api/fwd2m {speed}` (blocks until 2 m/stop/timeout); progress reads `GET /api/wheel_odom`. | `data-chassis-feature="fwd2m"` — hidden when `features.fwd2m` false. Pressing a WASD key mid-drive sends a stop. |
| **REC** | `trackRecBtn` | Toggles GPS-track + camera recording. Turns solid red and reads "STOP" while recording; draws a green trace on the map and keeps the robot centered. | Start → `POST /api/record/start` + collects GPS points; Stop → `POST /api/record/stop` and `POST /api/track/save`. | Always available. |
| **Detection** | `detBtn` | Toggles YOLO person-detection overlay; swaps the main feed to the detection stream and shows the detection info bar. | `toggleDetectionBtn()` → reloads cameras with `/api/detection/stream`; polls detection JSON. | Always available. |

### 2.4 GPS map (right column)

![GPS map](images/index-map.png)
*Leaflet/OpenStreetMap map with the robot marker; Google-Maps-style zoom, compass, and locate controls overlaid.*

A Leaflet map (OpenStreetMap tiles) showing the robot position. Position comes
from `GET /api/gnss`, polled every 1 s. Overlay controls:

| Control | Element id | Action |
|---------|-----------|--------|
| **Compass / 2D-3D toggle** | `compassBtn` | `toggleMapMode()` — switches between flat "2D" and heading-up "HDG" mode. In HDG mode the map rotates to the robot's GPS heading; the needle and mode label update. |
| **Zoom in** (+) | — | `mapZoomIn()`. |
| **Zoom out** (−) | — | `mapZoomOut()`. |
| **Locate / center** | `gnssLocateBtn` | `centerOnGps()` — recenters on the robot at street zoom. Turns red with a "?" when there is no GPS fix. |

Map markers: a blue pulsing dot for a valid fix (amber dot when no fix), a
translucent accuracy circle sized from HDOP, a green polyline while recording a
track, and red pins at logged seedling locations.

### 2.5 Telemetry strip (bottom-right)

![Telemetry strip](images/index-telemetry.png)
*Read-only telemetry: latitude, longitude, altitude, planted count, and chassis motion state.*

Read-only tiles, updated from the GNSS poll and drive state:

| Tile | Element id | Source |
|------|-----------|--------|
| **Latitude** | `lat` | `GET /api/gnss` |
| **Longitude** | `lon` | `GET /api/gnss` |
| **Altitude** | `alt` | `GET /api/gnss` |
| **Planted** | `planted` | Local seedling counter (`data-chassis-feature="actuators"`). |
| **Chassis** | `chassisVal` | "Idle" / "Moving" from drive state; "NO LINK" (red) when the server is unreachable. |

### 2.6 Advanced Settings panel

![Advanced Settings](images/index-settings.png)
*Slide-in settings panel: speed sliders, system status cards, planting job code, PLC machine setup and robot arm, PLC alerts, GNSS, and odometry.*

Opened by the gear button. Sections top to bottom:

**Speed Controls**
- **Modbus Speed (Master)** slider (`slModbus`, `data-chassis-feature="modbus_slider"`)
  — raw Modbus register per wheel (0–32767). Drives the other sliders live
  (`onModbusSlider`). Agrobot-only feature flag.
- **Max Forward Speed** slider (`slLinear`, 0.1–10 m/s) — `onLinearSlider`.
- **Max Turn Rate** slider (`slAngular`, 0.1–3 rad/s) — `onAngularSlider`.
- **Apply & Update Robot** button (`applyBtn`) — `applySettings()` →
  `POST /api/settings {maxLinear, maxAngular, modbusSpeed}`.

**System Status** — read-only status cards, polled every 2 s while the panel is
open (`_pollSpStatus`):

| Card | Element id | Source | Feature flag |
|------|-----------|--------|--------------|
| RealSense (Rear) | `stRS` | `GET /api/camera/status` | — |
| ZED 2i (Front) | `stZED` | `GET /api/zed/status` | — |
| GPS Receiver | `stGPS` | `GET /api/gnss` | — |
| Chassis Link | `stChassis` | `GET /api/wheel_odom` | `wheel_odom` |
| Chassis Battery | `stChassisBatt` | `GET /api/chassis_battery` | `battery` |

**Planting** (`data-chassis-feature="actuators"`) — **Job Code** text field
(`jobCodeInput`) + **Save** button → `POST /api/settings {jobCode}`; the code is
attached to every planted-seedling record.

**Machine Setup (PLC)** (`data-chassis-feature="plc"` — agrobot only). Buttons post
allow-listed commands to `POST /api/plc/machine`:

| Button | Command sent |
|--------|--------------|
| **Set Manual** (`plcSetManual`) | `SET_MANUAL` (disabled when already Manual) |
| **Set Auto** (`plcSetAuto`) | `SET_AUTO` (disabled when already Auto) |
| **Fault Reset** (`plcFaultReset`, red) | `FAULT_RESET` |
| **Home All** (`plcHomeAll`) | `HOME_ALL` |

Subsystem rows each carry an enabled pill (`plcAugerEn` / `plcPlanterEn` /
`plcRobotEn` / `plcAmrEn`) and Enable/Disable buttons posting
`ENABLE_*`/`DISABLE_*` (`AUGER`, `PLANTER`, `ROBOT`, `AMR`). These preconditions
(Auto mode + subsystem enabled + safety) are what a planting sequence needs
before it will actually move.

**Robot Arm** (`data-chassis-feature="plc"`) — nine pushbuttons posting to
`POST /api/plc/robot`: **Home**, **Motors On**, **Motors Off**, **Start**,
**Stop**, **Pause**, **Continue**, **Reset**, **Shutdown** (red; asks for
confirmation). All are **disabled until the Robot subsystem is enabled** in
Machine Setup.

**PLC Alerts** (`plcAlertsBody`, `data-chassis-feature="plc"`) — live fault /
warning banner text from `GET /api/plc/banner` (%MW1014 / %MW1030), polled every
5 s.

**Satellite Navigation (GNSS)** — read-only Fix Type / Satellites / HDOP cards
(`stGpsFix`, `stGpsSats`, `stGpsHdop`) from `GET /api/gnss`.

**Distance Traveled** (`data-chassis-feature="wheel_odom"`) — total odometry
distance (`odomDist`) and raw L/R pulse counts (`odomPulses`) from
`GET /api/wheel_odom`.

### 2.7 PLC Reference & Diagnostics panel

![PLC Reference panel](images/index-plc-reference.png)
*Read-only reference of every PLC tag the integration reads/writes, with live values, plus the full PLC symbol table.*

Opened by the CPU button (agrobot only). On first open it fetches
`GET /api/plc/tags` once and renders: a Gateway online/offline banner, the live
fault/warning banner, "Tags we read from the PLC" (each field shows a live value
polled every 1 s from `/api/plc/status`, `/api/plc/sequence`,
`/api/plc/auger_motor`), "Tags we write to the PLC" (the pushbutton command
sets), a "Reserved (declared, unused)" list, verification notes, and the full
PLC symbol table. This panel is documentation + diagnostics — it has no control
buttons of its own.

### 2.8 Event Log panel

![Event Log panel](images/index-event-log.png)
*Slide-in event log with All / Warn / Error filters and a clear button; entries expand to show a suggested fix.*

Opened by the bell button. Toolbar:

| Control | Action |
|---------|--------|
| **All** (`lfAll`) | Show all levels (`_setLogFilter('ALL')`). |
| **Warn** (`lfWarn`) | Filter to warnings. |
| **Error** (`lfError`) | Filter to errors. |
| **Clear** | Empty the client log (`_clearLog()`). |

Entries combine client-side events (connectivity, PLC, GNSS transitions, etc.)
and server events from `GET /api/events` (polled every 3 s). Each entry shows
timestamp, level, source, and message; clicking an entry with a suggestion
expands a "💡" hint line. A red badge on the bell shows unread warn/error count.

---

## 3. Adaptive UI behavior

On load, `index.html` calls **`GET /api/config`** (`_applyChassisConfig`) and
adapts itself to the active chassis (`agrobot` or `jackal`):

- **Feature hiding.** Every element tagged `data-chassis-feature="X"` is hidden
  when the config's `features.X === false`. So the same HTML serves both robots.
- **Velocity clamping.** `limits.maxLinear` / `limits.maxAngular` become
  `CFG.hardMaxLinear` / `hardMaxAngular`, which `teleop.js` uses to clamp every
  `/api/cmd_vel` command — the UI never exceeds the server's accepted range.
- **Scaling from the server.** `scaling.linearScale` (Modbus units per m/s) and
  `scaling.pulsePerM` (encoder calibration) come from the chassis YAML via the
  response; the JS must never hardcode them.
- **Wide mode.** When the server runs with `--wide` (`ui.wide` true), the video
  `<img>` elements switch to letterboxed `object-fit: contain`.

Which panels are **agrobot-only** (hidden on jackal via feature flags):

| Feature flag | Panels/controls it gates |
|--------------|--------------------------|
| `plc` | PLC status strip, PLC Reference button/panel, Machine Setup, Robot Arm, PLC Alerts. |
| `fwd2m` | The **→ 2 m** auto-drive button. |
| `modbus_slider` | The Modbus Speed (Master) slider in settings. |
| `wheel_odom` | Chassis Link status card, Distance Traveled section. |
| `battery` | Chassis Battery status card. |
| `actuators` | Planter/Auger/Both buttons, the Planted telemetry tile, and the Planting job-code section. **Shown on both chassis** (jackal keeps them cosmetic). |

Shown on **both** chassis regardless of flags: the cameras, WASD keypad, speed
readout/presets, REC, Detection, the GPS map + telemetry, and the GNSS / speed
sections of settings.

On `plc_combined.html` the same `data-chassis-feature` mechanism hides the
sidebar battery gauge when `features.battery` is false, and a separate
**`features.teleop === false`** ("mobile" / `--no-teleop`) mode adds
`body.no-drive`: it hides all wheel-drive controls, prevents the page from ever
publishing a `cmd_vel`, lands on the PLC tab, and shows the big Auger/Planter/Both
buttons instead.

---

## 4. Keyboard controls

Driving comes from the shared transport in `dashboard/js/teleop.js` plus each
page's WASD handler. The transport exposes `publishCmdVel(lin, ang)` and
`publishStop()`, clamps to `CFG.hardMaxLinear` / `hardMaxAngular`, and posts to
`POST /api/cmd_vel`. Commands are fire-and-forget at ~20 Hz (a 50 ms interval),
backed by the server's 0.5 s deadman.

Common behavior across `index.html`, `plc_combined.html`, and
`battery_test.html`:

| Key | Effect |
|-----|--------|
| **W** | Drive forward (`+linear`). |
| **S** | Drive backward (`−linear`). |
| **A** | Turn left (`+angular`). |
| **D** | Turn right (`−angular`). |

- Keys can be **combined** (e.g. W+A to arc). The active key(s) highlight.
- **Hold to drive** — a 50 ms loop republishes the current command while any key
  is held. On release there is a short (120–150 ms) debounce, then when no key
  remains held the page sends a single **stop** (`publishStop()`).
- On-screen W/A/S/D buttons mirror the keys via pointer/touch events.
- Typing in a text `<input>` is ignored by the key handler (so job-code entry
  doesn't drive the robot).
- On `index.html`, pressing any WASD key **cancels an in-progress 2 m auto-drive**
  (sends a stop; the server-side loop returns).

Page-specific keys:

- **`battery_test.html`** — **Spacebar** triggers the big STOP (`stopAll()`):
  clears keys, publishes stop, and halts the auto endurance loop.
- **`plc_combined.html`** — **Spacebar** cancels an in-progress 2 m drive
  (`_cancelFwd()`). WASD is ignored entirely in mobile/`no-drive` mode.

---

## 5. PLC dashboard — `plc_combined.html` (port 8769)

A single-page app: a left **sidebar** (navigation + persistent drive/actuator
controls + battery) and a **content area** that swaps between seven views. The
header shows two live pills: **AMR** (MOVING/IDLE, derived from drive velocity)
and **PLC** (link status + latency, from `GET /api/amr/poll`).

### 5.1 Sidebar (always visible)

![PLC page sidebar](images/plc-sidebar.png)
*Sidebar: view navigation, WASD keypad, speed presets, live velocity readout, Fwd/Bwd/REC/Det controls, actuator buttons, and battery gauge.*

**Navigation** — seven buttons calling `switchView(name)`: **Cameras**, **GPS**,
**PLC**, **HMI**, **Connectivity**, **Settings**, **Event Logs**.

**WASD block:**
- WASD keypad (`kw`/`ka`/`ks`/`kd`) — same drive behavior as [§4](#4-keyboard-controls).
- Speed presets **Slow** / **Med** / **Fast** (`setPreset(1|2|4)`) scaling the
  linear command; Med is the default.
- Live velocity readout: **Lin m/s** (`velLin`) and **Ang r/s** (`velAng`).

**Extra controls (2×2 grid):**

| Button | Element id | Action | Notes |
|--------|-----------|--------|-------|
| **2m Fwd** (label reflects configured distance) | `fwd2mBtn` | `moveForward2m('forward')` → `POST /api/fwd2m {speed, direction}`. Shows live progress. | Distance is set in Settings (`DRIVE_DISTANCE`, 0.1–20 m). |
| **2m Bwd** | `bwd2mBtn` | `moveForward2m('backward')`. | Both 2 m buttons disable while either runs. |
| **REC** | `trackRecBtn` | `toggleTrackRecord()` — GPS track + camera recording (`/api/record/start|stop`, `/api/track/save`). | Turns red while recording. |
| **Det** | `detBtn` | `toggleDetectionBtn()` — YOLO detection overlay on the camera view. | Turns blue while on. |

In mobile/`no-drive` mode the WASD keys, speed presets, velocity readout, and
2 m buttons are hidden; REC/Det remain.

**Actuator block** — three buttons, each with a status pill:

| Button | Element id | Action |
|--------|-----------|--------|
| **Auger** | `augerBtn` | `toggleActuator('auger')` |
| **Planter** | `planterBtn` | `toggleActuator('planter')` |
| **Auger + Planter** | `bothBtn` | `toggleActuator('both')` |

These fire the **same Clear-of-Ground handshake** as the battery-test page: a
press writes a momentary start pulse to the AMR→PLC command word
(`POST /api/amr/write {reg:5110|5111, value:1, pulse:true}`, self-clearing to 0),
then the button shows **"Working"** until the Clear-of-Ground status bit
(`%MW5100`/`5101` bit 1) returns to 1 = finished. State is driven by the 500 ms
`_pollPlc()` loop, so the sidebar and the PLC tab never disagree. Buttons are
**disabled when the PLC is offline** (`_plcConn !== true`); firing offline shows
"PLC offline". A completed **planter** cycle logs a seedling
(`POST /api/plant`) and drops a map pin.

**Battery gauge** (`sidebarBatt`, `data-chassis-feature="battery"`) — percentage,
bar, and voltage from `GET /api/chassis_battery` over the configured
`[minV, maxV]` range.

### 5.2 Cameras view

![Cameras tab](images/plc-camera-tab.png)
*Split camera feed (front full-frame + rear picture-in-picture) with an optional detection info bar.*

The same split camera + draggable PiP + swap behavior as the main dashboard
(§2.2). Detection is toggled from the sidebar **Det** button; the detection info
bar (`detBar`) shows front/rear person counts.

### 5.3 GPS view

![GPS tab](images/plc-gps-tab.png)
*Leaflet map on the left; a GNSS detail panel (status, satellites, HDOP, lat/lon in decimal + DMS, altitude, speed, heading) on the right.*

- **Map** with three overlay buttons: **+** (`mapZoomIn`), **−** (`mapZoomOut`),
  and **◕ re-centre** (`mapLocate` — recenters on the robot).
- **GNSS detail panel** (right) — a status LED (green fix / amber signal / grey),
  an age readout, and rows for Status, Satellites, HDOP, Latitude (decimal +
  DMS), Longitude (decimal + DMS), Altitude, Speed, and Heading. All from
  `GET /api/gnss` (polled ~1 s).

### 5.4 PLC view (handshake)

![PLC handshake tab](images/plc-handshake-tab.png)
*Two columns — PLC→AMR status registers with per-bit LEDs, and AMR→PLC command registers with fire/state buttons — plus a register quick reference, alert banner, and event log.*

The core AMR↔PLC handshake screen over registers **%MW5100–5112** (see
[`plc.md`](plc.md)). Two columns:

**Status (PLC → AMR), read every 500 ms** (`_applyStatus`) — cards for
**%MW5100 (Auger Status)** and **%MW5101 (Planter Status)**, each with a per-bit
LED + badge:
- bit 0 — Sequence Start Handshake
- bit 1 — Clear of Ground
- bit 2 — Cycle Complete

**Commands (AMR → PLC), written on change** (`_applyCmds`):

| Card | Element | Action | Notes |
|------|---------|--------|-------|
| **%MW5110 — Auger Command** | `cb-mw5110` | **Fire Auger Cycle** → `_fireActuator('auger')` (momentary pulse via `POST /api/amr/write`). Shows "Auger working…" while armed. | Same handshake as the sidebar actuators. |
| **%MW5111 — Planter Command** | `cb-mw5111` | **Fire Planter Cycle** → `_fireActuator('planter')`. | Completed planter cycle logs a seedling. |
| **%MW5112 — AMR State** | `cb-mw5112-0` / `-1` | **Stationary** (`_setCmd(5112,…,1)`), **Moving** (`…,2)`), and **Clear** (`…,0`). | Marked *auto-managed* — the server writes this from WASD activity; the buttons are a manual override. |

All command buttons are **disabled while the PLC link is down** (the 500 ms poll
is the heartbeat). Each write appends a line to the PLC event log with FC06
write + FC04 readback confirmation.

Below the columns:
- **Register Quick Reference** cheat-sheet — the bit meanings of %MW5100/5101/
  5110/5111/5112.
- **PLC Alerts banner** (`plc-banner-box`) — active fault/warning from
  %MW1014/%MW1030 (`GET /api/plc/banner`).
- **Event Log** (`plc-log-body`) — the low-level PLC read/write/change log with a
  **Clear** button.

### 5.5 HMI view

![HMI tab](images/plc-hmi-tab.png)
*Read-only mirror of the physical machine HMI: a menu of screens, live indicator lamps, value fields, gauges, and (on control screens) pushbutton/jog controls.*

A live, styled mirror of the machine's HMI screens (`GET /api/hmi/screens` for
the menu tree, `GET /api/hmi/read?screen=<id>` polled ~2.5 Hz per open screen).
See [`hmi.md`](hmi.md). Navigation mirrors the physical panel: a **MENU
SELECTION** button, per-screen menu buttons, and a **‹ Back** button walk a
navigation stack. A connection pill shows "PLC live" / "PLC offline", and a live
clock ticks.

Most of the HMI is **read-only** (indicator lamps, value fields, gauges,
disabled mirror buttons shown for fidelity). The exceptions are the **control
screens**, which do issue writes:
- **Pushbutton** presses → `POST /api/hmi/press {block, button}` (momentary pulse) —
  e.g. Servo ON/OFF, Home/Approach/Work position selection.
- **Hold-to-jog** buttons (▲ / ▼) → `POST /api/hmi/jog {block, button, action}`
  held down, refreshed every 200 ms, released on mouseup/touchend/blur; a
  server-side deadman clears the bit if refreshes stop.
- A few machine-mode buttons post to `POST /api/plc/machine`.

Writes are allow-listed server-side (PB words %MW5400–6500, FC06); anything
below %MW5000 is refused. The view returns HMI-unavailable text off a chassis
without a PLC.

### 5.6 Connectivity view

![Connectivity tab](images/plc-connectivity-tab.png)
*Subsystem status cards — PLC, rear camera, front ZED, GPS, and chassis — each with a live badge and last-seen timestamp.*

Read-only status cards (`_pollConn`, ~5 s), one per subsystem, each with an
on/off/partial/unknown badge, a detail line (address/endpoint), and a
last-updated timestamp:

| Card | Subsystem | Backing endpoint |
|------|-----------|------------------|
| PLC (Modbus TCP) | `%MW5100–5112 · 192.168.1.2:502` | `/api/amr/poll` |
| Rear Camera | ZED 2i rear | `/api/camera/status` |
| Front Camera (ZED 2i) | ZED 2i wide-angle | `/api/zed/status` |
| GPS / GNSS | GeoAstra RTU608BT | `/api/gnss` |
| Chassis (Robot Base) | Wheel encoders · Modbus RTU | `/api/wheel_odom` |

### 5.7 Settings view

![Settings tab](images/plc-settings-tab.png)
*Speed and drive-distance sliders (each with a matching numeric field) and a planting job-code field.*

**Speed Controls** — each row pairs a slider with an editable numeric field:
- **Modbus Motor Speed** (`slModbus` / `slModbusInput`, 0–32767 units).
- **Forward Speed (max)** (`slLinear` / `slLinInput`, 0.1–10 m/s).
- **Turn Speed (max angular)** (`slAngular` / `slAngInput`, 0.1–3 rad/s).
- **Drive Distance (Fwd/Bwd buttons)** (`slDistance` / `slDistInput`, 0.1–20 m) —
  sets how far the sidebar 2m Fwd/Bwd buttons drive; the button labels update
  live.
- **Apply & Update Robot** (`applyBtn`) → `POST /api/settings {maxLinear,
  maxAngular, modbusSpeed, driveDistance}`.

**Planting** — **Job Code** field (`jobCodeInput`) + **Save** →
`POST /api/settings {jobCode}`.

### 5.8 Event Logs view

![Event Logs tab](images/plc-logs-tab.png)
*Application-wide event log with category filter buttons and a clear button.*

The application-wide log (`_appLog`). A toolbar filters by category — **All**,
**Camera**, **GPS**, **PLC**, **Chassis**, **Settings**, **Conn** — with an entry
count and a **Clear** button. Entries carry a timestamp, category, message, and
optional debug hint. This is the browser-side aggregate log; the PLC tab's log
(§5.4) is a separate lower-level register log that also pipes into here.

---

## 6. Battery drain test — `battery_test.html` (port 8770)

![Battery test overview](images/battery-test-overview.png)
*The battery drain-test page: battery header, 2 m drives, manual actuators, WASD, speed slider, endurance cycle with STOP, and the cycle counters.*

A single scrolling page of cards for endurance testing. A battery header shows a
live percentage, bar, and voltage from `GET /api/chassis_battery` (polled every
5 s, mapped over the configured `[minV, maxV]`).

### 6.1 2 m Auto-Drive card

| Button | Element id | Action | Preconditions |
|--------|-----------|--------|---------------|
| **⬆ 2m Fwd** | `fwdBtn` | `drive2m('forward')` → `POST /api/fwd2m {speed, direction:"forward"}`. Shows a status line ("Forward 2 m …", "Done — X m", etc.). | Disabled while the Auto loop runs, another 2 m drive is in flight, or an actuator is mid-cycle. |
| **⬇ 2m Bwd** | `bwdBtn` | `drive2m('backward')`. | Same lockout. |

### 6.2 Actuators card

![Battery test actuators](images/battery-test-actuators.png)
*Manual Auger / Planter / Both buttons, each showing Idle / Working / Offline; drives lock out while an actuator runs.*

`data-chassis-feature="plc"` — **the whole card is hidden on a chassis without a
PLC** (`features.plc === false`).

| Button | Element id | Action |
|--------|-----------|--------|
| **🌀 Auger** | `augerBtn` | `fireActuator('auger')` |
| **🌱 Planter** | `planterBtn` | `fireActuator('planter')` |
| **⚙ Both** | `bothBtn` | `fireActuator('both')` (fires auger + planter) |

Each fires a **momentary start pulse** via `POST /api/amr/write {reg:5110|5111,
value:1, pulse:true}` and then tracks completion with the **Clear-of-Ground**
bit (`%MW5100`/`5101` bit 1), polled every 500 ms from `GET /api/amr/poll`. The
status text under each button reads **Idle → Working → Idle** (or **Offline**).
Preconditions/behavior:
- No firing while the Auto loop or a manual 2 m drive owns the robot.
- Firing an actuator **locks out the drive controls** (WASD + 2 m) until it
  completes — no drive/plant overlap.
- If Clear-of-Ground never drops within 8 s it reports "did not start (check
  Enable + Auto)"; a run exceeding 180 s reports a timeout.

### 6.3 Manual Drive (WASD) card

W/A/S/D on-screen buttons plus physical keys, per [§4](#4-keyboard-controls). The
keys are **disabled while the Auto loop runs, a 2 m drive is in flight, or an
actuator is mid-cycle**.

### 6.4 Speed card

A single **speed slider** (`speed`, 0.05 up to the chassis ceiling, capped at
3.0 m/s) with a live value readout. This one speed feeds manual WASD, the 2 m
drives, and the Auto loop (`onSpeed` sets `driveSpeed`).

### 6.5 Endurance Cycle card

![Endurance cycle](images/battery-test-auto.png)
*The Auto endurance loop control: AUTO/RUNNING button, a live phase indicator, and the large always-live STOP button.*

| Control | Element id | Action |
|---------|-----------|--------|
| **▶ AUTO** (becomes "● RUNNING") | `autoBtn` | `startAuto()` → `POST /api/battery_test/start {speed}`. Runs the server-side loop **forward 2 m → auger → planter → backward 2 m → auger → planter**, repeating until the pack voltage hits the cutoff. Disabled while running or during a manual drive. |
| **Phase indicator** | `phase` | Live text + colored dot from `GET /api/battery_test/status` (~0.7 s): "Driving forward 2 m", "Driving backward 2 m", "Planting — auger/planter", or "Idle". |
| **■ STOP** (big red, always live) | `stopBtn` | `stopAll()` → clears WASD, publishes stop, and `POST /api/battery_test/stop` (kills the loop, stops the robot, releases the auger/planter bits). Also bound to **Spacebar**. |

### 6.6 Cycle Counter card

Six read-only counters from `GET /api/battery_test/status`: **Forward** (`cFwd`),
**Backward** (`cBwd`), **Cycles** (`cCyc`), **Augers** (`cAug`), **Planters**
(`cPln`), and **Plant T/O** (plant timeouts, `cTo`).

---

## 7. Controls whose behavior could not be fully determined from the code

- **`index.html` "Both" actuator on the PLC path** — on a PLC chassis it
  posts `POST /api/plc/both {START}`; the exact machine motion that results is
  defined by the PLC ladder, not this repo, so "what the machine physically
  does" is out of scope of the UI code.
- **`%MW5112` AMR State manual buttons (PLC tab, §5.4)** — labeled *auto-managed*;
  pressing Stationary/Moving/Clear does write the register, but since the server
  also auto-writes %MW5112 from drive activity, a manual value may be
  immediately overwritten. The code does not document how long a manual override
  persists.
- **HMI control-screen writes (§5.5)** — the UI issues the press/jog writes, but
  the physical effect of each pushbutton/jog on the machine is governed by the
  PLC ladder and is not derivable from the dashboard source.

All other controls in this guide map directly to the code paths cited.
