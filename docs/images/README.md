# Screenshot capture guide — dashboard UI

This directory holds the screenshots referenced by [`../ui-guide.md`](../ui-guide.md).
None are committed yet — the placeholders below must be captured on the Jetson
with live cameras, GPS, and (ideally) a reachable PLC so the panels show real
data instead of dashes. Until a file exists, the `![...]` reference in
`ui-guide.md` renders as broken-image alt text; that is expected.

## Why a human has to do this

The dashboard only renders meaningfully when the ROS stack and camera/GPS/PLC
hardware are running on the device. A headless or hardware-free capture shows
empty video panes, `—` telemetry, and offline PLC chips, which defeats the
purpose of the screenshots. Capture from a machine that can see the live UI.

## How to capture

1. **On the Jetson**, start the stack for the page you want to shoot (see the
   table below). Each script prints the LAN URL and port on startup.
2. **View the page** one of two ways:
   - On the Jetson desktop over NoMachine / a local browser, or
   - From a laptop/tablet on the same network at `http://<jetson-ip>:<port>`.
   The printed URL uses the resolved port; the defaults are 8766 / 8769 / 8770.
3. **Drive/enable as needed** so the panel under test shows real state — e.g.
   open the ⚙ PLC tab only once the PLC link is up, toggle Detection on before
   shooting the detection overlay, acquire a GPS fix before the map shots.
4. **Capture** the full browser viewport (or crop to the panel named in the
   caption). A 1280×800 or larger window matches the layout the guide describes.
5. **Save** into this directory using the exact filename from the list below
   (lowercase, hyphenated, `.png`). Filenames must match `ui-guide.md` exactly
   or the embed stays broken.
6. Re-open `ui-guide.md` in a Markdown viewer to confirm each image resolves.

| Page | Launch script | Default port |
|------|---------------|--------------|
| `index.html` (main teleop) | `./launch_dashboard.sh` (or `./launch_dashboard_wide.sh`) | 8766 |
| `plc_combined.html` (4-tab PLC) | `./launch_dashboard_plc.sh` | 8769 |
| `battery_test.html` (drain test) | `./launch_dashboard_battery_test.sh` | 8770 |

Tip: the layout is responsive. For the mobile/`no-drive` variants noted in the
guide, narrow the browser window (or use device emulation) before capturing.

## Expected filenames

### Main dashboard — `index.html` (port 8766)

| Filename | Shows |
|----------|-------|
| `index-overview.png` | Whole page: header, split camera, drive controls, map, telemetry |
| `index-camera.png` | Camera area — front full-frame + rear picture-in-picture, LIVE badge |
| `index-drive-controls.png` | WASD keypad, speed readout + Slow/Normal/Fast presets, action cluster (Planter/Auger/Both, 2 m, REC, Detection), PLC status strip |
| `index-map.png` | Leaflet map with GPS marker, zoom / compass / locate overlay buttons |
| `index-telemetry.png` | Bottom telemetry strip (Lat/Lon/Alt/Planted/Chassis) |
| `index-settings.png` | Advanced Settings panel (speed sliders, System Status, Planting, Machine Setup, Robot Arm, PLC Alerts, GNSS, Odometry) |
| `index-plc-reference.png` | PLC Reference & Diagnostics side panel |
| `index-event-log.png` | Event Log side panel with filter buttons |

### PLC dashboard — `plc_combined.html` (port 8769)

| Filename | Shows |
|----------|-------|
| `plc-sidebar.png` | Left sidebar: nav, WASD, speed presets, velocity readout, Fwd/Bwd/REC/Det, actuator buttons, battery gauge |
| `plc-camera-tab.png` | Cameras view (split feed + PiP) |
| `plc-gps-tab.png` | GPS view — map plus right-hand GNSS detail panel |
| `plc-handshake-tab.png` | PLC view — Status / Commands columns, register quick reference, alerts, event log |
| `plc-hmi-tab.png` | HMI view — read-only mirror of a machine HMI screen |
| `plc-connectivity-tab.png` | Connectivity view — subsystem status cards |
| `plc-settings-tab.png` | Settings view — speed / distance sliders, Planting job code |
| `plc-logs-tab.png` | Event Logs view with category filters |
| `plc-mobile-actuators.png` | (Optional) `--no-teleop` mobile mode: big Auger/Planter/Both buttons on the PLC tab |

### Battery drain test — `battery_test.html` (port 8770)

| Filename | Shows |
|----------|-------|
| `battery-test-overview.png` | Whole page: battery header, 2 m drives, actuators, WASD, speed, endurance cycle, STOP, counters |
| `battery-test-actuators.png` | Manual Auger / Planter / Both cards with status text |
| `battery-test-auto.png` | Endurance Cycle card mid-run (phase indicator + counters) |
