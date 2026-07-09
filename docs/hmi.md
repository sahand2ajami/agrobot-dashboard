# HMI live mirror (read-only)

The tree-planter has a physical HMI panel that reads a set of UDT instances in
the LS Electric PLC's M area and shows their members across ~28 screens,
navigated by an on-screen MENU of buttons (and an I/O sub-menu). The dashboard
mirrors both the navigation and the screens **read-only**: same buttons, same
live values,
polled over the same Modbus TCP link the rest of the PLC integration uses. The
HMI buttons in the dashboard only navigate — nothing here writes to the PLC.

It is served by `launch_dashboard_plc.sh` (`serve_plc.py` → `plc_combined.html`,
default port 8769) as a new **HMI** item in the left sidebar. On a chassis
without `plc.enabled` the routes 503 and the panel says so; a downed PLC is a
normal 200 with `connected:false` and every value shown as `—`.

## How the mapping works

Each HMI screen field is a member of a UDT instance. `plc_client.py` holds the
whole mapping as the single source of truth:

- **`HMI_UDT`** — member layout per UDT type: `(name, datatype, "byte.bit")`,
  transcribed verbatim from the XG5000 UDT editor. LS addresses are
  **byte.bit within the struct**, so a member decodes at
  `word = byte // 2`, `bit_in_word = (byte % 2) * 8 + bit`.
  (Cross-check: `EstopOkFL @4.0` in `ud_HMI_IND` → %MW1000 word 2 bit 0 →
  %MX16032, which is exactly `_REG["IND_ESTOP_OK_FL"]`.)
- **`HMI_BLOCKS`** — the read-relevant UDT *instances* (symbol → type + %MW
  base), mirroring the read rows of `docs/plc/GTS_Tree_Planter_symbols.csv`.
  The write-only `*PB` instances are intentionally excluded.
- **`HMI_SINGLES`** — standalone tags not inside a UDT. Currently
  `NodeCommsNOk` (%MW1048): one *not-OK* bit per EtherNet/IP node, inverted so
  the mirror shows comms-OK.
- **`HMI_SCREENS`** — the ~24 screens grouped by section, each listing the
  block(s) and/or explicit rows it shows. `_hmi_expand_layout` turns a screen
  into concrete panels/rows without touching the PLC (structure survives a
  PLC-down poll).

### Reads

All values come from **FC04** word reads of the block's span (chunked to stay
under the ~125-register FC04 limit — the 130-word `ud_HMI_Parameters` block
takes two reads). BOOLs are extracted as bits of those same words, so a whole
block is one coherent snapshot with no extra FC02 round-trips. `Fault_Result`
/ `Warning_Result` STRING banners are **not** decoded here — `get_banner()`
already serves them.

### Datatypes

`bool` (1 bit) · `uint`/`int`/`word` (1 word) · `udint`/`dint`/`real`
(2 words). 32-bit values decode **low-word-first** (`_HMI_WORD_LOW_FIRST`);
flip that flag if a bench check shows swapped halves — see "Verify on live PLC".

## HTTP API

| Method | Path | Notes |
|--------|------|-------|
| GET | `/api/hmi/screens` | Navigation tree: `{menus:{root:{title,columns:[{header,buttons:[{label,target}]}]}, io:{…}}, root:"root", titles:{id:title}}`. Mirrors the physical **MENU** (pp.2) button columns + the **IO MENU** sub-menu (pp.6). A button `target` is `screen:<id>` (open a data screen) or `menu:<key>` (open a sub-menu). |
| GET | `/api/hmi/read?screen=<id>` | Screen data + live values: `{connected, screen, title, section, layout, blocks:{SYMBOL:{member:value}}, singles:{name:value}, panels:[{title, rows:[{label, ref, kind, unit, value, ip?}]}]}`. Unknown screen → 404. |

Both 503 on a chassis without `plc.enabled`. The browser opens on the **MENU**
screen (mirrors the panel exactly: six button columns). Pressing a menu button
either opens a data screen or descends into the **IO MENU** sub-menu; a `‹ Back`
control (subbar) pops one level (the panel's on-screen **RETURN**), and the
top-right **MENU SELECTION** jumps straight back to the root menu. The active
screen polls `/api/hmi/read` at ~2.5 Hz (pauses when the HMI view is hidden).

### Frontend rendering (`plc_combined.html`)

Each screen carries a `layout` hint (from the screen def, else `HMI_LAYOUT`,
else `panels`). A bespoke template per PDF page family renders from `blocks` /
`panels`:

- `main` (pp.1) — auger/planter sequence boxes, mode + start/stop, AMR, safety lamps.
- `comms` (pp.3) — Ethernet/IP node list (name · IP · comms lamp, red = failed).
- `gauges` (pp.5) — pitch/roll dials + distances.
- `motion` (pp.13–21) — three columns (control buttons · measurements · status
  lamps); auto-detects Teknic slide vs LA36 axis; position-selection buttons
  vary per axis (`_posBtns`).
- `motor` (pp.16), `robot` (pp.11), `jaws` (pp.22), `enable` (pp.30).
- `panels` (default) — generic lamp + value cards (I/O lists, parameters,
  tolerances, safety layout, robot I/O).

On-screen control buttons are **always disabled** — this is a read-only mirror;
they only reflect state. The top banner shows the live fault/warning text
(reused from `/api/plc/banner`) and a clock.

## Adding / editing screens

- **New field on an existing screen** — add it to that screen's `members` list
  (or a `rows` entry) in `HMI_SCREENS`. The member must exist in the block's
  `HMI_UDT` layout; a test asserts every screen ref resolves.
- **New UDT instance** — add it to `HMI_BLOCKS` (and its type to `HMI_UDT` if
  new), then reference it from a screen.
- **New standalone tag** — add it to `HMI_SINGLES` and reference it as
  `single:<name>`.
- **New screen** — add it to `HMI_SCREENS` *and* wire a button to it in
  `HMI_MENU` (a test asserts every screen is reachable from the menu tree, and
  every menu target resolves). Give it a `layout` if it needs a bespoke
  template; otherwise it renders with `panels`.

Never hardcode a register or offset anywhere else. Run `pytest
tests/test_plc_client.py` — the `TestHmiLayout` / `TestHmiDecode` /
`TestHmiScreenRead` classes cover addressing, decode, and a simulated read.

## Display formatting (C-more fractional digits)

Numeric fields are shown with the same precision as the physical C-more panel,
from `HMI_DECIMALS` (keyed by block instance + member, since the same UDT is
shown with different precision on different screens). For an **integer** tag the
fractional digits are an *implied decimal* — the value is `raw / 10**frac`
(velocity `600000` → `6000.00`, torque `800` → `8.00`); for a **REAL** tag the
value is already engineering units, so it only rounds. `_hmi_fmt_value` renders
these as fixed-decimals strings in `read_hmi_screen`; the browser prints them
verbatim. The C-more entry min/max limits aren't stored — they clamp operator
entry on the panel and don't change a displayed value (this mirror is read-only).

## Address provenance & the auger-motor exception

Every UDT layout and instance base is transcribed from the PDF (pp.40–79), and
17 of them are independently corroborated by the bench-confirmed `_REG` map
(e-stops, enables, sequences, steps — a test pins these). **One block differs:**
`ud_HMI_MotorIND` (`HMI_IND_Auger` @ %MW2500). The PDF (p.73) lists status bits
first then velocities; the live PLC (`_REG.AUGER_MOTOR_*`, bench-confirmed) has
**velocities first (word +0/+1) and the status bits at word +2** with
Run/Fwd/Faulted at bits 0/1/2. The mirror follows `_REG` here. Only those three
bits are confirmed, so the PDF's Rev/On/Off/CW/CCW are **not** decoded (positions
unknown). `test_hmi_addresses_match_bench_reg` locks the whole mirror to `_REG`
so the two maps can't drift.

## Verify on the live PLC (read-only, no writes)

Two assumptions were made from the UDT layouts and should be confirmed once
against the real PLC — both are pure reads:

1. **32-bit / REAL word order.** Open a screen with a known non-zero measured
   value (e.g. Auger Gimbal X `PositionMeasured`). If positions/velocities read
   as absurd magnitudes, set `_HMI_WORD_LOW_FIRST = False` in `plc_client.py`.
2. **Node-comms bit order & polarity.** On the Communications screen, pull one
   device's cable and confirm the matching node (and only it) goes red. The
   mapping assumes `NodeCommsNOk` bits 0–9 in the device order on the physical
   Communications screen, inverted (bit set = failed).
