# HMI live mirror (read-only)

The tree-planter has a physical HMI panel that reads a set of UDT instances in
the LS Electric PLC's M area and shows their members across ~24 screens. The
dashboard mirrors those screens **read-only**: it shows the same live values,
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
| GET | `/api/hmi/screens` | Menu structure: `{sections:[{section, screens:[{id,title}]}]}` — drives the on-screen **MENU** of buttons. |
| GET | `/api/hmi/read?screen=<id>` | Screen data + live values: `{connected, screen, title, section, layout, blocks:{SYMBOL:{member:value}}, singles:{name:value}, panels:[{title, rows:[{label, ref, kind, unit, value}]}]}`. Unknown screen → 404. |

Both 503 on a chassis without `plc.enabled`. The browser opens on the **MENU**
screen (mirrors the panel's menu), each button navigates to a screen, and each
screen has a **MENU SELECTION** button back. The active screen polls
`/api/hmi/read` at ~2.5 Hz (pauses when the HMI view is hidden).

### Frontend rendering (`plc_combined.html`)

Each screen carries a `layout` hint. Bespoke templates reproduce the physical
HMI layout from `blocks` (three-column `motion` screens with control buttons /
measurements / indicator lamps, `main` with sequence boxes + mode/start-stop +
safety lamps, `gauges` with pitch/roll dials, `motor`, `robot`); everything
else falls back to the generic `panels` renderer (lamp + value cards). The
on-screen control buttons are **always disabled** — this is a read-only mirror.
The top banner shows the live fault/warning text (reused from
`/api/plc/banner`) and a clock.

## Adding / editing screens

- **New field on an existing screen** — add it to that screen's `members` list
  (or a `rows` entry) in `HMI_SCREENS`. The member must exist in the block's
  `HMI_UDT` layout; a test asserts every screen ref resolves.
- **New UDT instance** — add it to `HMI_BLOCKS` (and its type to `HMI_UDT` if
  new), then reference it from a screen.
- **New standalone tag** — add it to `HMI_SINGLES` and reference it as
  `single:<name>`.

Never hardcode a register or offset anywhere else. Run `pytest
tests/test_plc_client.py` — the `TestHmiLayout` / `TestHmiDecode` /
`TestHmiScreenRead` classes cover addressing, decode, and a simulated read.

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
