# PLC reference (GTS Tree Planter)

Source-of-truth files for the auger / planter / robot-arm PLC that the dashboard
drives **directly over Modbus TCP** — the old gRPC gateway is gone (see
`dashboard/plc_client.py`). For the integration itself, read the **PLC
Integration** section in the repo [README](../../README.md) and
[DEVELOPMENT.md](../../DEVELOPMENT.md), the full [PLC integration guide](../plc.md), and the
official [LS Electric manufacturer documentation](manufacturer-docs.md).

| File | What it is |
|------|------------|
| `GTS_Tree_Planter_26006_20260608.xgwx` | The LS Electric XG5000 PLC project from the controls engineer (binary container). |
| `GTS_Tree_Planter_26006_20260608.csv` | Full tag / comment export from the `.xgwx` project. |
| `GTS_Tree_Planter_symbols.csv` | The PLC's global symbol table (name / type / `%MW` address), the equivalent of XG5000's "export variables to CSV". |
| [`manufacturer-docs.md`](manufacturer-docs.md) | Official LS Electric websites + XGT / FEnet / XG5000 manuals. |

## Compatibility status (verified against this project)

Every command/status register the dashboard uses maps to the correct PLC struct:

- `HMI_PB` @ `%MW5000` (machine pushbuttons) · `HMI_PB_Auger` @ `%MW6500` ·
  `HMI_PB_Robot` @ `%MW6200`
- `HMI_IND` @ `%MW1000` (E-stop / gate / fault / mode / enables) ·
  `AugerSeq` / `PlanterSeq` @ `%MW2700` / `%MW2800` (in-cycle / step) ·
  `HMI_IND_Auger` @ `%MW2500` (VFD telemetry)
- **AMR ↔ PLC handshake @ `%MW5100`–`%MW5112`** — auger / planter **start commands**
  at `%MW5110` / `%MW5111`, **status** at `%MW5100` / `%MW5101`, **AMR state** at
  `%MW5112`. This is the block the auger/planter buttons and the battery test use.

Two items can only be confirmed against the live PLC:

1. **Exact bit index within each shared word** (e.g. `SET_AUTO` = bit 0,
   `START` = bit 6, `ENABLE_PLANTER` = bit 13 of `HMI_PB`). The struct/word is
   verified; the precise bit order is packed in the binary UDT definition.
   Bench-check: press each button, watch the matching indicator flip in XG5000.
2. **Modbus address base** — the PLC's FEnet Modbus-TCP server must expose `%MW` at
   the offsets the dashboard assumes (FC04 read = addr − 1000, FC06 write =
   addr − 5000; standard for LS XGT, but config-dependent).

> **Old vs. current handshake map.** The `AMR_2_PLC` (`%MW100`) / `PLC_2_AMR`
> (`%MW200`) arrays declared in the symbol table are the **old, unused** map: they
> sit **below** the FEnet write base (`%MW5000`) and can never be written over
> Modbus. The **active** AMR handshake is the bench-confirmed `%MW5100`–`%MW5112`
> block above. Do not wire anything to `%MW100` / `%MW200` — a test in
> `tests/test_plc_client.py` rejects any write target below `%MW5000`, so the old
> map cannot silently return. See [../plc.md](../plc.md) for the full rationale.
