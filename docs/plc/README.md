# PLC reference (GTS Tree Planter)

Source-of-truth files for the auger / planter / robot-arm PLC that the dashboard
drives through the gRPC gateway (see the **PLC integration** section in the repo
[README](../../README.md) and [DEVELOPMENT.md](../../DEVELOPMENT.md)).

| File | What it is |
|------|------------|
| `GTS_Tree_Planter_26006_20260608.xgwx` | The LS Electric XG5000 PLC project from the controls engineer (binary container). |
| `GTS_Tree_Planter_symbols.csv` | The PLC's global symbol table (name / type / `%MW` address), extracted from the `.xgwx` — the equivalent of XG5000's "export variables to CSV". |

## Compatibility status (verified against this project)

Every command/status register the gateway uses maps to the correct PLC struct:

- `HMI_PB` @ `%MW5000` (machine pushbuttons) · `HMI_PB_Auger` @ `%MW6500` ·
  `HMI_PB_Robot` @ `%MW6200`
- `HMI_IND` @ `%MW1000` (E-stop / gate / fault / mode / enables) ·
  `AugerSeq`/`PlanterSeq` @ `%MW2700`/`%MW2800` (in-cycle / step) ·
  `HMI_IND_Auger` @ `%MW2500` (VFD telemetry)

Two items can only be confirmed against the live PLC:

1. **Exact bit index within each shared word** (e.g. `SET_AUTO`=bit 0, `START`=bit 6,
   `ENABLE_PLANTER`=bit 13 of `HMI_PB`). The struct/word is verified; the precise bit
   order is packed in the binary UDT definition. Bench-check: press each button, watch
   the matching indicator flip in XG5000.
2. **Modbus address base** — the PLC's Modbus-TCP/FEnet server must expose `%MW` at the
   offsets the gateway assumes (standard for LS XGK, but config-dependent).

> The `AMR_2_PLC` (`%MW100`) / `PLC_2_AMR` (`%MW200`) arrays are declared in the PLC but
> **not referenced anywhere in the ladder** in this build — they are reserved for a future
> AMR handshake and are not used by the current integration (which drives the `HMI_PB_*`
> registers).
