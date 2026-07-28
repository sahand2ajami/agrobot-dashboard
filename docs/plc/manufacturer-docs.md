# PLC Manufacturer Documentation — LS Electric XGT

The PLC in the Agrobot tree-planting robot is made by **LS Electric** (formerly
LS Industrial Systems, a spin-off of LG) and belongs to the **XGT** family of
programmable logic controllers. The ladder program is written and maintained in
LS Electric's **XG5000** engineering software, and the Jetson dashboard talks to
the CPU's Ethernet port over **Modbus TCP** (with an LS Electric **FEnet**
Ethernet module also present on the machine).

This page collects the official LS Electric websites and manuals so a first-time
reader can go straight to the manufacturer's own documentation instead of
third-party mirrors. It does **not** repeat our project-specific wiring:

- **Our Modbus register map and address offsets** (FC04 reads at address − 1000,
  FC06 writes at address − 5000, the `%MW5100–5112` handshake block, etc.) are
  documented in [`../plc.md`](../plc.md).
- **The actual PLC project files** — the XG5000 project
  `GTS_Tree_Planter_26006_20260608.xgwx` (`.xgwx` is XG5000's project format) and
  the symbol/tag exports `GTS_Tree_Planter_26006_20260608.csv` and
  `GTS_Tree_Planter_symbols.csv` — live alongside this file in
  [`docs/plc/`](.).

> All links below are on LS Electric's own domains (`ls-electric.com` and its
> automation portal `sol.ls-electric.com`). Prefer these over the many
> third-party PLC-download sites, which host outdated or repackaged copies.

---

## Verified official links

| Resource | URL | What you'll find |
|----------|-----|------------------|
| LS Electric — global home | https://www.ls-electric.com | The manufacturer's corporate site (company, news, contact, regional links). |
| Solution Square — automation product catalog | https://sol.ls-electric.com/ww/en/product/category/0 | LS Electric's automation portal; browse the PLC / XGT product lines, HMI, drives, and communication modules. |
| Solution Square — Download Center | https://sol.ls-electric.com/ww/en/dlcenter | The document library / download center. **This is where XG5000 software and all XGT/XGK/XGI CPU and module manuals are downloaded** — search by product name or model number. |
| XGT FEnet I/F module manual (XGL-EFMTB) | https://sol.ls-electric.com/uploads/document/16411765512530/XGL-EFMTB_T8_Manual_V3.2_202011_EN.pdf | **Most important for our integration.** The XGT-series Fast Ethernet (FEnet) module manual — covers the FEnet I/F acting as a server for both the XGT dedicated protocol *and* **Modbus TCP**, i.e. exactly the settings that define our register/offset mapping. |
| XGB FEnet I/F module manual (XBL-EMTA) | https://sol.ls-electric.com/uploads/document/16735978115430/User_s%20Manual_XBL-EMTA_ENG_V1.8_202210.pdf | The XGB-family FEnet manual — the equivalent Ethernet/Modbus-TCP reference if the unit turns out to be an XGB CPU rather than an XGK/XGI. Consult only the one that matches the confirmed model (see below). |
| XGK/XGB instructions & programming manual | https://sol.ls-electric.com/uploads/document/16411828568550/XGK_XGB_Instruction_Manual_202012_V2.9_EN.pdf | The instruction set / ladder-programming reference for XGK and XGB CPUs — useful when reading or editing the ladder in XG5000. |

### Link verification notes

- **Resolve-verified** (fetched and confirmed to return the real page/PDF, not a
  404): the global home, the Solution Square product catalog, the Download
  Center, and all three PDF manuals (`XGL-EFMTB`, `XBL-EMTA`, `XGK_XGB`
  instructions). The FEnet/instruction PDFs are large (>10 MB / 5.6 MB) so they
  were confirmed to download rather than fully parsed.
- **Best-effort landing pages, not deep links:** LS Electric does not publish a
  single stable "XGT product page" or "XG5000 download" deep link that survives
  site updates. For those, use the **Solution Square product catalog** (XGT
  browsing) and the **Download Center** (XG5000 software + all manuals) above —
  both are verified — and search within them by model number. This avoids
  linking a fragile URL that could rot.

---

## Model confirmation

The exact CPU and FEnet part numbers are **not pinned anywhere in this repo** —
we know the platform is LS Electric **XGT** (the project is an XG5000 `.xgwx`
file and communicates over an FEnet module + the CPU Ethernet port), but the
specific CPU family (**XGK** ladder-based, **XGI** IEC-61131, or **XGB** compact)
and the FEnet model are unconfirmed. Confirm them before ordering spares or
picking the exact manual to follow:

1. **Physical nameplate.** Read the model number printed on the front of each
   module in the control cabinet:
   - CPU module — e.g. `XGK-CPUx…`, `XGI-CPUx…`, or `XGB-…`.
   - FEnet/Ethernet module — e.g. `XGL-EFMTB` (XGT) or `XBL-EMTA` (XGB).
2. **XG5000 project.** Open `GTS_Tree_Planter_26006_20260608.xgwx` in XG5000 and
   look at the **I/O configuration / base module list** — the configured CPU and
   the FEnet module in each slot are named there.
3. **Online in XG5000.** With XG5000 connected to the live PLC
   (**Online → Connection Settings**, then **Connect**), read the CPU type and
   the communication-module configuration directly off the running controller.

Once the model is known, pick the matching manual from the Download Center (XGK
vs. XGI vs. XGB CPU manual; `XGL-EFMTB` vs. `XBL-EMTA` FEnet manual) rather than
assuming.

---

## How this maps to our setup

Concretely, here is how the manufacturer docs connect to what we run (full
detail in [`../plc.md`](../plc.md)):

- **Modbus TCP on port 502.** The Jetson dashboard (`dashboard/plc_client.py`)
  connects to the **CPU's Ethernet port at `192.168.1.2:502`** and speaks plain
  Modbus TCP. The FEnet card at `192.168.1.1` speaks LS Electric's *own* XGT
  dedicated protocol and does **not** serve Modbus on 502 — so all our reads and
  writes target the CPU port. The FEnet manual (`XGL-EFMTB`) is the reference for
  how LS Electric implements Modbus TCP server mode on this platform.
- **The FEnet Modbus address mapping.** Our offsets — **FC04 (read input
  registers) at address − 1000** and **FC06 (write single register) at
  address − 5000**, with reads using FC04 (FC03 returns zeros on this unit) — are
  a consequence of how the FEnet Modbus server maps LS `%MW` device addresses to
  Modbus addresses. The FEnet module manual is where that mapping is specified;
  our concrete register table is in [`../plc.md`](../plc.md).
- **XG5000 for the ladder.** XG5000 (from the Download Center) is the tool used
  to open our `.xgwx` project, read/edit the ladder, and — via LS Electric's
  XG5000 simulator — run the program without the physical machine when
  bench-testing the dashboard against it.
