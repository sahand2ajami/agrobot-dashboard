# PLC Integration — Field Notes & Pending Changes

Running log of everything learned during the June 2026 bench sessions.
This file is the reference for future work — read it before touching any PLC-related code.

---

## Network Map

| IP | MAC | Device | Role |
|----|-----|--------|------|
| `192.168.1.2` | `00:0b:29:83:5c:52` | LS Electric FEnet (PLC CPU Ethernet) | Modbus TCP server for PLC M-memory |
| `192.168.1.4` | `00:01:fc:a0:19:21` | Contec industrial device | Robot arm controller ("Agrobot Tree Planter") |
| `192.168.1.6` | `00:07:46:51:aa:4e` | Unknown industrial device | Returns Modbus exception 2 (illegal address) for most registers |

**Jetson:** `192.168.1.100/24` on `eno1`.

> **VMware IP conflict warning:** The user's laptop LAN cable connects to the same switch.
> A VMware VM on the laptop is configured to `192.168.1.2` (same as FEnet). When the
> laptop's LAN cable is plugged in, the VM answers ARP for `.2` first and intercepts all
> **FC03 (holding register)** requests, returning empty data. The Jetson never reaches the
> real PLC until the VM is powered off or the laptop's LAN cable is unplugged. The VM does
> NOT handle FC04 (input registers), so FC04 requests pass through to the real FEnet.
> Confirmed by MAC OUI `00:0b:29` (exclusively VMware, Inc.).

---

## FEnet Modbus Address Mapping

Configured in XG5000 → Online → Standard Settings → FEnet → Driver Setting → Modbus Settings:

| Area | Base PLC Address | Modbus Function Code | Offset Formula |
|------|-----------------|----------------------|----------------|
| Word Read | `%MW1000` | **FC04** (input registers) | `reg = plc_addr - 1000` |
| Word Write | `%MW5000` | **FC06/FC16** (write holding registers) | `reg = plc_addr - 5000` |
| Bit Read | `%MX0` | FC02 (discrete inputs) | no offset |
| Bit Write | `%MX1000` | FC05/FC15 (write coils) | `reg = plc_addr - 1000` |

**Critical:** The FEnet exposes its read area via **FC04 (input registers)**, NOT FC03
(holding registers). Using FC03 for reads returns 0 even when the PLC has live data.
This was confirmed on 2026-06-17 by reading `%MW1410` (a live register that fluctuates
between 7 and 8): FC03 returned 0 every time; FC04 returned the correct live value.

**Read/write use the same underlying PLC M-memory.** So after writing `%MW5000` via
FC06, you can confirm it by reading `%MW5000` via FC04 (reg = 5000 - 1000 = 4000).
A matching readback means the write landed; a mismatch means the PLC ladder overwrote it.

---

## Connection Behavior

The FEnet drops the TCP connection periodically (observed `ConnectionResetError: [Errno 104]
Connection reset by peer`). The FEnet is configured for 15 s idle timeout (3 connections max).

All Modbus calls must catch `ConnectionResetError`, reconnect, and retry once before
reporting failure. This is now implemented in `plc_test.py` and must be added to
`plc_client.py`.

---

## Problem 1 — IP Conflict: VMware VM Intercepting FC03 Requests  *(root cause confirmed)*

**What happened:** During early testing, all reads via FC03 returned 0 and all writes
"succeeded" (no Modbus error) but never appeared in XG5000. A full scan of 6000 registers
returned no live values. Setting a value in XG5000 (`%MW5170 = 12345`) and scanning for it
from the Jetson found nothing.

**Root cause:** A VMware VM on the laptop (MAC `00:0b:29:83:5c:52`) claimed `192.168.1.2`
on the LAN. Since the laptop's LAN cable was plugged into the switch, the VM answered
ARP for `.2` before the PLC FEnet could. The Jetson's entire session was talking to the
VM's isolated Modbus register buffer, with zero connection to PLC memory.

The VM handles FC03 (holding registers) — responding with 0 for reads and ACK for writes
into a flat buffer. It does not handle FC04 (input registers), which is why switching to
FC04 immediately started returning live PLC data once the function code was corrected.

**Confirmed by:**
- `arp -n 192.168.1.2` returned `00:0b:29:83:5c:52` (VMware OUI)
- Write/readback at different offsets showed independent buffers (write @ reg 0, readback @ reg 4000 = 0)
- Switching to FC04 at reg 410 returned 7/8 fluctuating values matching XG5000's `%MW1410`

**Ongoing risk:** Every time the laptop's LAN cable is connected to the switch, this VM
may reappear and intercept FC03 requests. Keep the VM powered off when doing PLC work,
or remove it from the network by unplugging the LAN cable.

---

## Problem 2 — All Code Was Using FC03 Instead of FC04 for Reads  *(fixed)*

`plc_read.py`, `plc_test.py`, and `dashboard/plc_client.py` all used
`read_holding_registers` (FC03) for every read. The FEnet serves reads via FC04.

**Fixed in:** `plc_read.py` (2026-06-17), `plc_test.py` (2026-06-17),
`dashboard/plc_client.py` (2026-06-18 — `_read()` changed to `read_input_registers`
for `%MW` and `read_discrete_inputs` for `%MX`).

---

## Problem 3 — FEnet Address Offsets Were Wrong in All Code  *(fixed)*

The original code assumed offset 0 (Modbus register N = PLC `%MWN`). The FEnet
applies offsets: reads subtract 1000, writes subtract 5000.

**Corrected offset formulas:**
```
Read  %MWx:  Modbus reg = x - 1000   (min x = 1000)
Write %MWx:  Modbus reg = x - 5000   (min x = 5000)
```

**Fixed in:** `plc_read.py`, `plc_test.py`, `dashboard/plc_client.py` (2026-06-18).
Module-level constants `_FENET_READ_WORD_BASE = 1000` / `_FENET_WRITE_WORD_BASE = 5000`
added. The `_write()` path now logs a clear warning and returns `False` (rather than
silently writing the wrong register) for any `%MW` address below 5000 — correct
behaviour until Problem 4 is resolved.

**Current correct register table:**

| Symbol | PLC addr | Wrong reg (current) | Correct read reg (FC04) | Correct write reg (FC06) |
|--------|----------|---------------------|------------------------|--------------------------|
| `IND_MODE_STATUS` | `%MW1000` | 1000 | **0** | — |
| `AUGER_MOTOR_VEL_TARGET` | `%MW2500` | 2500 | **1500** | — |
| `AUGER_MOTOR_VEL_ACTUAL` | `%MW2501` | 2501 | **1501** | — |
| `AUGER_STEP` | `%MW2701` | 2701 | **1701** | — |
| `PLANTER_STEP` | `%MW2801` | 2801 | **1801** | — |
| `HMI_PB_MachineCtrl` | `%MW5000` | 5000 | 4000 (readback) | **0** |
| `HMI_PB_MachineCtrl2` | `%MW5001` | 5001 | 4001 (readback) | **1** |
| `ROBOT_PB_CMD` | `%MW6200` | 6200 | 5200 (readback) | **1200** |
| `AUGER_AMR_WORD` | `%MW100` | 100 | — | **impossible (see Problem 4)** |
| `PLANTER_AMR_WORD` | `%MW101` | 101 | — | **impossible (see Problem 4)** |

---

## Problem 4 — %MW100 / %MW101 Not Writable via Current FEnet Config  *(unresolved)*

`AMR_2_PLC[0]` (`%MW100`) and `AMR_2_PLC[1]` (`%MW101`) are the auger/planter
handshake registers. The FEnet write area starts at `%MW5000`, so `%MW100` requires
Modbus write register `100 - 5000 = -4900` — impossible.

**Options for the PLC engineer:**

| Option | Change | Notes |
|--------|--------|-------|
| A (preferred) | Set FEnet Word Write Area to `%MW0` | All M-memory writable; write offset becomes 0 |
| B | Move `AMR_2_PLC[]` to `%MW5002`/`%MW5003` | Ladder references must be updated too |

Until one of these is done, auger/planter control via the dashboard cannot work.

---

## Problem 5 — `plc_test.py` Rejected All Non-Binary Values  *(fixed)*

Original code had `if value not in (0, 1)` which blocked all machine commands
(e.g. `START = 64`, `STOP = 128`, `ENABLE_AUGER = 2048`). Fixed to accept 0–65535.

---

## Problem 6 — Auger Jog Register Addresses Unknown  *(unresolved)*

The PLC HMI (`dashboard/plc_hmi.html`) has fully-wired press-and-hold jog buttons
(▲ Jog Up / ▼ Jog Down) with a 600 ms watchdog auto-stop. The server side
(`dashboard/plc_hmi_serve.py`) accepts the jog REST calls but cannot write to the
PLC because the `%MW` addresses for auger jog up/down have not been confirmed.

**To wire up:**
1. PLC engineer identifies the auger jog registers (e.g., bits in `AMR_2_PLC` or a
   dedicated jog word in the FEnet write area `%MW5000`+).
2. Add `AUGER_JOG_UP` and `AUGER_JOG_DOWN` to `dashboard/plc_client._REG`.
3. Add a `jog_auger(direction, active)` method to `PlcClient`:
   - `active=True` → write the jog value to the register (sustained, not pulsed)
   - `active=False` → write 0
4. Call it from `plc_hmi_serve.py`'s `/api/hmi/auger/jog` handler.

---

## What Has Been Fixed (as of 2026-06-18)

| File | Changes |
|------|---------|
| `plc_read.py` | FC04 for reads; `READ_OFFSET = 1000`; rejects addresses below `%MW1000` |
| `plc_test.py` | FC04 for reads; FC06 for writes; correct offsets; full 16-bit value range; readback now confirms write via FC04; reconnect-on-`ConnectionResetError` |
| `dashboard/plc_client.py` | FC04 for `%MW` reads; FC02 for `%MX` reads; correct FEnet offsets in `_read()` and `_write()`; `_write()` warns + fails gracefully for out-of-range addresses |

---

## What Still Needs to Be Done

1. **PLC engineer:** Resolve Problem 4 (lower write area to `%MW0` or relocate `AMR_2_PLC`).
2. **PLC engineer:** Define auger jog register addresses (Problem 6).
3. **Verify on real hardware:** Run `plc_test.py` against physical PLC, write a value to
   `%MW5000`+, confirm it appears in XG5000 at the correct `%MW` address.
4. **Confirm bit layout:** Bench-verify the bit indices in `%MW5000`/`%MW5001` for machine
   commands (`SET_AUTO`, `START`, `ENABLE_AUGER`, etc.) against XG5000 ladder monitor.
5. **Wire jog:** Once jog registers confirmed, add to `plc_client._REG` and implement
   `PlcClient.jog_auger()` (see Problem 6).
6. **Add Planter & Machine Status pages** to the PLC HMI (`dashboard/plc_hmi.html`).
