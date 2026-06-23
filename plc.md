# PLC Integration Guide

The Agrobot tree-planting robot uses an **LS Electric PLC** to control the auger and planter.
The Jetson communicates with the PLC directly over **Modbus TCP** on the wired LAN.

---

## Network Setup

| Device | IP | Role |
|--------|----|------|
| Jetson (this machine) | `192.168.1.100` on `eno1` | Dashboard / Modbus client |
| PLC CPU Ethernet port | `192.168.1.2:502` | Modbus TCP server |
| Robot arm controller | `192.168.1.4` | Contec, separate protocol |

> **Laptop LAN cable warning:** If a Windows laptop is plugged into the same switch,
> confirm no VMware VM is running with IP `192.168.1.2`. A running VM intercepts Modbus
> requests and silently returns fake data. Power off the VM or unplug the laptop's LAN cable
> before doing any PLC work. You can verify you're talking to the real PLC by checking
> `arp -n 192.168.1.2` — the MAC should NOT start with `00:0b:29` (VMware OUI).

---

## How Modbus Addressing Works on This PLC

The PLC's FEnet Ethernet module maps PLC memory to Modbus registers using fixed offsets:

| Operation | Function Code | Formula | Example |
|-----------|--------------|---------|---------|
| **Read** a word | FC04 (input registers) | `modbus_reg = plc_addr − 1000` | `%MW5100` → FC04 reg **4100** |
| **Write** a word | FC06 (single register write) | `modbus_reg = plc_addr − 5000` | `%MW5110` → FC06 reg **110** |

**Important:** Reads use **FC04**, not FC03. FC03 returns 0 on this FEnet even when the
PLC has live data — this is a known quirk of the XG5000 FEnet configuration.

After writing a register with FC06, you can read it back with FC04 to confirm the write
landed. Both function codes access the same underlying PLC M-memory.

---

## AMR ↔ PLC Handshake Registers

These are the only registers the dashboard reads and writes. All addresses are above the
FEnet floors (%MW1000 read, %MW5000 write), so no PLC reconfiguration is needed.

### PLC → AMR: Status Registers (read-only, FC04)

The PLC writes these. The dashboard reads them every 500 ms.

#### `%MW5100` — Auger Status (FC04 reg 4100)

| Bit | Decimal Value | Meaning |
|-----|--------------|---------|
| 0 | **1** | Sequence Start Handshake |
| 1 | **2** | Auger Clear of Ground |
| 2 | **4** | Auger Cycle Complete |

Example: if you read `%MW5100 = 6`, bits 1 and 2 are set → auger is clear of ground AND cycle is complete.

#### `%MW5101` — Planter Status (FC04 reg 4101)

| Bit | Decimal Value | Meaning |
|-----|--------------|---------|
| 0 | **1** | Sequence Start Handshake |
| 1 | **2** | Planter Clear of Ground |
| 2 | **4** | Planter Cycle Complete |

---

### AMR → PLC: Command Registers (read/write, FC06 write · FC04 readback)

The dashboard writes these. The PLC reads them. After each write, the server reads back
the register to confirm the value landed.

#### `%MW5110` — Auger Command (FC06 reg 110, readback FC04 reg 4110)

| Write Value | Meaning |
|-------------|---------|
| **1** | Bit 0 set — Auger Start Sequence active |
| **0** | Idle (clear the command) |

#### `%MW5111` — Planter Command (FC06 reg 111, readback FC04 reg 4111)

| Write Value | Meaning |
|-------------|---------|
| **1** | Bit 0 set — Planter Start Sequence active |
| **0** | Idle (clear the command) |

#### `%MW5112` — AMR State (FC06 reg 112, readback FC04 reg 4112)

| Write Value | Meaning |
|-------------|---------|
| **1** | Bit 0 set — AMR is Stationary |
| **2** | Bit 1 set — AMR is Moving |
| **0** | Unknown / not reporting |

---

## Tools

Three tools are available. Use them in the order below depending on what you need.

### 1. `plc_read.py` — Quick read-only spot check

Connects to the PLC and reads a predefined set of registers. Use this to verify
connectivity and see the current values of the handshake registers at a glance.

```bash
cd /home/jetson/dual/dual-robot-dashboard
python3 plc_read.py
```

Output format:
```
%MW5100 = 4  (FC04 reg 4100)    # Auger Cycle Complete bit set
%MW5101 = 0  (FC04 reg 4101)    # Planter idle
```

Values are always in **decimal**. This script is read-only — it never writes to the PLC.

---

### 2. `plc_test.py` — Interactive read/write terminal

Use this for manual testing: read any register, write a value, and immediately get a
readback confirmation. This is the lowest-level tool and the best way to verify that
writes are reaching the PLC ladder.

```bash
cd /home/jetson/dual/dual-robot-dashboard
python3 plc_test.py
```

You will see a prompt (`>`). Commands:

| Command | What it does | Example |
|---------|-------------|---------|
| `r <plc_addr>` | Read one word via FC04 | `r 5100` |
| `<plc_addr> <value>` | Write one word via FC06, then read back | `5110 1` |
| `q` | Quit | |

All addresses are **PLC addresses** (the `%MW` number). The tool computes the
Modbus register number for you and prints it in the output.

**Example session — start the auger sequence:**
```
> r 5100
  %MW5100 = 0  (FC04 reg 4100)          ← auger idle

> 5110 1
  wrote %MW5110 = 1  (FC06 write reg 110)
  readback %MW5110 = 1  (FC04 read reg 4110)  ✓ confirmed   ← write landed

> r 5100
  %MW5100 = 1  (FC04 reg 4100)          ← PLC confirmed sequence start

> 5110 0
  wrote %MW5110 = 0  (FC06 write reg 110)
  readback %MW5110 = 0  (FC04 read reg 4110)  ✓ confirmed   ← command cleared
```

**Example session — report AMR state:**
```
> 5112 2
  wrote %MW5112 = 2  (FC06 write reg 112)
  readback %MW5112 = 2  (FC04 read reg 4112)  ✓ confirmed   ← AMR Moving

> 5112 1
  wrote %MW5112 = 1  (FC06 write reg 112)
  readback %MW5112 = 1  (FC04 read reg 4112)  ✓ confirmed   ← AMR Stationary
```

If you see `✗ mismatch`, the PLC ladder overwrote the register immediately after the
write — this is expected if the PLC program has logic that clears the command bits.

---

### 3. `launch_plc2.sh` — Live web dashboard

Starts a local web server (port 8768) with a browser HMI showing all handshake
registers in real time and buttons to write command values. This is the primary
operator interface.

```bash
cd /home/jetson/dual/dual-robot-dashboard
./launch_plc2.sh
```

Then open a browser at:
- **Local (on Jetson):** `http://localhost:8768`
- **From another device on the network:** `http://192.168.1.100:8768`

**Options:**

| Option | Default | Description |
|--------|---------|-------------|
| `--port N` | 8768 | HTTP listen port |
| `--plc-host H` | 192.168.1.2 | PLC Modbus TCP host |
| `--plc-port N` | 502 | PLC Modbus TCP port |
| `--headless` | off | Skip opening a browser (use when running remotely or as a service) |

**Examples:**
```bash
./launch_plc2.sh                           # default settings, opens browser
./launch_plc2.sh --headless                # serve only, no browser
./launch_plc2.sh --plc-host 192.168.1.2   # explicit PLC address
```

**What the dashboard shows:**

- **PLC → AMR panel (left):** Live LED indicators for each bit of `%MW5100` and `%MW5101`.
  Green LED = bit is 1. Polled every 500 ms.
- **AMR → PLC panel (right):** Buttons to write values to `%MW5110`, `%MW5111`, `%MW5112`.
  The current register value is always read back and shown after each write.
- **Event log (bottom):** Every poll change and every write is logged with timestamps and
  the exact Modbus register numbers and decimal values used, matching `plc_test.py` format:
  ```
  14:23:01.123  wrote %MW5110 = 1  (FC06 reg 110)
  14:23:01.134  readback %MW5110 = 1  (FC04 reg 4110)  ✓ confirmed
  14:23:00.456  %MW5100 bit 1 (Clear of Ground): 0 → 1
  ```

The dashboard degrades gracefully if the PLC is unreachable — it shows "PLC offline"
and keeps retrying in the background. It never crashes on a lost connection.

---

## Troubleshooting

**Dashboard shows "PLC offline" / `plc_test.py` says "CONNECT FAILED"**
1. Check the Jetson has an address on `eno1`: `ip addr show eno1` should show `192.168.1.100/24`.
2. Ping the PLC: `ping 192.168.1.2`. If it times out, check the LAN cable.
3. Check for the VMware conflict: `arp -n 192.168.1.2`. MAC starting with `00:0b:29` = VM in the way — power it off.

**Reads return 0 for everything**
Almost always the VMware conflict. See above.

**Write says `✓ confirmed` but PLC doesn't act on the command**
The write reached the PLC M-memory (`%MW5110` etc.), but the PLC ladder logic gates
actual motion on Auto mode + subsystem enabled + safety interlocks. Check XG5000 ladder
monitor to confirm the PLC is in Auto mode and all enables are set.

**`✗ mismatch` on readback**
The PLC ladder wrote a different value to the register between the FC06 write and the
FC04 readback (~10 ms later). This is normal if the PLC program clears command bits
after acknowledging them.

**Connection drops mid-session**
The FEnet closes idle TCP connections after ~15 seconds. All three tools reconnect
automatically on the next operation — you don't need to restart anything.
