"""Interactive Modbus TCP register writer/reader for PLC testing.

FEnet address mapping (XG5000 Standard Settings → FEnet):
  Word Read Area:  %MW1000  →  Modbus INPUT  register 0 = PLC %MW1000  (FC04)
  Word Write Area: %MW5000  →  Modbus WRITE  register 0 = PLC %MW5000  (FC06)

  Read  formula: modbus_reg = plc_address - 1000  (min readable:  %MW1000)
  Write formula: modbus_reg = plc_address - 5000  (min writable:  %MW5000)

The FEnet serves its read area via FC04 (input registers). Writes use FC06
(write single holding register) against the write area. Both map to the same
PLC M-memory, so reading %MW5000 via FC04 (reg 4000) after writing confirms
the value landed correctly.

WARNING: %MW100 / %MW101 (AMR_2_PLC handshake registers) are BELOW the FEnet
write area floor (%MW5000) and cannot be written via this FEnet configuration.
The PLC engineer must lower the FEnet Word Write Area to %MW0 (or move those
registers to %MW5002+) before Jetson can control the auger/planter.

Usage: python3 plc_test.py
"""

from pymodbus.client import ModbusTcpClient

HOST         = "192.168.1.2"
PORT         = 502
READ_OFFSET  = 1000   # FEnet Word Read Area base  (%MW1000)
WRITE_OFFSET = 5000   # FEnet Word Write Area base (%MW5000)

c = ModbusTcpClient(HOST, port=PORT, timeout=3)
if not c.connect():
    print(f"CONNECT FAILED → {HOST}:{PORT}")
    exit(1)

print(f"Connected → {HOST}:{PORT}")


def _reconnect():
    c.close()
    if c.connect():
        print("  (reconnected)")
        return True
    print("  RECONNECT FAILED")
    return False
print(f"FEnet read area: %MW{READ_OFFSET}+  |  write area: %MW{WRITE_OFFSET}+")
print()
print("Commands:  <plc_addr> <value>    write word   (e.g. '5000 64')")
print("           r <plc_addr>          read word    (e.g. 'r 1000')")
print("           q                     quit")
print()
print("Value range: 0–65535 (16-bit word). Use bit values directly, e.g. 64 for bit 6.")
print()


def _read(addr):
    """Read %MW{addr} via FC04 (input registers). Returns value or None on error."""
    if addr < READ_OFFSET:
        print(f"  ERROR: %MW{addr} is below the FEnet read area (%MW{READ_OFFSET}+) — not readable.")
        return None
    reg = addr - READ_OFFSET
    try:
        res = c.read_input_registers(reg, count=1)
    except ConnectionResetError:
        if not _reconnect():
            return None
        res = c.read_input_registers(reg, count=1)
    if res.isError():
        print(f"  read error (FC04 reg {reg}): {res}")
        return None
    return res.registers[0]


while True:
    try:
        line = input("> ").strip()
    except (EOFError, KeyboardInterrupt):
        break

    if not line:
        continue
    if line.lower() == 'q':
        break

    parts = line.split()

    # ── read ──────────────────────────────────────────────────────────────────
    if parts[0].lower() == 'r' and len(parts) == 2:
        try:
            addr = int(parts[1])
        except ValueError:
            print("  Usage: r <plc_address>")
            continue
        val = _read(addr)
        if val is not None:
            print(f"  %MW{addr} = {val}  (FC04 reg {addr - READ_OFFSET})")
        continue

    # ── write ─────────────────────────────────────────────────────────────────
    if len(parts) == 2:
        try:
            addr  = int(parts[0])
            value = int(parts[1])
        except ValueError:
            print("  Usage: <plc_address> <value>")
            continue

        if addr < WRITE_OFFSET:
            print(f"  ERROR: %MW{addr} is below the FEnet write area (%MW{WRITE_OFFSET}+) — not writable.")
            print(f"  To write %MW{addr}, the PLC engineer must set the FEnet Word Write Area to %MW0 in XG5000.")
            continue

        if not (0 <= value <= 65535):
            print("  Value must be 0–65535 (16-bit word).")
            continue

        write_reg = addr - WRITE_OFFSET
        try:
            res = c.write_register(write_reg, value & 0xFFFF)
        except ConnectionResetError:
            if not _reconnect():
                continue
            res = c.write_register(write_reg, value & 0xFFFF)
        if res.isError():
            print(f"  WRITE FAILED (FC06 reg {write_reg}): {res}")
        else:
            print(f"  wrote %MW{addr} = {value}  (FC06 write reg {write_reg})")
            # Read back via FC04 — both sides map to the same PLC M memory,
            # so a matching value confirms the write landed.
            rb = _read(addr)
            if rb is not None:
                ok = "✓ confirmed" if rb == value else "✗ mismatch — PLC ladder may have overwritten it"
                print(f"  readback %MW{addr} = {rb}  (FC04 read reg {addr - READ_OFFSET})  {ok}")
        continue

    print("  Usage: <plc_address> <value>  |  r <plc_address>  |  q")

c.close()
print("Disconnected.")
