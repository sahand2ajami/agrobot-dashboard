"""Interactive Modbus TCP register reader for PLC testing.

FEnet address mapping (XG5000 Standard Settings → FEnet):
  Word Read Area: %MW1000  →  Modbus INPUT register 0 = PLC %MW1000  (FC04)
  Offset formula: modbus_register = plc_address - 1000

The FEnet serves its read area via FC04 (input registers), NOT FC03 (holding
registers). Using FC03 returns 0 from the FEnet even when the PLC has real data.

Enter PLC addresses (e.g. 1000 for %MW1000). Minimum readable address: %MW1000.
Addresses below %MW1000 are outside the FEnet read window and will be rejected.

Usage: python3 plc_read.py
"""

from pymodbus.client import ModbusTcpClient

HOST        = "192.168.1.2"
PORT        = 502
READ_OFFSET = 1000   # FEnet Word Read Area base (%MW1000)

c = ModbusTcpClient(HOST, port=PORT, timeout=3)
if not c.connect():
    print(f"CONNECT FAILED → {HOST}:{PORT}")
    exit(1)

print(f"Connected → {HOST}:{PORT}")
print(f"FEnet read area: %MW{READ_OFFSET}+ (Modbus reg 0 = PLC %MW{READ_OFFSET})")
print()
print("Commands:  <plc_addr>            read single  (e.g. '1000' for %MW1000)")
print("           <start>-<end>         read range   (e.g. '1000-1010')")
print("           q                     quit")
print()

def _read(plc_addr, count=1):
    if plc_addr < READ_OFFSET:
        print(f"  ERROR: %MW{plc_addr} is below the FEnet read area (%MW{READ_OFFSET}+) — not readable.")
        return None
    modbus_reg = plc_addr - READ_OFFSET
    res = c.read_input_registers(modbus_reg, count=count)
    if res.isError():
        print(f"  read error (Modbus reg {modbus_reg}): {res}")
        return None
    return res.registers

while True:
    try:
        line = input("> ").strip()
    except (EOFError, KeyboardInterrupt):
        break

    if not line:
        continue
    if line.lower() == 'q':
        break

    if '-' in line:
        try:
            s, e = line.split('-', 1)
            start, end = int(s), int(e)
            count = end - start + 1
        except ValueError:
            print("  Usage: <start>-<end>  e.g. 1000-1010")
            continue
        regs = _read(start, count)
        if regs is not None:
            for i, v in enumerate(regs):
                print(f"  %MW{start + i} = {v}  (Modbus reg {start + i - READ_OFFSET})")
    else:
        try:
            addr = int(line)
        except ValueError:
            print("  Usage: <plc_address>  e.g. 1000")
            continue
        regs = _read(addr)
        if regs is not None:
            print(f"  %MW{addr} = {regs[0]}  (Modbus reg {addr - READ_OFFSET})")

c.close()
print("Disconnected.")
