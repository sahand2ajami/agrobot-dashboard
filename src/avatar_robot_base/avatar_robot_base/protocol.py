"""Modbus-RTU protocol for the Agrobot chassis — pure functions, no ROS.

This module is deliberately importable without rclpy or pyserial so the
frame-building and register-parsing logic can be unit-tested anywhere
(tests/test_robot_base.py). robot_base_node.py owns the serial port and
timing; everything about the wire format lives here.
"""
import struct

SLAVE = 0x01
SPEED_REG = 0x0016
SENSOR_REG = 0x0019
SENSOR_N = 7


def crc16(data: bytes) -> bytes:
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return struct.pack('<H', crc)


def write_speed_frame(speed_l: int, speed_r: int) -> bytes:
    """FC16 write of [left, right, 0] to the speed registers."""
    pdu = struct.pack('>BBHHB', SLAVE, 0x10, SPEED_REG, 3, 6)
    pdu += struct.pack('>hhh', int(speed_l), int(speed_r), 0)
    return pdu + crc16(pdu)


def read_sensors_frame() -> bytes:
    """FC03 read of the 7-register sensor block."""
    pdu = struct.pack('>BBHH', SLAVE, 0x03, SENSOR_REG, SENSOR_N)
    return pdu + crc16(pdu)


def sign_extend_32(raw: int) -> int:
    """Two unsigned 16-bit registers combine to an unsigned 32-bit value;
    sign-extend it so the encoder count wraps correctly across zero
    (0xFFFF_FFFF → -1, not 4294967295)."""
    return raw if raw < 0x8000_0000 else raw - 0x1_0000_0000


def parse_sensor_regs(regs) -> dict:
    """Decode the sensor block (registers 0x0019..0x001F).

    Layout confirmed by passive bus capture:
      [0] odom_L hi  [1] odom_L lo  [2] odom_R hi
      [3] odom_R lo  [4] battery×100 (V)  [5] fault code  [6] oil %
    """
    return {
        "odom_l": sign_extend_32((regs[0] << 16) | regs[1]),
        "odom_r": sign_extend_32((regs[2] << 16) | regs[3]),
        "battery_v": regs[4] / 100.0,
        "error_code": regs[5],
        "oil_pct": int(regs[6]) & 0xFF,
    }
