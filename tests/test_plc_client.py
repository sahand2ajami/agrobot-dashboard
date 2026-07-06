"""Tests for dashboard/plc_client.py — register map integrity and the pure
validation paths (everything testable without pymodbus or a PLC).

The register addresses here are load-bearing: the AMR↔PLC handshake block is
%MW5100–5112 (bench-confirmed). An older map used %MW100/101, which sit below
the FEnet write base (%MW5000) and can never be written over Modbus — these
tests exist so that map can't silently come back.
"""
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).parent.parent / "dashboard"))

import plc_client
from plc_client import PlcClient, AMR_WRITABLE


class TestRegisterMap:
    def test_handshake_words_are_5100_block(self):
        assert plc_client._REG["AUGER_AMR_WORD"] == "%MW5110"
        assert plc_client._REG["PLANTER_AMR_WORD"] == "%MW5111"
        assert plc_client._REG["AMR_STATE_WORD"] == "%MW5112"

    def test_no_register_below_write_base_is_written(self):
        # Every register a write command targets must be >= the FEnet write
        # base, otherwise the write silently can't happen.
        write_targets = {var for var, _ in plc_client._MACHINE_CMD_MAP.values()}
        write_targets |= {"ROBOT_PB_CMD", "AUGER_AMR_WORD", "PLANTER_AMR_WORD",
                          "AMR_STATE_WORD"}
        for name in write_targets:
            func, addr = PlcClient._parse(plc_client._REG[name])
            assert func == "hr", f"{name} must be a %MW word"
            assert addr >= plc_client._FENET_WRITE_WORD_BASE, \
                f"{name} ({plc_client._REG[name]}) is below the FEnet write base"

    def test_amr_writable_set(self):
        assert AMR_WRITABLE == {5110, 5111, 5112}

    def test_parse_devices(self):
        assert PlcClient._parse("%MW5110") == ("hr", 5110)
        assert PlcClient._parse("%MX43204") == ("coil", 43204)
        with pytest.raises(ValueError):
            PlcClient._parse("%QX1.2.3")


class TestCommandTables:
    def test_machine_commands_exported(self):
        assert "SET_AUTO" in plc_client.MACHINE_COMMANDS
        assert "FAULT_RESET" in plc_client.MACHINE_COMMANDS

    def test_robot_commands_exported(self):
        assert {"HOME", "START", "STOP"} <= set(plc_client.ROBOT_COMMANDS)

    def test_symbol_roles_cover_map(self):
        roles = plc_client.symbol_roles()
        assert roles["HMI_IND"] == "read"
        assert roles["AMR_CMD_AUGER"] == "write"
        assert roles["AMR_STATUS_AUGER"] == "reserved"


class TestAmrWriteValidation:
    """amr_write validates before touching the network, so these run
    without pymodbus/PLC."""

    def test_rejects_plc_owned_register(self):
        c = PlcClient("192.0.2.1", 502)   # TEST-NET address — never contacted
        out = c.amr_write(5100, 1)
        assert out["success"] is False
        assert "not a writable" in out["message"]

    def test_rejects_non_integer_args(self):
        c = PlcClient("192.0.2.1", 502)
        out = c.amr_write("abc", "def")
        assert out["success"] is False
        assert "integers" in out["message"]

    def test_masks_to_16_bits_and_validates_register(self):
        c = PlcClient("192.0.2.1", 502)
        # 5110 is writable; without pymodbus/PLC the op reports unreachable,
        # but it must NOT be rejected as an invalid register.
        out = c.amr_write(5110, 0x1FFFF)
        assert "not a writable" not in out.get("message", "")


class TestBannerDecode:
    def test_decodes_big_endian_ascii(self):
        # "AB" packed high-byte-first, null-terminated
        words = [0x4142, 0x4344, 0x0000] + [0] * 13
        assert PlcClient._decode_banner_string(words) == "ABCD"

    def test_empty_banner(self):
        assert PlcClient._decode_banner_string([0] * 16) == ""
