"""Tests for dashboard/plc_client.py — register map integrity and the pure
validation paths (everything testable without pymodbus or a PLC).

The register addresses here are load-bearing: the AMR↔PLC handshake block is
%MW5100–5112 (bench-confirmed). An older map used %MW100/101, which sit below
the FEnet write base (%MW5000) and can never be written over Modbus — these
tests exist so that map can't silently come back.
"""
import struct
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


class TestHmiLayout:
    """The HMI read-only mirror: byte.bit addressing, screen integrity, decode."""

    def test_addr_byte_bit_to_word(self):
        # struct byte.bit → (word, bit_in_word); cross-checked vs _REG below
        assert plc_client._hmi_addr("0.0") == (0, 0)
        assert plc_client._hmi_addr("4.0") == (2, 0)
        assert plc_client._hmi_addr("5.0") == (2, 8)
        assert plc_client._hmi_addr("5.4") == (2, 12)

    def test_ind_bits_match_reg(self):
        # EstopOkFL @4.0 in ud_HMI_IND (base %MW1000) must land on the same
        # physical bit as _REG["IND_ESTOP_OK_FL"] = %MX16032.
        w, bit = plc_client._hmi_addr("4.0")
        base_mw = plc_client.HMI_BLOCKS["HMI_IND"][1]        # 1000
        assert (base_mw + w) * 16 + bit == 16032

    def test_every_screen_ref_resolves(self):
        for s in plc_client.HMI_SCREENS:
            for panel in plc_client._hmi_expand_layout(s)["panels"]:
                for row in panel["rows"]:
                    ref = row["ref"]
                    if ref.startswith("single:"):
                        assert ref[len("single:"):] in plc_client.HMI_SINGLES
                    else:
                        sym, _, rest = ref.partition(".")
                        member = rest.split("#", 1)[0]
                        udt = plc_client.HMI_BLOCKS[sym][0]
                        assert member in {m[0] for m in plc_client.HMI_UDT[udt]}

    def test_screen_ids_unique(self):
        ids = [s["id"] for s in plc_client.HMI_SCREENS]
        assert len(ids) == len(set(ids))

    def test_screens_meta_grouped(self):
        meta = PlcClient.hmi_screens_meta()
        assert meta["sections"][0]["section"] == "Overview"
        flat = [sc["id"] for sec in meta["sections"] for sc in sec["screens"]]
        assert flat == [s["id"] for s in plc_client.HMI_SCREENS]


class TestHmiDecode:
    def test_real_little_word_first(self):
        lo, hi = struct.unpack("<HH", struct.pack("<f", 123.45))
        words = [0, 0, lo, hi]                       # value at word 2
        assert plc_client._hmi_decode(words, "real", 2, 0) == pytest.approx(123.45, abs=1e-2)

    def test_dint_negative(self):
        v = (-12345) & 0xFFFFFFFF
        words = [v & 0xFFFF, (v >> 16) & 0xFFFF]
        assert plc_client._hmi_decode(words, "dint", 0, 0) == -12345

    def test_int_sign_and_uint(self):
        assert plc_client._hmi_decode([0xFFFF], "int", 0, 0) == -1
        assert plc_client._hmi_decode([0xFFFF], "uint", 0, 0) == 65535

    def test_bool_bit(self):
        assert plc_client._hmi_decode([0b100000], "bool", 0, 5) is True
        assert plc_client._hmi_decode([0b100000], "bool", 0, 4) is False

    def test_out_of_range_is_none(self):
        assert plc_client._hmi_decode([1, 2], "real", 3, 0) is None


class TestHmiScreenRead:
    """read_hmi_screen with a simulated word bus (no pymodbus)."""

    def _connected_client(self, monkeypatch, words_by_base):
        c = PlcClient("192.0.2.1", 502)
        monkeypatch.setattr(c, "_ensure_client", lambda: object())
        monkeypatch.setattr(
            c, "_read_words_raw",
            lambda addr, count: list(words_by_base.get(addr, [0] * count))[:count])
        return c

    def test_unknown_screen(self):
        c = PlcClient("192.0.2.1", 502)
        assert c.read_hmi_screen("nope").get("error")

    def test_offline_keeps_layout_null_values(self):
        # default client: no pymodbus → unreachable, but structure survives
        c = PlcClient("192.0.2.1", 502)
        out = c.read_hmi_screen("auger_gimbal_x")
        assert out["connected"] is False
        assert out["panels"] and out["panels"][0]["rows"]
        assert all(r["value"] is None for r in out["panels"][0]["rows"])

    def test_live_decode_la36(self, monkeypatch):
        lo, hi = struct.unpack("<HH", struct.pack("<f", 55.5))
        words = [0] * 14
        words[0] = 0b1                      # AtHome (0.0)
        words[4], words[5] = lo, hi         # PositionMeasured REAL @8.0 → word 4
        c = self._connected_client(monkeypatch, {1600: words})
        out = c.read_hmi_screen("auger_gimbal_x")
        assert out["connected"] is True
        vals = {r["label"]: r["value"] for r in out["panels"][0]["rows"]}
        assert vals["AtHome"] is True
        assert vals["PositionMeasured"] == pytest.approx(55.5, abs=1e-2)

    def test_node_comms_inverted(self, monkeypatch):
        # NodeCommsNOk @%MW1048: bit set = NOT ok → mirror shows False
        c = self._connected_client(monkeypatch, {1048: [0b1]})   # bit0 set
        out = c.read_hmi_screen("communications")
        rows = {r["ref"]: r["value"] for r in out["panels"][0]["rows"]}
        assert rows["single:Node0CommsOk"] is False   # bit0 NOk → not ok
        assert rows["single:Node1CommsOk"] is True    # bit1 clear → ok

    def test_io_digital_bit_extraction(self, monkeypatch):
        # LocalIn WORD @%MW3000, IN02 = bit 2
        c = self._connected_client(monkeypatch, {3000: [0b100, 0, 0, 0, 0, 0]})
        out = c.read_hmi_screen("io_digital")
        rows = {r["label"]: r["value"] for r in out["panels"][0]["rows"]}
        assert rows["IN02 Gate Locked"] is True
        assert rows["IN00 Spare"] is False
