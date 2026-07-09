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
    def test_decodes_little_endian_ascii(self):
        # LS PLC packs strings low-byte-first: 'A'=0x41,'B'=0x42 → word 0x4241.
        words = [0x4241, 0x4443, 0x0000] + [0] * 13
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

    def test_screens_meta_menu_tree(self):
        meta = PlcClient.hmi_screens_meta()
        assert meta["root"] == "root"
        assert "root" in meta["menus"] and "io" in meta["menus"]
        # every screen has a title in the flat map
        assert set(meta["titles"]) == {s["id"] for s in plc_client.HMI_SCREENS}

    def test_menu_targets_resolve(self):
        ids = {s["id"] for s in plc_client.HMI_SCREENS}
        for key, menu in plc_client.HMI_MENU.items():
            for col in menu["columns"]:
                for btn in col["buttons"]:
                    kind, _, tgt = btn["target"].partition(":")
                    if kind == "screen":
                        assert tgt in ids, f"{key}: bad screen target {tgt}"
                    elif kind == "menu":
                        assert tgt in plc_client.HMI_MENU, f"{key}: bad menu target {tgt}"
                    else:
                        raise AssertionError(f"unknown target kind {btn['target']}")

    def test_every_screen_reachable_from_menu(self):
        reached = set()
        for menu in plc_client.HMI_MENU.values():
            for col in menu["columns"]:
                for btn in col["buttons"]:
                    kind, _, tgt = btn["target"].partition(":")
                    if kind == "screen":
                        reached.add(tgt)
        assert reached == {s["id"] for s in plc_client.HMI_SCREENS}

    def test_screen_has_layout(self):
        for s in plc_client.HMI_SCREENS:
            assert plc_client._hmi_expand_layout(s)["layout"]

    def test_hmi_addresses_match_bench_reg(self):
        """Every HMI-mirror tag that also appears in the bench-confirmed _REG map
        must resolve to the same %MW/%MX address. This pins the transcription to
        hardware truth so the two maps can't silently drift (esp. the auger-motor
        block, whose word order follows _REG, not the PDF)."""
        def member_mx(block, member):
            udt, base = plc_client.HMI_BLOCKS[block]
            for name, dt, addr in plc_client.HMI_UDT[udt]:
                if name == member:
                    w, bit = plc_client._hmi_addr(addr)
                    return (base + w) * 16 + bit, "MX" if dt == "bool" else "MW", base + w
            raise KeyError(member)

        def parse(reg):
            r = reg.strip().upper().lstrip("%")
            return r[:2], int(r[2:])

        # bench _REG key → (HMI block, member)
        pairs = {
            "IND_MODE_STATUS": ("HMI_IND", "ModeStatus"),
            "IND_ESTOP_OK_FL": ("HMI_IND", "EstopOkFL"),
            "IND_GATE_OK": ("HMI_IND", "GateOk"),
            "IND_FAULTED": ("HMI_IND", "Faulted"),
            "IND_AUGER_ENABLED": ("HMI_IND", "AugerEnabled"),
            "IND_PLANTER_ENABLED": ("HMI_IND", "PlanterEnabled"),
            "IND_ROBOT_ENABLED": ("HMI_IND", "RobotEnabled"),
            "IND_AMR_ENABLED": ("HMI_IND", "AMREnabled"),
            "AUGER_HOME": ("AugerSeq", "Home"),
            "AUGER_SETUP_OK": ("AugerSeq", "SetupOk"),
            "AUGER_OK_START": ("AugerSeq", "OkToStart"),
            "AUGER_ENABLED": ("AugerSeq", "Enabled"),
            "AUGER_IN_CYCLE": ("AugerSeq", "InCycle"),
            "AUGER_COMPLETE": ("AugerSeq", "Complete"),
            "AUGER_STEP": ("AugerSeq", "Step"),
            "PLANTER_HOME": ("PlanterSeq", "Home"),
            "PLANTER_STEP": ("PlanterSeq", "Step"),
            "AUGER_MOTOR_VEL_TARGET": ("HMI_IND_Auger", "VelocityTarget"),
            "AUGER_MOTOR_VEL_ACTUAL": ("HMI_IND_Auger", "VelocityMeasured"),
            "AUGER_MOTOR_RUN": ("HMI_IND_Auger", "Run"),
            "AUGER_MOTOR_FWD": ("HMI_IND_Auger", "Fwd"),
            "AUGER_MOTOR_FAULTED": ("HMI_IND_Auger", "Faulted"),
        }
        for key, (blk, mem) in pairs.items():
            kind, num = parse(plc_client._REG[key])
            mx, mkind, word = member_mx(blk, mem)
            got = mx if kind == "MX" else word
            assert kind == mkind and got == num, (
                f"{key}: _REG={plc_client._REG[key]} but mirror {blk}.{mem} "
                f"→ %{'MX'+str(mx) if kind=='MX' else 'MW'+str(word)}")


class TestConnectNegativeCache:
    """A downed PLC must fast-fail: one blocking connect per cooldown window,
    not one per poll (which starved the HMI/AMR polls past their fetch timeout)."""

    def test_failed_connect_is_cached(self, monkeypatch):
        import types
        calls = {"n": 0}

        class FakeClient:
            def __init__(self, *a, **k): pass
            def connect(self): calls["n"] += 1; return False
            def close(self): pass

        fake = types.ModuleType("pymodbus.client")
        fake.ModbusTcpClient = FakeClient
        monkeypatch.setitem(sys.modules, "pymodbus.client", fake)

        now = [1000.0]
        monkeypatch.setattr(plc_client.time, "monotonic", lambda: now[0])

        c = PlcClient("192.0.2.1", 502)
        with c._lock:
            assert c._ensure_client() is None
            assert c._ensure_client() is None     # within cooldown → no 2nd connect
        assert calls["n"] == 1

        now[0] += c._CONNECT_COOLDOWN + 0.1        # cooldown elapsed → retry once
        with c._lock:
            assert c._ensure_client() is None
        assert calls["n"] == 2


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

    def test_transport_error_resets_socket(self, monkeypatch):
        # A broken pipe mid-read must drop the shared socket so the next poll
        # reconnects (matches _op); a bad-address None must NOT reset it.
        c = PlcClient("192.0.2.1", 502)
        monkeypatch.setattr(c, "_ensure_client", lambda: object())
        reset = {"n": 0}
        monkeypatch.setattr(c, "_reset", lambda: reset.__setitem__("n", reset["n"] + 1))

        def boom(addr, count):
            raise OSError("[Errno 32] Broken pipe")
        monkeypatch.setattr(c, "_read_words_raw", boom)
        out = c.read_hmi_screen("auger_gimbal_x")
        assert out["connected"] is True                 # we did have a socket
        assert all(r["value"] is None for r in out["panels"][0]["rows"])
        assert reset["n"] == 1                           # socket dropped for next poll

        reset["n"] = 0
        monkeypatch.setattr(c, "_read_words_raw", lambda a, n: None)  # isError, socket fine
        c.read_hmi_screen("auger_gimbal_x")
        assert reset["n"] == 0                            # bad address ≠ reset
