"""
Tests for scripts/gnss_rtu608bt_read.py — NMEA parsing, checksum validation,
and JSON coordinate output.
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent / "scripts"))

import pytest
from gnss_rtu608bt_read import (
    nmea_to_decimal,
    parse_gngga,
    parse_gnrmc,
    parse_gsv,
    strip_checksum,
    validate_checksum,
)


# ── nmea_to_decimal ─────────────────────────────────────────────────────────

class TestNmeaToDecimal:
    def test_north_lat(self):
        assert nmea_to_decimal("5130.0000", "N") == pytest.approx(51.5, abs=0.01)

    def test_south_lat(self):
        result = nmea_to_decimal("5130.0000", "S")
        assert result == pytest.approx(-51.5, abs=0.01)

    def test_east_lon(self):
        result = nmea_to_decimal("00007.5000", "E")
        assert result == pytest.approx(0.125, abs=0.001)

    def test_west_lon(self):
        result = nmea_to_decimal("00007.5000", "W")
        assert result == pytest.approx(-0.125, abs=0.001)

    def test_empty_value_returns_none(self):
        assert nmea_to_decimal("", "N") is None

    def test_zero_zero(self):
        assert nmea_to_decimal("0000.0000", "N") == pytest.approx(0.0)

    def test_realistic_london(self):
        # 51°30'27"N  → 5130.4500,N
        lat = nmea_to_decimal("5130.4500", "N")
        assert 51.4 < lat < 51.6


# ── parse_gngga ─────────────────────────────────────────────────────────────

class TestParseGngga:
    def _sentence(self, lat="5130.4500", ns="N", lon="00007.5000", ew="W",
                  fix="4", sats="12", hdop="0.9", alt="50.1"):
        # $GNGGA,time,lat,NS,lon,EW,fix,sats,hdop,alt,...
        return ["$GNGGA", "120000.00", lat, ns, lon, ew,
                fix, sats, hdop, alt, "M", "0", "M", "", ""]

    def test_valid_rtk_fixed(self):
        r = parse_gngga(self._sentence())
        assert r["fix"] == 4
        assert r["sats"] == 12
        assert r["hdop"] == pytest.approx(0.9)
        assert r["alt"] == pytest.approx(50.1)
        assert -0.13 < r["lon"] < -0.12
        assert 51.4 < r["lat"] < 51.6

    def test_no_fix(self):
        r = parse_gngga(self._sentence(fix="0"))
        assert r["fix"] == 0

    def test_too_few_parts_returns_empty(self):
        assert parse_gngga(["$GNGGA", "120000"]) == {}

    def test_empty_lat_returns_empty(self):
        assert parse_gngga(self._sentence(lat="", ns="")) == {}

    def test_malformed_hdop_uses_default(self):
        parts = self._sentence(hdop="")
        r = parse_gngga(parts)
        assert r["hdop"] == 99.9

    def test_malformed_alt_uses_default(self):
        parts = self._sentence(alt="")
        r = parse_gngga(parts)
        assert r["alt"] == 0.0

    def test_corrupt_lat_returns_empty(self):
        assert parse_gngga(self._sentence(lat="NOTNMEA")) == {}


# ── parse_gnrmc ─────────────────────────────────────────────────────────────

class TestParseGnrmc:
    def _sentence(self, active="A", speed="1.0", heading="270.5"):
        # $GNRMC,time,active,lat,NS,lon,EW,speed,heading,date,...
        return ["$GNRMC", "120000.00", active, "5130.4500", "N",
                "00007.5000", "W", speed, heading, "260526", "", ""]

    def test_active(self):
        r = parse_gnrmc(self._sentence())
        assert r["active"] is True

    def test_void(self):
        r = parse_gnrmc(self._sentence(active="V"))
        assert r["active"] is False

    def test_speed_conversion(self):
        r = parse_gnrmc(self._sentence(speed="1.0"))
        assert r["speed_kmh"] == pytest.approx(1.852, rel=0.001)

    def test_zero_speed(self):
        r = parse_gnrmc(self._sentence(speed="0.0"))
        assert r["speed_kmh"] == pytest.approx(0.0)

    def test_heading(self):
        r = parse_gnrmc(self._sentence(heading="90.0"))
        assert r["heading"] == pytest.approx(90.0)

    def test_empty_heading_defaults_zero(self):
        r = parse_gnrmc(self._sentence(heading=""))
        assert r["heading"] == pytest.approx(0.0)

    def test_too_few_parts_returns_empty(self):
        assert parse_gnrmc(["$GNRMC", "time", "A"]) == {}


# ── parse_gsv (satellites in view) ───────────────────────────────────────────

class TestParseGsv:
    def test_basic_in_view(self):
        # $GPGSV,3,1,11,...  -> 11 satellites in view
        parts = ["$GPGSV", "3", "1", "11", "01", "40", "083", "46"]
        assert parse_gsv(parts) == 11

    def test_zero_in_view(self):
        assert parse_gsv(["$GLGSV", "1", "1", "00"]) == 0

    def test_too_few_parts(self):
        assert parse_gsv(["$GPGSV", "1", "1"]) is None

    def test_malformed_count(self):
        assert parse_gsv(["$GPGSV", "1", "1", "xx"]) is None


# ── validate_checksum ────────────────────────────────────────────────────────

class TestValidateChecksum:
    def test_valid_checksum(self):
        # Build a known-good sentence manually
        body = "GNGGA,120000.00,5130.4500,N,00007.5000,W,4,12,0.9,50.1,M,0,M,,"
        chk = 0
        for c in body:
            chk ^= ord(c)
        sentence = f"${body}*{chk:02X}"
        assert validate_checksum(sentence) is True

    def test_wrong_checksum(self):
        sentence = "$GNGGA,test*FF"
        assert validate_checksum(sentence) is False

    def test_no_checksum_field_accepted(self):
        assert validate_checksum("$GNGGA,data,without,checksum") is True

    def test_empty_string(self):
        assert validate_checksum("") is True

    def test_malformed_checksum_hex(self):
        assert validate_checksum("$GNGGA,data*ZZ") is False


# ── strip_checksum ────────────────────────────────────────────────────────────

class TestStripChecksum:
    def test_strips_checksum(self):
        assert strip_checksum("$GNGGA,data*2B") == "$GNGGA,data"

    def test_no_checksum_unchanged(self):
        assert strip_checksum("$GNGGA,data") == "$GNGGA,data"
