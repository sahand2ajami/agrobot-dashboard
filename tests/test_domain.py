"""Unit tests for agrobot_dashboard.domain — the pure business logic.

Everything here runs on a bare Python install: no ROS, no hardware, no HTTP.
Timestamps are supplied explicitly, so behaviour is fully deterministic.
"""
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).parent.parent))

from agrobot_dashboard.domain import auto_drive, geo, kinematics
from agrobot_dashboard.domain.battery import MedianVoltageFilter
from agrobot_dashboard.domain.odometry import OdometryAccumulator, sign_extend_32


# ── kinematics ───────────────────────────────────────────────────────────────

class TestKinematics:
    AGROBOT = dict(linear_scale=3000, angular_scale=1000, speed_max=32767)

    def test_forward(self):
        assert kinematics.twist_to_wheel_speeds(0.5, 0.0, **self.AGROBOT) == (1500, 1500)

    def test_turn_left_is_minus_plus(self):
        l, r = kinematics.twist_to_wheel_speeds(0.0, 1.0, **self.AGROBOT)
        assert (l, r) == (-1000, 1000)

    def test_clamped_to_speed_max(self):
        assert kinematics.twist_to_wheel_speeds(999.0, 0.0, **self.AGROBOT) == (32767, 32767)

    def test_scaling_comes_from_arguments(self):
        # A different chassis config must change the output — no hidden constants.
        l, r = kinematics.twist_to_wheel_speeds(1.0, 0.0, linear_scale=500,
                                                angular_scale=100, speed_max=1000)
        assert (l, r) == (500, 500)


# ── odometry ─────────────────────────────────────────────────────────────────

class TestOdometryAccumulator:
    def test_accumulates_center_displacement(self):
        o = OdometryAccumulator()
        o.update(0, 0, now=1.0)
        o.update(100, 200, now=1.1)
        assert o.mileage_pulses == pytest.approx(150.0)
        assert (o.left, o.right) == (100, 200)

    def test_reverse_adds_absolute_mileage(self):
        o = OdometryAccumulator()
        o.update(0, 0, now=1.0)
        o.update(-100, -100, now=1.1)
        assert o.mileage_pulses == pytest.approx(100.0)

    def test_outlier_delta_rejected(self):
        o = OdometryAccumulator(max_delta=3000)
        o.update(0, 0, now=1.0)
        o.update(50_000, 50_000, now=1.1)     # register jump / bad read
        assert o.mileage_pulses == 0.0
        # ...but the raw position still tracks the latest reading
        assert o.left == 50_000

    def test_reconnect_resets_mileage(self):
        o = OdometryAccumulator(reconnect_gap_s=5.0)
        o.update(0, 0, now=1.0)
        o.update(100, 100, now=1.1)
        assert o.mileage_pulses > 0
        o.update(100, 100, now=10.0)          # > 5 s gap → chassis power-cycle
        assert o.mileage_pulses == 0.0

    def test_first_update_adds_nothing(self):
        o = OdometryAccumulator()
        o.update(500, 500, now=1.0)
        assert o.mileage_pulses == 0.0


class TestSignExtend:
    def test_matches_int32_semantics(self):
        assert sign_extend_32(0xFFFF_FFFF) == -1
        assert sign_extend_32(0x8000_0000) == -2_147_483_648
        assert sign_extend_32(0x7FFF_FFFF) == 2_147_483_647


# ── battery ──────────────────────────────────────────────────────────────────

class TestMedianVoltageFilter:
    def test_median_of_window(self):
        f = MedianVoltageFilter(recompute_s=0.0)
        for i, v in enumerate([48.0, 49.0, 47.5]):
            f.add(v, now=1.0 + i * 0.1)
        assert f.smoothed == 48.0

    def test_zero_readings_never_enter_window(self):
        f = MedianVoltageFilter(recompute_s=0.0)
        f.add(0.0, now=1.0)
        assert f.smoothed == 0.0 and not f.window
        assert f.last_reading == 1.0          # still counts as "chassis alive"

    def test_out_of_range_excluded_from_median(self):
        f = MedianVoltageFilter(recompute_s=0.0, min_valid=30.0, max_valid=70.0)
        f.add(48.0, now=1.0)
        f.add(120.0, now=1.1)                 # impossible for a 14S pack
        f.add(48.0, now=1.2)
        assert f.smoothed == 48.0

    def test_recompute_throttled(self):
        f = MedianVoltageFilter(recompute_s=10.0)
        f.add(48.0, now=1.0)
        first = f.smoothed_at
        f.add(60.0, now=2.0)                  # only 1 s later — no recompute
        assert f.smoothed_at == first

    def test_window_expires(self):
        f = MedianVoltageFilter(window_s=15.0, recompute_s=0.0)
        f.add(40.0, now=1.0)
        f.add(50.0, now=20.0)                 # first reading now outside window
        assert f.smoothed == 50.0


# ── auto drive ───────────────────────────────────────────────────────────────

class TestAutoDrivePlan:
    TARGET, SLOW = 6422.0, 1605.5             # 2 m / 0.5 m at 3211 pulses/m

    def test_cruise_far_from_target(self):
        assert auto_drive.plan_speed(0, self.TARGET, self.SLOW, 0.5, 0.08) == 0.5

    def test_crawl_in_slow_zone(self):
        assert auto_drive.plan_speed(5000, self.TARGET, self.SLOW, 0.5, 0.08) == 0.08

    def test_done_at_target(self):
        assert auto_drive.plan_speed(6422, self.TARGET, self.SLOW, 0.5, 0.08) is None

    def test_done_past_target(self):
        assert auto_drive.plan_speed(7000, self.TARGET, self.SLOW, 0.5, 0.08) is None


# ── geo ──────────────────────────────────────────────────────────────────────

class TestDms:
    def test_north_east(self):
        assert geo.dms(51.5074, True) == "51°30'26.64\"N"

    def test_south_west(self):
        assert geo.dms(-33.8688, True).endswith('"S')
        assert geo.dms(-70.6693, False).endswith('"W')

    def test_none_is_empty(self):
        assert geo.dms(None, True) == ""
