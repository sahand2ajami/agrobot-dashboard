"""
Tests for robot_base_node.py and dashboard/serve.py motor utilities.

These tests run without ROS — they only exercise pure-Python helper
functions (no rclpy.init required).
"""
import sys
from pathlib import Path
import pytest

# ── twist_to_wheel_speeds from serve.py ─────────────────────────────────────

sys.path.insert(0, str(Path(__file__).parent.parent / "dashboard"))

from serve import twist_to_wheel_speeds, SPEED_MAX, LINEAR_SCALE, ANGULAR_SCALE


class TestTwistToWheelSpeeds:
    def test_forward(self):
        l, r = twist_to_wheel_speeds(0.5, 0.0)
        assert l == r
        assert l > 0

    def test_backward(self):
        l, r = twist_to_wheel_speeds(-0.5, 0.0)
        assert l == r
        assert l < 0

    def test_turn_left(self):
        # angular_z > 0 = turn left: left wheel slower / reverse, right faster
        l, r = twist_to_wheel_speeds(0.0, 1.0)
        assert r > 0 and l < 0

    def test_turn_right(self):
        l, r = twist_to_wheel_speeds(0.0, -1.0)
        assert l > 0 and r < 0

    def test_stop(self):
        assert twist_to_wheel_speeds(0.0, 0.0) == (0, 0)

    def test_speed_max_clamped(self):
        l, r = twist_to_wheel_speeds(999.0, 0.0)
        assert l == SPEED_MAX
        assert r == SPEED_MAX

    def test_reverse_max_clamped(self):
        l, r = twist_to_wheel_speeds(-999.0, 0.0)
        assert l == -SPEED_MAX
        assert r == -SPEED_MAX

    def test_linear_scale(self):
        l, r = twist_to_wheel_speeds(1.0, 0.0)
        assert l == r == min(LINEAR_SCALE, SPEED_MAX)

    def test_angular_scale(self):
        l, r = twist_to_wheel_speeds(0.0, 1.0)
        # l = lin - ang = 0 - ANGULAR_SCALE, r = lin + ang = 0 + ANGULAR_SCALE
        assert l == max(-SPEED_MAX, -ANGULAR_SCALE)
        assert r == min(SPEED_MAX, ANGULAR_SCALE)

    def test_output_are_integers(self):
        l, r = twist_to_wheel_speeds(0.3, 0.1)
        assert isinstance(l, int)
        assert isinstance(r, int)


# ── Int32 sign-extension logic from robot_base_node._publish ─────────────────
# We extract the formula into a standalone function for testability.

def _sign_extend_32(raw: int) -> int:
    """Replicate the sign-extension logic in robot_base_node._publish."""
    return raw if raw < 0x8000_0000 else raw - 0x1_0000_0000


class TestOdomSignExtension:
    def test_zero(self):
        assert _sign_extend_32(0) == 0

    def test_max_positive(self):
        assert _sign_extend_32(0x7FFF_FFFF) == 2_147_483_647

    def test_minus_one(self):
        # 0xFFFF_FFFF unsigned → -1 signed
        assert _sign_extend_32(0xFFFF_FFFF) == -1

    def test_min_negative(self):
        # 0x8000_0000 unsigned → -2147483648 signed
        assert _sign_extend_32(0x8000_0000) == -2_147_483_648

    def test_small_positive(self):
        assert _sign_extend_32(100) == 100

    def test_just_over_half(self):
        # Anything >= 2^31 should come back negative
        val = _sign_extend_32(0x8000_0001)
        assert val < 0

    def test_combined_hi_lo(self):
        # Simulate encoder register combination: hi=0xFFFF, lo=0xFFFF → uint32=0xFFFFFFFF → -1
        hi, lo = 0xFFFF, 0xFFFF
        raw = (hi << 16) | lo
        assert _sign_extend_32(raw) == -1

    def test_boundary_stays_positive(self):
        # 0x7FFF_0000 should stay positive
        assert _sign_extend_32(0x7FFF_0000) > 0

    @pytest.mark.parametrize("hi,lo,expected", [
        (0x0000, 0x0001,  1),
        (0x0000, 0x0064,  100),
        (0xFFFF, 0xFFFF, -1),
        (0x8000, 0x0000, -2_147_483_648),
        (0x7FFF, 0xFFFF,  2_147_483_647),
    ])
    def test_register_combinations(self, hi, lo, expected):
        raw = (hi << 16) | lo
        assert _sign_extend_32(raw) == expected
