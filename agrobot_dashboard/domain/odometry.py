"""Wheel-encoder odometry: sign extension and mileage accumulation.

The chassis reports each encoder as two unsigned 16-bit Modbus registers.
Combined they form an unsigned 32-bit value that must be sign-extended so the
count wraps correctly across zero (0xFFFF_FFFF → -1, not 4294967295).
"""


def sign_extend_32(raw):
    """Sign-extend an unsigned 32-bit register pair to a signed int32."""
    return raw if raw < 0x8000_0000 else raw - 0x1_0000_0000


class OdometryAccumulator:
    """Accumulates travelled mileage (in encoder pulses) from raw counts.

    Pure logic — the caller supplies monotonic timestamps, so behaviour is
    fully deterministic in tests.

    - Deltas larger than ``max_delta`` are rejected as outliers (a bad Modbus
      read or register jump must not add phantom mileage).
    - A gap longer than ``reconnect_gap_s`` between updates is treated as a
      chassis power-cycle/reconnect: mileage restarts from zero because the
      encoder counters may have reset.
    """

    def __init__(self, max_delta=3000, reconnect_gap_s=5.0):
        self.max_delta = max_delta
        self.reconnect_gap_s = reconnect_gap_s
        self.mileage_pulses = 0.0
        self.left = 0
        self.right = 0
        self.last_update = 0.0
        self._prev_l = None
        self._prev_r = None

    def update(self, left, right, now):
        """Feed one encoder reading (signed counts) taken at monotonic ``now``."""
        if self.last_update > 0 and (now - self.last_update) > self.reconnect_gap_s:
            self.mileage_pulses = 0.0
            self._prev_l = None
            self._prev_r = None
        if self._prev_l is not None:
            dl = left - self._prev_l
            dr = right - self._prev_r
            if abs(dl) < self.max_delta and abs(dr) < self.max_delta:
                self.mileage_pulses += abs((dl + dr) / 2.0)
        self._prev_l = left
        self._prev_r = right
        self.left = left
        self.right = right
        self.last_update = now
