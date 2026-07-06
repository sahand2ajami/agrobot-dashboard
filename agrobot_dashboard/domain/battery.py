"""Chassis pack-voltage smoothing.

Raw voltage readings from the chassis are noisy and occasionally garbage
(a bad Modbus read can produce 0 V or a wild value). The gauge therefore
shows a median over a sliding window, recomputed at a slower cadence so the
displayed value is steady.
"""


class MedianVoltageFilter:
    """Sliding-window median filter for pack voltage.

    Pure logic — the caller supplies monotonic timestamps.

    - Readings ≤ 0 V never enter the window (sensor glitch).
    - The median only considers values inside (min_valid, max_valid); for the
      default ~48 V / 14S pack anything outside 30–70 V is physically
      impossible and treated as a misread.
    - ``smoothed`` is recomputed at most every ``recompute_s`` seconds.
    """

    def __init__(self, window_s=15.0, recompute_s=10.0,
                 min_valid=30.0, max_valid=70.0):
        self.window_s = window_s
        self.recompute_s = recompute_s
        self.min_valid = min_valid
        self.max_valid = max_valid
        self.window = []          # [(monotonic_ts, volts), ...]
        self.smoothed = 0.0       # last computed median (0.0 = no data yet)
        self.smoothed_at = 0.0
        self.last_reading = 0.0   # monotonic ts of the last raw reading

    def add(self, volts, now):
        """Feed one raw voltage reading taken at monotonic ``now``."""
        self.last_reading = now
        if volts > 0:
            self.window.append((now, volts))
        cutoff = now - self.window_s
        self.window = [(t, v) for t, v in self.window if t > cutoff]
        if now - self.smoothed_at >= self.recompute_s:
            valid = sorted(v for _, v in self.window
                           if self.min_valid < v < self.max_valid)
            if valid:
                self.smoothed = valid[len(valid) // 2]
                self.smoothed_at = now
