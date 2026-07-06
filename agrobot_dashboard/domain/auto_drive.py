"""Speed planning for the server-side fixed-distance auto-drive.

The drive is two-phase: cruise, then crawl for the final slow zone so the
robot doesn't coast past the target. The stop decision is made here (on the
server) — never in the browser.
"""


def plan_speed(traveled_pulses, target_pulses, slow_zone_pulses,
               cruise_speed, crawl_speed):
    """Return the next commanded speed (m/s), or None once the target
    distance has been reached."""
    if traveled_pulses >= target_pulses:
        return None
    remaining = target_pulses - traveled_pulses
    return crawl_speed if remaining <= slow_zone_pulses else cruise_speed
