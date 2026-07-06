"""Differential-drive kinematics.

This is the single implementation of the Twist → wheel-speed conversion.
The scale factors come from the active chassis config
(config/chassis/<name>.yaml); callers must not hardcode their own copies.
"""


def twist_to_wheel_speeds(linear_x, angular_z,
                          linear_scale, angular_scale, speed_max):
    """Convert Twist (m/s, rad/s) to (left, right) wheel speed integers.

    Differential-drive convention (matches teleop_keyboard.py):
      forward  W: [+,+]   backward S: [-,-]
      left     A: [-,+]   right    D: [+,-]   (positive angular_z = left)
    """
    lin = int(linear_x * linear_scale)
    ang = int(angular_z * angular_scale)
    left = max(-speed_max, min(speed_max, lin - ang))
    right = max(-speed_max, min(speed_max, lin + ang))
    return left, right
