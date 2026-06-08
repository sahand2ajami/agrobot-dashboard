"""
Keyboard teleoperation for the Avatar robot chassis.

Controls:
  w / Arrow Up    : Forward
  s / Arrow Down  : Backward
  a / Arrow Left  : Spin left
  d / Arrow Right : Spin right
  q / e           : Diagonal turns (forward + curve)
  Space           : Stop
  +/-             : Increase / decrease speed
  Ctrl+C          : Quit
"""

import sys
import tty
import termios
import select
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16MultiArray

SPEED_DEFAULT = 500
SPEED_STEP = 50
SPEED_MAX = 800
SPEED_MIN = 100

BANNER = """
==========================================
   Avatar Robot Keyboard Teleoperation
==========================================
  w / ↑  : Forward       s / ↓  : Back
  a / ←  : Spin Left     d / →  : Spin Right
  q       : Fwd-Left      e      : Fwd-Right
  Space   : STOP
  +/-     : Speed up/down
  Ctrl+C  : Quit
------------------------------------------
"""

ESC = '\x1b'
ARROW_UP    = '\x1b[A'
ARROW_DOWN  = '\x1b[B'
ARROW_RIGHT = '\x1b[C'
ARROW_LEFT  = '\x1b[D'


def get_key(timeout=0.05):
    """Return the next keypress as a string, or '' on timeout."""
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        rlist, _, _ = select.select([sys.stdin], [], [], timeout)
        if not rlist:
            return ''
        ch = sys.stdin.read(1)
        if ch == ESC:
            rlist2, _, _ = select.select([sys.stdin], [], [], 0.02)
            if rlist2:
                ch += sys.stdin.read(1)
                rlist3, _, _ = select.select([sys.stdin], [], [], 0.02)
                if rlist3:
                    ch += sys.stdin.read(1)
        return ch
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


class TeleopNode(Node):
    def __init__(self):
        super().__init__('teleop_keyboard')
        self.pub = self.create_publisher(Int16MultiArray, '/avatar_robot/speed_cmd', 10)
        self.speed = SPEED_DEFAULT
        self.create_timer(1 / 20.0, self._publish)
        self._left = 0
        self._right = 0

    def set_cmd(self, left, right):
        self._left = left
        self._right = right

    def _publish(self):
        msg = Int16MultiArray()
        msg.data = [self._left, self._right]
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TeleopNode()

    print(BANNER)
    print(f"Speed: {node.speed}")

    try:
        while rclpy.ok():
            key = get_key(timeout=0.05)

            if key in ('\x03',):  # Ctrl+C
                break

            if key in ('w', ARROW_UP):
                node.set_cmd(node.speed, node.speed)
            elif key in ('s', ARROW_DOWN):
                node.set_cmd(-node.speed, -node.speed)
            elif key in ('a', ARROW_LEFT):
                node.set_cmd(-node.speed, node.speed)
            elif key in ('d', ARROW_RIGHT):
                node.set_cmd(node.speed, -node.speed)
            elif key == 'q':
                node.set_cmd(int(node.speed * 0.5), node.speed)
            elif key == 'e':
                node.set_cmd(node.speed, int(node.speed * 0.5))
            elif key == ' ':
                node.set_cmd(0, 0)
            elif key in ('+', '='):
                node.speed = min(node.speed + SPEED_STEP, SPEED_MAX)
                print(f"\rSpeed: {node.speed}   ", end='', flush=True)
            elif key == '-':
                node.speed = max(node.speed - SPEED_STEP, SPEED_MIN)
                print(f"\rSpeed: {node.speed}   ", end='', flush=True)
            elif key == '':
                # Timeout — no key held, stop the robot
                node.set_cmd(0, 0)

            rclpy.spin_once(node, timeout_sec=0.0)

    except KeyboardInterrupt:
        pass
    finally:
        node.set_cmd(0, 0)
        rclpy.spin_once(node, timeout_sec=0.1)
        node.destroy_node()
        rclpy.shutdown()
        print("\nStopped.")


if __name__ == '__main__':
    main()
