import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int32MultiArray, UInt16, UInt8, Int16MultiArray
import os
import serial
import struct
import threading
import time

from .protocol import (SLAVE, SENSOR_N, write_speed_frame, read_sensors_frame,
                       parse_sensor_regs)

# Serial-port resolution. AGROBOT_BASE_PORT (set by launch_dashboard.sh) wins;
# otherwise the first candidate that exists is used. /dev/agrobot_base is the
# stable udev symlink pinned to the chassis FTDI by serial
# (config/udev/99-agrobot-serial.rules); /dev/ttyUSB0 is the fallback on a box
# without the rule. Resolved at every (re)open so a late-appearing or
# reconnected device is picked up automatically.
PORT_CANDIDATES = ('/dev/agrobot_base', '/dev/ttyUSB0')
BAUD        = 38400
CMD_TIMEOUT = 1.5
RECONNECT_S    = 2.0    # min seconds between serial (re)open attempts
BAD_READ_LIMIT = 50     # consecutive bad reads (~5 s) → force a port reopen


class RobotBaseNode(Node):
    def __init__(self):
        super().__init__('robot_base_node')

        self.speed_l = 0
        self.speed_r = 0
        self.last_cmd = time.time()

        self.battery_pub     = self.create_publisher(Float32,         '/avatar_robot/battery',     10)
        self.odom_pub        = self.create_publisher(Int32MultiArray, '/avatar_robot/wheel_odom',  10)
        self.error_pub       = self.create_publisher(UInt16,          '/avatar_robot/error',       10)
        self.oil_pub         = self.create_publisher(UInt8,           '/avatar_robot/oil',         10)

        self.create_subscription(
            Int16MultiArray, '/avatar_robot/speed_cmd', self._speed_cb, 10)

        self.ser  = None
        self.port = None
        self._bad_reads     = 0       # consecutive bad/short sensor reads
        self._last_open     = 0.0     # monotonic time of last open attempt
        self._last_open_log = -1e9    # monotonic time of last open-failure log
        self._err_count     = 0       # consecutive comm errors
        self._last_err_log  = 0.0     # monotonic time of last comm-error log
        self._open_serial()           # best effort; the loop keeps retrying

        self.running = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def _open_serial(self):
        """(Re)open the chassis serial port. Returns True on success.

        Throttled to one attempt per RECONNECT_S so a missing device neither
        spins the CPU nor floods the log. The port is resolved fresh each call
        so a device that appears late (or a symlink whose target changed) is
        picked up without a restart."""
        now = time.monotonic()
        if now - self._last_open < RECONNECT_S:
            return False
        self._last_open = now

        port = os.environ.get('AGROBOT_BASE_PORT') or next(
            (p for p in PORT_CANDIDATES if os.path.exists(p)), None)
        if port is None:
            if now - self._last_open_log >= 10.0:
                self.get_logger().warn(
                    'Chassis serial port not found (looked for '
                    f'{", ".join(PORT_CANDIDATES)}) — retrying')
                self._last_open_log = now
            return False

        try:
            self.ser = serial.Serial(port, BAUD, timeout=0.12)
        except Exception as e:
            self.ser = None
            if now - self._last_open_log >= 10.0:
                self.get_logger().warn(f'Serial open failed ({port}): {e} — retrying')
                self._last_open_log = now
            return False

        self.port = port
        self._bad_reads = 0
        self.get_logger().info(f'Serial {port} @ {BAUD} baud opened')
        return True

    def _close_serial(self):
        """Close and drop the port so the loop reopens it on the next pass."""
        if self.ser is not None:
            try:
                self.ser.close()
            except Exception:
                pass
        self.ser = None

    def _log_comm_error(self, e):
        self._err_count += 1
        now = time.monotonic()
        # Log first occurrence immediately, then at most once every 10 s.
        if self._err_count == 1 or now - self._last_err_log >= 10.0:
            self.get_logger().warn(f'Comm error (×{self._err_count}): {e}')
            self._err_count = 0
            self._last_err_log = now

    def _speed_cb(self, msg):
        if len(msg.data) >= 2:
            self.speed_l = int(msg.data[0])
            self.speed_r = int(msg.data[1])
            self.last_cmd = time.time()

    def _loop(self):
        # The robot's internal Modbus master sends a 50-byte cycle every 100ms.
        # The active-transmit window is ~13ms; the silent gap is ~87ms.
        # We flush, wait 50ms for any in-flight frame to finish, then send in the gap.
        while self.running:
            if self.ser is None:
                if not self._open_serial():
                    time.sleep(0.2)
                continue

            if time.time() - self.last_cmd > CMD_TIMEOUT:
                self.speed_l = 0
                self.speed_r = 0

            try:
                self.ser.reset_input_buffer()
                time.sleep(0.05)   # let any in-flight autonomous frame finish

                # Write speed
                self.ser.write(write_speed_frame(self.speed_l, self.speed_r))
                ack = self.ser.read(8)
                if len(ack) != 8 or ack[0] != SLAVE or ack[1] != 0x10:
                    self.get_logger().debug(f'Speed ack bad ({len(ack)}B): {ack.hex()}')
                    # Flush stale bytes, then still attempt the sensor read so
                    # battery/odom data is not blocked by a missed speed ACK.
                    self.ser.reset_input_buffer()

                time.sleep(0.003)

                # Read sensors
                self.ser.write(read_sensors_frame())
                resp = self.ser.read(3 + SENSOR_N * 2 + 2)
                if len(resp) == 19 and resp[0] == SLAVE and resp[1] == 0x03:
                    regs = struct.unpack(f'>{SENSOR_N}H', resp[3:17])
                    self._publish(regs)
                    self._bad_reads = 0
                else:
                    self.get_logger().debug(f'Sensor read bad ({len(resp)}B): {resp.hex()}')
                    self._bad_reads += 1

            except (serial.SerialException, OSError) as e:
                # The port vanished or errored (USB unplug, driver reset). Drop
                # it and let the loop reopen — this is what makes the link
                # self-heal instead of wedging until a manual restart.
                self._log_comm_error(e)
                self._close_serial()
                continue
            except Exception as e:
                self._log_comm_error(e)
            else:
                self._err_count = 0   # reset streak on a successful cycle

            # Self-heal a wedged-but-open port: if sensor reads stay bad for a
            # sustained stretch (e.g. framing left corrupt after a bus
            # collision), reopen it rather than sit silently disconnected.
            if self._bad_reads >= BAD_READ_LIMIT:
                self.get_logger().warn(
                    f'{self._bad_reads} consecutive bad reads — reopening {self.port}')
                self._close_serial()
                continue

            time.sleep(0.03)  # ~10 Hz total

    def _publish(self, regs):
        sensors = parse_sensor_regs(regs)

        msg = Float32();         msg.data = sensors["battery_v"]
        self.battery_pub.publish(msg)

        msg = Int32MultiArray(); msg.data = [sensors["odom_l"], sensors["odom_r"]]
        self.odom_pub.publish(msg)

        msg = UInt16();          msg.data = sensors["error_code"]
        self.error_pub.publish(msg)

        msg = UInt8();           msg.data = sensors["oil_pct"]
        self.oil_pub.publish(msg)

    def destroy_node(self):
        self.running = False
        if self.ser:
            try:
                self.ser.write(write_speed_frame(0, 0))
                time.sleep(0.05)
            except Exception:
                pass
            self.ser.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RobotBaseNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    node.destroy_node()
    rclpy.shutdown()
