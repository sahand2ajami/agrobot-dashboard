import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int32MultiArray, UInt16, UInt8, Int16MultiArray
import serial
import struct
import threading
import time

from .protocol import (SLAVE, SENSOR_N, write_speed_frame, read_sensors_frame,
                       parse_sensor_regs)

PORT        = '/dev/ttyUSB0'
BAUD        = 38400
CMD_TIMEOUT = 1.5


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

        try:
            self.ser = serial.Serial(PORT, BAUD, timeout=0.12)
            self.get_logger().info(f'Serial {PORT} @ {BAUD} baud opened')
        except Exception as e:
            self.get_logger().error(f'Serial open failed: {e}')
            self.ser = None

        self.running = True
        self._err_count  = 0       # consecutive comm errors
        self._last_err_log = 0.0   # monotonic time of last logged error
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

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
                time.sleep(0.1)
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
                else:
                    self.get_logger().debug(f'Sensor read bad ({len(resp)}B): {resp.hex()}')


            except Exception as e:
                self._err_count += 1
                now = time.monotonic()
                # Log first occurrence immediately, then at most once every 10 s.
                if self._err_count == 1 or now - self._last_err_log >= 10.0:
                    self.get_logger().warn(
                        f'Comm error (×{self._err_count}): {e}')
                    self._err_count  = 0
                    self._last_err_log = now
            else:
                self._err_count = 0   # reset streak on a successful cycle

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
