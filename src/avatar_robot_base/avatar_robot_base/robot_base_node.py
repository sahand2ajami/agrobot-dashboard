import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int32MultiArray, UInt16, UInt8, Int16MultiArray
import serial
import struct
import threading
import time

PORT        = '/dev/ttyUSB0'
BAUD        = 38400
SLAVE       = 0x01
SPEED_REG   = 0x0016
SENSOR_REG  = 0x0019
SENSOR_N    = 7
BATT_PCT_REG = 0x0033   # battery percentage (0-100), confirmed by register scan
CMD_TIMEOUT = 1.5


def _crc16(data: bytes) -> bytes:
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return struct.pack('<H', crc)


def _write_cmd(speed_l, speed_r):
    pdu = struct.pack('>BBHHB', SLAVE, 0x10, SPEED_REG, 3, 6)
    pdu += struct.pack('>hhh', int(speed_l), int(speed_r), 0)
    return pdu + _crc16(pdu)


def _read_cmd():
    pdu = struct.pack('>BBHH', SLAVE, 0x03, SENSOR_REG, SENSOR_N)
    return pdu + _crc16(pdu)

def _batt_pct_cmd():
    pdu = struct.pack('>BBHH', SLAVE, 0x03, BATT_PCT_REG, 1)
    return pdu + _crc16(pdu)


class RobotBaseNode(Node):
    def __init__(self):
        super().__init__('robot_base_node')

        self.speed_l = 0
        self.speed_r = 0
        self.last_cmd = time.time()

        self.battery_pub     = self.create_publisher(Float32,         '/avatar_robot/battery',     10)
        self.battery_pct_pub = self.create_publisher(UInt8,           '/avatar_robot/battery_pct', 10)
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
                self.ser.write(_write_cmd(self.speed_l, self.speed_r))
                ack = self.ser.read(8)
                if len(ack) != 8 or ack[0] != SLAVE or ack[1] != 0x10:
                    self.get_logger().debug(f'Speed ack bad ({len(ack)}B): {ack.hex()}')
                    # Flush stale bytes, then still attempt the sensor read so
                    # battery/odom data is not blocked by a missed speed ACK.
                    self.ser.reset_input_buffer()

                time.sleep(0.003)

                # Read sensors
                self.ser.write(_read_cmd())
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
        # Sensor register layout (0x0019 .. 0x001F) — confirmed by passive bus capture:
        #   [0] odom_L hi  [1] odom_L lo  [2] odom_R hi
        #   [3] odom_R lo  [4] battery×100 (V)  [5] fault code  [6] oil%
        raw_l     = (regs[0] << 16) | regs[1]
        raw_r     = (regs[2] << 16) | regs[3]
        # Registers are unsigned 16-bit; combined value is unsigned 32-bit.
        # Int32MultiArray requires signed int32. Sign-extend so the encoder
        # wraps correctly across zero (e.g. 0xFFFF_FFFF → -1, not a publish error).
        odom_l    = raw_l if raw_l < 0x8000_0000 else raw_l - 0x1_0000_0000
        odom_r    = raw_r if raw_r < 0x8000_0000 else raw_r - 0x1_0000_0000
        battery_v = regs[4] / 100.0
        oil_pct   = regs[6]
        error_code = regs[5]

        msg = Float32();         msg.data = battery_v
        self.battery_pub.publish(msg)

        msg = Int32MultiArray(); msg.data = [odom_l, odom_r]
        self.odom_pub.publish(msg)

        msg = UInt16();          msg.data = error_code
        self.error_pub.publish(msg)

        msg = UInt8();           msg.data = int(oil_pct) & 0xFF
        self.oil_pub.publish(msg)

    def destroy_node(self):
        self.running = False
        if self.ser:
            try:
                self.ser.write(_write_cmd(0, 0))
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
