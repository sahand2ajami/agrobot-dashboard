"""Dead-reckoning odometry for the Agrobot chassis variants.

Publishes nav_msgs/Odometry on /odom and the odom→base_footprint TF from
wheel feedback. The chassis variant is selected by the `car_type` ROS
parameter (falling back to the CAR_TYPE environment variable for
compatibility with the shell launchers).

Variant feedback sources:
  T3          — wheel RPMs on /avatar_robot/vel_raw (Twist: linear.x = left
                RPM, angular.x = right RPM — a legacy packing kept for the T3
                firmware). NOTE: robot_base_node does not publish this topic;
                T3 odometry only works with firmware that does.
  T13 / T17E  — encoder pulse counts on /avatar_robot/wheel_odom
                (Int32MultiArray [left, right]).
"""
import math
import os

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, TransformStamped
from std_msgs.msg import Int32MultiArray
from tf2_ros import TransformBroadcaster

# Per-variant geometry. wheel_radius/axle_width in metres.
VARIANTS = {
    "T3":   {"wheel_radius": 0.065 / 2.0, "axle_width": 0.29,
             "reduction": 14.0},                       # RPM feedback
    "T13":  {"wheel_radius": 0.18 / 2.0,  "axle_width": 0.854,
             "pulse_per_round": 1000.0},               # encoder feedback
    "T17E": {"wheel_radius": 0.26 / 2.0,  "axle_width": 1.30,
             "pulse_per_round": 3000.0},               # encoder feedback
}

# Dead zones: sub-noise readings are treated as standstill so the pose
# doesn't drift in RViz while the robot is parked.
WHEEL_VEL_DEADZONE = 0.0005   # m/s per wheel
YAW_RATE_DEADZONE  = 0.001    # rad/s


class OdomCalculation(Node):
    def __init__(self):
        super().__init__('odom_calculation')

        default = os.environ.get('CAR_TYPE', 'T3')
        self.declare_parameter('car_type', default)
        self.car_type = self.get_parameter('car_type').value

        if self.car_type not in VARIANTS:
            raise ValueError(
                f"unknown car_type '{self.car_type}' — expected one of "
                f"{sorted(VARIANTS)} (set the car_type parameter or CAR_TYPE env)")
        v = VARIANTS[self.car_type]
        self.wheel_radius = v["wheel_radius"]
        self.axle_width   = v["axle_width"]

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = self.get_clock().now()
        self.current_linear = 0.0
        self.current_angular = 0.0

        self.tf_broadcaster = TransformBroadcaster(self)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)

        if self.car_type == "T3":
            self.reduction = v["reduction"]
            self.create_subscription(Twist, '/avatar_robot/vel_raw',
                                     self._on_rpm, 10)
            self.get_logger().warning(
                "car_type=T3 expects wheel RPMs on /avatar_robot/vel_raw — "
                "robot_base_node does not publish it; odometry stays zero "
                "unless the T3 firmware bridge is running")
        else:
            self.pulse_per_m = v["pulse_per_round"] / (math.pi * 2.0 * self.wheel_radius)
            self._last_pulses = None     # (left, right) at the previous message
            self._last_pulse_time = None
            self.create_subscription(Int32MultiArray, '/avatar_robot/wheel_odom',
                                     self._on_pulses, 10)

        self.get_logger().info(f"car_type: {self.car_type}")
        self.create_timer(0.1, self.publish_odom)

    # ── feedback callbacks ────────────────────────────────────────────────────
    def _on_rpm(self, msg):
        """T3: wheel RPMs packed as Twist(linear.x=left, angular.x=right)."""
        peri = 2 * math.pi * self.wheel_radius
        v_l = (msg.linear.x  / self.reduction * peri) / 60.0
        v_r = (msg.angular.x / self.reduction * peri) / 60.0
        self.update_odom(v_l, v_r)

    def _on_pulses(self, msg):
        """T13/T17E: cumulative encoder pulse counts [left, right]."""
        if len(msg.data) < 2:
            return
        now = self.get_clock().now()
        pulses = (msg.data[0], msg.data[1])
        if self._last_pulses is None:
            self._last_pulses, self._last_pulse_time = pulses, now
            return
        # Measured interval, not an assumed 10 Hz — the bus cycle drifts with
        # Modbus latency, and assuming 0.1 s made velocity disagree with the
        # dt the integrator used.
        dt = (now - self._last_pulse_time).nanoseconds / 1e9
        if dt <= 1e-4:
            return
        dl = (pulses[0] - self._last_pulses[0]) / self.pulse_per_m
        dr = (pulses[1] - self._last_pulses[1]) / self.pulse_per_m
        self._last_pulses, self._last_pulse_time = pulses, now

        v_l, v_r = dl / dt, dr / dt
        if abs(v_l) < WHEEL_VEL_DEADZONE:
            v_l = 0.0
        if abs(v_r) < WHEEL_VEL_DEADZONE:
            v_r = 0.0
        self.update_odom(v_l, v_r)

    # ── integration + publishing ──────────────────────────────────────────────
    def update_odom(self, v_l, v_r):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now

        vx  = (v_l + v_r) * 0.5
        vth = (v_r - v_l) / self.axle_width
        if abs(vth) < YAW_RATE_DEADZONE:
            vth = 0.0

        self.theta += vth * dt
        self.x += vx * math.cos(self.theta) * dt
        self.y += vx * math.sin(self.theta) * dt
        self.current_linear = vx
        self.current_angular = vth

    def publish_odom(self):
        now = self.get_clock().now().to_msg()
        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = "odom"
        t.child_frame_id = "base_footprint"
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0

        q = self.euler_to_quat(0, 0, self.theta)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        self.tf_broadcaster.sendTransform(t)

        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_footprint"
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation = t.transform.rotation
        odom.twist.twist.linear.x = self.current_linear
        odom.twist.twist.angular.z = self.current_angular
        self.odom_pub.publish(odom)

    @staticmethod
    def euler_to_quat(roll, pitch, yaw):
        cy, sy = math.cos(yaw * 0.5),   math.sin(yaw * 0.5)
        cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
        cr, sr = math.cos(roll * 0.5),  math.sin(roll * 0.5)
        return [sr * cp * cy - cr * sp * sy,
                cr * sp * cy + sr * cp * sy,
                cr * cp * sy - sr * sp * cy,
                cr * cp * cy + sr * sp * sy]


def main(args=None):
    rclpy.init(args=args)
    node = OdomCalculation()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    node.destroy_node()
    rclpy.shutdown()
