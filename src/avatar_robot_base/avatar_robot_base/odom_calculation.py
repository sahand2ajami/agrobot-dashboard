import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, TransformStamped
from tf2_ros import TransformBroadcaster
import math
import os
from std_msgs.msg import Int32MultiArray

class OdomCalculation(Node):
    def __init__(self):
        super().__init__('odom_calculation')

        self.CAR_TYPE = os.environ.get('CAR_TYPE', 'T3')
        self.get_logger().info(f'✅ 车型: {self.CAR_TYPE}')

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = self.get_clock().now()

        self.current_linear = 0.0
        self.current_angular = 0.0

        # T3
        if self.CAR_TYPE == "T3":
            self.W_R = 0.065 / 2.0
            self.W_DIST = 0.29
            self.REDUCTION = 14.0

        # T13
        elif self.CAR_TYPE == "T13":
            self.W_R = 0.18 / 2.0
            self.W_DIST = 0.854
            self.PULSE_PER_ROUND = 1000.0
            self.PULSE_PER_M = self.PULSE_PER_ROUND / (math.pi * 0.18)
            self.last_L = 0
            self.last_R = 0

        # T17E  【和T13完全一样逻辑，仅尺寸不同】
        elif self.CAR_TYPE == "T17E":
            self.W_R = 0.26 / 2.0
            self.W_DIST = 1.30
            self.PULSE_PER_ROUND = 3000.0
            self.PULSE_PER_M = self.PULSE_PER_ROUND / (math.pi * 0.26)
            self.last_L = 0
            self.last_R = 0

        self.tf_broadcaster = TransformBroadcaster(self)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)

        if self.CAR_TYPE == "T3":
            self.create_subscription(Twist, '/avatar_robot/vel_raw', self.cb_t3, 10)
        else:
            self.create_subscription(Int32MultiArray, '/avatar_robot/wheel_odom', self.cb_pulse, 10)

        self.create_timer(0.1, self.publish_odom)

    def cb_t3(self, msg):
        rpm_l = msg.linear.x
        rpm_r = msg.angular.x
        peri = 2 * math.pi * self.W_R
        v_l = (rpm_l / self.REDUCTION * peri) / 60.0
        v_r = (rpm_r / self.REDUCTION * peri) / 60.0
        self.update_odom(v_l, v_r)

    def cb_pulse(self, msg):
        if len(msg.data) < 2:
            return

        Lp = msg.data[0]
        Rp = msg.data[1]

        # T13 和 T17E 使用 **完全相同的计算方式**
        if self.CAR_TYPE == "T13" or self.CAR_TYPE == "T17E":
            dL = (Lp - self.last_L) / self.PULSE_PER_M
            dR = (Rp - self.last_R) / self.PULSE_PER_M
            self.last_L = Lp
            self.last_R = Rp

            v_l = dL / 0.1
            v_r = dR / 0.1

            # 静止死区，防止RViz自己跑
            if abs(v_l) < 0.0005:
                v_l = 0.0
            if abs(v_r) < 0.0005:
                v_r = 0.0

        self.update_odom(v_l, v_r)

    def update_odom(self, v_l, v_r):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now

        vx = (v_l + v_r) * 0.5
        vth = (v_r - v_l) / self.W_DIST

        # 防自转
        if abs(vth) < 0.001:
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

    def euler_to_quat(self, roll, pitch, yaw):
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy
        return [qx, qy, qz, qw]

def main(args=None):
    rclpy.init(args=args)
    node = OdomCalculation()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
