import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped

class PathPublisher(Node):
    def __init__(self):
        super().__init__('path_pub')
        self.get_logger().info("✅ 轨迹发布节点启动")
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.path_pub = self.create_publisher(Path, '/odom_path', 10)
        self.path = Path()
        self.path.header.frame_id = 'odom'

    def odom_cb(self, msg):
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose
        self.path.poses.append(pose)
        self.path_pub.publish(self.path)

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(PathPublisher())
    rclpy.shutdown()
