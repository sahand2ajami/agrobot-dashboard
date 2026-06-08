import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty

class ResetService(Node):
    def __init__(self):
        super().__init__('reset_service')
        self.srv = self.create_service(Empty, '/avatar_robot/reset_position', self.cb)
        self.get_logger().info("✅ 里程清零服务已启动")

    def cb(self, req, res):
        self.get_logger().info("🔄 里程计已清零")
        return res

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(ResetService())
    rclpy.shutdown()
