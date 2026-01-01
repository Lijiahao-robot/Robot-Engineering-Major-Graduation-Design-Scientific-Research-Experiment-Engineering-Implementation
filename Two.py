# node_watchdog.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time
import os

class WatchDog(Node):
    def __init__(self):
        super().__init__('watchdog')

        self.last_heartbeat = time.time()
        self.sub = self.create_subscription(
            String, '/heartbeat', self.cb, 10)

        self.timer = self.create_timer(2.0, self.check)

    def cb(self, msg):
        self.last_heartbeat = time.time()

    def check(self):
        if time.time() - self.last_heartbeat > 5.0:
            self.get_logger().warn("Node timeout! Restarting...")
            os.system("ros2 launch your_pkg your_node.launch.py")

def main():
    rclpy.init()
    node = WatchDog()
    rclpy.spin(node)
    rclpy.shutdown()
