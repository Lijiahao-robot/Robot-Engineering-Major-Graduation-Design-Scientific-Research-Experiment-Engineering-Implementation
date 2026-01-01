# strategy_manager_node.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import numpy as np

class StrategyManager(Node):
    def __init__(self):
        super().__init__('strategy_manager')

        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)

        self.cmd_pub = self.create_publisher(
            Twist, '/cmd_vel', 10)

        self.strategy = "PID"  # PID / RL / EMERGENCY

    def scan_callback(self, scan):
        ranges = np.array(scan.ranges)
        ranges = ranges[np.isfinite(ranges)]

        obstacle_density = np.sum(ranges < 1.0) / len(ranges)

        if obstacle_density < 0.1:
            self.strategy = "PID"
        elif obstacle_density < 0.4:
            self.strategy = "RL"
        else:
            self.strategy = "EMERGENCY"

        self.publish_cmd()

    def publish_cmd(self):
        cmd = Twist()

        if self.strategy == "PID":
            cmd.linear.x = 0.3
            cmd.angular.z = 0.0

        elif self.strategy == "RL":
            #  rl_avoidance 
            cmd.linear.x = 0.2
            cmd.angular.z = 0.6

        else:  # EMERGENCY
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0

        self.cmd_pub.publish(cmd)

def main():
    rclpy.init()
    node = StrategyManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
