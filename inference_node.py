import rclpy
from rclpy.node import Node
import torch
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import numpy as np

from .policy import RobotPolicy

class PolicyNode(Node):
    def __init__(self):
        super().__init__('policy_node')

        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)

        self.model = RobotPolicy()
        self.model.load_state_dict(torch.load(
            'weights/policy.pth', map_location='cpu'))
        self.model.eval()

    def scan_callback(self, msg):
        obs = np.array(msg.ranges[:10], dtype=np.float32)
        obs = torch.tensor(obs).unsqueeze(0)

        with torch.no_grad():
            action = self.model(obs).squeeze().numpy()

        cmd = Twist()
        cmd.linear.x = float(action[0])
        cmd.angular.z = float(action[1])
        self.pub.publish(cmd)

def main():
    rclpy.init()
    rclpy.spin(PolicyNode())
    rclpy.shutdown()
