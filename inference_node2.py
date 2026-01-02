import rclpy
from rclpy.node import Node
import torch
import numpy as np

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

from .policy import RobotPolicy
from .safety import clip_action

class PolicyNode(Node):
    def __init__(self):
        super().__init__('policy_node')

        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)

        self.model = RobotPolicy()
        self.model.load_state_dict(
            torch.load('weights/policy.pth', map_location='cpu'))
        self.model.eval()

        self.obs = None
        self.timer = self.create_timer(0.05, self.control_loop)  # 20Hz

    def scan_callback(self, msg):
        scan = np.array(msg.ranges, dtype=np.float32)
        scan = np.nan_to_num(scan, nan=5.0, posinf=5.0)
        self.obs = scan[:10]  # 取前 10 个激光

    def control_loop(self):
        if self.obs is None:
            return

        obs = torch.tensor(self.obs).unsqueeze(0)

        with torch.no_grad():
            action = self.model(obs).squeeze().numpy()

        v, w = clip_action(action)

        cmd = Twist()
        cmd.linear.x = float(v)
        cmd.angular.z = float(w)
        self.pub.publish(cmd)

def main():
    rclpy.init()
    rclpy.spin(PolicyNode())
    rclpy.shutdown()
