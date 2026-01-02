import rclpy
from rclpy.node import Node
import torch
import cv2
import numpy as np

from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge

from .vision_model import VisionPolicy

class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')

        self.bridge = CvBridge()
        self.sub = self.create_subscription(
            Image, '/camera/color/image_raw',
            self.image_callback, 10)

        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.model = VisionPolicy()
        self.model.load_state_dict(
            torch.load('weights/vision_policy.pth',
                       map_location='cpu'))
        self.model.eval()

        self.timer = self.create_timer(0.1, self.run)
        self.last_img = None

    def image_callback(self, msg):
        img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        img = cv2.resize(img, (224, 224))
        img = img.astype(np.float32) / 255.0
        img = np.transpose(img, (2, 0, 1))
        self.last_img = torch.tensor(img).unsqueeze(0)

    def run(self):
        if self.last_img is None:
            return

        with torch.no_grad():
            action = self.model(self.last_img).squeeze().numpy()

        cmd = Twist()
        cmd.linear.x = float(np.clip(action[0], -0.2, 0.2))
        cmd.angular.z = float(np.clip(action[1], -1.0, 1.0))
        self.pub.publish(cmd)

def main():
    rclpy.init()
    rclpy.spin(VisionNode())
    rclpy.shutdown()
