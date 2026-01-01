# fusion_pose_node.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import numpy as np

class FusionPose(Node):
    def __init__(self):
        super().__init__('fusion_pose')

        self.lidar_sub = self.create_subscription(
            PoseStamped, '/lidar_pose', self.lidar_cb, 10)
        self.vision_sub = self.create_subscription(
            PoseStamped, '/vision_pose', self.vision_cb, 10)

        self.pub = self.create_publisher(
            PoseStamped, '/fusion_pose', 10)

        self.lidar_pose = None
        self.vision_pose = None

    def lidar_cb(self, msg):
        self.lidar_pose = msg
        self.fuse()

    def vision_cb(self, msg):
        self.vision_pose = msg
        self.fuse()

    def fuse(self):
        if self.lidar_pose is None or self.vision_pose is None:
            return

        w_lidar = 0.7
        w_vision = 0.3

        fused = PoseStamped()
        fused.header.stamp = self.get_clock().now().to_msg()

        fused.pose.position.x = (
            w_lidar * self.lidar_pose.pose.position.x +
            w_vision * self.vision_pose.pose.position.x
        )
        fused.pose.position.y = (
            w_lidar * self.lidar_pose.pose.position.y +
            w_vision * self.vision_pose.pose.position.y
        )

        self.pub.publish(fused)

def main():
    rclpy.init()
    node = FusionPose()
    rclpy.spin(node)
    rclpy.shutdown()
