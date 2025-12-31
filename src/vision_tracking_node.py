#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection2DArray
from geometry_msgs.msg import Twist
import numpy as np

class VisionTrackingNode(Node):
    def __init__(self):
        super().__init__("vision_tracking_node")
        # 追踪参数
        self.track_target = 0  # 追踪目标：0=人员，39=纸箱
        self.safe_distance = 1.0  # 与追踪目标的安全距离
        self.angular_gain = 0.5  # 角度控制增益

        # 订阅检测结果，发布控制指令（联动避障）
        self.detection_sub = self.create_subscription(Detection2DArray, "/vision_detections", self.track_callback, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel_vision", 10)  # 单独发布视觉控制指令
        self.get_logger().info("视觉目标追踪节点已启动，当前追踪目标：人员")

    def track_callback(self, msg):
        twist = Twist()
        # 查找目标类别
        target_det = None
        for det in msg.detections:
            if det.results[0].id == self.track_target:
                target_det = det
                break

        if target_det is not None:
            # 计算目标偏移量（图像中心为原点）
            target_x = target_det.bbox.center.x - 0.5  # -0.5~0.5
            # 角度控制：偏移量转为角速度
            twist.angular.z = -target_x * self.angular_gain
            # 距离控制：默认缓慢前进，保持安全距离（后续与激光避障融合）
            twist.linear.x = 0.1
        else:
            # 未检测到目标，缓慢旋转搜索
            twist.angular.z = 0.3

        self.cmd_vel_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = VisionTrackingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
