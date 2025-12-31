#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import numpy as np
import cv2

class MapOptimizeNode(Node):
    def __init__(self):
        super().__init__("map_optimize_node")
        self.map_sub = self.create_subscription(OccupancyGrid, "/map", self.map_callback, 10)
        self.optimized_map_pub = self.create_publisher(OccupancyGrid, "/optimized_map", 10)
        self.get_logger().info("地图后处理节点已启动")

    def map_callback(self, msg):
        # 1. 将地图转为二维数组
        map_data = np.array(msg.data).reshape(msg.info.height, msg.info.width)
        # 2. 中值滤波去除噪声
        map_data = cv2.medianBlur(map_data.astype(np.uint8), 3)
        # 3. 二值化处理（0=空闲，100=障碍）
        _, map_data = cv2.threshold(map_data, 50, 100, cv2.THRESH_BINARY)
        # 4. 形态学闭运算填充小空洞
        kernel = np.ones((3,3), np.uint8)
        map_data = cv2.morphologyEx(map_data, cv2.MORPH_CLOSE, kernel)
        # 5. 发布优化后的地图
        optimized_msg = msg
        optimized_msg.data = map_data.flatten().tolist()
        self.optimized_map_pub.publish(optimized_msg)
        self.get_logger().info("地图优化完成，已发布至 /optimized_map")

def main(args=None):
    rclpy.init(args=args)
    node = MapOptimizeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
