# 移除冗余依赖
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2
from geometry_msgs.msg import Twist
import numpy as np
# 注释掉 pclpy，改用 numpy 轻量化处理点云
# import pclpy
# from pclpy import pcl

class SensorFusionAvoidNode(Node):
    def __init__(self):
        super().__init__('sensor_fusion_avoid_node')
        # 简化参数，只保留核心避障参数
        self.declare_parameters(
            namespace='',
            parameters=[
                ('safe_radius', 0.6),
                ('laser_weight', 0.7),
                ('buffer_size', 3)  # 减小缓存大小，降低内存占用
            ]
        )
        self.safe_radius = self.get_parameter('safe_radius').value
        self.laser_weight = self.get_parameter('laser_weight').value
        self.buffer_size = self.get_parameter('buffer_size').value
        self.camera_weight = 1 - self.laser_weight  # 简化计算，避免重复参数

        # 其他初始化代码不变...

    # 改用 numpy 轻量化处理点云，替代 pclpy
    def process_camera_data(self, msg):
        """轻量化点云处理：只提取 xy 平面距离"""
        # 解析 PointCloud2 数据（仅保留核心字段）
        points = np.frombuffer(msg.data, dtype=np.float32).reshape(-1, 4)
        # 只取 x, y 坐标计算距离
        ranges = np.sqrt(points[:, 0]**2 + points[:, 1]**2)
        # 截断无效数据
        ranges = np.clip(ranges, 0.0, self.safe_radius * 2)
        return ranges[:360]  # 固定长度，匹配激光数据

    # 修改 fusion_avoid_logic 中的视觉数据处理
    def fusion_avoid_logic(self):
        # ... 原有逻辑 ...
        if self.camera_data is not None:
            camera_ranges = self.process_camera_data(self.camera_data)
        # ... 其他逻辑不变 ...
