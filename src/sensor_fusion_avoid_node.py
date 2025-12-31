#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2
from geometry_msgs.msg import Twist
import numpy as np
import pclpy
from pclpy import pcl
from rclpy.parameter import Parameter

class SensorFusionAvoidNode(Node):
    def __init__(self):
        super().__init__('sensor_fusion_avoid_node')
        # ========== 参数声明 ==========
        self.declare_parameters(
            namespace='',
            parameters=[
                ('safe_radius', 0.6),
                ('laser_weight', 0.7),
                ('camera_weight', 0.3),
                ('use_camera', True),
                ('buffer_size', 5),
                ('linear_vel', 0.2),
                ('angular_vel', 0.5)
            ]
        )

        # ========== 获取参数 ==========
        self.safe_radius = self.get_parameter('safe_radius').value
        self.laser_weight = self.get_parameter('laser_weight').value
        self.camera_weight = self.get_parameter('camera_weight').value
        self.use_camera = self.get_parameter('use_camera').value
        self.buffer_size = self.get_parameter('buffer_size').value
        self.linear_vel = self.get_parameter('linear_vel').value
        self.angular_vel = self.get_parameter('angular_vel').value

        # ========== 订阅器与发布器 ==========
        self.laser_sub = self.create_subscription(
            LaserScan, '/scan', self.laser_callback, 10
        )
        self.camera_sub = self.create_subscription(
            PointCloud2, '/camera/depth/color/points', self.camera_callback, 10
        )
        self.cmd_vel_pub = self.create_publisher(Twist, '/vla_cmd_vel', 10)

        # ========== 数据缓存（滑动平均滤波） ==========
        self.laser_buffer = []
        self.camera_buffer = []
        self.laser_data = None
        self.camera_data = None

        # ========== 定时器 ==========
        self.timer = self.create_timer(0.02, self.fusion_avoid_logic)

        # ========== 动态参数回调 ==========
        self.add_on_set_parameters_callback(self.param_change_callback)

        self.get_logger().info("Sensor Fusion Avoid Node Initialized!")
        self.get_logger().info(f"Safe Radius: {self.safe_radius}m | Buffer Size: {self.buffer_size}")

    def param_change_callback(self, params):
        """动态参数回调函数"""
        for param in params:
            if param.name == 'safe_radius' and param.type_ == Parameter.Type.DOUBLE:
                self.safe_radius = param.value
            elif param.name == 'laser_weight' and param.type_ == Parameter.Type.DOUBLE:
                self.laser_weight = param.value
            elif param.name == 'camera_weight' and param.type_ == Parameter.Type.DOUBLE:
                self.camera_weight = param.value
        return rclpy.parameter.SetParametersResult(successful=True)

    def laser_callback(self, msg):
        """激光雷达数据回调"""
        self.laser_buffer.append(msg)
        if len(self.laser_buffer) > self.buffer_size:
            self.laser_buffer.pop(0)
        self.laser_data = self.laser_buffer[-1] if self.laser_buffer else None

    def camera_callback(self, msg):
        """深度相机数据回调"""
        if not self.use_camera:
            return
        self.camera_buffer.append(msg)
        if len(self.camera_buffer) > self.buffer_size:
            self.camera_buffer.pop(0)
        self.camera_data = self.camera_buffer[-1] if self.camera_buffer else None

    def stop_robot(self):
        """机器人紧急停止"""
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.0
        cmd_vel.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd_vel)
        self.get_logger().error("Both Sensors Lost - Robot Stopped!")

    def only_laser_avoid(self):
        """纯激光避障逻辑"""
        if not self.laser_data:
            self.stop_robot()
            return
        
        # 滑动平均滤波
        laser_ranges_list = []
        for msg in self.laser_buffer:
            ranges = np.array(msg.ranges)[:360]
            ranges = np.nan_to_num(ranges, nan=self.safe_radius*2, posinf=self.safe_radius*2)
            laser_ranges_list.append(ranges)
        laser_ranges = np.mean(laser_ranges_list, axis=0)

        # 避障决策
        min_dist = np.min(laser_ranges)
        min_index = np.argmin(laser_ranges)
        cmd_vel = Twist()

        if min_dist < self.safe_radius:
            cmd_vel.linear.x = 0.05
            cmd_vel.angular.z = self.angular_vel if min_index < len(laser_ranges)//2 else -self.angular_vel
            self.get_logger().warn(f"Laser Obstacle: {min_dist:.2f}m - Turning")
        else:
            cmd_vel.linear.x = self.linear_vel
            cmd_vel.angular.z = 0.0
        
        self.cmd_vel_pub.publish(cmd_vel)

    def only_camera_avoid(self):
        """纯视觉避障逻辑"""
        if not self.camera_data:
            self.stop_robot()
            return
        
        # 滑动平均滤波
        camera_ranges_list = []
        for msg in self.camera_buffer:
            cloud = pcl.PointCloud.PointXYZ()
            pclpy.io.fromROSMsg(msg, cloud)
            ranges = [np.linalg.norm([p.x, p.y]) for p in cloud if np.linalg.norm([p.x, p.y]) < self.safe_radius*2][:360]
            camera_ranges_list.append(ranges)
        
        # 对齐长度
        max_len = max([len(r) for r in camera_ranges_list]) if camera_ranges_list else 0
        camera_ranges_padded = []
        for r in camera_ranges_list:
            padded = np.pad(r, (0, max_len - len(r)), 'edge')[:360]
            camera_ranges_padded.append(padded)
        camera_ranges = np.mean(camera_ranges_padded, axis=0) if camera_ranges_padded else np.zeros(360)

        # 避障决策
        min_dist = np.min(camera_ranges)
        min_index = np.argmin(camera_ranges)
        cmd_vel = Twist()

        if min_dist < self.safe_radius:
            cmd_vel.linear.x = 0.05
            cmd_vel.angular.z = self.angular_vel if min_index < len(camera_ranges)//2 else -self.angular_vel
        else:
            cmd_vel.linear.x = self.linear_vel
            cmd_vel.angular.z = 0.0
        
        self.cmd_vel_pub.publish(cmd_vel)

    def fusion_avoid_logic(self):
        """激光-视觉融合避障主逻辑"""
        # 纯激光模式
        if not self.use_camera:
            self.only_laser_avoid()
            return
        
        # 传感器容错
        if self.laser_data is None and self.camera_data is not None:
            self.only_camera_avoid()
            return
        elif self.camera_data is None and self.laser_data is not None:
            self.only_laser_avoid()
            return
        elif self.laser_data is None or self.camera_data is None:
            self.stop_robot()
            return

        # ========== 激光数据处理 ==========
        laser_ranges_list = []
        for msg in self.laser_buffer:
            ranges = np.array(msg.ranges)[:360]
            ranges = np.nan_to_num(ranges, nan=self.safe_radius*2, posinf=self.safe_radius*2)
            laser_ranges_list.append(ranges)
        laser_ranges = np.mean(laser_ranges_list, axis=0)

        # ========== 视觉数据处理 ==========
        camera_ranges_list = []
        for msg in self.camera_buffer:
            cloud = pcl.PointCloud.PointXYZ()
            pclpy.io.fromROSMsg(msg, cloud)
            ranges = [np.linalg.norm([p.x, p.y]) for p in cloud if np.linalg.norm([p.x, p.y]) < self.safe_radius*2][:360]
            camera_ranges_list.append(ranges)
        
        max_len = max([len(r) for r in camera_ranges_list])
        camera_ranges_padded = []
        for r in camera_ranges_list:
            padded = np.pad(r, (0, max_len - len(r)), 'edge')[:360]
            camera_ranges_padded.append(padded)
        camera_ranges = np.mean(camera_ranges_padded, axis=0)

        # ========== 加权融合 ==========
        if len(camera_ranges) < len(laser_ranges):
            camera_ranges = np.pad(camera_ranges, (0, len(laser_ranges)-len(camera_ranges)), 'edge')
        fusion_ranges = self.laser_weight * laser_ranges + self.camera_weight * camera_ranges

        # ========== 避障决策 ==========
        min_dist = np.min(fusion_ranges)
        min_index = np.argmin(fusion_ranges)
        cmd_vel = Twist()

        if min_dist < self.safe_radius:
            cmd_vel.linear.x = 0.05
            cmd_vel.angular.z = self.angular_vel if min_index < len(fusion_ranges)//2 else -self.angular_vel
            self.get_logger().info(f"Fusion Obstacle: {min_dist:.2f}m - Turning")
        else:
            cmd_vel.linear.x = self.linear_vel
            cmd_vel.angular.z = 0.0
        
        self.cmd_vel_pub.publish(cmd_vel)

def main(args=None):
    rclpy.init(args=args)
    node = SensorFusionAvoidNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
