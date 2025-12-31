#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2
from nav_msgs.msg import Odometry
import message_filters
from message_filters import ApproximateTimeSynchronizer

class SensorSyncNode(Node):
    def __init__(self):
        super().__init__('sensor_sync_node')
        # 订阅未同步的传感器话题
        laser_sub = message_filters.Subscriber(self, LaserScan, '/scan_raw')
        camera_sub = message_filters.Subscriber(self, PointCloud2, '/camera/depth/color/points_raw')
        odom_sub = message_filters.Subscriber(self, Odometry, '/odom_raw')

        # 近似时间同步：允许 100ms 时间差
        self.ts = ApproximateTimeSynchronizer(
            [laser_sub, camera_sub, odom_sub], queue_size=10, slop=0.1
        )
        self.ts.registerCallback(self.sync_callback)

        # 发布同步后的话题
        self.laser_pub = self.create_publisher(LaserScan, '/scan', 10)
        self.camera_pub = self.create_publisher(PointCloud2, '/camera/depth/color/points', 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)

        self.get_logger().info("Sensor Time Synchronization Node Started!")

    def sync_callback(self, laser_msg, camera_msg, odom_msg):
        """时间同步回调：统一时间戳为激光雷达时间戳"""
        sync_time = laser_msg.header.stamp
        # 更新所有消息的时间戳
        camera_msg.header.stamp = sync_time
        odom_msg.header.stamp = sync_time
        # 发布同步后的消息
        self.laser_pub.publish(laser_msg)
        self.camera_pub.publish(camera_msg)
        self.odom_pub.publish(odom_msg)

def main(args=None):
    rclpy.init(args=args)
    node = SensorSyncNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
