#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2
from nav_msgs.msg import Odometry
import rosbag2_py
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions

class DataRecordNode(Node):
    def __init__(self):
        super().__init__("data_record_node")
        # 配置 bag 存储
        self.storage_options = StorageOptions(uri="recorded_data", storage_id="sqlite3")
        self.converter_options = ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr")
        self.writer = rosbag2_py.SequentialWriter()
        self.writer.open(self.storage_options, self.converter_options)

        # 声明要录制的话题
        topics = [
            ("/scan", "sensor_msgs/msg/LaserScan"),
            ("/camera/depth/color/points", "sensor_msgs/msg/PointCloud2"),
            ("/odom", "nav_msgs/msg/Odometry")
        ]
        for topic_name, topic_type in topics:
            self.writer.create_topic(rosbag2_py.TopicMetadata(name=topic_name, type=topic_type, serialization_format="cdr"))
            # 订阅话题并录制
            self.create_subscription(eval(topic_type), topic_name, lambda msg, tn=topic_name: self.write_bag(tn, msg), 10)

        self.get_logger().info("数据录制已启动，保存至 recorded_data 目录")

    def write_bag(self, topic_name, msg):
        self.writer.write(topic_name, msg, self.get_clock().now().nanoseconds)

def main(args=None):
    rclpy.init(args=args)
    node = DataRecordNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
