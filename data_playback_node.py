#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import rosbag2_py
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions

class DataPlaybackNode(Node):
    def __init__(self):
        super().__init__("data_playback_node")
        # 配置 bag 读取
        self.storage_options = StorageOptions(uri="recorded_data", storage_id="sqlite3")
        self.converter_options = ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr")
        self.reader = SequentialReader()
        self.reader.open(self.storage_options, self.converter_options)

        # 创建发布者
        self.pubs = {
            "/scan": self.create_publisher(eval("sensor_msgs/msg/LaserScan"), "/scan", 10),
            "/camera/depth/color/points": self.create_publisher(eval("sensor_msgs/msg/PointCloud2"), "/camera/depth/color/points", 10),
            "/odom": self.create_publisher(eval("nav_msgs/msg/Odometry"), "/odom", 10)
        }

        # 定时器循环回放
        self.timer = self.create_timer(0.1, self.playback_loop)
        self.get_logger().info("离线数据回放已启动")

    def playback_loop(self):
        if self.reader.has_next():
            topic_name, msg, timestamp = self.reader.read_next()
            if topic_name in self.pubs:
                self.pubs[topic_name].publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = DataPlaybackNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
