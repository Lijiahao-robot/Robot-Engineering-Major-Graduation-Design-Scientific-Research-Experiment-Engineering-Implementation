#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TransformStamped
import tf2_ros

class FusionFallbackNode(Node):
    def __init__(self):
        super().__init__("fusion_fallback_node")
        # 定位模式：0=激光SLAM（默认），1=轮式里程计，2=惯性导航
        self.loc_mode = 0
        self.laser_loss_time = 0.0
        self.LASER_LOSS_THRESHOLD = 2.0  # 激光丢失 2s 触发 fallback

        # 订阅话题
        self.laser_sub = self.create_subscription(LaserScan, "/scan", self.laser_callback, 10)
        self.slam_odom_sub = self.create_subscription(Odometry, "/slam_odom", self.slam_odom_callback, 10)
        self.wheel_odom_sub = self.create_subscription(Odometry, "/wheel_odom", self.wheel_odom_callback, 10)
        self.imu_odom_sub = self.create_subscription(Odometry, "/imu_odom", self.imu_odom_callback, 10)

        # 发布融合后定位结果
        self.fusion_odom_pub = self.create_publisher(Odometry, "/fused_odom", 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # 定时器检测激光状态
        self.timer = self.create_timer(0.1, self.mode_check_loop)

    def laser_callback(self, msg):
        # 检测激光数据有效性（无有效点则判定为丢失）
        valid_ranges = [r for r in msg.ranges if not (np.isnan(r) or r < 0.1 or r > 5.0)]
        if len(valid_ranges) < 10:
            self.laser_loss_time += 0.1
        else:
            self.laser_loss_time = 0.0
            self.loc_mode = 0  # 激光恢复，切回 SLAM 定位

    def mode_check_loop(self):
        # 触发 fallback 机制
        if self.laser_loss_time > self.LASER_LOSS_THRESHOLD:
            if self.loc_mode == 0:
                self.loc_mode = 1
                self.get_logger().warn("激光丢失，切换至轮式里程计定位")
            # 轮式里程计漂移过大时，切惯性导航（需结合 IMU 数据判断漂移）
            elif self.loc_mode == 1 and self.check_wheel_drift():
                self.loc_mode = 2
                self.get_logger().warn("轮式里程计漂移过大，切换至惯性导航")

    def check_wheel_drift(self):
        # 简单漂移判断：相邻时刻角速度差值超过阈值
        if hasattr(self, "last_angular_z"):
            drift = abs(self.current_angular_z - self.last_angular_z)
            self.last_angular_z = self.current_angular_z
            return drift > 0.5
        return False

    def slam_odom_callback(self, msg):
        if self.loc_mode == 0:
            self.fusion_odom_pub.publish(msg)
            self.broadcast_tf(msg)

    def wheel_odom_callback(self, msg):
        self.current_angular_z = msg.twist.twist.angular.z
        if self.loc_mode == 1:
            self.fusion_odom_pub.publish(msg)
            self.broadcast_tf(msg)

    def imu_odom_callback(self, msg):
        if self.loc_mode == 2:
            self.fusion_odom_pub.publish(msg)
            self.broadcast_tf(msg)

    def broadcast_tf(self, odom_msg):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        t.transform.translation.x = odom_msg.pose.pose.position.x
        t.transform.translation.y = odom_msg.pose.pose.position.y
        t.transform.rotation = odom_msg.pose.pose.orientation
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = FusionFallbackNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
