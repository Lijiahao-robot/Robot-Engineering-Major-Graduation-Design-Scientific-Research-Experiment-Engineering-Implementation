#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import LaserScan
import numpy as np
import matplotlib.pyplot as plt
import csv

class PerformanceEvaluationNode(Node):
    def __init__(self):
        super().__init__("performance_evaluation_node")
        # 初始化评估指标
        self.ground_truth = (5.0, 5.0)  # 目标点坐标
        self.position_history = []
        self.collision_count = 0
        self.path_length = 0.0
        self.last_pose = None

        # 订阅话题
        self.create_subscription(Odometry, "/fused_odom", self.odom_callback, 10)
        self.create_subscription(LaserScan, "/scan", self.laser_callback, 10)
        self.create_subscription(Path, "/planned_path", self.path_callback, 10)

        # 定时器：评估完成后生成报告
        self.timer = self.create_timer(30.0, self.generate_report)

    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.position_history.append((x, y))
        if self.last_pose is not None:
            self.path_length += np.hypot(x - self.last_pose[0], y - self.last_pose[1])
        self.last_pose = (x, y)

    def laser_callback(self, msg):
        min_dist = min([r for r in msg.ranges if not np.isnan(r)])
        if min_dist < 0.2:  # 距离小于0.2m判定为碰撞
            self.collision_count += 1

    def path_callback(self, msg):
        self.planned_path_length = sum([
            np.hypot(
                msg.poses[i].pose.position.x - msg.poses[i-1].pose.position.x,
                msg.poses[i].pose.position.y - msg.poses[i-1].pose.position.y
            ) for i in range(1, len(msg.poses))
        ])

    def generate_report(self):
        # 计算定位误差
        final_pose = self.position_history[-1]
        positioning_error = np.hypot(final_pose[0] - self.ground_truth[0], final_pose[1] - self.ground_truth[1])
        # 计算避障成功率
        obstacle_avoidance_success_rate = 1 - (self.collision_count / len(self.position_history))
        # 计算路径规划效率（实际路径长度/规划路径长度）
        path_efficiency = self.path_length / self.planned_path_length if self.planned_path_length > 0 else 0

        # 保存数据到 CSV
        with open("performance_report.csv", "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["指标", "数值"])
            writer.writerow(["定位误差(m)", positioning_error])
            writer.writerow(["碰撞次数", self.collision_count])
            writer.writerow(["避障成功率", obstacle_avoidance_success_rate])
            writer.writerow(["路径规划效率", path_efficiency])

        # 绘制轨迹图
        plt.figure(figsize=(8, 6))
        x = [p[0] for p in self.position_history]
        y = [p[1] for p in self.position_history]
        plt.plot(x, y, label="实际轨迹")
        plt.scatter(self.ground_truth[0], self.ground_truth[1], color="red", label="目标点")
        plt.xlabel("X(m)")
        plt.ylabel("Y(m)")
        plt.title("机器人运动轨迹")
        plt.legend()
        plt.savefig("trajectory.png")

        self.get_logger().info("性能评估报告已生成：performance_report.csv + trajectory.png")
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = PerformanceEvaluationNode()
    rclpy.spin(node)

if __name__ == "__main__":
    main()
