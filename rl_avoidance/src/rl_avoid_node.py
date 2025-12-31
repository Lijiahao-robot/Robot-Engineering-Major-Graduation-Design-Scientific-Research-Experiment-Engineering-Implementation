#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from stable_baselines3 import PPO
import numpy as np

class RLAvoidNode(Node):
    def __init__(self):
        super().__init__("rl_avoid_node")
        self.laser_sub = self.create_subscription(LaserScan, "/scan", self.laser_callback, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.laser_data = None

        # 加载训练好的 PPO 模型
        self.model = PPO.load("./models/ppo_robot_avoid")

        self.timer = self.create_timer(0.02, self.inference_loop)
        self.get_logger().info("强化学习避障节点已启动！")

    def laser_callback(self, msg):
        self.laser_data = np.array(msg.ranges)[:360]
        self.laser_data = np.nan_to_num(self.laser_data, nan=5.0, posinf=5.0)

    def inference_loop(self):
        if self.laser_data is None:
            return
        # 模型推理
        action, _states = self.model.predict(self.laser_data, deterministic=True)
        # 发布控制指令
        cmd_vel = Twist()
        cmd_vel.linear.x = action[0]
        cmd_vel.angular.z = action[1]
        self.cmd_vel_pub.publish(cmd_vel)

def main(args=None):
    rclpy.init(args=args)
    node = RLAvoidNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
