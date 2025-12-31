import gymnasium as gym
from gymnasium import spaces
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class RobotRLAvoidEnv(gym.Env):
    metadata = {"render_modes": ["human"], "render_fps": 10}

    def __init__(self):
        super().__init__()
        # 动作空间：线速度[0,0.3]，角速度[-1,1]
        self.action_space = spaces.Box(low=np.array([0.0, -1.0]), high=np.array([0.3, 1.0]), dtype=np.float32)
        # 观测空间：激光雷达 360 个距离点
        self.observation_space = spaces.Box(low=0.0, high=5.0, shape=(360,), dtype=np.float32)

        # ROS2 初始化
        rclpy.init(args=None)
        self.node = Node("rl_avoid_env")
        self.cmd_vel_pub = self.node.create_publisher(Twist, "/cmd_vel", 10)
        self.laser_sub = self.node.create_subscription(LaserScan, "/scan", self.laser_callback, 10)

        self.laser_data = None
        self.episode_step = 0
        self.max_steps = 500

    def laser_callback(self, msg):
        self.laser_data = np.array(msg.ranges)[:360]
        self.laser_data = np.nan_to_num(self.laser_data, nan=5.0, posinf=5.0)

    def step(self, action):
        self.episode_step += 1
        # 发布动作指令
        cmd_vel = Twist()
        cmd_vel.linear.x = action[0]
        cmd_vel.angular.z = action[1]
        self.cmd_vel_pub.publish(cmd_vel)

        # 获取观测
        rclpy.spin_once(self.node, timeout_sec=0.01)
        obs = self.laser_data if self.laser_data is not None else np.ones(360)*5.0

        # 计算奖励
        min_dist = np.min(obs)
        reward = 0.0
        if min_dist < 0.3:
            reward = -100.0  # 碰撞惩罚
            terminated = True
        elif min_dist > 1.0:
            reward = 10.0   # 远离障碍物奖励
            terminated = False
        else:
            reward = 1.0    # 安全距离奖励
            terminated = False

        # 步数超限终止
        truncated = True if self.episode_step >= self.max_steps else False

        return obs, reward, terminated, truncated, {}

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        self.episode_step = 0
        # 停止机器人
        cmd_vel = Twist()
        self.cmd_vel_pub.publish(cmd_vel)
        # 获取初始观测
        rclpy.spin_once(self.node, timeout_sec=0.1)
        obs = self.laser_data if self.laser_data is not None else np.ones(360)*5.0
        return obs, {}

    def close(self):
        self.node.destroy_node()
        rclpy.shutdown()
