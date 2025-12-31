#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import yaml
import os

class SceneSwitchNode(Node):
    def __init__(self):
        super().__init__("scene_switch_node")
        # 加载场景配置
        config_path = os.path.join(os.path.dirname(__file__), "../config/scene_params.yaml")
        with open(config_path, "r") as f:
            self.scene_params = yaml.safe_load(f)["scenes"]
        
        # 订阅激光数据
        self.laser_sub = self.create_subscription(LaserScan, "/scan", self.laser_callback, 10)
        # 定时器 1s 检测一次场景
        self.timer = self.create_timer(1.0, self.scene_detect)
        # 当前场景
        self.current_scene = "empty_room"
        self.get_logger().info("场景自适应切换节点已启动，当前场景：empty_room")

    def laser_callback(self, msg):
        self.laser_ranges = np.array(msg.ranges)

    def scene_detect(self):
        if not hasattr(self, "laser_ranges"):
            return
        # 计算激光数据的方差（方差小=走廊，方差大=空旷，中等=仓储）
        ranges = self.laser_ranges[~np.isnan(self.laser_ranges)]
        ranges = ranges[ranges < 5.0]  # 只考虑5m内的数据
        if len(ranges) == 0:
            return
        range_var = np.var(ranges)

        # 场景判断逻辑
        if range_var < 0.5:
            new_scene = "corridor"
        elif range_var > 2.0:
            new_scene = "empty_room"
        else:
            new_scene = "warehouse"
        
        # 场景切换时更新参数
        if new_scene != self.current_scene:
            self.current_scene = new_scene
            self.update_params()
            self.get_logger().info(f"场景切换至：{new_scene}")

    def update_params(self):
        # 获取当前场景参数
        params = self.scene_params[self.current_scene]
        # 更新 PID 参数
        self.set_parameter(rclpy.Parameter("pid_velocity_node.kp", rclpy.Parameter.Type.DOUBLE, params["pid_kp"]))
        self.set_parameter(rclpy.Parameter("pid_velocity_node.ki", rclpy.Parameter.Type.DOUBLE, params["pid_ki"]))
        self.set_parameter(rclpy.Parameter("pid_velocity_node.kd", rclpy.Parameter.Type.DOUBLE, params["pid_kd"]))
        # 更新避障参数
        self.set_parameter(rclpy.Parameter("sensor_fusion_avoid_node.safe_radius", rclpy.Parameter.Type.DOUBLE, params["safe_radius"]))
        # 更新节能参数
        self.set_parameter(rclpy.Parameter("pid_velocity_node.energy_save_vel", rclpy.Parameter.Type.DOUBLE, params["energy_save_vel"]))

def main(args=None):
    rclpy.init(args=args)
    node = SceneSwitchNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
