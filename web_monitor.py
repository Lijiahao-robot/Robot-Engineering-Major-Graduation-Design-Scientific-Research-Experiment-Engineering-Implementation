#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
import asyncio
import websockets

# 全局变量存储机器人状态
robot_state = {
    "x": 0.0,
    "y": 0.0,
    "fault": "normal"
}

class WebMonitorNode(Node):
    def __init__(self):
        super().__init__("web_monitor_node")
        # 订阅核心话题
        self.create_subscription(Odometry, "/fused_odom", self.odom_callback, 10)
        self.create_subscription(String, "/fault_info", self.fault_callback, 10)
        # WebSocket 服务器配置
        self.ws_port = 8765
        self.loop = asyncio.get_event_loop()
        self.loop.run_until_complete(self.start_ws_server())

    def odom_callback(self, msg):
        robot_state["x"] = msg.pose.pose.position.x
        robot_state["y"] = msg.pose.pose.position.y

    def fault_callback(self, msg):
        robot_state["fault"] = msg.data

    async def ws_handler(self, websocket):
        while True:
            # 每秒发送一次状态
            await websocket.send(str(robot_state))
            await asyncio.sleep(1.0)

    async def start_ws_server(self):
        async with websockets.serve(self.ws_handler, "0.0.0.0", self.ws_port):
            self.get_logger().info(f"WebSocket Server started on port {self.ws_port}")
            await asyncio.Future()

def main(args=None):
    rclpy.init(args=args)
    node = WebMonitorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
