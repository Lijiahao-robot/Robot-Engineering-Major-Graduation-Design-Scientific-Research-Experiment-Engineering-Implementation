#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import asyncio
import websockets
import json

class AppControlNode(Node):
    def __init__(self):
        super().__init__("app_control_node")
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.ws_port = 8766
        self.loop = asyncio.get_event_loop()
        self.loop.create_task(self.start_ws_server())
        self.get_logger().info(f"手机 APP 控制节点已启动，端口：{self.ws_port}")

    async def ws_handler(self, websocket):
        async for message in websocket:
            cmd = json.loads(message)
            twist = Twist()
            twist.linear.x = cmd.get("linear_x", 0.0)
            twist.angular.z = cmd.get("angular_z", 0.0)
            self.cmd_vel_pub.publish(twist)
            # 回传机器人状态
            await websocket.send(json.dumps({"status": "success"}))

    async def start_ws_server(self):
        async with websockets.serve(self.ws_handler, "0.0.0.0", self.ws_port):
            await asyncio.Future()

def main(args=None):
    rclpy.init(args=args)
    node = AppControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
