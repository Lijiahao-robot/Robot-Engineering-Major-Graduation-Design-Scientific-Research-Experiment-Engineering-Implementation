#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import vosk
import pyaudio
import json

class VoiceControlNode(Node):
    def __init__(self):
        super().__init__("voice_control_node")
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        
        # 加载 vosk 轻量级模型（需自行下载中文模型）
        self.model = vosk.Model("./model/vosk-model-small-cn-0.22")
        self.recognizer = vosk.KaldiRecognizer(self.model, 16000)
        
        # 初始化音频流
        self.p = pyaudio.PyAudio()
        self.stream = self.p.open(
            format=pyaudio.paInt16,
            channels=1,
            rate=16000,
            input=True,
            frames_per_buffer=8192
        )
        self.stream.start_stream()

        self.get_logger().info("语音控制节点已启动，支持指令：前进/后退/左转/右转/停止")

        # 定时器监听语音
        self.timer = self.create_timer(0.1, self.voice_listen_loop)

    def voice_listen_loop(self):
        data = self.stream.read(4096, exception_on_overflow=False)
        if self.recognizer.AcceptWaveform(data):
            result = json.loads(self.recognizer.Result())
            if "text" in result and result["text"] != "":
                self.parse_voice_cmd(result["text"])

    def parse_voice_cmd(self, cmd):
        twist = Twist()
        if "前进" in cmd:
            twist.linear.x = 0.2
            self.get_logger().info("执行指令：前进")
        elif "后退" in cmd:
            twist.linear.x = -0.2
            self.get_logger().info("执行指令：后退")
        elif "左转" in cmd:
            twist.angular.z = 0.5
            self.get_logger().info("执行指令：左转")
        elif "右转" in cmd:
            twist.angular.z = -0.5
            self.get_logger().info("执行指令：右转")
        elif "停止" in cmd:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.get_logger().info("执行指令：停止")
        self.cmd_vel_pub.publish(twist)

    def destroy_node(self):
        self.stream.stop_stream()
        self.stream.close()
        self.p.terminate()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = VoiceControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
