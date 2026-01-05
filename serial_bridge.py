import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Int32MultiArray
import serial
import struct
import time

# CRC8 校验算法（适配STM32端，抗干扰性强）
def crc8_calc(data: bytes) -> int:
    crc = 0x00
    poly = 0x31  # 多项式：x^8 + x^5 + x^4 + 1
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x80:
                crc = (crc << 1) ^ poly
            else:
                crc <<= 1
            crc &= 0xFF
    return crc

class SerialBridgeNode(Node):
    def __init__(self):
        super().__init__('serial_bridge_node')
        self.get_logger().info("Serial Bridge Node Started! [Debug Mode Enabled]")
        
        # 串口配置（可通过ROS2参数服务器修改，无需改代码）
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 115200)
        self.port = self.get_parameter('serial_port').value
        self.baud = self.get_parameter('baud_rate').value
        
        # 串口初始化 + 异常重连机制
        self.ser = None
        self.serial_connect()
        
        # ROS2 话题订阅/发布
        self.motor_speed_sub = self.create_subscription(
            Float32MultiArray, '/motor_speed', self.motor_speed_callback, 10)
        self.sensor_data_pub = self.create_publisher(
            Int32MultiArray, '/sensor_data', 10)
        
        # 定时任务：100Hz 读取串口 + 串口状态检测
        self.timer = self.create_timer(0.01, self.serial_task)
        self.timeout_cnt = 0  # 数据超时计数器

    def serial_connect(self):
        """串口连接/重连"""
        try:
            self.ser = serial.Serial(
                port=self.port,
                baudrate=self.baud,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                bytesize=serial.EIGHTBITS,
                timeout=0.005,  # 缩短超时，提升实时性
                writeTimeout=0.005
            )
            self.get_logger().info(f"Connected to {self.port} @ {self.baud} bps")
        except Exception as e:
            self.get_logger().error(f"Serial connect failed: {e}")
            self.ser = None

    def motor_speed_callback(self, msg):
        """ROS2 速度指令 → STM32 串口下发（打包+CRC校验）"""
        if not self.ser or not self.ser.isOpen():
            self.serial_connect()
            return
        if len(msg.data) != 4:
            self.get_logger().warn('Motor speed data length must be 4!')
            return
        
        # 统一帧格式（总长度 18 字节）
        header = struct.pack('<2B', 0xAA, 0x55)
        speed_data = struct.pack('<4f', *msg.data)
        crc = crc8_calc(header + speed_data)
        data_pack = header + speed_data + struct.pack('<B', crc)
        
        # ✅ 新增：调试打印（下发的电机速度）
        self.get_logger().debug(
            f"Send to STM32: LF={msg.data[0]:.1f} | RF={msg.data[1]:.1f} | LB={msg.data[2]:.1f} | RB={msg.data[3]:.1f} | CRC={crc:#02x}"
        )
        
        # 发送数据（避免阻塞，增加发送失败判断）
        try:
            self.ser.write(data_pack)
        except Exception as e:
            self.get_logger().error(f"Send failed: {e}")
            self.ser.close()

    def serial_task(self):
        """串口读取 + 传感器数据发布（定时执行，避免丢包）"""
        if not self.ser or not self.ser.isOpen():
            return
        
        # 读取STM32发送的一帧数据（18字节，与下发帧格式统一）
        if self.ser.in_waiting >= 18:
            raw_data = self.ser.read(18)
            self.timeout_cnt = 0
            
            # 帧头校验
            h1, h2 = struct.unpack('<2B', raw_data[:2])
            if h1 != 0x55 or h2 != 0xAA:
                self.ser.flushInput()
                self.get_logger().debug("Frame header error!")
                return
            
            # CRC8 校验
            recv_crc = raw_data[-1]
            calc_crc = crc8_calc(raw_data[:-1])
            if recv_crc != calc_crc:
                self.get_logger().debug(f"CRC check failed! Recv={recv_crc:#02x} | Calc={calc_crc:#02x}")
                return
            
            # 解析4个编码器数据（int32_t）
            sensor_data = struct.unpack('<4i', raw_data[2:18-1])
            sensor_msg = Int32MultiArray()
            sensor_msg.data = list(sensor_data)
            self.sensor_data_pub.publish(sensor_msg)
            
            # ✅ 新增：调试打印（接收的编码器数据）
            self.get_logger().debug(
                f"Recv from STM32: Enc1={sensor_data[0]} | Enc2={sensor_data[1]} | Enc3={sensor_data[2]} | Enc4={sensor_data[3]} | CRC={recv_crc:#02x}"
            )
        else:
            self.timeout_cnt += 1
            if self.timeout_cnt >= 50:  # 500ms 无数据，提示超时
                self.get_logger().warn("No data from STM32, check serial!")
                self.timeout_cnt = 0
                self.ser.flushInput()

def main(args=None):
    rclpy.init(args=args)
    node = SerialBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    # 关闭资源
    if node.ser and node.ser.isOpen():
        node.ser.close()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
