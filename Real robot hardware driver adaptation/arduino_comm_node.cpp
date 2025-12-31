#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <serial/serial.h>
#include <string>
#include <sstream>

class ArduinoCommNode : public rclcpp::Node
{
public:
    ArduinoCommNode() : Node("arduino_comm_node")
    {
        // 声明串口参数
        this->declare_parameter<std::string>("serial_port", "/dev/ttyACM0");
        this->declare_parameter<int>("baud_rate", 115200);
        
        std::string port = this->get_parameter("serial_port").as_string();
        int baud = this->get_parameter("baud_rate").as_int();

        // 初始化串口
        try {
            ser_.setPort(port);
            ser_.setBaudrate(baud);
            serial::Timeout to = serial::Timeout::simpleTimeout(1000);
            ser_.setTimeout(to);
            ser_.open();
            RCLCPP_INFO(this->get_logger(), "Serial port %s opened successfully", port.c_str());
        } catch (serial::IOException& e) {
            RCLCPP_ERROR(this->get_logger(), "Unable to open serial port: %s", e.what());
            rclcpp::shutdown();
        }

        // 订阅速度指令
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10, std::bind(&ArduinoCommNode::cmd_vel_callback, this, std::placeholders::_1)
        );

        // 发布里程计数据（可选）
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
    }

    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        // 将速度指令转为串口指令：格式 "v:线速度,w:角速度\n"
        float linear_x = msg->linear.x;
        float angular_z = msg->angular.z;
        std::stringstream ss;
        ss << "v:" << linear_x << ",w:" << angular_z << "\n";
        if (ser_.isOpen()) {
            ser_.write(ss.str());
            RCLCPP_DEBUG(this->get_logger(), "Sent: %s", ss.str().c_str());
        }

        // 读取 Arduino 反馈（可选）
        if (ser_.available()) {
            std::string response = ser_.readline();
            RCLCPP_DEBUG(this->get_logger(), "Received: %s", response.c_str());
            // 解析反馈数据并发布里程计（需根据 Arduino 代码格式修改）
        }
    }

private:
    serial::Serial ser_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ArduinoCommNode>());
    rclcpp::shutdown();
    return 0;
}
