#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <chrono>
#include <cmath>
#include <algorithm>

class PIDVelocityController : public rclcpp::Node
{
public:
    PIDVelocityController() : Node("pid_velocity_controller")
    {
        // ========== 参数声明与初始化 ==========
        this->declare_parameter<double>("kp", 0.5);
        this->declare_parameter<double>("ki", 0.1);
        this->declare_parameter<double>("kd", 0.05);
        this->declare_parameter<double>("target_vel", 0.2);
        this->declare_parameter<double>("odom_timeout", 0.5);
        this->declare_parameter<double>("integral_threshold", 0.1);
        this->declare_parameter<double>("integral_limit", 1.0);
        this->declare_parameter<double>("output_limit", 0.5);

        // 读取参数
        this->get_parameter("kp", kp_);
        this->get_parameter("ki", ki_);
        this->get_parameter("kd", kd_);
        this->get_parameter("target_vel", target_vel_);
        this->get_parameter("odom_timeout", odom_timeout_);
        this->get_parameter("integral_threshold", integral_threshold_);
        this->get_parameter("integral_limit", integral_limit_);
        this->get_parameter("output_limit", output_limit_);

        // ========== ROS2 通信初始化 ==========
        // 发布速度控制指令
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        // 订阅里程计数据
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&PIDVelocityController::odom_callback, this, std::placeholders::_1)
        );

        // ========== 定时器与回调 ==========
        // 10ms 定时器执行 PID 控制
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10), std::bind(&PIDVelocityController::pid_control_loop, this)
        );
        // 动态参数回调
        param_callback_handle_ = this->add_on_set_parameters_callback(
            std::bind(&PIDVelocityController::param_change_cb, this, std::placeholders::_1)
        );

        // ========== 变量初始化 ==========
        last_odom_time_ = std::chrono::steady_clock::now();
        current_vel_ = 0.0;
        integral_ = 0.0;
        last_error_ = 0.0;

        RCLCPP_INFO(this->get_logger(), "PID Velocity Controller Initialized!");
        RCLCPP_INFO(this->get_logger(), "KP: %.2f, KI: %.2f, KD: %.2f, Target Vel: %.2f m/s", kp_, ki_, kd_, target_vel_);
    }

private:
    // ========== 动态参数回调函数 ==========
    rcl_interfaces::msg::SetParametersResult param_change_cb(const std::vector<rclcpp::Parameter> &params)
    {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        for (const auto &param : params)
        {
            if (param.get_name() == "kp") kp_ = param.as_double();
            else if (param.get_name() == "ki") ki_ = param.as_double();
            else if (param.get_name() == "kd") kd_ = param.as_double();
            else if (param.get_name() == "target_vel") target_vel_ = param.as_double();
            RCLCPP_INFO(this->get_logger(), "Parameter [%s] updated to %.3f", param.get_name().c_str(), param.as_double());
        }
        return result;
    }

    // ========== 里程计回调函数 ==========
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        current_vel_ = msg->twist.twist.linear.x;
        last_odom_time_ = std::chrono::steady_clock::now();
        RCLCPP_DEBUG(this->get_logger(), "Current Velocity: %.3f m/s", current_vel_);
    }

    // ========== PID 控制主循环 ==========
    void pid_control_loop()
    {
        if (!rclcpp::ok()) return;

        // 1. 看门狗：里程计超时检测
        auto now = std::chrono::steady_clock::now();
        double dt = std::chrono::duration<double>(now - last_odom_time_).count();
        if (dt > odom_timeout_)
        {
            geometry_msgs::msg::Twist cmd_vel;
            cmd_vel.linear.x = 0.0;
            cmd_vel.angular.z = 0.0;
            cmd_vel_pub_->publish(cmd_vel);
            RCLCPP_WARN(this->get_logger(), "Odom Timeout (%.2fs) - Robot Stopped!", dt);
            integral_ = 0.0; // 重置积分项
            last_error_ = 0.0;
            return;
        }

        // 2. PID 核心计算
        double error = target_vel_ - current_vel_;
        double dt_control = 0.01; // 控制周期 10ms

        // 积分分离：误差小于阈值时关闭积分，防止超调
        if (fabs(error) > integral_threshold_)
        {
            integral_ += error * dt_control;
            // 积分限幅：防止积分饱和
            integral_ = std::clamp(integral_, -integral_limit_, integral_limit_);
        }
        else
        {
            integral_ = 0.0;
        }

        // 微分计算：消除高频噪声
        double derivative = (error - last_error_) / dt_control;
        last_error_ = error;

        // PID 输出计算
        double output = kp_ * error + ki_ * integral_ + kd_ * derivative;
        // 输出限幅：保护电机
        output = std::clamp(output, -output_limit_, output_limit_);

        // 3. 发布控制指令
        geometry_msgs::msg::Twist cmd_vel;
        cmd_vel.linear.x = output;
        cmd_vel.angular.z = 0.0;
        cmd_vel_pub_->publish(cmd_vel);

        // 4. 调试日志
        RCLCPP_DEBUG(this->get_logger(), "Error: %.3f | Integral: %.3f | Derivative: %.3f | Output: %.3f",
                     error, integral_, derivative, output);
    }

    // ========== 成员变量 ==========
    // PID 参数
    double kp_, ki_, kd_;
    double target_vel_, current_vel_;
    double integral_, last_error_;
    double odom_timeout_, integral_threshold_, integral_limit_, output_limit_;

    // ROS2 句柄
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

    // 时间戳
    std::chrono::time_point<std::chrono::steady_clock> last_odom_time_;
};

// ========== 主函数 ==========
int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PIDVelocityController>());
    rclcpp::shutdown();
    return 0;
}
