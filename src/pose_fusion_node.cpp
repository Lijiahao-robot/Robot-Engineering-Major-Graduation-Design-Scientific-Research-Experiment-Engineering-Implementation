#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include <Eigen/Dense>
#include <chrono>
#include <memory>

using namespace Eigen;
using namespace std::chrono_literals;

class PoseFusionNode : public rclcpp::Node
{
public:
    PoseFusionNode() : Node("pose_fusion_node")
    {
        // ========== 订阅器初始化 ==========
        // 订阅 Cartographer SLAM 位姿
        slam_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odometry/filtered", 10, std::bind(&PoseFusionNode::slam_odom_cb, this, std::placeholders::_1)
        );
        // 订阅 VLM 定位位姿
        vlm_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/vlm_odom", 10, std::bind(&PoseFusionNode::vlm_odom_cb, this, std::placeholders::_1)
        );

        // ========== 发布器初始化 ==========
        // 发布融合后的位姿
        fused_odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/fused_odom", 10);

        // ========== 卡尔曼滤波初始化 ==========
        x_ = Vector3d::Zero(); // 状态向量 [x, y, yaw]
        P_ = Matrix3d::Identity() * 0.1; // 协方差矩阵
        F_ = Matrix3d::Identity(); // 状态转移矩阵
        H_ = Matrix3d::Identity(); // 观测矩阵
        Q_ = Matrix3d::Identity() * 0.01; // 过程噪声协方差
        R_ = Matrix3d::Identity() * 0.05; // 观测噪声协方差

        // ========== 定时器 ==========
        // 20ms 执行一次融合
        timer_ = this->create_wall_timer(20ms, std::bind(&PoseFusionNode::fusion_loop, this));

        // ========== 标志位初始化 ==========
        has_slam_ = false;
        has_vlm_ = false;

        RCLCPP_INFO(this->get_logger(), "Pose Fusion Node Initialized (Kalman Filter)");
    }

private:
    // ========== 回调函数 ==========
    void slam_odom_cb(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        slam_pose_ = *msg;
        has_slam_ = true;
    }

    void vlm_odom_cb(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        vlm_pose_ = *msg;
        has_vlm_ = true;
    }

    // ========== 卡尔曼融合主循环 ==========
    void fusion_loop()
    {
        if (!has_slam_ || !has_vlm_ || !rclcpp::ok())
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Waiting for SLAM/VLM data...");
            return;
        }

        // 1. 预测步
        x_ = F_ * x_;
        P_ = F_ * P_ * F_.transpose() + Q_;

        // 2. 构造观测值：SLAM 权重 0.7，VLM 权重 0.3
        double x_slam = slam_pose_.pose.pose.position.x;
        double y_slam = slam_pose_.pose.pose.position.y;
        double x_vlm = vlm_pose_.pose.pose.position.x;
        double y_vlm = vlm_pose_.pose.pose.position.y;

        Vector3d z;
        z << 0.7 * x_slam + 0.3 * x_vlm,
             0.7 * y_slam + 0.3 * y_vlm,
             0.0;

        // 3. 更新步
        Matrix3d K = P_ * H_.transpose() * (H_ * P_ * H_.transpose() + R_).inverse();
        x_ = x_ + K * (z - H_ * x_);
        P_ = (Matrix3d::Identity() - K * H_) * P_;

        // 4. 发布融合后的位姿
        nav_msgs::msg::Odometry fused_odom = slam_pose_;
        fused_odom.pose.pose.position.x = x_(0);
        fused_odom.pose.pose.position.y = x_(1);
        fused_odom.header.stamp = this->get_clock()->now();
        fused_odom.header.frame_id = "map";
        fused_odom.child_frame_id = "base_link";
        fused_odom_pub_->publish(fused_odom);

        // 5. 调试日志
        RCLCPP_DEBUG(this->get_logger(), "Fused Pose: X=%.3f, Y=%.3f", x_(0), x_(1));
    }

    // ========== 成员变量 ==========
    // 卡尔曼滤波参数
    Vector3d x_;
    Matrix3d P_, F_, H_, Q_, R_;

    // 位姿数据
    nav_msgs::msg::Odometry slam_pose_, vlm_pose_;
    bool has_slam_, has_vlm_;

    // ROS2 句柄
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr slam_odom_sub_, vlm_odom_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr fused_odom_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

// ========== 主函数 ==========
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PoseFusionNode>());
    rclcpp::shutdown();
    return 0;
}
