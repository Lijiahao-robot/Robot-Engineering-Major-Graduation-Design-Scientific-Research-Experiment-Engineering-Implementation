#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "yaml-cpp/yaml.h"
#include <unordered_map>
#include <chrono>
#include <fstream>

struct FaultRule {
    std::string name;
    std::string type; // "topic" or "node"
    std::string target;
    double timeout;
    std::string level;
};

class FaultDiagnosisNode : public rclcpp::Node
{
public:
    FaultDiagnosisNode() : Node("fault_diagnosis_node")
    {
        // 加载故障配置
        load_fault_config("src/fault_diagnosis/src/fault_config.yaml");

        // 发布故障信息
        fault_pub_ = this->create_publisher<std_msgs::msg::String>("/fault_info", 10);

        // 定时器 500ms 检测一次
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500), std::bind(&FaultDiagnosisNode::diagnose_loop, this)
        );

        // 记录话题最后更新时间
        for (auto &rule : fault_rules_) {
            if (rule.type == "topic") {
                topic_last_time_[rule.target] = this->get_clock()->now();
                // 订阅话题以更新时间戳
                create_topic_subscription(rule.target);
            }
        }

        RCLCPP_INFO(this->get_logger(), "Fault Diagnosis Node Initialized!");
    }

private:
    void load_fault_config(const std::string &config_path)
    {
        YAML::Node config = YAML::LoadFile(config_path);
        for (auto &rule_node : config["fault_rules"]) {
            FaultRule rule;
            rule.name = rule_node["name"].as<std::string>();
            rule.target = rule_node["topic"].as<std::string>();
            rule.timeout = rule_node["timeout"].as<double>();
            rule.level = rule_node["level"].as<std::string>();
            rule.type = "topic";
            if (rule_node["node"]) {
                rule.type = "node";
                rule.target = rule_node["node"].as<std::string>();
            }
            fault_rules_.push_back(rule);
        }
    }

    void create_topic_subscription(const std::string &topic_name)
    {
        auto callback = [this, topic_name](const rclcpp::SerializedMessage::ConstSharedPtr) {
            topic_last_time_[topic_name] = this->get_clock()->now();
        };
        auto sub = this->create_subscription<rclcpp::SerializedMessage>(
            topic_name, 10, callback
        );
        topic_subs_.push_back(sub);
    }

    void diagnose_loop()
    {
        std_msgs::msg::String fault_msg;
        for (auto &rule : fault_rules_) {
            if (rule.type == "topic") {
                auto now = this->get_clock()->now();
                double dt = (now - topic_last_time_[rule.target]).seconds();
                if (dt > rule.timeout) {
                    fault_msg.data = "[" + rule.level + "] " + rule.name + ": " + rule.target + " timeout (" + std::to_string(dt) + "s)";
                    fault_pub_->publish(fault_msg);
                    RCLCPP_WARN(this->get_logger(), "%s", fault_msg.data.c_str());
                }
            } else if (rule.type == "node") {
                auto node_list = rclcpp::Node::get_node_names();
                bool node_exists = false;
                for (auto &node_name : node_list) {
                    if (node_name.find(rule.target) != std::string::npos) {
                        node_exists = true;
                        break;
                    }
                }
                if (!node_exists) {
                    fault_msg.data = "[" + rule.level + "] " + rule.name + ": " + rule.target + " is down";
                    fault_pub_->publish(fault_msg);
                    RCLCPP_ERROR(this->get_logger(), "%s", fault_msg.data.c_str());
                }
            }
        }
    }

    std::vector<FaultRule> fault_rules_;
    std::unordered_map<std::string, rclcpp::Time> topic_last_time_;
    std::vector<rclcpp::Subscription<rclcpp::SerializedMessage>::SharedPtr> topic_subs_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr fault_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FaultDiagnosisNode>());
    rclcpp::shutdown();
    return 0;
}
