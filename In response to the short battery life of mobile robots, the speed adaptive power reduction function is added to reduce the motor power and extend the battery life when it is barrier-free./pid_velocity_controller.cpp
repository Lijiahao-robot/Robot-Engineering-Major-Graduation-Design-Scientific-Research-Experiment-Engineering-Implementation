// 新增成员变量
double energy_save_threshold_ = 1.5;  // 无障碍物距离阈值
double energy_save_vel_ = 0.1;        // 节能模式线速度
bool energy_save_mode_ = false;
// 订阅激光雷达数据（用于判断是否有障碍物）
rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;

// 构造函数中初始化
laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "/scan", 10, std::bind(&PIDVelocityController::laser_callback, this, std::placeholders::_1)
);
this->declare_parameter<double>("energy_save_threshold", 1.5);
this->declare_parameter<double>("energy_save_vel", 0.1);
this->get_parameter("energy_save_threshold", energy_save_threshold_);
this->get_parameter("energy_save_vel", energy_save_vel_);

// 新增激光雷达回调函数
void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    std::vector<float> ranges = msg->ranges;
    float min_dist = *min_element(ranges.begin(), ranges.end());
    // 无障碍物时进入节能模式
    energy_save_mode_ = (min_dist > energy_save_threshold_);
}

// 修改 PID 控制循环，添加节能逻辑
void pid_control_loop()
{
    // ... 原有看门狗逻辑 ...

    // 节能模式：动态调整目标速度
    double target_vel = energy_save_mode_ ? energy_save_vel_ : target_vel_;
    double error = target_vel - current_vel_;

    // ... 原有 PID 计算逻辑 ...
}
