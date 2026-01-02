#!/usr/bin/env python3
# 月球机器人 ROS 导航控制代码（发布速度指令）
# 运行环境：Ubuntu 20.04 + ROS Noetic
# 依赖安装：sudo apt-get install ros-noetic-geometry-msgs

import rospy
from geometry_msgs.msg import Twist

class LunarRoverROS:
    def __init__(self):
        rospy.init_node('lunar_rover_nav', anonymous=True)  # 初始化ROS节点
        self.vel_pub = rospy.Publisher('/lunar_rover/cmd_vel', Twist, queue_size=10)
        self.rate = rospy.Rate(10)  # 10Hz 发布频率
        self.twist = Twist()

    # 正向移动
    def move_forward(self, linear_speed=0.2):
        self.twist.linear.x = linear_speed
        self.twist.angular.z = 0.0
        print("ROS月球车：正向移动")

    # 原地左转
    def turn_left(self, angular_speed=0.1):
        self.twist.linear.x = 0.0
        self.twist.angular.z = angular_speed
        print("ROS月球车：原地左转")

    # 停止移动
    def stop(self):
        self.twist.linear.x = 0.0
        self.twist.angular.z = 0.0
        print("ROS月球车：停止")

    # 运行控制循环
    def run(self):
        while not rospy.is_shutdown():
            self.move_forward()
            self.vel_pub.publish(self.twist)
            self.rate.sleep()
            rospy.sleep(3)  # 移动3秒

            self.turn_left()
            self.vel_pub.publish(self.twist)
            self.rate.sleep()
            rospy.sleep(2)  # 左转2秒

            self.stop()
            self.vel_pub.publish(self.twist)
            rospy.sleep(1)
            break

if __name__ == '__main__':
    try:
        rover = LunarRoverROS()
        rover.run()
    except rospy.ROSInterruptException:
        pass
