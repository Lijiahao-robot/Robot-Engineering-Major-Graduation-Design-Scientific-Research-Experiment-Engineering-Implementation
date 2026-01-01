#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist

def gait_control():
    rospy.init_node('humanoid_gait_optimized', anonymous=True)
    pub = rospy.Publisher('/humanoid/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(10) 

    vel_msg = Twist()
    center_offset = 0.1 
    adaptive_stride = True
    while not rospy.is_shutdown():
      
        if adaptive_stride:
            vel_msg.linear.x = 0.2 * 0.8  
        else:
            vel_msg.linear.x = 0.2
        vel_msg.angular.z = 0.5  
        if vel_msg.angular.z != 0:
            vel_msg.linear.x *= (1 - center_offset)  

        pub.publish(vel_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        gait_control()
    except rospy.ROSInterruptException:
        pass
