#!/usr/bin/env python

import rospy
import roslaunch
from geometry_msgs.msg import Twist

rospy.init_node("spin")
pub = rospy.Publisher("/mobile_base_controller/cmd_vel", Twist, queue_size=5)
spin_msg = Twist()
spin_msg.linear.x = 0
spin_msg.linear.y = 0
spin_msg.linear.z = 0
spin_msg.angular.x = 0
spin_msg.angular.y = 0
spin_msg.angular.z = float(roslaunch.sys.argv[1])
rate = rospy.Rate(3)

while not rospy.is_shutdown():
    pub.publish(spin_msg)
    rate.sleep()
