#!/usr/bin/env python

import rospy
import roslaunch
from std_msgs.msg import String


def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)


# if __name__ == '__init__':
#    rospy.loginfo("test.py __init__ on")
#    rospy.init_node("testnode")
#    rospy.Subscriber("/chatter", String, callback)
#    rospy.spin()

rospy.init_node("testnode")
rospy.Subscriber("/chatter", String, callback)

while not rospy.is_shutdown():
    rospy.loginfo(roslaunch.sys.argv[1])
    rospy.sleep(1)
