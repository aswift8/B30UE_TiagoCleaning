#!/usr/bin/env python

import rospy
import roslaunch
import xml.etree.cElementTree
from geometry_msgs.msg import Twist


def increment(_index, _msgs):
    _index += 1
    if _index >= len(_msgs):
        _index = 0
    return _index


rospy.init_node("xml_reader")
rospy.loginfo("file: " + roslaunch.sys.argv[1])
tree = xml.etree.cElementTree.parse(roslaunch.sys.argv[1])
info = tree.getroot()
delay = float(info.attrib["delay"])

position_list = []
for position in info:
    msg = Twist()
    linear = position.find("linear")
    msg.linear.x = float(linear.find("x").text)
    msg.linear.y = float(linear.find("y").text)
    msg.linear.z = float(linear.find("z").text)
    angular = position.find("angular")
    msg.angular.x = float(angular.find("x").text)
    msg.angular.y = float(angular.find("y").text)
    msg.angular.z = float(angular.find("z").text)
    position_list.append(msg)

rospy.loginfo("delay: " + str(delay))
for msg in position_list:
    rospy.loginfo("message: " + str(msg))

rate = rospy.Rate(3)

pub = rospy.Publisher("/mobile_base_controller/cmd_vel", Twist, queue_size=10)
index = 0
counter = 0
while not rospy.is_shutdown():
    rospy.loginfo("loop " + str(index) + ": " + str(position_list[index]))
    pub.publish(position_list[index])
    rate.sleep()
    counter += 1
    if counter > delay:
        index = increment(index, position_list)
        counter = 0
