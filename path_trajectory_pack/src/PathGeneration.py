#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import xml.etree.cElementTree


def getxmlroot(filename):
    tree = xml.etree.cElementTree.parse(filename)
    return tree.getroot()


rospy.init_node("path_generation")
sub_instruction = rospy.Subscriber("/path/instruction", String, queue_size=5)
pub_new_path = rospy.Publisher("/path/new_path", String, queue_size=2)
pub_logger = rospy.Publisher("/path/logger", String, queue_size=5)

rate = rospy.Rate(3)

while not rospy.is_shutdown():
    # if new instruction:
        # if instruction's first word is "path_generation":
            # publish to logger: "instruction receieved: <instruction>"
            # if instruction = path_generation read_file file.xml


    rate.sleep()
