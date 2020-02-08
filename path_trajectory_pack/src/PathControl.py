#!/usr/bin/env python

import rospy
import roslaunch
import xml.etree.cElementTree
from std_msgs.msg import String
from geometry_msgs.msg import Vector3


# TODO define path reading method
def readPath(filename):
    tree = xml.etree.cElementTree.parse(filename)
    root = tree.getroot()

    return [(0, 0)]


# TODO define velocity transformation method
def velocityPlaneToEndEffectorTransform(plane_velocity):
    return plane_velocity


# TODO define velocity calculation method
def calculateVelocity():
    vector = Vector3()
    vector.x = 0
    vector.y = 0
    vector.z = 0
    return vector

rospy.init_node("path_controller")
sub_instruction = rospy.Subscriber("/path/instruction", String, queue_size=5)
pub_desired_velocity = rospy.Publisher("/path/desired_velocity", Vector3, queue_size=5)
pub_logger = rospy.Publisher("/path/logger", String, queue_size=5)
rate = rospy.Rate(10)
calculate_velocity = False
path_points = []

# if an xml file is specified, use that to create the first path upon start-up
if len(roslaunch.sys.argv) > 1:
    path_points = readPath(roslaunch.sys.argv[1])

while not rospy.is_shutdown(): # TODO read instruction string
    # if new instruction:
        # publish to logger: "instruction receieved: <instruction>"

    instruction = "test"
    instruction.split(" ")
    if instruction[0] is "read":
        path_points = readPath(instruction[1])
    elif instruction[0] is "start":
        calculate_velocity = True
    elif instruction[0] is "stop":
        calculate_velocity = False

    if calculate_velocity:
        plane_velocity = calculateVelocity()
        end_effector_velocity = velocityPlaneToEndEffectorTransform(plane_velocity)
        pub_desired_velocity.publish(end_effector_velocity)

    rate.sleep()

pub_logger.publish("path_controller shutting down")
