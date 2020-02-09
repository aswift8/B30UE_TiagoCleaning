#!/usr/bin/env python

import rospy
import roslaunch
import xml.etree.cElementTree
import math
from std_msgs.msg import String
from geometry_msgs.msg import Vector3


# holds the current instruction before being processed
class CurrentInstruction:
    instruction = ""


# TODO define path reading method
def readPath(filename):
    root = xml.etree.cElementTree.parse(filename).getroot()
    paths = []
    for point in root:
        paths.append(tuple(point))
    return paths


def calculateVelocity(start, end, speed):
    vector = Vector3()
    vector.z = 0
    delta_x = end[0] - start[0]
    if delta_x > 0:
        vector.x = 1
    elif delta_x < 0:
        vector.x = -1
    else:
        vector.x = 0
    delta_y = end[1] - start[1]
    if delta_y > 0:
        vector.y = 1
    elif delta_y < 0:
        vector.y = -1
    else:
        vector.y = 0
    # normalises velocity
    if vector.x != 0 and vector.y != 0:
        magnitude = math.sqrt(math.pow(vector.x, 2) + math.pow(vector.y, 2))
        vector.x *= speed / magnitude
        vector.y *= speed / magnitude
    else:
        vector.x *= speed
        vector.y *= speed
    return vector


def getGradient(a, b):
    delta_x = b[0] - a[0]
    if delta_x == 0:
        return "inf"
    else:
        return (b[1] - a[1]) / delta_x


def inverseGradient(m):
    if m == "inf":
        return 0
    elif m == 0:
        return "inf"
    else:
        return -1 / m


# where a and b are points on the line, and c is the point to check
# if line is horizontal, returns true if the point is above line
def pointIsLeft(a, b, c):
    return ((b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])) >= 0


def callback(data):
    CurrentInstruction.instruction = data.data


rospy.init_node("path_controller")
rospy_rate = 10
rospy.Subscriber("/path/instruction", String, callback)
pub_desired_velocity = rospy.Publisher("/path/desired_velocity", Vector3, queue_size=5)
pub_logger = rospy.Publisher("/path/logger", String, queue_size=5)
rate = rospy.Rate(rospy_rate)
calculate_velocity = False
path_points = []

# if an xml file is specified, use that to create the first path upon start-up
if len(roslaunch.sys.argv) > 1:
    path_points = readPath(roslaunch.sys.argv[1])

current_path_section = 0
path_section_count = 0
estimated_position = (0, 0)
while not rospy.is_shutdown():
    instruction = CurrentInstruction.instruction.split(" ")
    if len(instruction) > 0:
        if instruction[0] is "read":
            path_points = readPath(instruction[1])
            path_section_count = len(path_points) - 1
            CurrentInstruction.instruction = ""
            pub_logger.publish("new path imported")
        elif instruction[0] is "start" and path_section_count > 0:
            calculate_velocity = True
            CurrentInstruction.instruction = ""
            pub_logger.publish("velocity calculation started")
        elif instruction[0] is "stop":
            calculate_velocity = False
            CurrentInstruction.instruction = ""
            pub_logger.publish("velocity calculation ended")
        elif instruction[0] is "reset":
            calculate_velocity = False
            current_path_section = 0
            path_section_count = 0
            estimated_position = (0, 0)
            CurrentInstruction.instruction = ""
            pub_logger.publish("path_controller reset")

    if calculate_velocity:
        # identify current path section
        # stops moving end-effector if the end of the path has been reached
        plane_velocity = Vector3()
        section_identified = False
        while not section_identified:
            if current_path_section >= path_section_count:
                plane_velocity.x = 0
                plane_velocity.y = 0
                plane_velocity.z = 0
                section_identified = True
            else:
                start = path_points[current_path_section]
                end = path_points[current_path_section + 1]
                crossover_line_gradient = inverseGradient(getGradient(start, end))
                end2_x = end[1] + 1
                if crossover_line_gradient == "inf":
                    end2 = (end[0], end2_x)
                else:
                    c = end[1] / (crossover_line_gradient * end[0])
                    end2 = ((crossover_line_gradient*end2_x) + c, end2_x)
                start_crossover_bool = pointIsLeft(end, end2, start)
                current_crossover_bool = pointIsLeft(end, end2, estimated_position)
                if start_crossover_bool == current_path_section:
                    # we are on this current line
                    plane_velocity = calculateVelocity(start, end, 0.1)
                    section_identified = True
                else:
                    # we have moved to the next section
                    current_path_section += 1
        # publish this velocity
        pub_desired_velocity.publish(plane_velocity)
        # update record of location TODO have a better localisation method
        estimated_position[0] += plane_velocity.x / rospy_rate
        estimated_position[1] += plane_velocity.y / rospy_rate
    rate.sleep()

pub_logger.publish("path_controller shutting down")
