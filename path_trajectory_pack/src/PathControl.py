#!/usr/bin/env python

import rospy
import roslaunch
import xml.etree.cElementTree
import math
from std_msgs.msg import String, Float32MultiArray
from geometry_msgs.msg import Vector3, Point


class Memory:
    instruction = ""
    correction_threshold = 0.01
    z_abs_boundary_min = 0
    z_abs_boundary_max = 2
    threshold = 0.1
    current_point_3d = Point(0, 0, 0)
    current_rotation_3d = Float32MultiArray()
    current_point_2d = (0, 0)


def distance_between_points(point_a, point_b):
    delta_x_sq = math.pow(point_a.x - point_b.x, 2)
    delta_y_sq = math.pow(point_a.y - point_b.y, 2)
    delta_z_sq = math.pow(point_a.z - point_b.z, 2)
    return math.sqrt(delta_x_sq + delta_y_sq + delta_z_sq)


def update_position(data):
    current_position = data
    if distance_between_points(Memory.current_point_3d, current_position) > Memory.threshold:
        Memory.current_point_3d = current_position
        Memory.current_point_2d = transform_z(Memory.current_point_3d, False)


def update_rotation(data):
    Memory.current_rotation_3d = data


def transform_z(vector, mode):
    if mode:
        vector_3d = Vector3()
        vector_3d.x = vector[0]
        vector_3d.y = vector[1]
        vector_3d.z = 0
        return vector_3d
    else:
        return vector.x, vector.y


def read_path(filename):
    root = xml.etree.cElementTree.parse(filename).getroot()
    paths = []
    for point in root:
        x = float(point.find("x").text)
        y = float(point.find("y").text)
        paths.append((x, y))
    return paths


# TODO include error correction and rotational correction math
def calculate_velocity(start_point, end_point, speed):
    vector = Vector3()
    vector.x = end_point[0] - start_point[0]
    vector.y = end_point[1] - start_point[1]

    # error correction calculation
    correction_magnitude = point_left_by(start_point, end_point, Memory.current_point_2d)
    if abs(correction_magnitude) > Memory.correction_threshold:
        correction_m = inverse_gradient(get_gradient(start_point, end_point))
        if correction_m == "inf":
            vector.y += correction_magnitude
        else:
            theta = math.atan(correction_m)
            vector.x += correction_magnitude * math.cos(theta)
            vector.y += correction_magnitude * math.sin(theta)

    # z rotation correction
    vector.z = -math.atan2(Memory.current_rotation_3d.data[1], Memory.current_rotation_3d.data[0])
    if vector.z != 0:
        new_x = vector.x * math.cos(vector.z) + vector.y * math.sin(vector.z)
        new_y = vector.y * math.cos(vector.z) + vector.x * math.sin(vector.z)
        vector.x = new_x
        vector.y = new_y

    # normalises velocity
    if vector.x != 0 or vector.y != 0:
        magnitude = math.sqrt(math.pow(vector.x, 2) + math.pow(vector.y, 2))
        vector.x *= speed / magnitude
        vector.y *= speed / magnitude
    return vector


def get_gradient(a, b):
    delta_x = b[0] - a[0]
    if delta_x == 0:
        return "inf"
    else:
        return (b[1] - a[1]) / delta_x


def inverse_gradient(m):
    if m == "inf":
        return 0
    elif m == 0:
        return "inf"
    else:
        return -1 / m


# where a and b are points on the line, and c is the point to check
# if line is horizontal, returns true if the point is above line
def point_is_left(a, b, c):
    return ((b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])) >= 0


def point_left_by(a, b, c):
    return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])


def new_instruction(data):
    Memory.instruction = data.data


rospy.init_node("path_controller")
rospy_rate = 10
rospy.Subscriber("/path/instruction", String, new_instruction)
rospy.Subscriber("/path/current_position", Vector3, update_position)
rospy.Subscriber("/path/current_rotation", Float32MultiArray, update_rotation)
pub_desired_velocity = rospy.Publisher("/path/desired_velocity", Vector3, queue_size=5)
pub_logger = rospy.Publisher("/path/logger", String, queue_size=5)
rate = rospy.Rate(rospy_rate)
move_end_effector = False
path_points = []
Memory.current_rotation_3d.data = [0, 0, 0, 0, 0, 0, 0, 0, 0]
current_path_section = 0
path_section_count = 0

pub_logger.publish(String("path_controller started-up"))

# if an xml file is specified, use that to create the first path upon start-up
if len(roslaunch.sys.argv) > 1:
    path_points = read_path(roslaunch.sys.argv[1])
    path_section_count = len(path_points) - 1
    pub_logger.publish("new path imported: " + str(path_points))

while not rospy.is_shutdown():
    instruction = Memory.instruction.split(" ")
    if len(instruction[0]) > 0: # if an instruction exists
        pub_logger.publish("new instruction: " + str(instruction[0]))
        if instruction[0] == "read":
            if len(instruction) < 2: # read command must have a parameter
                pub_logger.publish("read command requires a link to read from")
                break
            else:
                path_points = read_path(instruction[1])
                path_section_count = len(path_points) - 1
                Memory.instruction = ""
                move_end_effector = False
                pub_logger.publish("new path imported: " + str(path_points))
        elif instruction[0] == "start" and path_section_count > 0:
            move_end_effector = True
            Memory.instruction = ""
            pub_logger.publish("velocity calculation started")
        elif instruction[0] == "stop":
            move_end_effector = False
            Memory.instruction = ""
            pub_logger.publish("velocity calculation ended")
        elif instruction[0] == "reset":
            move_end_effector = False
            current_path_section = 0
            path_section_count = 0
            Memory.instruction = ""
            pub_logger.publish("path_controller reset")

    if move_end_effector:
        # identify current path section
        # stops moving end-effector if the end of the path has been reached
        plane_velocity = Vector3()
        section_identified = False
        while not section_identified:
            if current_path_section >= path_section_count: # if the end-effector has reached the end of its path
                plane_velocity.x = 0
                plane_velocity.y = 0
                plane_velocity.z = 0
                section_identified = True
                rospy.loginfo("path end reached")
            else:
                start = path_points[current_path_section]
                end = path_points[current_path_section + 1]
                crossover_line_gradient = inverse_gradient(get_gradient(start, end))
                rospy.loginfo(str(crossover_line_gradient))
                end2_x = end[1] + 1
                if crossover_line_gradient == "inf":
                    end2 = (end[0], end2_x)
                elif crossover_line_gradient == 0:
                    end2 = (end[0] + 1, end[1])
                else:
                    c = end[1] / (crossover_line_gradient * end[0])
                    end2 = ((crossover_line_gradient * end2_x) + c, end2_x)

                start_crossover_bool = point_is_left(end, end2, start)
                current_crossover_bool = point_is_left(end, end2, Memory.current_point_2d)
                rospy.loginfo(str(start_crossover_bool) + " " + str(current_crossover_bool))
                if start_crossover_bool == current_path_section:
                    # we are on this current line
                    plane_velocity = calculate_velocity(start, end, 0.5)
                    section_identified = True
                else:
                    # we have moved to the next section
                    current_path_section += 1
        # publish this velocity
        pub_desired_velocity.publish(plane_velocity)
    rate.sleep()

pub_logger.publish("path_controller shutting down")
