#!/usr/bin/env python

import rospy
import roslaunch
import xml.etree.cElementTree
import math
from geometry_msgs.msg import Vector3
import numpy as np

"""
Origin Axes:
x axis = forward
y axis = left
z axis = up

End-Effector Axes:
x axis = line along end-effector
y axis = line between pincers
z axis = line intersecting pincers

Note: force sensor axis are different from end-effector
"""

# static variables that can be accessed from any method in the script
class Memory:
    # operation parameters
    run_rate = 10
    speed = 1
    correction_threshold = 0.01
    velocity_threshold = 0.01
    rel_origin_boundaries = np.array([[-10, 10], [-10, 10], [-10, 10]])
    # static variables
    instruction = ""
    global_point_3d = Vector3(0, 0, 0)
    plane_point_2d = (0, 0)


# more efficent than using math.pow(a,2)
def squared(a):
    return a * a


def update_position(data):
    Memory.global_point_3d = data
    Memory.plane_point_2d = Memory.global_point_3d.x, Memory.global_point_3d.y


def read_path(filename):
    root = xml.etree.cElementTree.parse(filename).getroot()
    paths = []
    for point in root:
        x = float(point.find("x").text)
        y = float(point.find("y").text)
        paths.append((x, y))
    return paths


def calculate_velocity(start_point, end_point,
                       linear_correct_multi, slow_on_approach_distance):
    # translation calculation
    vector = Vector3(x=end_point[0] - start_point[0], y=end_point[1] - start_point[1], z=0)

    # error correction calculation
    if linear_correct_multi > 0:
        correction_magnitude = point_left_by(start_point, end_point, Memory.plane_point_2d)
        if abs(correction_magnitude) > Memory.correction_threshold:
            correction_m = inverse_gradient(get_gradient(start_point, end_point))
            if correction_m == "inf":
                # TODO confirm this lines works as intended
                vector.y -= correction_magnitude) * linear_correct_multi
            elif correction_m == 0:
                vector.x += correction_magnitude * linear_correct_multi
            else:
                theta = math.atan(correction_m)  # neg gradient => neg radian => pos cosine
                delta_x = abs(correction_magnitude * math.cos(theta) * linear_correct_multi)
                delta_y = abs(correction_magnitude * math.sin(theta) * linear_correct_multi)
                if correction_m > 0:
                    if correction_magnitude > 0:
                        vector.x += delta_x
                        vector.y += delta_y
                        rospy.loginfo("check: + +")
                    else:
                        vector.x -= delta_x
                        vector.y -= delta_y
                        rospy.loginfo("check: + -")
                else:
                    if correction_magnitude > 0:
                        vector.x += delta_x
                        vector.y -= delta_y
                        rospy.loginfo("check: - +")
                    else:
                        vector.x -= delta_x
                        vector.y += delta_y
                        rospy.loginfo("check: - -")

    return normalise_2d_vector(vector, slow_on_approach_distance, Memory.speed)


#
def normalise_2d_vector(vector, slow_on_approach_distance, multi):
    magnitude = math.sqrt(squared(vector.x) + squared(vector.y))
    if magnitude < Memory.velocity_threshold:
        vector.x = 0
        vector.y = 0
    else:
        vector.x *= multi
        vector.y *= multi
        if magnitude > slow_on_approach_distance:
            vector.x /= magnitude
            vector.y /= magnitude
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
    return point_left_by(a, b, c) >= 0


def point_left_by(a, b, c):
    if a[1] > b[1]:
        a, b = b, a
    return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])


def new_instruction(data):
    Memory.instruction = data.data


def calculate_border_points(path_coordinates):
    i = 0
    end2_coordinates = []
    while i < len(path_coordinates) - 1:
        start = path_coordinates[i]
        end = path_coordinates[i + 1]
        crossover_line_gradient = inverse_gradient(get_gradient(start, end))
        if crossover_line_gradient == "inf":
            end2 = (end[0], end[1] + 1)
        elif crossover_line_gradient == 0:
            end2 = (end[0] + 1, end[1])
        else:
            end2_y = end[1] + 1
            c = end[1] - (crossover_line_gradient * end[0])
            end2 = ((end2_y - c) / crossover_line_gradient, end2_y)
        end2_coordinates.append(end2)
        i += 1
    return end2_coordinates


def offset_coordinates(points, offset):
    rel_points = []
    i = 0
    while i < len(points):
        point = (points[i][0] + offset[0], points[i][1] + offset[1])
        rel_points.append(point)
        i += 1
    return rel_points


def global_point_3d_outside_boundary():
    if Memory.global_point_3d.x < Memory.rel_origin_boundaries[0][0] \
        or Memory.global_point_3d.x > Memory.rel_origin_boundaries[0][1] \
        or Memory.global_point_3d.y < Memory.rel_origin_boundaries[1][0] \
        or Memory.global_point_3d.y > Memory.rel_origin_boundaries[1][1] \
        or Memory.global_point_3d.z < Memory.rel_origin_boundaries[2][0] \
        or Memory.global_point_3d.z > Memory.rel_origin_boundaries[2][1]:
        return True
    else:
        return False


# variable initialisation
follow_path = False
move_to_point = False
move_to_target = (0, 0)
path_points = []
rel_path_points = []
end2_points = []
current_path_section = 0
path_section_count = 0

# ros networking setup
rospy.init_node("path_controller")
rate = rospy.Rate(Memory.run_rate)
rospy.Subscriber("/path/instruction", String, new_instruction)
rospy.Subscriber("/cleaning/forward_kinematics/position", Vector3, update_position)
pub_desired_velocity = rospy.Publisher("/path/desired_velocity", Vector3, queue_size=5)
pub_logger = rospy.Publisher("/path/logger", String, queue_size=5)

pub_logger.publish(String("path_controller started-up"))

# if an xml file is specified, use that to create the first path upon start-up
if len(roslaunch.sys.argv) > 1:
    Memory.instruction = "read " + roslaunch.sys.argv[1]

# wait for one second
for i in range(0, Memory.run_rate):
    rate.sleep()

while not rospy.is_shutdown():
    # read instruction if a new one is present
    instruction = Memory.instruction.split(" ")
    if len(instruction[0]) > 0:  # if an instruction exists
        pub_logger.publish("new instruction: " + str(Memory.instruction))
        if instruction[0] == "read":
            if len(instruction) < 2:  # read command must have a parameter
                pub_logger.publish("read command requires a link to read from")
            else:
                try:
                    path_points = read_path(instruction[1])
                except:
                    pub_logger.publish("ERROR - file could not be read")
                Memory.instruction = "reset"
                pub_logger.publish("new path imported: " + str(path_points))
                continue
        elif instruction[0] == "start":
            if path_section_count > 0:
                follow_path = True
                move_to_point = False
                pub_logger.publish("velocity calculation started")
            else:
                pub_logger.publish("start command: invalid path")
        elif instruction[0] == "stop":
            pub_desired_velocity.publish(Vector3(0, 0, 0))
            follow_path = False
            move_to_point = False
            pub_logger.publish("velocity calculation ended")
        elif instruction[0] == "reset":
            follow_path = False
            move_to_point = False
            current_path_section = 0
            path_section_count = len(path_points) - 1
            rel_path_points = offset_coordinates(path_points, Memory.plane_point_2d)
            end2_points = calculate_border_points(rel_path_points)
            pub_logger.publish("path_controller reset")
        elif instruction[0] == "moveTo":
            move_to_point = True
            try:
                move_to_target = (float(instruction[1]), float(instruction[2]))
            except ValueError:
                pub_logger.publish("invalid paramters - must be two numbers specifying x and y coordinates")
        elif instruction[0] == "moveBy":
            move_to_point = True
            try:
                move_to_target = (Memory.plane_point_2d[0] + float(instruction[1]),
                                  Memory.plane_point_2d[1] + float(instruction[2]))
            except ValueError:
                pub_logger.publish("invalid paramters - must be two numbers specifying x and y coordinates")
        else:
            # for unrecognised commands
            pub_logger.publish("unrecognised command: " + Memory.instruction)
        Memory.instruction = ""

    # stops the end-effector from moving if it's outside the defined boundary area
    if global_point_3d_outside_boundary() and (move_to_point or follow_path):
        rospy.loginfo("end-effector left boundary area")
        Memory.instruction = "stop"
        continue

    # moves the end-effector
    if move_to_point:
        pub_desired_velocity.publish(calculate_velocity(Memory.plane_point_2d, move_to_target, 0, 1))
    elif follow_path:
        # identify current path section
        # stops moving end-effector if the end of the path has been reached
        while True:
            if current_path_section >= path_section_count:
                # if the end-effector has reached the end of its path
                Memory.instruction = "stop"
                rospy.loginfo("path end reached")
                break
            else:
                start = rel_path_points[current_path_section]
                end = rel_path_points[current_path_section + 1]
                start_left_of_border = point_is_left(end, end2_points[current_path_section], start)
                current_left_of_border = point_is_left(end, end2_points[current_path_section], Memory.plane_point_2d)
                if start_left_of_border == current_left_of_border:
                    # we are on this current line
                    pub_desired_velocity.publish(calculate_velocity(start, end, 1, 0))
                    break
                else:
                    # we have moved to the next section
                    rospy.loginfo("path section complete: " + str(current_path_section))
                    current_path_section += 1

    rate.sleep()

pub_logger.publish("path_controller shutting down")
