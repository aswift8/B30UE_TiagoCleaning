#!/usr/bin/env python

import rospy
import roslaunch
import xml.etree.cElementTree
import math
from std_msgs.msg import String, Float32MultiArray
from geometry_msgs.msg import Vector3, Point


class Memory:
    # operation parameters
    run_rate = 10
    correction_threshold = 0.01
    velocity_threshold = 0.01
    z_abs_boundary_min = 0
    z_abs_boundary_max = 2
    position_update_threshold = 0.1
    # static variables (for access inside methods)
    instruction = ""
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
    #if distance_between_points(Memory.current_point_3d, current_position) > Memory.position_update_threshold:
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


def calculate_velocity(start_point, end_point, desired_speed,
                       linear_correct_multi, angular_correct_multi, slow_on_approach_distance):
    vector = Vector3()
    vector.x = end_point[0] - start_point[0]
    vector.y = end_point[1] - start_point[1]
    vector = normalise_vector(vector, slow_on_approach_distance, desired_speed)

    # TODO test alternate direction gradients (absolute path test)
    # error correction calculation
    if linear_correct_multi > 0:
        correction_magnitude = point_left_by(start_point, end_point, Memory.current_point_2d)
        if abs(correction_magnitude) > Memory.correction_threshold:
            correction_m = inverse_gradient(get_gradient(start_point, end_point))
            if correction_m == "inf":
                vector.y -= abs(correction_magnitude) * linear_correct_multi
            elif correction_m == 0:
                vector.x += correction_magnitude * linear_correct_multi
            else:
                theta = math.atan(correction_m) # neg gradient => neg radian => pos cosine
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
            vector = normalise_vector(vector, slow_on_approach_distance, desired_speed)

    # TODO test rotation transformation
    # z rotation correction
    # theta = -math.atan2(Memory.current_rotation_3d.data[7], -Memory.current_rotation_3d.data[6])
    # r = 1 + Memory.current_rotation_3d.data[8]
    # vector.z = r*(-math.cos(theta) - Memory.current_rotation_3d.data[2]) \
    #            + (1-r)*(math.asin(Memory.current_rotation_3d.data[3]))
    # if vector.z != 0:
    #     new_x = vector.x * math.cos(vector.z) + vector.y * math.sin(vector.z)
    #     new_y = vector.y * math.cos(vector.z) + vector.x * math.sin(vector.z)
    #     vector.x = new_x
    #     vector.y = new_y
    #     vector.z *= angular_correct_multi

    return vector


def normalise_vector(vector, slow_on_approach_distance, multi):
    magnitude = math.sqrt((vector.x * vector.x) + (vector.y * vector.y))
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


# variable initialisation
follow_path = False
move_to_point = False
move_to_target = (0, 0)
path_points = []
rel_path_points = []
end2_points = []
Memory.current_rotation_3d.data = [0, 0, 0, 0, 0, 0, 0, 0, 0]
current_path_section = 0
path_section_count = 0

# ros networking setup
rospy.init_node("path_controller")
rate = rospy.Rate(Memory.run_rate)
rospy.Subscriber("/path/instruction", String, new_instruction)
rospy.Subscriber("/cleaning/forward_kinematics/position", Vector3, update_position)
rospy.Subscriber("/cleaning/forward_kinematics/orientation", Float32MultiArray, update_rotation)
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
    if len(instruction[0]) > 0: # if an instruction exists
        pub_logger.publish("new instruction: " + str(Memory.instruction))
        if instruction[0] == "read":
            if len(instruction) < 2: # read command must have a parameter
                pub_logger.publish("read command requires a link to read from")
            else:
                path_points = read_path(instruction[1])
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
            follow_path = False
            move_to_point = False
            pub_logger.publish("velocity calculation ended")
        elif instruction[0] == "reset":
            # TODO zero path onto end-effector's current starting position
            follow_path = False
            move_to_point = False
            current_path_section = 0
            path_section_count = len(path_points) - 1
            rel_path_points = offset_coordinates(path_points, Memory.current_point_2d)
            end2_points = calculate_border_points(rel_path_points)
            pub_logger.publish("path_controller reset")
        elif instruction[0] == "moveTo":
            move_to_point = True
            move_to_target = (float(instruction[1]), float(instruction[2]))
        elif instruction[0] == "moveBy":
            move_to_point = True
            move_to_target = (Memory.current_point_2d[0] + float(instruction[1]),
                              Memory.current_point_2d[1] + float(instruction[2]))
        else:
            # for unrecognised commands
            pub_logger.publish("unrecognised command: " + Memory.instruction)
        Memory.instruction = ""

    if move_to_point:
        pub_desired_velocity.publish(calculate_velocity(Memory.current_point_2d, move_to_target, 1, 0, 1, 1))
    elif follow_path:
        # identify current path section
        # stops moving end-effector if the end of the path has been reached
        while True:
            if current_path_section >= path_section_count:
                # if the end-effector has reached the end of its path
                Memory.instruction = "stop"
                pub_desired_velocity.publish(Vector3(0, 0, 0))
                rospy.loginfo("path end reached")
                break
            else:
                start = rel_path_points[current_path_section]
                end = rel_path_points[current_path_section + 1]
                start_left_of_border = point_is_left(end, end2_points[current_path_section], start)
                current_left_of_border = point_is_left(end, end2_points[current_path_section], Memory.current_point_2d)
                if start_left_of_border == current_left_of_border:
                    # we are on this current line
                    pub_desired_velocity.publish(calculate_velocity(start, end, 1, 1, 1, 0))
                    break
                else:
                    # we have moved to the next section
                    rospy.loginfo("path section complete: " + str(current_path_section))
                    current_path_section += 1

    rate.sleep()

pub_logger.publish("path_controller shutting down")
