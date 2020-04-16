#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Vector3
import numpy as np


class Memory:
    current_rotation = np.array([[ 1,  0,  0],
                                 [ 0,  1,  0],
                                 [ 0,  0,  1]])
    current_position = Vector3(0, 0, 0)
    current_velocity = Vector3(0, 0, 0)
    receive_velocity_from = "/path/desired_velocity"
    give_position_to = "/cleaning/forward_kinematics/position"
    give_rotation_to = "/cleaning/forward_kinematics/orientation"
    rate = 10


def update_velocity(velocity_data):
    Memory.current_velocity = velocity_data


def update_pose():
    Memory.current_position.x += (Memory.current_velocity.x / Memory.rate)
    Memory.current_position.y += (Memory.current_velocity.y / Memory.rate)
    Memory.current_position.z += (Memory.current_velocity.z / Memory.rate)
    Memory.current_velocity = Vector3(0, 0, 0)


def get_current_position():
    return Memory.current_position


def get_rotation_array():
    return Float32MultiArray(
    data=[current_rotation[0], current_rotation[3], current_rotation[6],
          current_rotation[1], current_rotation[4], current_rotation[7]
          current_rotation[2], current_rotation[5], current_rotation[8]])


rospy.init_node("end_effector_sim")
rate = rospy.Rate(Memory.rate)
pub_pos = rospy.Publisher(Memory.give_position_to, Vector3, queue_size=5)
pub_rot = rospy.Publisher(Memory.give_rotation_to, Float32MultiArray, queue_size=5)
rospy.Subscriber(Memory.receive_velocity_from, Vector3, update_velocity)

# wait for one second
# for i in range(0, Memory.rate):
#     rate.sleep()

while not rospy.is_shutdown():
    update_pose()
    pub_pos.publish(get_current_position())
    pub_rot.publish(get_rotation_array())
    rate.sleep()
