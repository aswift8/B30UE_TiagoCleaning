#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Vector3
import numpy as np

class Memory:
    current_position = Vector3(0, 0, 0)
    current_velocity = Vector3(0, 0, 0)
    receive_velocity_from = "/path/desired_velocity"
    give_position_to = "/cleaning/forward_kinematics/position"
    rate = 10


def update_velocity(velocity_data):
    Memory.current_velocity = velocity_data


def update_position():
    Memory.current_position.x += (Memory.current_velocity.x / Memory.rate)
    Memory.current_position.y += (Memory.current_velocity.y / Memory.rate)
    Memory.current_velocity = Vector3(0, 0, 0)


def get_current_position():
    return Memory.current_position


rospy.init_node("end_effector_sim")
rate = rospy.Rate(Memory.rate)
pub_pos = rospy.Publisher(Memory.give_position_to, Vector3, queue_size=5)
rospy.Subscriber(Memory.receive_velocity_from, Vector3, update_velocity)

# wait for one second
# for i in range(0, Memory.rate):
#     rate.sleep()

while not rospy.is_shutdown():
    update_position()
    pub_pos.publish(get_current_position())
    rate.sleep()
