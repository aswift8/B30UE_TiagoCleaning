#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Vector3


class Memory:
    current_position = Vector3(0, 0, 0)
    current_rotation = Float32MultiArray()
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


def get_current_rotation():
    return Memory.current_rotation


Memory.current_rotation.data = [0, 0, 0, 0, 0, 0, 0, 0, 0]

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
    pub_rot.publish(get_current_rotation())
    rate.sleep()
