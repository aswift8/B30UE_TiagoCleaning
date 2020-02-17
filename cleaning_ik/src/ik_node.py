#! /usr/bin/env python2.7

# ROS imports
import rospy
from rospy.rostime import Duration
from control_msgs.msg import JointTrajectoryControllerState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Vector3
from std_msgs.msg import Float32MultiArray
# Python imports
import numpy as np


# Linear x, y, z, Angular x, y, z
end_effector_desired_vel = np.array([0, 0, 0, 0, 0, 0], dtype=float)
# Joints 1 -> 7
joint_angles = np.array([0, 0, 0, 0, 0, 0, 0], dtype=float)
# Arm inverse Jacobian
inverse_jacobian = np.zeros((7, 6), dtype=float)

def cb_path(msg):
	global end_effector_desired_vel
	end_effector_desired_vel[0] = msg.x	# Linear x
	end_effector_desired_vel[1] = msg.y	# Linear y
	end_effector_desired_vel[5] = msg.z	# Angular z
def cb_correction(msg):
	global end_effector_desired_vel
	end_effector_desired_vel[3] = msg.x	# Angular x
	end_effector_desired_vel[4] = msg.y	# Angular y
	end_effector_desired_vel[2] = msg.z	# Linear z
def cb_joints(msg):
	global joint_angles
	joint_angles = np.array(msg.actual.positions)
def cb_jacobian(msg):
	global inverse_jacobian
	jacobian = np.ndarray(shape=(6,7), dtype=float, order='F', buffer=np.array(msg.data))
	inverse_jacobian = np.linalg.pinv(jacobian)
	print("Inverse jacobian:", inverse_jacobian)

# TODO: find & import message types
sub_path = rospy.Subscriber("/path/desired_velocity", Vector3, cb_path)
sub_correction = rospy.Subscriber("/end_effector/correction_velocity", Vector3, cb_correction)
sub_joints = rospy.Subscriber("/arm_controller/state", JointTrajectoryControllerState, cb_joints)
sub_jacobian = rospy.Subscriber("/cleaning/jacobian", Float32MultiArray, cb_jacobian)

pub_control = rospy.Publisher("/arm_controller/command", JointTrajectory, queue_size=1)

msg_arm = JointTrajectory()
msg_arm.joint_names = ["arm_1_joint", "arm_2_joint", "arm_3_joint", "arm_4_joint", "arm_5_joint", "arm_6_joint", "arm_7_joint"]
msg_arm_point = JointTrajectoryPoint()

rospy.init_node("ik_node")
freq = 10
r = rospy.Rate(freq)

duration_mult = 2
msg_arm_point.time_from_start = Duration(nsecs=1e9/freq*duration_mult)

while not rospy.is_shutdown():
	r.sleep()
	# Inverse Jacobian from inverse_jacobian
	# Update desired joint angles
	joint_desired_vel = inverse_jacobian.dot(end_effector_desired_vel)
	joint_desired_vel = np.array([1, 1, 1, 1, 1, 1, 1], dtype=float)
	joint_angle_new = joint_angles + joint_desired_vel * duration_mult / float(freq)
	print("New angles:", joint_angle_new)
	# Publish joint angle message
"""
	msg_arm_point.positions = joint_angle_new.tolist()
	msg_arm.points = [msg_arm_point]
	pub_control.publish(msg_arm)
"""

