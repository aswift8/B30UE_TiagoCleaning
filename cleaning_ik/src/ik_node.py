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


# Linear x, y, z, Angular x, y, z, force-torque sensor frame of reference
end_effector_desired_vel = np.zeros(shape=(6,1), dtype=float)
# Joints 1 -> 7
joint_angles = None
# Arm inverse Jacobian
inverse_jacobian = None
# 6*6 matrix to rotate from force-torque sensor frame of reference to end-effector frame of reference
rot_mat_small = np.ndarray(shape=(3,3), dtype=float, order='F', buffer=np.array([0,1,0,0,0,1,1,0,0], dtype=float))
rot_mat = np.zeros(shape=(6,6), dtype=float)
rot_mat[:3,:3] = rot_mat_small
rot_mat[3:,3:] = rot_mat_small

def cb_path(msg):
	global end_effector_desired_vel			# In force-torque sensor frame of reference
	end_effector_desired_vel[0] = msg.x		# Linear x
	end_effector_desired_vel[1] = msg.y		# Linear y
	end_effector_desired_vel[5] = msg.z		# Angular z
def cb_correction(msg):
	global end_effector_desired_vel			# In force-torque sensor frame of reference
	end_effector_desired_vel[3] = msg.x		# Angular x
	end_effector_desired_vel[4] = msg.y		# Angular y
	end_effector_desired_vel[2] = msg.z		# Linear z
def cb_joints(msg):
	global joint_angles				# (7x1) numpy matrix/vector
	joint_angles = np.ndarray(shape=(7,1), dtype=float, buffer=np.array(msg.actual.positions, dtype=float))
def cb_jacobian(msg):
	global inverse_jacobian				# (7x6) numpy matrix
	jacobian = np.ndarray(shape=(6,7), dtype=float, order='F', buffer=np.array(msg.data, dtype=float))
	inverse_jacobian = np.linalg.pinv(jacobian)	# Matrix isn't square, so pseudoinverse has to be calculated (instead of inverse)
def cb_orientation(msg):
	global rot_mat					# (6x6) numpy matrix
	"""
	The cb_path and cb_correction velocities use the force-torque sensor frame of reference (i.e. z-axis is outwards towards the surface)
	The joints use a different frame of reference with the x-axis towards the surface
	This matrix converts from force-torque frame of reference to joint frame of reference for the end-effector
	"""
	rot_mat_corr = np.transpose(np.ndarray(shape=(3,3), dtype=float, order='F', buffer=np.array([0,0,1,1,0,0,0,1,0], dtype=float)))
	rot_mat_small = np.ndarray(shape=(3,3), dtype=float, order='F', buffer=np.array(msg.data, dtype=float))
	rot_mat_small = np.dot(rot_mat_small, rot_mat_corr)
	rot_mat[:3,:3] = rot_mat_small			# Convert linear velocities to correct axes
	rot_mat[3:,3:] = rot_mat_small			# Convert angular velocities to correct axes


sub_path = rospy.Subscriber("/path/desired_velocity", Vector3, cb_path)						# Path following component callback
sub_correction = rospy.Subscriber("/end_effector/correction_velocity", Vector3, cb_correction)			# Orientation correction component callback
sub_joints = rospy.Subscriber("/arm_controller/state", JointTrajectoryControllerState, cb_joints)		# Current position needed to add velocity * deltatime to
sub_jacobian = rospy.Subscriber("/cleaning/jacobian", Float32MultiArray, cb_jacobian)				# Jacobian required for inverse differential kinematics
sub_ori = rospy.Subscriber("/cleaning/forward_kinematics/orientation", Float32MultiArray, cb_orientation)	# Current orientation required to translate commands into correct frame of reference

pub_control = rospy.Publisher("/arm_controller/safe_command", JointTrajectory, queue_size=1)		# Arm trajectory commands published here

# Joint trajectory message is used to specify joint angles
msg_arm = JointTrajectory()
msg_arm.joint_names = ["arm_1_joint", "arm_2_joint", "arm_3_joint", "arm_4_joint", "arm_5_joint", "arm_6_joint", "arm_7_joint"]
msg_arm_point = JointTrajectoryPoint()

rospy.init_node("ik_node")
freq = 50								# Frequency for new messages to be published
duration_mult = 2							# Used to create an 'overlap' between messages, keep above 1 (otherwise arm will try to
									#   move to the next position before the next message, causing it to rapidly start/stop
joint_vel_max = 1.0							# Maximum velocity of a single joint (rad/sec)
r = rospy.Rate(freq)

msg_arm_point.time_from_start = Duration(nsecs=1e9*duration_mult/freq)	# (duration_mult / freq) = deltatime

while not rospy.is_shutdown():
	r.sleep()
	if (joint_angles is None or inverse_jacobian is None):	# joint_angles and inverse_jacobian are required for calculation, need these to be published first
		continue
	# Transform end-effector velocity to correct frame of reference
	end_effector_vel_command = np.dot(rot_mat, end_effector_desired_vel)
	# Use jacobian pseudoinverse to calculate desired joint velocity
	joint_vel_command = np.dot(inverse_jacobian, end_effector_vel_command)
	# Scale joint velocity down if too high
	if joint_vel_max < np.max(joint_vel_command):
		joint_vel = joint_vel_command / np.max(joint_vel_command) * joint_vel_max
	else:
		joint_vel = joint_vel_command
	# Calculate new joint angles from current angles and velocity
	joint_angles_new = joint_angles + joint_vel * duration_mult / float(freq)

	# Publish joint angle message
	msg_arm_point.positions = joint_angles_new.flatten().tolist()
	msg_arm.points = [msg_arm_point]
	pub_control.publish(msg_arm)
