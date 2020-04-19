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

# Used for reading values
pub_reading = None
# Joint positions
joint_angles = None			# np.array([1, 2, 3, 4, 5, 6, 7])

def cb_arm_state(msg):
	global joint_angles, pub_reading
	joint_angles = np.ndarray(shape=(7,1), dtype=float, order='F', buffer=np.array(msg.actual.positions, dtype=float))
	if pub_reading is None:
		return
	readings = [(msg.header.stamp.secs + msg.header.stamp.nsecs * 1e-9),]
	readings.extend(joint_angles)
	new_msg = Float32MultiArray()
	new_msg.data = readings
	pub_reading.publish(new_msg)



rospy.init_node("joint_vel_test")
pub_reading = rospy.Publisher("/readings", Float32MultiArray, queue_size=1)
sub_joints = rospy.Subscriber('/arm_controller/state', JointTrajectoryControllerState, cb_arm_state)
pub_control = rospy.Publisher("/arm_controller/command", JointTrajectory, queue_size=1)
freq = 20		# Number of commands published per second
joint_vel_max = 1.0	# Maximum velocity of a single joint - if this is exceeded all joint velocities are scaled down
duration_mult = 2	# Velocities are set as points to reach in the future - this multiplies how far ahead to make that point. At 1 the goal is at (1/freq), causing a lot of jittery motion in the arm
r = rospy.Rate(freq)

msg_arm = JointTrajectory()
msg_arm.joint_names = ["arm_1_joint", "arm_2_joint", "arm_3_joint", "arm_4_joint", "arm_5_joint", "arm_6_joint", "arm_7_joint"]
msg_arm_point = JointTrajectoryPoint()
msg_arm_point.time_from_start = Duration(nsecs=int(1e9/freq*duration_mult))

joint_vel_command = np.ndarray(shape=(7,1), dtype=float, buffer=np.array([1, 1, 0, 0, 0, 0, 0], dtype=float)*joint_vel_max)

while not rospy.is_shutdown():
	r.sleep()
	if (joint_angles is None):
		# Skip if not enough callbacks yet
		continue
	if np.all(np.equal(joint_vel_command, np.zeros(shape=(7,1)))):
		# Skip if command is 0
		continue
	if joint_vel_max < np.max(joint_vel_command):
		joint_desired_vel = joint_vel_command / np.max(joint_vel_command) * joint_vel_max
	else:
		joint_desired_vel = joint_vel_command
	joint_angles_new = joint_angles + joint_desired_vel * duration_mult / float(freq)

	# Publish joint angle message
	msg_arm_point.positions = joint_angles_new.flatten().tolist()
	msg_arm.points = [msg_arm_point]
	pub_control.publish(msg_arm)
