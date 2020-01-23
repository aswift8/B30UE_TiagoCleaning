#! /usr/bin/env python2.7

# Python imports
from copy import deepcopy
# ROS imports
import rospy
from rospy.rostime import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


# Message on its own often fails to publish first time - this solves that
def publish_once(publisher, message):
	rospy.logdebug("Looking for subscriber")
	while publisher.get_num_connections() == 0:
		rospy.logdebug("Waiting for subscriber")
		rospy.sleep(1)
	rospy.logdebug("Publishing")
	publisher.publish(message)


# Create node
rospy.init_node("test_movement", log_level=rospy.DEBUG)

# -- HEAD CONTROL --
# Create publisher
pub_head = rospy.Publisher("/head_controller/command", JointTrajectory, queue_size=1)
# Create message
msg_head = JointTrajectory()
msg_head.joint_names = ["head_1_joint", "head_2_joint"]
# Needs point
point_head = JointTrajectoryPoint()
point_head.positions = [0, 0.5]
point_head.time_from_start = Duration(nsecs=1)	# If time_from_start is 0 nothing happens
# Add point to message - multiple points can be added
# 			but need different time_from_start durations!
msg_head.points = [point_head]

# -- TORSO CONTROL --
# Create publisher
pub_torso = rospy.Publisher("/torso_controller/command", JointTrajectory, queue_size=1)
# Create message
msg_torso = JointTrajectory()
msg_torso.joint_names = ["torso_lift_joint"]
# Needs point
point_torso_1 = JointTrajectoryPoint()
point_torso_1.positions = [0.5]
point_torso_1.time_from_start = Duration(nsecs=1)
point_torso_2 = JointTrajectoryPoint()
point_torso_2.positions = [0]
point_torso_2.time_from_start = Duration(secs=5)
# Add point to message
msg_torso.points = [point_torso_1, point_torso_2]

# -- ARM CONTROL --
# IMPORTANT - if points list includes *any* point that is self-collision safe_command will NOT run
# Create publisher
pub_arm = rospy.Publisher("/arm_controller/safe_command", JointTrajectory, queue_size=1)
# Create message
msg_arm = JointTrajectory()
msg_arm.joint_names = ["arm_1_joint", "arm_2_joint", "arm_3_joint", "arm_4_joint", "arm_5_joint", "arm_6_joint", "arm_7_joint"]
# Needs point
"""
point_arm_0 = JointTrajectoryPoint()
point_arm_0.positions = [0, 0, 0, 0, 0, 0, 0]
point_arm_0.time_from_start = Duration(secs=3)
point_arm_1 = JointTrajectoryPoint()
point_arm_1.positions = [1, 0, 0, 0, 0, 0, 0]
point_arm_1.time_from_start = Duration(secs=6)
point_arm_2 = JointTrajectoryPoint()
point_arm_2.positions = [0, 1, 0, 0, 0, 0, 0]
point_arm_2.time_from_start = Duration(secs=9)
point_arm_3 = JointTrajectoryPoint()
point_arm_3.positions = [0, 0, 1, 0, 0, 0, 0]
point_arm_3.time_from_start = Duration(secs=12)
point_arm_4 = JointTrajectoryPoint()
point_arm_4.positions = [0, 0, 0, 1, 0, 0, 0]
point_arm_4.time_from_start = Duration(secs=15)
point_arm_5 = JointTrajectoryPoint()
point_arm_5.positions = [0, 0, 0, 0, 1, 0, 0]
point_arm_5.time_from_start = Duration(secs=18)
point_arm_6 = JointTrajectoryPoint()
point_arm_6.positions = [0, 0, 0, 0, 0, 1, 0]
point_arm_6.time_from_start = Duration(secs=21)
point_arm_7 = JointTrajectoryPoint()
point_arm_7.positions = [0, 0, 0, 0, 0, 0, 1]
point_arm_7.time_from_start = Duration(secs=24)
point_arm_8 = JointTrajectoryPoint()
point_arm_8.positions = [0, 0, 0, 0, 0, 0, 0]
point_arm_8.time_from_start = Duration(secs=27)
# Add point to message
msg_arm.points = [point_arm_0,point_arm_1,point_arm_2,point_arm_3,point_arm_4,point_arm_5,point_arm_6,point_arm_7,point_arm_8]
"""
"""
point_arm_0 = JointTrajectoryPoint()
point_arm_0.positions = [0, 0, 0, 0, 0, 0, 0]
point_arm_0.time_from_start = Duration(secs=3)
point_arm_1 = JointTrajectoryPoint()
point_arm_1.positions = [0, -1, 0, 0, 0, 0, 0]
point_arm_1.time_from_start = Duration(secs=6)
point_arm_2 = JointTrajectoryPoint()
point_arm_2.positions = [0, -0.5, 0, 0, 0, 0, 0]
point_arm_2.time_from_start = Duration(secs=9)
point_arm_3 = JointTrajectoryPoint()
point_arm_3.positions = [0, 0, 0, 0, 0, 0, 0]
point_arm_3.time_from_start = Duration(secs=12)
msg_arm.points = [point_arm_0,point_arm_1,point_arm_2,point_arm_3]
"""
point_arm_0 = JointTrajectoryPoint()
point_arm_0.positions = [0, 0, 0, 0, 0, 0, 0]
point_arm_0.time_from_start = Duration(nsecs=1)
point_arm_1 = JointTrajectoryPoint()
point_arm_1.positions = [2, 0, 0, 0, 0, 0, 0]
point_arm_1.time_from_start = Duration(secs=1)
point_arm_2 = JointTrajectoryPoint()
point_arm_2.positions = [0, 0, 0, 0, 0, 0, 0]
point_arm_2.time_from_start = Duration(secs=2)
msg_arm.points = [point_arm_0, point_arm_1, point_arm_2]

# Publish message
#publish_once(pub_head, msg_head)
#publish_once(pub_torso, msg_torso)
publish_once(pub_arm, msg_arm)

r = rospy.Rate(5)

while not rospy.is_shutdown():
	#pub_arm.publish(msg_arm)
	rospy.logdebug("Still here")
	r.sleep()
	#pub_head.publish(msg_head)

