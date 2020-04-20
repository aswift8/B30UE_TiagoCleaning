#! /usr/bin/env python2.7

#roslib.load_manifest('turtlebot_teleop') # No longer needed in catkin!
import roslib
import numpy as np
import rospy

from rospy.rostime import Duration
from control_msgs.msg import JointTrajectoryControllerState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Vector3
from std_msgs.msg import Float32MultiArray
from pynput.keyboard import Key, Controller

rospy.init_node("moving_arm_joint_6")

#Goal is to work with arm_6 & arm_7 to show PID control

# Linear x, y, z, Angular x, y, z
#end_effector_desired_vel = np.array([0, 0, 0, 0, 0, 0], dtype=float)

# Joints 1 -> 7
joint_angles = np.array([0, 0, 0, 0, 0, 0, 0], dtype=float)

joint_angles_6_positive = np.array([0, 0, 0, 0, 0, 0.1, 0], dtype=float)
joint_angles_6_negative = np.array([0, 0, 0, 0, 0, -0.1, 0], dtype=float)
joint_angles_7_postiive = np.array([0, 0, 0, 0, 0, 0.1, 0], dtype=float)
joint_angles_7_negative = np.array([0, 0, 0, 0, 0, -0.1, 0], dtype=float)

keyboard = Controller ()

#keyboard.press('w')
#keyboard.release('w')

#moveBindings = {
#        'w': arm_joint_6_positive,
#        'a': arm_joint_7_positive,
#        's': arm_joint_6_negative,
#       'd': arm_joint_7_negative,
#}


#def cb_joints(msg):
#	global joint_angles
#	joint_angles = np.array(msg.actual.positions)
#	cb_correction(msg)
	
#def cb_correction(msg):
#end_effector_desired_vel[3] = msg.x	# Angular x
#end_effector_desired_vel[4] = msg.y	# Angular y
#end_effector_desired_vel[2] = msg.z	# Linear z 
#      print(end_effector_desired_vel[2])
#print("Hello")


# ROS publishers
pub_control = rospy.Publisher("/arm_controller/command", JointTrajectory, queue_size=1)

# Subscriber
#sub_correction = rospy.Subscriber("/end_effector/correction_velocity", Vector3, cb_correction)
#sub_joints = rospy.Subscriber("/arm_controller/state", JointTrajectoryControllerState, cb_joints)

msg_arm = JointTrajectory()
msg_arm.joint_names = ["arm_1_joint", "arm_2_joint", "arm_3_joint", "arm_4_joint", "arm_5_joint", "arm_6_joint", "arm_7_joint"]
msg_arm_point = JointTrajectoryPoint()

freq = 10
r = rospy.Rate(freq)

duration_mult = 2
msg_arm_point.time_from_start = Duration(nsecs=1e9/freq*2.0)

while not rospy.is_shutdown():	# Create a loop that will go until someone stops the program execution
        r.sleep()	
	if keyboard.read_key() == "w":
        	print("You pressed w")
		joint_angles = joint_angles + joint_angles_6_positive
	elif keyboard.read_key() == "a":
        	print("You pressed a")
       	        joint_angles = joint_angles + joint_angles_7_positive
	elif keyboard.read_key() == "s":
        	print("You pressed s")
 	        joint_angles = joint_angles + joint_angles_6_negative
	elif keyboard.read_key() == "d":
        	print("You pressed d")
       	        joint_angles = joint_angles + joint_angles_7_negative
	else:
		joint_angles = joint_angles

	msg_arm_point.positions = joint_angles.tolist()
	msg_arm.points = [msg_arm_point]
	pub_control.publish(msg_arm)

#if __name__=="__main__":

