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
rospy.init_node("testnode", log_level=rospy.DEBUG)

# Create publisher
pub = rospy.Publisher("/torso_controller/command", JointTrajectory, queue_size=1)

# Create message
msg = JointTrajectory()
# Needs point
point = JointTrajectoryPoint()
point.positions = [0]
point.time_from_start = Duration(nsecs=1)	# If time_from_start is 0 nothing happens
# Move torso
msg.joint_names = ["torso_lift_joint"]
msg.points = [point]

# Publish message
publish_once(pub, msg)

while not rospy.is_shutdown():
	rospy.logdebug("Still here")
	rospy.sleep(1)

