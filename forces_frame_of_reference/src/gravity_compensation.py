#! /usr/bin/env python2.7

import rospy
from geometry_msgs.msg import WrenchStamped
from std_msgs.msg import Float32MultiArray

import numpy as np
import math


# The goal of this code is to compensate for the gravity affecting the end effector. 
# To achieve this there are 2 majors parts:
#
# Firstly, the code finds how gravity affects the end effector as if it were a point mass. 
# This results in a 3*1 column matrix with the values of gravity in linear the X,Y,Z (from end 
# effectors persepective).  
# 
# Secondly, the new 3*1 column matrix of gravity is moved from being at the point mass to being at 
# the location of the force sensor. This force readings at the wrist sensor are read and have gravity # sumbstracted from them. The resultant forces are then published to the node 
# sensorFrame_2_surfaceFrame on the topic /wrist_ft_gravity_compensated



# GRAVITY MATRICES

# 3x1
gravity_column_matrix_orientation = np.array([[0],[0],[-9.81]]) # be carful, as this assums point mass has mass 1kg

# 3x3
rotational_matrix =  np.array([[0,0,0],[0,0,0],[0,0,0]])

# 6x1
gravity_compensation = np.array([[0],[0],[0],[0],[0],[0]])
force_at_wrist = np.array([[0],[0],[0],[0],[0],[0]])

# TRANSFORM MATRIX

#6x6 transformation matrix FPG   
# We use this matrix to describe the shift in reference frame from gravity point mass frame to
# force sensor Frame. We judge it's a shift in 10cm in the +Z direction.
pointmass_2_sensor_transfrom = np.array([[1,0,0,0,0,0],[0,1,0,0,0,0],[0,0,1,0,0,0],[0,-0.1,0,1,0,0],[0.1,0,0,0,1,0],[0,0,0,0,0,1]])
##### the above needs hanging as we don't use the same x,y,z!!! ####

# PUBLISHER DECLARE 
pub = rospy.Publisher('/wrist_ft_compensated',WrenchStamped, queue_size=1) 


# is looped in main method
def gravityCompensation():
	rotational_matrix = getRotationMatrix()
 	gravity_compensation = getGravity_for_EndEffector_pointmass(rotational_matrix)
	gravity_compensation = getGravity_at_end_effector_sensor(gravity_compensation)
	gravity_compensation = correctedForceReading(gravity_compensation)
	velocitiesPublisher(gravity_compensation)

# step 1) get rotation matrix of base to end effector.
def getRotationMatrix():
	return rotational_matrix

# step 2) create the gravity values from the perspective of the point mass
def getGravity_for_EndEffector_pointmass(rotational_matrix):

	gravity_at_pointMass = np.array([[0],[0],[0]])

	# Mutiply rotation matrix of base to end effector, by gravity at the base. 
	gravity_at_pointMass = rotational_matrix.dot(gravity_column_matrix_orientation)
	
	# The next step requiers a 6x1 matrix. 
	# So we feed in the linear x,y,z values we found
	gravity_compensation[0][0] = gravity_at_pointMass[0][0] # x
	gravity_compensation[1][0] = gravity_at_pointMass[1][0] # y 
	gravity_compensation[2][0] = gravity_at_pointMass[2][0] # z

	# there are no angular forces on a pointmass, so we leave the default 0 values.
	
	return gravity_compensation

# step 3) using gravity values from point mass reference to find gravity values at sensor reference
def getGravity_at_end_effector_sensor(gravity_compensation):
	# we multiply the gravity forces at point mass by the appropriate transfrom
	gravity_compensation = pointmass_2_sensor_transfrom.dot(gravity_compensation)
	return gravity_compensation

# step 4) substracting the force of gravity from the force readings in the wrist
def correctedForceReading(gravity_compensation):
	gravity_compensation = force_at_wrist - gravity_compensation
	return gravity_compensation


# REQUIERED for step 1) 
def getRotationMatrix_base_to_endEffector(msg):
	global rotational_matrix
	# create a numpy ndarray
	rotational_matrix = np.ndarray( shape=(3,3), buffer=np.array(msg.data),order='F',dtype=float)
	
# REQUIERED for step 4) 
def getForceReading(msg):
	global force_at_wrist

	#readings from force/torque sensor
        f = msg.wrench.force
	t = msg.wrench.torque
	#print(f.x, f.y, f.z, t.x, t.y, t.z)

	#force/torque sensor, Ff readings in the form of 6x1 matrix
	force_at_wrist = np.array([[f.x],[f.y],[f.z],[t.x],[t.y],[t.z]])



# REQUIERED for publisher.  Methode to correctly format the output message.
def formatOutput(Sf,compensatedForces):
	#this is the transformed value of the linear forces. This represents forces at surface.
	compensatedForces.wrench.force.x = Sf[0][0]
	compensatedForces.wrench.force.y = Sf[1][0]
	compensatedForces.wrench.force.z = Sf[2][0]
	
	#this is the transformed value of the angular forces. This represents moments at surface.
	compensatedForces.wrench.torque.x = Sf[3][0]
	compensatedForces.wrench.torque.y = Sf[4][0]
	compensatedForces.wrench.torque.z = Sf[5][0]

# SUBSCRIBERS	

# Subscribe to end-effector position and angle
sub_endFrame = rospy.Subscriber("/cleaning/forward_kinematics/orientation", WrenchStamped, getRotationMatrix_base_to_endEffector)

# Subscribe to force sensor
sub_wrist = rospy.Subscriber("/wrist_ft", WrenchStamped, getForceReading)



# PUBLISHERS

# handles the publishing of output message
def velocitiesPublisher(gravity_compensation):
	global pub
	compensatedForces = WrenchStamped()
	formatOutput(gravity_compensation,compensatedForces)
	#print('compensatedForces: ', compensatedForces)
	pub.publish(compensatedForces)

# TYPICAL NODE SETUP

# Initialise node
rospy.init_node("gravity_compensation")


# Loop to keep the node alive
r = rospy.Rate(5)              
while not rospy.is_shutdown():	# Create a loop that will go until someone stops the program execution
      gravityCompensation()
      r.sleep()			# Make sure the publish rate maintains at 5 Hz

