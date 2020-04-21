#! /usr/bin/env python2.7
import rospy
from geometry_msgs.msg import WrenchStamped
from std_msgs.msg import Float32MultiArray

# Other imports
import numpy as np
import math

gravity = np.array([[0],[0],[-9.80]]) # We assume the are is 1kg, can change this value to change the weight
#angle_x = 0
#angle_y = 0
#angle_z = math.pi/2e

Ff = np.array([[0],[0],[0],[0],[0],[0]])
Ff_gravity = np.array([[0],[0],[0],[0],[0],[0]])

# 6x6 transformation matrix SPF   
# We use this matrix to describe the shift in reference frame from force sensor frame to
# end-effector contact with surface Frame. We judge it's a shift in 23.5 cm in the +Z direction.
SPF = np.array([[1,0,0,0,0,0],[0,1,0,0,0,0],[0,0,1,0,0,0],[0,-0.235,0,1,0,0],[0.235,0,0,0,1,0],[0,0,0,0,0,1]])

# 6x6 transformation matrix FPG   
# We use this matrix to describe the shift in reference frame from gravity point mass frame to
# force sensor Frame. We judge it's a shift in 10cm in the +Z direction.
FPG = np.array([[1,0,0,0,0,0],[0,1,0,0,0,0],[0,0,1,0,0,0],[0,-0.1,0,1,0,0],[0.1,0,0,0,1,0],[0,0,0,0,0,1]])

# 3x3 static rotational matrix for end-effector frame to gravity point mass frame with reference to direction of +x and +y
# direction of force sensor frame 
static_rotational_matrix = np.array([[0,0,-1],[0,1,0],[1,0,0]])

# Centre of mass of TIAGo's end-effector
tool_mass = 0.1 #need to find

# Distance between centre point mass and end-effector frame of reference 
length = np.array([[0],[0],[0.1]])#need to find  

#TIAGo's constant error
constant_error = np.array([[4.5],[0.8],[27.5],[0.33],[-0.15],[-0.15]])

def getForceReadings(msg):
	
	global Ff, Ff_gravity, constant_error

	#readings from force/torque sensor
        f = msg.wrench.force
	t = msg.wrench.torque
	#print(f.x, f.y, f.z, t.x, t.y, t.z)

	#force/torque sensor, Ff readings in the form of 6x1 matrix
	Ff= (np.array([[f.x],[f.y],[f.z],[t.x],[t.y],[t.z]])+(Ff_gravity)-(constant_error))

	
	transformOfSensorFrame2surfaceFrame()


def getEndPosition(msg):

	global tool_mass, gravity, length, point_frame_matrix

	# Numpy ndarray in column-major order
	# Buffer assist to queue up the 9 elements obtained from Alex rotation matrix which is in a line
	rot_mat = np.ndarray( shape=(3,3), buffer=np.array(msg.data), order='F', dtype=float)	

	# Force in end-effector frame of reference
	end_frame_force = mass*(rot_mat.dot(gravity))

	# Torque in end-effector frame of reference
	end_frame_torque = end_frame_force.dot(length)

	# Force in point mass frame of reference
	point_frame_force = end_frame_force.dot(static_rotational_matrix)

	# Torque in point mass frame of reference
	point_frame_torque = end_frame_torque.dot(static_rotational_matrix)
	
	# 6x1 matrix in point mass frame of reference
        point_frame_matrix = np.array([[0],[0],[point_frame_force],[point_frame_torque],[point_frame_torque],[0]])

	gravityAtSensor()
	

# Deals with the transofmation of frames. Sensor ---> contact between sponge and surface
def transformOfSensorFrame2surfaceFrame():

	#gravityCompensator()

        #6x1 matrix for actual reading, Sf
        #obtained by multiplying SPF with Ff 
        #Sf = SPF.dot(Ff_gravity)

	Sf = SPF.dot(Ff)
	
	#publish the forces/torques with new frame of reference 
	velocitiesPublisher(Sf)
	
	

# handles the copensation of gravity, takes readings at force sensor and removes gravity       
def gravityCompensator():
	#global angle_x = 0
        #global angle_y = math.pi/4
	#global angle_z = 0
	pointMassGravity(angle_x, angle_y, angle_z) 
	gravityAtSensor(X_gravity,Y_gravity, Z_gravity)

	#Ff = Ff - Ff_gravity

# gives the force of gravity from the frame of reference of the center of mass
#def pointMassGravity(angle_x, angle_y, angle_z):

#	global X_gravity
#	global Y_gravity
#	global Z_gravity
	
	# we assume are is a point mass, this there are no torques from gravity. 
	# these are the linear values of gravity compensating for the rotation of the end-effector
#	X_gravity = Gravity*(math.cos(angle_z))*math.cos(angle_y)
#	Y_gravity = Gravity*(math.sin(angle_z))*math.cos(angle_x)
#s	Z_gravity = Gravity*(math.sin(angle_y))*math.cos(angle_x)
	# THIS MAKES THE ASSUMTION THAT THE BASE ANGLE HAS X up&down / Y left&right / Z forward/backwards
	
			

# gives the force of gravity from the force sensor position
def gravityAtSensor(): #X_gravity,Y_gravity, Z_gravity):
	#print('value of x: ',X_gravity)
	#print('value of y: ',Y_gravity)
	#print('value of z: ',Z_gravity)

	global Ff_gravity, point_frame_matrix

	#Ff_gravity = np.array([[X_gravity],[Y_gravity],[Z_gravity],[0],[0],[0]])

	#print('We are printing Ff-gravity: ',Ff_gravity)

	#6x1 matrix for actual reading, Sf
        #obtained by multiplying SPF with Ff 
        #Ff_gravity = FPG.dot(Ff_gravity)
	#print('We are printing Ff-gravity: ',Ff_gravity)

	Ff_gravity = FPG.dot(point_frame_matrix)
	print('We are printing Ff-gravity: ',Ff_gravity)


	




# handles the publishing of output message
def velocitiesPublisher(Sf):
	pub = rospy.Publisher('/end_effector/forces_surfaceFrame',WrenchStamped, queue_size=1)
	transformedInputForces = WrenchStamped()


	formatOutput(Sf,transformedInputForces)
	#print('transformedInputForces ', transformedInputForces)
	pub.publish(transformedInputForces)

# methode to correctly format the output message
def formatOutput(Sf,transformedInputForces):
	#this is the transformed value of the linear forces. This represents forces at surface.
	transformedInputForces.wrench.force.x = Sf[0][0]
	transformedInputForces.wrench.force.y = Sf[1][0]
	transformedInputForces.wrench.force.z = Sf[2][0]
	
	#this is the transformed value of the angular forces. This represents moments at surface.
	transformedInputForces.wrench.torque.x = Sf[3][0]
	transformedInputForces.wrench.torque.y = Sf[4][0]
	transformedInputForces.wrench.torque.z = Sf[5][0]


# Initialise node
rospy.init_node("sensorFrame_2_surfaceFrame")

# Subscribe to force sensor
sub_wrist = rospy.Subscriber("/wrist_ft", WrenchStamped, getForceReadings)

# Subscribe to end-effector 3x3 rotational matrix obtained from Alex part
sub_endFrame = rospy.Subscriber("/cleaning/forward_kinematics/orientation", WrenchStamped, getEndPosition)

# Loop to keep the node alive
r = rospy.Rate(20s)              
while not rospy.is_shutdown():	# Create a loop that will go until someone stops the program execution
      r.sleep()			# Make sure the publish rate maintains at 5 Hz


