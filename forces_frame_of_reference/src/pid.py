#! /usr/bin/env python2.7
import rospy
from geometry_msgs.msg import WrenchStamped
from geometry_msgs.msg import Vector3

# Other imports
import numpy as np

#Proportional gain
Kp= 0.01

#Integral gain
Ki= 0.001

#Derivative gain
Kd= 0.001

#iterations
i = 0

#seconds per repeat to read the value of error
time = 1

#list
error_list = []

#6x1 matrix for desired input, Sfd
Sfd = np.array([[0],[0],[-1],[0],[0],[0]])



def callback(msg):

	#readings from force/torque sensor
        f = msg.wrench.force
	t = msg.wrench.torque
	#print(f.x, f.y, f.z, t.x, t.y, t.z)

	#force/torque sensor, Ff readings in the form of 6x1 matrix
	Sf= np.array([[f.x],[f.y],[f.z],[t.x],[t.y],[t.z]])

	pid(Sf)

def pid(Sf):
	# Python doesn't have a do while loop, so we use an I to fork the code instead. 
	# We need this to obtain a last-error for the derivative to work
	global i, error

	#callback class start reading from this part  
	if (i==0):

		#difference between desired input and actual reading
	        error = Sfd - Sf
	        #print('Loop1')
		i=1

	#error = np.array.zeros
	if (i>0):
		#print('Loop2')
 		last_error = error

	        #difference between desired input and actual reading
	        error = Sfd - Sf

 	        #function of .append is to add an item to the end of the list
		error_list.append(error)

		proportional = Kp*error

		#integral = sum(error_list) * Ki / (len(error_list) * 1.0/20.0)

		#derivative = (Kd/20.0)*(error - last_error)
	
			
		# For PID control, we add the P + I + D = output (of type velocities in our case)
		output = proportional #+ derivative 

		#print(output)
		velocitiesPublisher(output)


	

def velocitiesPublisher(output):
	pub = rospy.Publisher('/end_effector/correction_velocity', Vector3, queue_size=1)
	correction_velocities = Vector3()
	formatOutput(output, correction_velocities)
	print('correction_velocities[] ', correction_velocities)
	pub.publish(correction_velocities)
	
def formatOutput(output, correction_velocities):
	correction_velocities.x = output[3]
	correction_velocities.y = output[4]
	correction_velocities.z = output[2]

# Initialise node
rospy.init_node("pid")

# Subscribe
sub_wrist = rospy.Subscriber("/end_effector/forces_surfaceFrame", WrenchStamped, callback)

# Loop
r = rospy.Rate(20)
while not rospy.is_shutdown():	# Create a loop that will go until someone stops the program execution
      r.sleep()			# Make sure the publish rate maintains at 5 Hz


