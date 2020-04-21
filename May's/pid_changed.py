#! /usr/bin/env python2.7
import rospy
from geometry_msgs.msg import WrenchStamped
from geometry_msgs.msg import Vector3
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import JointTrajectoryControllerState
from rospy.rostime import Duration

# Other imports
import numpy as np
import matplotlib.pyplot as plt

# Newly added
arm_current = None	
running = True		

# Arm state callbackx
#def cb_arm_state(msg):
#	global arm_current
#	arm_current = np.array(msg.actual.positions)

# Joints 1 -> 7
joint_angles = np.array([0, 0, 0, 0, 0, 0, 0], dtype=float)

#Newly added


#Proportional gain
Kp= 0.001

#Integral gain
#Ki= 0.1

#Derivative gain
Kd= 0.001

#iterations
i = 0

#seconds per repeat to read the value of error
time = 1

#list
error_list = []

# Lists for graphing
p_list = []
i_list = []
d_list = []
pid_list = []


# Figures and axes
fig=plt.figure()
p_ax = fig.add_subplot(2,2,1)   # Plotting in form of: (row,column,position)
i_ax = fig.add_subplot(2,2,2)
d_ax = fig.add_subplot(2,2,3)
pid_ax = fig.add_subplot(2,2,4)

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
	global i, error, error_list

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

		if len(error_list) > 500:	# Make this a variable
			error_list = error_list[1:]    #Cut of the first value if list > 500, so 501th become 500th in the list

		proportional = Kp*error
			
		#integral = sum(error_list) * Ki / (len(error_list) * 1.0/20.0) # t = 1/f , TIAGo's frequency runs at 20Hz

		#derivative = (Kd*1.0/20.0)*(error - last_error)
			
		# For PID control, we add the P + I + D = output (of type velocities in our case)
		output = proportional #+ derivative 

		# Using transpose matrix to convert velocity from surface frame of reference to end-effector frame of reference for Alex's part
		#surface_to_end_frame_velocity = output.transpose()

		p_list.append(proportional)
		#i_list.append(integral)
		#d_list.append(derivative)
		#pid_list.append(output)

		#print(output)
		velocitiesPublisher(output)


	
#publish to arm 6
def velocitiesPublisher(output):
	
	#Newly added 
	global arm_current
	
	freq = 20
	r = rospy.Rate(freq)

	# ROS publishers
	duration = Duration(nsecs=1e9/float(freq)*3.0)
	arm_pub = rospy.Publisher('/arm_controller/safe_command', JointTrajectory, queue_size=1)
	arm_msg = JointTrajectory()

	correction_velocities = Vector3()
	formatOutput(output, correction_velocities)
	print('correction_velocities[] ', correction_velocities)
	
	joint_angles_6 = correction_velocities.z
	joint_angles_7 = correction_velocities.x
	
	arm_pub.publish(correction_velocities)
	#arm_msg.joint_names = ["arm_1_joint", "arm_2_joint", "arm_3_joint", "arm_4_joint", "arm_5_joint", "arm_6_joint",
						  # "arm_7_joint"]
	arm_msg.joint_names = ["arm_6_joint", "arm_7_joint"]
	arm_point = JointTrajectoryPoint()
	arm_point.time_from_start = duration
	arm_msg.points = [arm_point]

	# Subscribers
	arm_sub = rospy.Subscriber('/arm_controller/state', JointTrajectoryControllerState, cb_arm_state)
 	#Newly added



	#pub = rospy.Publisher('/end_effector/correction_velocity', Vector3, queue_size=1)
	#correction_velocities = Vector3()
	#formatOutput(output, correction_velocities)
	#print('correction_velocities[] ', correction_velocities)
	#pub.publish(correction_velocities)
	
	

def formatOutput(output, correction_velocities):
	correction_velocities.x = output[3] # Rotational x
	correction_velocities.y = output[4] # Rotational y
	correction_velocities.z = output[2] # Linear z

# Initialise node
rospy.init_node("pid")

# Subscribe
sub_wrist = rospy.Subscriber("/end_effector/forces_surfaceFrame", WrenchStamped, callback)

# Loop
r = rospy.Rate(20)
time_axis = list(range(500))  #creates a list of range from 1 to 500 

while not rospy.is_shutdown():	# Create a loop that will go until someone stops the program execution
        r.sleep()			# Make sure the publish rate maintains at 5 Hz
	current_len = len(p_list)
	if current_len == 0:
		continue
	if current_len > 500:
		#pid_list = pid_list[500-current_len:]
		p_list = p_list[500-current_len:]
		#i_list = i_list[500-current_len:]
		#d_list = d_list[500-current_len:]
	
	p_ax.plot(time_axis, p_list, color='red')
	#i_ax.plot(time_axis, i_list, color='green')
	#d_ax.plot(time_axis, d_list, color='blue')
	#pid_ax.plot(time_axis, pid_list, color='cyan')

	plt.draw()
	plt.pause(0.001)



