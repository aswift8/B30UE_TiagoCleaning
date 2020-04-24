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

# Proportional gain
Kp= 0.001

# Integral gain
Ki= 0.1

# Derivative gain
Kd= 0.001

#iterations
i = 0

# 6x6 transformation matrix SPF   
# We use this matrix to describe the shift in reference frame from force sensor frame to
# end-effector contact with surface Frame. We judge it's a shift in 23.5 cm in the +Z direction.
SPF = np.array([[1,0,0,0,0,0],[0,1,0,0,0,0],[0,0,1,0,0,0],[0,-0.235,0,1,0,0],[0.235,0,0,0,1,0],[0,0,0,0,0,1]])

# 6x6 transformation matrix NPF   
# We use this matrix to describe the shift in reference frame from force sensor frame to
# end-effector contact with surface Frame. We judge it's a shift in 23.5 cm in the -Z direction.
NPF = np.array([[1,0,0,0,0,0],[0,1,0,0,0,0],[0,0,1,0,0,0],[0,0.235,0,1,0,0],[-0.235,0,0,0,1,0],[0,0,0,0,0,1]])

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

# 6x1 matrix for desired input, Sfd
Sfd = np.array([[0],[0],[-1],[0],[0],[0]])

# 3x1 matrix for end-effector velocities
end_effector_velocities = np.array([[0],[0],[0]], dtype=float)

def callback(Ff_compensated):

	# 6x1 matrix for actual reading, Sf
	# obtained by multplying SPF with Ff_compensated
    
	Sf = SPF.dot(Ff_compensated)

	pid(Sf)

def pid(Sf):
	# Python doesn't have a do while loop, so we use an I to fork the code instead. 
	# We need this to obtain a last-error for the derivative to work
	global i, error, error_list

	# Callback class start reading from this part  
	if (i==0)

		# Difference between desired input and actual reading
	        error = Sfd - Sf
	     
		i=1
	
	if (i>0):
		
 			last_error = error

	        # Difference between desired input and actual reading
	        error = Sfd - Sf

 	        # Function of .append is to add an item to the end of the list
		
			error_list.append(error)

			if len(error_list) > 500:	# Make this a variable
				error_list = error_list[1:]    # Cut of the first value if list > 500, so 501th become 500th in the list

			# Equations for each P.I.D
			proportional = Kp*error
			
			integral = sum(error_list) * Ki / (len(error_list) * 1.0/50.0) # t = 1/f , TIAGo's frequency runs at 50Hz

			derivative = (Kd*1.0/50.0)*(error - last_error)
			
			# For PID control, we add the P + I + D = output 
			output = proportional + integral + derivative 

			p_list.append(proportional)
			i_list.append(integral)
			d_list.append(derivative)
			pid_list.append(output)

			velocitiesPublisher(output)


	
#publish to "/end_effector/correction_velocities" (inverse differential kinematics section)
def velocitiesPublisher(output):
	
    # 6x1 matrix for end_effector work-space velocities
	# Obtained by multiplying NPF with output
	end_effector_velocities = NPS.dot(output)

	# Publishers
	pub = rospy.Publisher('/end_effector/correction_velocity', Vector3, queue_size=1)

	correction_velocities = Vector3()
	formatOutput(end_effector_velocities, correction_velocities)
	pub.publish(correction_velocities)
	

def formatOutput(end_effector_velocities, correction_velocities):

	# Will only take value from force in z-direction and torque in x and y directions
	# The values are inversed due to opposite direction of force 
	# as what we received from the force/torque sensor readings are forces applied from the surface towards the end-effector
	# But we want work-space velocities to apply from end-effector towards the surface
	correction_velocities.x = -end_effector[3] # Rotational x
	correction_velocities.y = -end_effector[4] # Rotational y
	correction_velocities.z = -end_effector[2] # Linear z

# Initialise node
rospy.init_node("pid")

# Subscriber
Ff_compensated = rospy.Subscriber("/wrist_ft_compensated", WrenchStamped, callback)

# Loop
r = rospy.Rate(20)
time_axis = list(range(500))  # Creates a list of range from 1 to 500 

while not rospy.is_shutdown():	# Creates a loop that will go until someone stops the program execution
  
    r.sleep()			        # Make sure the publish rate maintains at 5 Hz
	current_len = len(p_list)

	if current_len == 0:
		continue
	if current_len > 500:						# If current length is more than 500, the length will be cut
		pid_list = pid_list[500-current_len:]   # so that the current length is always equal or less than 500
		p_list = p_list[500-current_len:]
		i_list = i_list[500-current_len:]
		d_list = d_list[500-current_len:]

    # Actual plotting of the graphs	
	p_ax.plot(time_axis, p_list, color='red')
	i_ax.plot(time_axis, i_list, color='green')
	d_ax.plot(time_axis, d_list, color='blue')
	pid_ax.plot(time_axis, pid_list, color='cyan')

	plt.draw()
	plt.pause(0.001)



