#! /usr/bin/env python2.7

from inputs import devices, get_gamepad
import threading
import Tkinter as tk
import ttk as ttk
import numpy as np
import rospy
from geometry_msgs.msg import Twist
from rospy.rostime import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import JointTrajectoryControllerState


rospy.init_node("controller_manual")

torso_current = None		# [height]
head_current = None			# [x, y]
arm_current = None			# np.array([1, 2, 3, 4, 5, 6, 7])


button = [0] * 11
bA, bB, bX, bY, bSTART, bMODE, bSELECT, bLB, bRB, bLT, bRT = 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10
hat = [0, 0]
hX, hY = 0, 1
axis = [0] * 6
aLX, aLY, aLZ, aRX, aRY, aRZ = 0, 1, 2, 3, 4, 5

running = True


def close_window():
	global running, root, x
	running = False			# - should let everything close?
	print("Closing window")
	root.destroy()


def manage_gamepad_inputs():
	global running
	global button, bA, bB, bX, bY, bSTART, bSELECT, bLB, bRB, bLT, bRT
	global hat, hX, hY
	global axis, aLX, aLY, aLZ, aRX, aRY, aRZ
	while running:
		events = get_gamepad()
		for event in events:
			# print(event.ev_type, event.code, event.state)
			if event.code == 'ABS_X':				# JOYSTICKS		16-bit integer
				axis[aLX] = event.state / 32768.0				  # -32768 -> 32767
			elif event.code == 'ABS_RX':
				axis[aRX] = event.state / 32768.0
			elif event.code == 'ABS_Y':
				axis[aLY] = event.state / 32768.0
			elif event.code == 'ABS_RY':
				axis[aRY] = event.state / 32768.0
			elif event.code == 'ABS_Z':				# TRIGGERS			8-bit integer
				axis[aLZ] = event.state / 255.0
			elif event.code == 'ABS_RZ':
				axis[aRZ] = event.state / 255.0
			elif event.code == 'ABS_HAT0X':			# D-PAD				-1, 0, 1
				hat[hX] = event.state
			elif event.code == 'ABS_HAT0Y':
				hat[hY] = event.state
			elif event.code == 'BTN_THUMBL':		# THUMB BUTTONS		0, 1
				button[bLT] = event.state
			elif event.code == 'BTN_THUMBR':
				button[bRT] = event.state
			elif event.code == 'BTN_TL':			# BUMPER BUTTONS	0, 1
				button[bLB] = event.state
			elif event.code == 'BTN_TR':
				button[bRB] = event.state
			elif event.code == 'BTN_START':			# START			0, 1
				button[bSTART] = event.state
			elif event.code == 'BTN_MODE':			# MODE			0, 1
				button[bMODE] = event.state
			elif event.code == 'BTN_SELECT':		# SELECT		0, 1
				button[bSELECT] = event.state
			elif event.code == 'BTN_NORTH':			# BUTTONS		0, 1
				button[bX] = event.state
			elif event.code == 'BTN_SOUTH':
				button[bA] = event.state
			elif event.code == 'BTN_WEST':			# Ubuntu Kinetic 16.04 - NORTH and WEST swapped
				button[bY] = event.state
			elif event.code == 'BTN_EAST':
				button[bB] = event.state
			elif event.ev_type != "Sync":			# Other (non-sync) event
				print("Other button detected: ", event.ev_type, event.code, event.state)


# Torso state callback
def cb_torso_state(msg):
	global torso_current
	torso_current = msg.actual.positions


# Head state callback
def cb_head_state(msg):
	global head_current
	head_current = msg.actual.positions


# Arm state callbackx
def cb_arm_state(msg):
	global arm_current
	arm_current = np.array(msg.actual.positions)


def main():
	global running, root, x
	global button, bA, bB, bX, bY, bSTART, bSELECT, bLB, bRB, bLT, bRT
	global hat, hX, hY
	global axis, aLX, aLY, aLZ, aRX, aRY, aRZ
	global torso_current, head_current, arm_current
	print("The following devices are detected:")
	for device in devices:
		print("  " + str(device))

	print "Initialising publishers and subscribers"

	freq = 20
	r = rospy.Rate(freq)

	# ROS publishers
	duration = Duration(nsecs=1e9/float(freq)*3.0)
	base_pub = rospy.Publisher('/mobile_base_controller/cmd_vel', Twist, queue_size=1)
	base_msg = Twist()
	torso_pub = rospy.Publisher('/torso_controller/command', JointTrajectory, queue_size=1)
	torso_msg = JointTrajectory()
	torso_msg.joint_names = ["torso_lift_joint"]
	torso_point = JointTrajectoryPoint()
	torso_point.time_from_start = duration
	torso_msg.points = [torso_point]
	head_pub = rospy.Publisher('/head_controller/command', JointTrajectory, queue_size=1)
	head_msg = JointTrajectory()
	head_msg.joint_names = ["head_1_joint", "head_2_joint"]
	head_point = JointTrajectoryPoint()
	head_point.time_from_start = duration
	head_msg.points = [head_point]
	arm_pub = rospy.Publisher('/arm_controller/safe_command', JointTrajectory, queue_size=1)
	arm_msg = JointTrajectory()
	arm_msg.joint_names = ["arm_1_joint", "arm_2_joint", "arm_3_joint", "arm_4_joint", "arm_5_joint", "arm_6_joint", "arm_7_joint"]
	arm_point = JointTrajectoryPoint()
	arm_point.time_from_start = duration
	arm_msg.points = [arm_point]
	# Subscribers
	torso_sub = rospy.Subscriber('/torso_controller/state', JointTrajectoryControllerState, cb_torso_state)
	head_sub = rospy.Subscriber('/head_controller/state', JointTrajectoryControllerState, cb_head_state)
	arm_sub = rospy.Subscriber('/arm_controller/state', JointTrajectoryControllerState, cb_arm_state)

	print "Building gui"

	# Canvas constants
	canvas_size = 101
	canvas_centre = 51
	joy_move = 30
	joy_radius = 20
	dot_radius = 3
	circle_radius = 49
	# Colours
	col_button_down = "#505050"
	col_button_up = "#b0b0b0"
	# Create tkinter gui
	root = tk.Tk()
	root.title("Inputs")
	root.protocol("WM_DELETE_WINDOW", close_window)
	f_all = tk.Frame(master=root)
	# Left trigger + bumper
	f_trig_left = tk.Frame(master=f_all)
	f_trig_left.grid(column=0, row=0)
	pb_trig_left = ttk.Progressbar(master=f_trig_left, maximum=1)
	pb_trig_left.grid(column=0, row=0)
	l_bump_left = tk.Label(master=f_trig_left, bg=col_button_up)
	l_bump_left.grid(column=0, row=1, sticky="nsew")
	# Right trigger + bumper
	f_trig_right = tk.Frame(master=f_all)
	f_trig_right.grid(column=2, row=0)
	pb_trig_right = ttk.Progressbar(master=f_trig_right, maximum=1)
	pb_trig_right.grid(column=0, row=0)
	l_bump_right = tk.Label(master=f_trig_right, bg=col_button_up)
	l_bump_right.grid(column=0, row=1, sticky="nsew")
	# D-pad
	f_dpad = tk.Frame(master=f_all)
	f_dpad.grid(column=0, row=2)
	f_dpad.grid_columnconfigure(0, minsize=25)
	f_dpad.grid_columnconfigure(1, minsize=25)
	f_dpad.grid_columnconfigure(2, minsize=25)
	l_dpad_u = tk.Label(master=f_dpad, bg=col_button_up)
	l_dpad_u.grid(column=1, row=0, sticky="nsew")
	l_dpad_d = tk.Label(master=f_dpad, bg=col_button_up)
	l_dpad_d.grid(column=1, row=2, sticky="nsew")
	l_dpad_l = tk.Label(master=f_dpad, bg=col_button_up)
	l_dpad_l.grid(column=0, row=1, sticky="nsew")
	l_dpad_r = tk.Label(master=f_dpad, bg=col_button_up)
	l_dpad_r.grid(column=2, row=1, sticky="nsew")
	# ABXY buttons
	f_buttons = tk.Frame(master=f_all)
	f_buttons.grid(column=2, row=1)
	f_buttons.grid_columnconfigure(0, minsize=25)
	f_buttons.grid_columnconfigure(1, minsize=25)
	f_buttons.grid_columnconfigure(2, minsize=25)
	l_abxy_y = tk.Label(master=f_buttons, bg=col_button_up)
	l_abxy_y.grid(column=1, row=0, sticky="nsew")
	l_abxy_a = tk.Label(master=f_buttons, bg=col_button_up)
	l_abxy_a.grid(column=1, row=2, sticky="nsew")
	l_abxy_x = tk.Label(master=f_buttons, bg=col_button_up)
	l_abxy_x.grid(column=0, row=1, sticky="nsew")
	l_abxy_b = tk.Label(master=f_buttons, bg=col_button_up)
	l_abxy_b.grid(column=2, row=1, sticky="nsew")
	# Start/Select buttons - also Mode
	f_ss = tk.Frame(master=f_all)
	f_ss.grid(column=1, row=1)
	f_ss.grid_columnconfigure(0, minsize=25)
	f_ss.grid_columnconfigure(1, minsize=25)
	f_ss.grid_columnconfigure(2, minsize=25)
	l_select = tk.Label(master=f_ss, bg=col_button_up)
	l_select.grid(column=0, row=0, sticky="nsew")
	l_mode = tk.Label(master=f_ss, bg=col_button_up)
	l_mode.grid(column=1, row=0, sticky="nsew")
	l_start = tk.Label(master=f_ss, bg=col_button_up)
	l_start.grid(column=2, row=0, sticky="nsew")
	# Joystick setup constants - values from above
	joy_min = canvas_centre-joy_radius
	joy_max = canvas_centre+joy_radius
	dot_min = canvas_centre-dot_radius
	dot_max = canvas_centre+dot_radius
	circle_min = canvas_centre-circle_radius
	circle_max = canvas_centre+circle_radius
	# Left joystick canvas
	c_joy_left = tk.Canvas(master=f_all, width=canvas_size, height=canvas_size, background="#f0f0f0")
	c_joy_left.grid(column=0, row=1)
	c_joy_left.create_oval(circle_min, circle_min, circle_max, circle_max, fill="#ffffff")
	stick_left = c_joy_left.create_oval(joy_min, joy_min, joy_max, joy_max,	fill=col_button_down)
	c_joy_left.create_oval(dot_min, dot_min, dot_max, dot_max, fill="red")
	# Right joystick canvas
	c_joy_right = tk.Canvas(master=f_all, width=canvas_size, height=canvas_size, background="#f0f0f0")
	c_joy_right.grid(column=2, row=2)
	c_joy_right.create_oval(circle_min, circle_min, circle_max, circle_max, fill="#ffffff")
	stick_right = c_joy_right.create_oval(joy_min, joy_min, joy_max, joy_max, fill=col_button_down)
	c_joy_right.create_oval(dot_min, dot_min, dot_max, dot_max, fill="red")
	# Pack top frame
	f_all.grid()

	# Start gamepad input thread - should be option for daemon?
	print "Starting gamepad input thread"
	x = threading.Thread(target=manage_gamepad_inputs)
	x.start()
	print "Starting control loop"

	control_mode = "move_and_look"

	while running:
		r.sleep()
		root.update_idletasks()
		root.update()
		# Joysticks
		xl = canvas_centre + joy_move*axis[aLX]
		yl = canvas_centre + joy_move*axis[aLY]
		# If window closes with [X] this causes exception
		c_joy_left.coords(stick_left, xl-joy_radius, yl-joy_radius, xl+joy_radius, yl+joy_radius)
		l_fill = col_button_up if not button[bLT] else col_button_down
		c_joy_left.itemconfigure(stick_left, fill=l_fill)
		xr = canvas_centre + joy_move*axis[aRX]
		yr = canvas_centre + joy_move*axis[aRY]
		c_joy_right.coords(stick_right, xr-joy_radius, yr-joy_radius, xr+joy_radius, yr+joy_radius)
		r_fill = col_button_up if not button[bRT] else col_button_down
		c_joy_right.itemconfigure(stick_right, fill=r_fill)
		# Triggers
		pb_trig_left["value"] = axis[aLZ]
		pb_trig_right["value"] = axis[aRZ]
		# D-Pad
		l_dpad_u["bg"] = col_button_up if hat[hY] > -1 else col_button_down
		l_dpad_l["bg"] = col_button_up if hat[hX] > -1 else col_button_down
		l_dpad_d["bg"] = col_button_up if hat[hY] < 1 else col_button_down
		l_dpad_r["bg"] = col_button_up if hat[hX] < 1 else col_button_down
		# Bumpers
		l_bump_left["bg"] = col_button_down if button[bLB] else col_button_up
		l_bump_right["bg"] = col_button_down if button[bRB] else col_button_up
		# Start/Select
		l_start["bg"] = col_button_down if button[bSTART] else col_button_up
		l_mode["bg"] = col_button_down if button[bMODE] else col_button_up
		l_select["bg"] = col_button_down if button[bSELECT] else col_button_up
		# ABXY
		l_abxy_a["bg"] = col_button_down if button[bA] else col_button_up
		l_abxy_b["bg"] = col_button_down if button[bB] else col_button_up
		l_abxy_x["bg"] = col_button_down if button[bX] else col_button_up
		l_abxy_y["bg"] = col_button_down if button[bY] else col_button_up

		# Change mode
		if button[bSTART]:
			control_mode = "move_and_look"
		elif button[bMODE]:
			control_mode = "arm_1"

		if control_mode is "move_and_look":		# Control move base and head
			move_x = -axis[aLY]
			move_z = -axis[aLX]
			base_msg.linear.x = 0 if abs(move_x) < 0.16 else move_x
			base_msg.angular.z = 0 if abs(move_z) < 0.16 else move_z
			base_pub.publish(base_msg)

			if torso_current is not None :
				torso_vel = (axis[aLZ] - axis[aRZ]) / freq / 2
				torso_new = torso_current[0] + torso_vel/2.0
				torso_point.positions = [torso_new]
				torso_msg.points = [torso_point]
				torso_pub.publish(torso_msg)
			if head_current is not None:
				if abs(axis[aRX]) > 0.2 or abs(axis[aRY]) > 0.2:
					head_vel_x = -axis[aRX] / freq * 5
					head_vel_y = -axis[aRY] / freq * 5
					head_new_x = head_current[0] + head_vel_x/2.0
					head_new_y = head_current[1] + head_vel_y/2.0
					head_point.positions = [head_new_x, head_new_y]
				else:
					head_point.positions = head_current
				head_msg.points = [head_point]
				head_pub.publish(head_msg)
		elif control_mode is "arm_1":			# Control arm joints 1-4, 6
			new_vels = np.array([-axis[aLX], -axis[aLY], axis[aLZ] - axis[aRZ], axis[aRY], 0, axis[aRX], 0], dtype=float)
			nv_use = np.greater(np.absolute(new_vels), np.array([0.2]*7, dtype=float))
			new_vels = new_vels * nv_use / freq
			new_pos = arm_current + new_vels/2.0 * 5
			arm_point.positions = new_pos.tolist()
			arm_msg.points = [arm_point]
			arm_pub.publish(arm_msg)


if __name__ == '__main__':
	main()
