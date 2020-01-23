#! /usr/bin/env python2.7

# ROS imports
import rospy
from geometry_msgs.msg import WrenchStamped
# Other imports
import matplotlib.pyplot as plt
import numpy as np


# Time stamp for first time received
# Updated with first callback
time_start = False
secs_last = -1

fig = plt.figure()
axfx = fig.add_subplot(2,3,1)
axfy = fig.add_subplot(2,3,2)
axfz = fig.add_subplot(2,3,3)
axtx = fig.add_subplot(2,3,4)
axty = fig.add_subplot(2,3,5)
axtz = fig.add_subplot(2,3,6)

new_vals = 0
ts = []
fxs = []
fys = []
fzs = []
txs = []
tys = []
tzs = []


# Callback method
def cb_wrist(msg):
	global time_start, secs_last, msg_recent, ts, fxs, fys, fzs, txs, tys, tzs, new_vals
	if not time_start:
		time_start = msg.header.stamp
	f = msg.wrench.force
	t = msg.wrench.torque
	time = msg.header.stamp - time_start
	time_float = time.secs + float(time.nsecs)/1e9
	#print("F: {},{},{}\tT: {},{},{}\tTime: {}".format(f.x, f.y, f.z, t.x, t.y, t.z, time_float))
	ts.append(time_float)
	fxs.append(f.x)
	fys.append(f.y)
	fzs.append(f.z)
	txs.append(t.x)
	tys.append(t.y)
	tzs.append(t.z)
	new_vals += 1


# Set up graph
plt.ion()
plt.show()
# Initialise node
rospy.init_node("wrist_graph")
# Subscribe
sub_wrist = rospy.Subscriber("/wrist_ft", WrenchStamped, cb_wrist)

#vals_to_display = 100	# - fix
time_to_show = 10	# 10s

# Loop
r = rospy.Rate(5)
while not rospy.is_shutdown():
	entries = len(ts)
	if entries == 0:
		r.sleep()
		continue
	if entries > new_vals:
		# Remove old vals - matplotlib keeps them anyway(?)
		ts = ts[entries - new_vals:]
		fxs = fxs[entries - new_vals:]
		fys = fys[entries - new_vals:]
		fzs = fzs[entries - new_vals:]
		txs = txs[entries - new_vals:]
		tys = tys[entries - new_vals:]
		tzs = tzs[entries - new_vals:]
	axfx.plot(ts, fxs, color='red')
	axfy.plot(ts, fys, color='green')
	axfz.plot(ts, fzs, color='blue')
	axtx.plot(ts, txs, color='red')
	axty.plot(ts, tys, color='green')
	axtz.plot(ts, tzs, color='blue')
	t_r = ts[-1]
	t_l = max(t_r - time_to_show, 0)
	axfx.set_xlim(t_l, t_r)
	axfy.set_xlim(t_l, t_r)
	axfz.set_xlim(t_l, t_r)
	axtx.set_xlim(t_l, t_r)
	axty.set_xlim(t_l, t_r)
	axtz.set_xlim(t_l, t_r)
	plt.draw()
	plt.pause(0.001)

	r.sleep()
