#! /usr/bin/env python2.7

"""
Similar to wrist graph, but for a Float32MultiArray with the initial value being timestamp (secs)
Initial plan - use for forward kinematic position
"""

# ROS imports
import rospy
from std_msgs.msg import Float32MultiArray
# Other imports
import matplotlib.pyplot as plt
import numpy as np


topic_name = rospy.get_param("readingtopic")
graph_count = int(rospy.get_param("graphcount"))

# Time stamp for program start
time_start = False
secs_last = -1
has_printed = False

do_shutdown = False

fig = plt.figure(figsize=(23,5))
fig.canvas.set_window_title(topic_name + " readings")

ts = []
vals = [[] for i in range(graph_count)]
axes = []
for i in range(graph_count):
	axes.append(fig.add_subplot(1, graph_count, i+1))
	axes[i].title.set_text(str(i+1))

# Callback method
def cb_in(msg):
	global time_start, secs_last, ts, vals, time_to_show, has_printed
	if not time_start:
		time_start = msg.data[0]
	time_msg = msg.data[0] - time_start
	if time_to_show * 1.5 < time_msg:
		return
	ts.append(float(time_msg))
	for i in range(graph_count):
		vals[i].append(msg.data[i+1])
	if time_msg > time_to_show and has_printed == False:
		has_printed = True
		out = str(time_msg)
		for i in range(graph_count):
			out += '\t' + "{:.3f}".format(vals[i][-1])
		print out


def crop_lists():
	# List crop call happens when the first data value is approx. 2* x-axis length
	# Start search for first relevant time at the middle of the time data
	global time_to_show, ts, xs, ys, zs, dxs, dys, dzs
	time_latest = ts[-1]
	time_earliest = time_latest - time_to_show
	index_earliest = len(ts) / 2			# Start at middle
	while ts[index_earliest] < time_earliest:
		index_earliest += 1
	while ts[index_earliest] > time_earliest:
		index_earliest -= 1
	# Remove all unnecessary data points
	for i in range(graph_count):
		vals[i] = vals[i][index_earliest:]
	ts = ts[index_earliest:]

def handle_close(evt):
	global do_shutdown
	do_shutdown = True


time_to_show = 1.2	# Was previously used for auto-updating the graph, currently acts as x-axis limit
ylim = (0,1.2)
xlim = (0,time_to_show)

# Set up graph
fig.canvas.mpl_connect('close_event', handle_close)
plt.ion()
plt.show()
for i in range(graph_count):
	axes[i].set_xlim(*xlim)
	axes[i].set_ylim(*ylim)
# Initialise node
rospy.init_node("multi_graph", disable_signals=True)
# Subscribe
sub_v3 = rospy.Subscriber(topic_name, Float32MultiArray, cb_in)

#vals_to_display = 100	# - fix
last_time_crop = 0

# Loop
r = rospy.Rate(5)
has_last_drawn = False
while not rospy.is_shutdown():	
	if len(ts) == 0:			# Don't update if nothing to display
		r.sleep()
		continue
	if not has_last_drawn:
		try:
			for i in range(graph_count):
				axes[i].clear()
				axes[i].set_xlim(*xlim)
				axes[i].set_ylim(*ylim)
				axes[i].plot(ts, vals[i], color='blue')
			if has_printed:
				has_last_drawn = True
		except ValueError as e:
			print "VALUERROR OCCURRED"
	plt.draw()
	plt.pause(0.001)
	r.sleep()
	if do_shutdown:
		rospy.signal_shutdown("Window closed")
