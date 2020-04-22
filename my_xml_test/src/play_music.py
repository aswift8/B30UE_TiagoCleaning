#!/usr/bin/env python

import rospy
import roslaunch
from sound_play.libsoundplay import SoundClient

rospy.init_node("play_sound_file", anonymous=True)
sound_client = SoundClient()
rospy.sleep(2)
sound_client.playWave(roslaunch.sys.argv[1])
# sound = sound_client.waveSound(roslaunch.sys.argv[1])
# sound.Play()
rate = rospy.Rate(1)

while not rospy.is_shutdown():
    rate.sleep()

# to shut tiago up
# rosrun sound_play shutup.py
