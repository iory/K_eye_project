#!/usr/bin/env python
# -*- coding:utf-8 -*-

import os
import rospy
import sys

from sound_play.msg import SoundRequest
from sound_play.libsoundplay import SoundClient
from std_msgs.msg import Int32
from std_msgs.msg import Float32
from std_msgs.msg import Empty

def sleep(t):
    try:
        rospy.sleep(t)
    except:
        pass

wave_path = os.path.dirname(os.path.abspath(__file__)) + "/../sounds/camera.wav"
next_time = None

def main():
    rospy.init_node('eys_see', anonymous = True)
    global next_time
    next_time = rospy.Time.now()
    soundhandle = SoundClient()
    close_eye_publisher = rospy.Publisher("close_eye", Int32, queue_size=100)
    save_image_publisher = rospy.Publisher("save_trigger", Empty, queue_size=100)

    def continuous_time_callback(msg):
        global next_time
        continuous_time = msg.data
        if rospy.Time.now() < next_time:
            return
        else:
            next_time = rospy.Time.now() + rospy.Duration(1.0)
        if continuous_time >= 1.0:
            next_time = rospy.Time.now() + rospy.Duration(4.0)
            save_image_publisher.publish()
            close_eye_publisher.publish(1)
            soundhandle.playWave(wave_path)

    rospy.Subscriber("/continuous_time", Float32, continuous_time_callback)
    r = rospy.Rate(1)
    rospy.spin()
    soundhandle.stopAll()


if __name__ == "__main__":
    main()
