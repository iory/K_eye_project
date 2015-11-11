#!/usr/bin/env python
# -*- coding:utf-8 -*-

import os
import rospy
import sys

from sound_play.msg import SoundRequest
from sound_play.libsoundplay import SoundClient
from std_msgs.msg import Int32

def sleep(t):
    try:
        rospy.sleep(t)
    except:
        pass

def main():
    rospy.init_node('eys_move', anonymous = True)
    soundhandle = SoundClient()
    close_eye_publisher = rospy.Publisher("close_eye", Int32, queue_size=10)
    rospy.sleep(1)
    soundhandle.stopAll()

    close_eye_publisher.publish(Int32(data=2))

    wave_path = os.path.dirname(os.path.abspath(__file__)) + "/../sounds/camera.wav"
    soundhandle.playWave(wave_path)
    sleep(2)

if __name__ == "__main__":
    main()
