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

from dynamixel_msgs.msg import *
from std_msgs.msg import Float64
from dynamixel_msgs.msg import MotorStateList
import numpy as np
import random

def sleep(t):
    try:
        rospy.sleep(t)
    except:
        pass

wave_path = os.path.dirname(os.path.abspath(__file__)) + "/../sounds/camera.wav"
next_time = None
kyorokyoro_next_time = None

j1_servo_on = False
j2_servo_on = False
motor_limit = 30
def servo_state_callback1(msg):
    global j1_servo_on
    if msg.motor_ids[0] >= motor_limit:
        j1_servo_on = False
    else:
        j1_servo_on = True
            
def servo_state_callback2(msg):
    global j2_servo_on
    rospy.loginfo("{} {}".format(j2_servo_on, motor_limit))
    if msg.motor_ids[0] >= motor_limit:
        j2_servo_on = False
    else:
        j2_servo_on = True

def kyorokyoro_eye():
    # global j1_servo_on
    # if j1_servo_on is False:
    #     return
    pub1 = rospy.Publisher('/arm_j1_controller/command', Float64, queue_size=10)
    pub1.publish(Float64(np.deg2rad(-50)))
    rospy.sleep(1.0)
    pub1.publish(Float64(np.deg2rad(50)))
    rospy.sleep(1.0)    
    pub1.publish(Float64(np.deg2rad(0)))

def close_eye():
    pub2 = rospy.Publisher('/arm_j2_controller/command', Float64, queue_size=10)
    pub2.publish(Float64(np.deg2rad(-120)))
    rospy.sleep(0.3)
    pub2.publish(Float64(np.deg2rad(-100)))
    rospy.sleep(0.3)    
    pub2.publish(Float64(np.deg2rad(-120)))

def main():
    rospy.init_node('eys_see', anonymous = True)
    global next_time
    global kyorokyoro_next_time
    next_time = rospy.Time.now()
    kyorokyoro_next_time = rospy.Time.now()
    
    soundhandle = SoundClient()
    close_eye_publisher = rospy.Publisher("close_eye", Int32, queue_size=100)
    save_image_publisher = rospy.Publisher("save_trigger", Empty, queue_size=100)

    def continuous_time_callback(msg):
        # global j2_servo_on, j1_servo_on
        # if j2_servo_on is False:
        #     return

        global next_time
        global kyorokyoro_next_time
        continuous_time = msg.data
        if rospy.Time.now() < next_time:
            return
        else:
            next_time = rospy.Time.now() + rospy.Duration(1.0)
        if continuous_time >= 0.5: # kao no renzoku ninshiki jikan
            next_time = rospy.Time.now() + rospy.Duration(4.0)
            save_image_publisher.publish()
            close_eye_publisher.publish(1)
            close_eye()
            soundhandle.playWave(wave_path)
        elif kyorokyoro_next_time <= rospy.Time.now():
            kyorokyoro_next_time = rospy.Time.now() + rospy.Duration(random.randint(30, 60))
            kyorokyoro_eye()
            
    rospy.Subscriber("/continuous_time", Float32, continuous_time_callback)
    # rospy.Subscriber("/arm_j1_controller/state", JointState, servo_state_callback1)
    # rospy.Subscriber("/arm_j2_controller/state", JointState, servo_state_callback2)
    r = rospy.Rate(1)
    rospy.spin()
    soundhandle.stopAll()


if __name__ == "__main__":
    main()
