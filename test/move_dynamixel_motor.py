#!/usr/bin/env python
# -*- coding:utf-8 -*-

import rospy
from dynamixel_msgs.msg import *
from std_msgs.msg import Float64
import numpy as np

pub2 = None
def close_eye():
    global pub2

    pub2.publish(Float64(np.deg2rad(100)))
    rospy.sleep(0.3)
    pub2.publish(Float64(np.deg2rad(50)))
    rospy.sleep(0.3)
    pub2.publish(Float64(np.deg2rad(100)))

def main():
    global pub2
    rospy.init_node("test_dynamixel_motor")
    r = rospy.Rate(100)

    pub1 = rospy.Publisher('/arm_j1_controller/command', Float64, queue_size=10)
    pub2 = rospy.Publisher('/arm_j2_controller/command', Float64, queue_size=10)
    rospy.sleep(3.0)

    while True:
        angle = float(input())
        pub1.publish(Float64(np.deg2rad(angle)))

if __name__ == "__main__":
    main()
