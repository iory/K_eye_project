#!/usr/bin/env python
# -*- coding:utf-8 -*-

import rospy
from dynamixel_msgs.msg import *
from std_msgs.msg import Float64

def main():
    rospy.init_node("test_dynamixel_motor")
    r = rospy.Rate(10)

    pub1 = rospy.Publisher('/arm_j1_controller/command', Float64, queue_size=10)
    pub2 = rospy.Publisher('/arm_j2_controller/command', Float64, queue_size=10)

    while True:
        angle = float(input())
        pub1.publish(Float64(angle))

if __name__ == "__main__":
    main()
