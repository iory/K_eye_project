#!/usr/bin/env python
# -*- coding:utf-8 -*-

import rospy
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Empty

import os
import cv2

cascade_path = "/opt/ros/" + os.environ.get("ROS_DISTRO") + "/share/OpenCV/haarcascades/haarcascade_frontalface_alt.xml"
color = (255, 255, 255)

class face_position_publisher:
    def __init__(self):
        self.image_pub = rospy.Publisher("face_position", Point, queue_size=10)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("camera_node/image_raw",Image, self.callback, queue_size=10)
        self.shutter_sub = rospy.Subscriber("/close_eye", Empty,self.save_callback, queue_size=10)
        self.pub = rospy.Publisher('face/position', Point, queue_size=10)
        self.image_publisher = rospy.Publisher('face/recognized', Image, queue_size=10)
        self.cv_image = None
        self.cnt = 0
    def callback(self, data):
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            image_gray = cv2.cvtColor(self.cv_image, cv2.cv.CV_BGR2GRAY)
            cascade = cv2.CascadeClassifier(cascade_path)
            facerect = cascade.detectMultiScale(image_gray, scaleFactor=1.1, minNeighbors=10, minSize=(100, 100))
            if len(facerect) > 0:
                for rect in facerect:
                    cv2.rectangle(self.cv_image, tuple(rect[0:2]),tuple(rect[0:2]+rect[2:4]), color, thickness=2)
                    rospy.loginfo(rect)
                    self.pub.publish(Point(x=rect[0:1]+rect[1:2]/2,
                                           y=rect[2:3]+rect[3:]/2,
                                           z=0))
            # self.image_publisher.publish(self.bridge.cv2_to_imgmsg(cv_image, 'bgr8'))
        except CvBridgeError, e:
            print(e)
    def save_callback(self, msg):
        rospy.loginfo("Saving Image")
        cv2.imwrite("/tmp/face_{0:04}.jpg".format(self.cnt), self.cv_image)
        self.cnt += 1
        self.cnt %= 1000

def main():
    fcp = face_position_publisher()
    rospy.init_node('face_position_publisher')
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("shutting down")
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
