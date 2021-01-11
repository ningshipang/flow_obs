#!/usr/bin/env python
#!coding=utf-8

import rospy
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2

def callback(data):
    global bridge
    cv_img = bridge.imgmsg_to_cv2(data, "bgr8")
    cv2.imshow("frame" , cv_img)
    cv2.waitKey(1)


def display():
    rospy.init_node('camera_display', anonymous=True)
 
    # make a video_object and init the video object
    global count,bridge
    bridge = CvBridge()
    rospy.Subscriber('/camera/left', Image, callback)
    rospy.spin()

if __name__ == '__main__':
    display()