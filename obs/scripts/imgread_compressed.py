#!/usr/bin/env python
#!coding=utf-8

import rospy
import numpy as np
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError
import cv2

def callback(msg):
    np_arr = np.fromstring(msg.data, np.uint8)
    image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
    global bridge
    cv2.imshow("frame" , image)
    cv2.waitKey(1)


def display():
    rospy.init_node('camera_display', anonymous=True)

    # make a video_object and init the video object
    global bridge
    bridge = CvBridge()
    rospy.Subscriber('/camera/left/compressed', CompressedImage, callback)
    rospy.spin()

if __name__ == '__main__':
    display()