#!/usr/bin/env python
import rospy
import numpy as np
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from geometry_msgs.msg import Vector3
from tf import transformations as trans
from cv_bridge import CvBridge, CvBridgeError
import cv2

import sys


class computer_vision(object):
    """docstring for computer_vision."""

    def __init__(self):
        super(computer_vision, self).__init__()
        rospy.init_node('computer_vision', anonymous=True)
        rospy.loginfo("CV node started")

        self.image_sub = rospy.Subscriber(
            "/cv/resized/image_raw", Image, self.cb)
        self.result = rospy.Publisher(
            '/cv/result/image_raw', Image, queue_size=10)

        self.bridge = CvBridge()
        rospy.spin()

    def cb(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")

        except CvBridgeError as e:
            print(e)

        gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        ret, thresh1 = cv2.threshold(gray_image, 127, 255, cv2.THRESH_BINARY)
        canvas = np.zeros_like(cv_image)
        circles = cv2.HoughCircles(
            thresh1, cv2.HOUGH_GRADIENT, 1, 20, param1=50, param2=30, minRadius=0, maxRadius=0)

        if circles is None:
            return -1

        circles = np.uint16(np.around(circles))

        for i in circles[0, :]:
            cv2.circle(canvas, (i[0], i[1]), i[2], (0, 255, 0), 2)
            cv2.circle(canvas, (i[0], i[1]), 2, (0, 0, 255), 3)

        self.result.publish(
            self.bridge.cv2_to_imgmsg(canvas, "bgr8"))
        # cv2.imshow('detected circles', canvas)


if __name__ == '__main__':
    vo = computer_vision()
    # rospy.spin()
# /camera/image_raw
