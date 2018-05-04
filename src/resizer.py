#!/usr/bin/env python
import rospy
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import Float64

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

        self.image_sub = rospy.Subscriber("/camera/image_raw", Image, self.cb)
        self.scale_sub = rospy.Subscriber("/cv/scale", Float64, self.cb_scale)

        self.result_pub = rospy.Publisher(
            '/cv/resized/image_raw', Image, queue_size=10)

        self.bridge = CvBridge()
        self.rate = rospy.Rate(50)  # 10hz
        self.image = None
        self.scale = None

        while not rospy.is_shutdown():
            if self.scale is None:
                self.scale = 0.5

            if self.image is not None:
                rospy.loginfo("Current scale %f", self.scale)
                self.resized_image = cv2.resize(
                    self.image, None, fx=self.scale, fy=self.scale,
                    interpolation=cv2.INTER_CUBIC)

                self.result_pub.publish(
                    self.bridge.cv2_to_imgmsg(self.resized_image, "bgr8"))
                self.rate.sleep()

    def cb(self, data):
        try:
            self.image = self.bridge.imgmsg_to_cv2(data, "bgr8")

        except CvBridgeError as e:
            print(e)

    def cb_scale(self, data):
        try:
            self.scale = data.data

        except CvBridgeError as e:
            print(e)


if __name__ == '__main__':
    vo = computer_vision()
    # rospy.spin()
# /camera/image_raw
