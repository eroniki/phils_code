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
        self.scale_pub = rospy.Publisher('/cv/scale', Float64, queue_size=10)
        self.now = None
        self.last = rospy.get_time()
        self.rate = rospy.Rate(50)  # 10hz

        while not rospy.is_shutdown():
            if self.now is not None:
                self.scale = Float64()
                # self.scale = np.random.random()
                self.scale.data = self.create_scale(self.now, self.last)
                self.scale_pub.publish(self.scale)
                self.rate.sleep()

    def create_scale(self, tnow, tlast):
        # heyyo = 1 / (48 * (tnow - tlast)) + 0.1
        dt = tnow - tlast
        heyyo = 0.5
        rospy.loginfo("scale factor %f", heyyo)
        return heyyo

    def cb(self, data):
        try:
            self.now = rospy.get_time()
        except CvBridgeError as e:
            print(e)


if __name__ == '__main__':
    vo = computer_vision()
    # rospy.spin()
# /camera/image_raw
