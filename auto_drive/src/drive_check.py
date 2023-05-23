#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, rospkg
import numpy as np
import cv2, random, math
from cv_bridge import CvBridge
from xycar_msgs.msg import xycar_motor
from sensor_msgs.msg import Image

import sys
import os
import signal


def start():

    rospy.init_node('auto_drive')
    pub = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)

    print "---------- Xycar A2 v1.0 ----------"
    rospy.sleep(2)
    i, gain = 0, 1
    while True:
        msg = xycar_motor()
        msg.angle = 0

        if abs(i) >= 50:
            i = 0
            gain *= -1

        i += 10
        msg.angle = i * gain
        msg.speed = 10

        pub.publish(msg)
        print("angle : ", msg.angle," speed : ", msg.speed)

        rospy.sleep(2)

if __name__ == '__main__':
    start()


