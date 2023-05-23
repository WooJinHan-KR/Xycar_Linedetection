#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import cv2
import rospy
import numpy as np

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

bridge = CvBridge()
cv_image = np.empty(shape=[0])

def img_callback(data):
    global cv_image
    img = np.frombuffer(data.data, dtype=np.uint8).reshape(data.height, data.width, -1)
    cv_image = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)


rospy.init_node('depth_cam_view', anonymous=True)
rospy.Subscriber("/camera/color/image_raw/", Image, img_callback)

while not rospy.is_shutdown():

    if cv_image.size != (640*480*3):
        continue

    cv2.imshow("Depth Cam Viewer", cv_image)
    cv2.waitKey(1)

    cv_image = np.empty(shape=[0])

