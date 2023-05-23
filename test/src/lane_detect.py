#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import cv2
import time
import numpy as np
from std_msgs.msg import Float64
from xycar_msgs.msg import xycar_motor
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import sys
import os
import signal
#cap = cv2.VideoCapture(0)

def signal_handler(sig, frame):
    os.system('killall -9 python rosout')
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

bridge = CvBridge()
cv_image = np.empty(shape=[0])
ack_publisher = None
def img_callback(data):
    global cv_image
    try:
      cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

rospy.sleep(3)
bridge = CvBridge()
image_sub = rospy.Subscriber("/usb_cam/image_raw/",Image,img_callback)
rospy.init_node('lane_detection', anonymous=True)

# Constants
threshold_60 = 60
width_640 = 640
scan_width_200, scan_height_20 = 200, 20
lmid_200, rmid_440 = scan_width_200, width_640 - scan_width_200
area_width_20, area_height_10 = 20, 10
vertical_430 = 430
row_begin_5 = (scan_height_20 - area_height_10) // 2
row_end_15 = row_begin_5 + area_height_10
pixel_threshold_160 = 0.8 * area_width_20 * area_height_10

#curve_radius_pub = rospy.Publisher('/curve_radius', Float64, queue_size=1)
error_pub = rospy.Publisher('/error', Float64, queue_size=1)

rate = rospy.Rate(10)

while cv_image.size == (640*480*3):
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

    constant = cv_image.copy()

    roi = constant[vertical_430:vertical_430 + scan_height_20, :]

    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

    lbound = np.array([0, 0, threshold_60], dtype=np.uint8)
    ubound = np.array([131, 255, 255], dtype=np.uint8)

    bin = cv2.inRange(hsv, lbound, ubound)
    view = cv2.cvtColor(bin, cv2.COLOR_GRAY2BGR)

    left, right = -1, -1
    for l in range(area_width_20, lmid_200):
        area = bin[row_begin_5:row_end_15, l - area_width_20:l]
        if cv2.countNonZero(area) > pixel_threshold_160:
            left = l
            break

    for r in range(width_640 - area_width_20, rmid_440, -1):
        area = bin[row_begin_5:row_end_15, r:r + area_width_20]
        if cv2.countNonZero(area) > pixel_threshold_160:
            right = r
            break

    if left != -1 and right != -1:
        img_width = constant.shape[1]
        lane_width = right - left
        lane_center = (left + right) // 2
        img_center = img_width // 2
        distance_from_center = lane_center - img_width // 2
        #curve_radius = (2 * lane_width) / (lane_width ** 2 + (2 * distance_from_center) ** 2) ** 1.5
        error = lane_center - img_center
        
        #curve_radius_msg = Float64(curve_radius)
        error_msg = Float64(error)

        #curve_radius_pub.publish(curve_radius_msg)
        error_pub.publish(error_msg)
    
    img = cv2.line(constant, (lane_center,440),(lane_center,440), (0,0,255),5) # »¡°­ = Â÷¼± Áß¾Ó
    img = cv2.line(constant, (img_center,440),(img_center,440), (0,255,0),5) # ÃÊ·Ï = Ä«¸Þ¶ó Áß¾Ó
 
     
    cv2.imshow("view",img)
    cv2.waitKey(50)
    rate.sleep()
cv2.destroyAllWindows()


