#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy, rospkg
import numpy as np
import cv2, random, math
from xycar_msgs.msg import xycar_motor
from sensor_msgs.msg import Image

import sys
import os
import signal

class MovingAverage:
    def __init__(self, n):
        self.samples = n
        self.data = []
        self.weights = list(range(1, n+1))

    def add_sample(self, new_sample):
        if len(self.data) < self.samples:
            self.data.append(new_sample)
        else:
            self.data = self.data[1:] + [new_sample]
        #print("samples: %s" % self.data)

    def get_mm(self):
	    return float(sum(self.data)) / len(self.data)

    def get_wmm(self):
        s = 0
        for i, x in enumerate(self.data):
            s += x * self.weights[i]
        vector = np.vectorize(np.float)
        s = vector(s)
        return s / sum(self.weights[:len(self.data)])

class PID():
    def __init__(self, kp, ki, kd):
        self.Kp = kp
        self.Ki = ki
        self.Kd = kd
        self.p_error = 0.0
        self.i_error = 0.0
        self.d_error = 0.0

    def pid_control(self, cte):
        self.d_error = cte-self.p_error 
        self.p_error = cte 
        self.i_error += cte 

        return self.Kp*self.p_error + self.Ki*self.i_error + self.Kd*self.d_error

def signal_handler(sig, frame):
    os.system('killall -9 python rosout')
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

image = np.empty(shape=[0])
pub = None
Width = 640
Height = 480
Offset = 390
Gap = 40


def img_callback(data):
    global image
    image = np.frombuffer(data.data, dtype=np.uint8).reshape(data.height, data.width, -1)
    image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

# publish xycar_motor msg
def drive(Angle, Speed): 
    global pub

    msg = xycar_motor()
    msg.angle = int(Angle)
    msg.speed = int(Speed)

    pub.publish(msg)

# draw lines
def draw_lines(img, lines):
    global Offset
    for line in lines:
        x1, y1, x2, y2 = line[0]
        color = (random.randint(0, 255), random.randint(0, 255), random.randint(0, 255))
        img = cv2.line(img, (x1, y1+Offset), (x2, y2+Offset), color, 2)
    return img

# draw rectangle
def draw_rectangle(img, lpos, rpos, offset=0):
    center = (lpos + rpos) // 2
    #center2 = (lpos2 + rpos2) / 2

    cv2.rectangle(img, (lpos - 5, 15 + offset),
                       (lpos + 5, 25 + offset),
                       (0, 255, 0), 2)
    cv2.rectangle(img, (rpos - 5, 15 + offset),
                       (rpos + 5, 25 + offset),
                       (0, 255, 0), 2)
    cv2.rectangle(img, (center-5, 15 + offset),
                       (center+5, 25 + offset),
                       (0, 255, 0), 2)
    cv2.rectangle(img, (315, 15 + offset),
                       (325, 25 + offset),
                       (0, 0, 255), 2)
    return img

# left lines, right lines
def divide_left_right(lines):
    global Width

    low_slope_threshold = 0
    high_slope_threshold = 10

    # calculate slope & filtering with threshold
    slopes = []
    new_lines = []

    for line in lines:
        x1, y1, x2, y2 = line[0]

        if x2 - x1 == 0:
            slope = 0
        else:
            slope = float(y2-y1) / float(x2-x1)
        
        if abs(slope) > low_slope_threshold and abs(slope) < high_slope_threshold:
            slopes.append(slope)
            new_lines.append(line[0])

    # divide lines left to right
    left_lines = []
    right_lines = []

    for j in range(len(slopes)):
        Line = new_lines[j]
        slope = slopes[j]

        x1, y1, x2, y2 = Line

        if (slope < 0) and (x2 < Width/2 - 90):
            left_lines.append([Line.tolist()])
        elif (slope > 0) and (x1 > Width/2 + 90):
            right_lines.append([Line.tolist()])

    return left_lines, right_lines

# get average m, b of lines
def get_line_params(lines):
    # sum of x, y, m
    x_sum = 0.0
    y_sum = 0.0
    m_sum = 0.0

    size = len(lines)
    if size == 0:
        return 0, 0

    for line in lines:
        x1, y1, x2, y2 = line[0]

        x_sum += x1 + x2
        y_sum += y1 + y2
        m_sum += float(y2 - y1) / float(x2 - x1)

    x_avg = x_sum / (size * 2)
    y_avg = y_sum / (size * 2)
    m = m_sum / size
    b = y_avg - m * x_avg

    return m, b

# get lpos, rpos
def get_line_pos(img, lines, left=False, right=False):
    global Width, Height
    global Offset, Gap

    m, b = get_line_params(lines)

    if m == 0 and b == 0:
        if left:
            pos = 0
        if right:
            pos = Width
    else:
        y = Gap / 2
        pos = (y - b) / m

        b += Offset
        x1 = (Height - b) / float(m)
        x2 = ((Height/2) - b) / float(m)

        cv2.line(img, (int(x1), Height), (int(x2), (Height//2)), (255, 0,0), 3)

    return img, int(pos)

# show image and return lpos, rpos
def process_image(frame):
    global Width
    global Offset, Offset2, Gap

    # gray
    gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)

    gray = gray - 50

    alpha = 1
    dst = gray + (gray - 128) * alpha

    # blur
    kernel_size = 5
    blur_gray = cv2.GaussianBlur(dst,(kernel_size, kernel_size), 0)

    _, blur_gray = cv2.threshold(blur_gray, 120, 255, cv2.THRESH_BINARY)
    #cv2.imshow('binary',blur_gray)

    # canny edge
    low_threshold = 60
    high_threshold = 70
    edge_img = cv2.Canny(np.uint8(blur_gray), low_threshold, high_threshold)
    # cv2.imshow('canny',edge_img)

    # HoughLinesP
    roi = edge_img[Offset : Offset+Gap, 0 : Width]
    all_lines = cv2.HoughLinesP(roi,1,math.pi/180,35,35,10)

    # divide left, right lines
    if all_lines is None:
        return 0, 640
    left_lines, right_lines = divide_left_right(all_lines)

    # get center of lines
    frame, lpos = get_line_pos(frame, left_lines, left=True)
    frame, rpos = get_line_pos(frame, right_lines, right=True)


    # draw lines
    frame = draw_lines(frame, left_lines)
    frame = draw_lines(frame, right_lines)
    frame = cv2.line(frame, (230, 235), (410, 235), (255,255,255), 2)
                                 
    # draw rectangle
    frame = draw_rectangle(frame, lpos, rpos, offset=Offset)

    # show image
    # cv2.imshow('calibration', frame)
    
    return lpos, rpos

def avg_angle(angle, angle_list):
    a_list = angle_list
    
    if a_list :
        if angle * a_list[-1] < 0:
            a_list = []
            a_list.append(angle)
        else:
            if len(a_list) < 3:
                a_list.append(angle)
            else:
                a_list.append(angle)
                a_list = a_list[1:]

    else:
        a_list.append(angle)
    return a_list

def start():
    global pub
    global image
    global cap
    global Width, Height
    global Offset

    rospy.init_node('auto_drive')
    pub = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)

    image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, img_callback)
    print ("---------- Xycar A2 v1.0 ----------")
    rospy.sleep(2)
    
    angle = 2.5
    angle_temp = 2.5 
    speed = 10
    angle_list = []
    
    pid = PID(0.56, 0.0007, 0.2)
    mm1 = MovingAverage(20)
    while True:
        while not image.size == (640*480*3):
            continue
        cv2.imshow('img', image)
        lpos, rpos = process_image(image)


        center = (lpos + rpos) / 2
        #print(center)
        error = (center - Width/2) 

	    #angle_s = -(Width/2 - center)

        #mm1.add_sample(angle_s)
	    #wmm_angle = mm1.get_wmm()

        if lpos==0 and rpos==640: # can't recognize lanes
            angle_temp = angle
            drive(angle_temp, 5)
            continue
            #print(angle_temp)
            
        angle = pid.pid_control(error)
        #print("angle: ", angle)

        angle_list = avg_angle(angle, angle_list)
        mean_angle = np.mean(angle_list)
        #print('mean : ',mean_angle)

        if len(angle_list) == 3 and abs(mean_angle) >= 50:
            drive(mean_angle*2, 10)
            Offset = 380
        else:
            drive(mean_angle, 15)
            Offset = 390

        #print("angle_s", angle_s)
        #print("wm_angle", wmm_angle)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    rospy.spin()

if __name__ == '__main__':

    start()


