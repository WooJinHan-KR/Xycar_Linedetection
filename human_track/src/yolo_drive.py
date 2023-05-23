#!/usr/bin/env python3
# -*- coding: utf-8 -*-

####################################################################
# 프로그램명 : yolo_drive.py
# 모 델 명 : B
# 작 성 자 : 자이트론
# 생 성 일 : 2021년 03월 25일
# 수 정 일 : 2021년 03월 30일
# 검 수 인 : 조 이현
# 본 프로그램은 상업 라이센스에 의해 제공되므로 무단 배포 및 상업적 이용을 금합니다.
####################################################################

import time, random, cv2
from common_ros_cls import ros_module
from judge import judge
from safety_box import safety
from yolo_tracking import yolo

import numpy as np

time.sleep(10)

print("start")

r = ros_module("yolo_drive")
j = judge()
s = safety()
y = yolo()

detect_label = "person"
min_probability = 0
min_result_number = 0.3

while not r.rospy_is_shutdown():
    if not s.get_wait():
        y.set_frame(r.get_camera_data())
        obj_ea = y.boxes_data_reprocess(r.get_boundingbox_data(), detect_label, min_probability)

        ea = len(obj_ea)

        if ea > 0:
            print("yolo detect !!!")
            j.set_yolo_detect(True)

            detect_objects = []
            detect_number = []
    
            for box in obj_ea:
                left_up, right_down = box
                detect_objects.append(y.set_tracking_objects(y.object_pretreatment(left_up, right_down)))

            if str(type(y.get_select_object())) == "<type 'NoneType'>":
                y.set_host_object(detect_objects[random.randint(0, ea-1)])

            for detect_object in detect_objects:
                detect_number.append(y.diff_object(detect_object, cv2.HISTCMP_CORREL))
            
            max_result_number = max(detect_number)
            if max_result_number < min_result_number:
                y.set_host_object(detect_objects[detect_number.index(max_result_number)])

            left_up, right_down = obj_ea[detect_number.index(max_result_number)]
            X, Y = y.calculating_middle_point(left_up, right_down)
            angle = y.calculating_angle(X, Y, 640)
            j.set_yolo_angle(angle)

        else:
            print("yolo not detect !!!")
            y.set_host_object(None)
            j.set_yolo_detect(False)

        ultra_data = r.get_ultrasonic_data()
        lidar_data, lidar_increment = r.get_lidar_data()

        obstacle = j.obstacle_chk(ultra_data, lidar_data, lidar_increment)

        if obstacle == -1:
            continue
    
        remind_time = j.mode_and_action_checker(obstacle)

        s.set_safety_wait_time(remind_time)
        s.time_init()

    s.set_angle_and_speed(j.get_angle(), j.get_speed())
    angle, speed = s.safety_drive()
    r.motor_msg_pub(angle, speed)


