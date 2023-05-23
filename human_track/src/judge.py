#!/usr/bin/env python3
# -*- coding: utf-8 -*-

####################################################################
# 프로그램명 : judge.py
# 모 델 명 : B
# 작 성 자 : 자이트론
# 생 성 일 : 2021년 03월 25일
# 수 정 일 : 2021년 03월 30일
# 검 수 인 : 조 이현
# 본 프로그램은 상업 라이센스에 의해 제공되므로 무단 배포 및 상업적 이용을 금합니다.
####################################################################

import time, os
import numpy as np

class judge:
    mode = 0
    action = 0
    default_mode = 3
    last_mode = 0
    last_action = 0
    action_start_time = 0
    yolo_detect = False

    drive_mode = [
        [ { "angle": 0,  "speed":  30, "time":  float('inf') }                                                     ],
        [ { "angle": 0,  "speed": -25, "time":  2            }, { "angle": 50, "speed": 30, "time": 2            } ],
        [ { "angle": 0,  "speed":   0, "time":  float('inf') }                                                     ],
        [ { "angle": 50, "speed": -25, "time":  2            }, { "angle": 0, "speed": 30, "time": 2             } ],
        [ { "angle": 0,  "speed":  30, "time":  float('inf') }                                                     ]
    ]
    obstacle_mode = ["n", "f", "b", "fb"]
    speed_states  = ["s", "f", "b"]
    
    def __init__(self):
        self.lidar_type = int(os.environ["lidar_version"])
        self.lidar_setting = [
            [505, np.sin, np.cos, -1, 1],
            [720, np.cos, np.sin, -1, 1]
        ]
        self.mode = self.default_mode
        self.speed = self.drive_mode[self.default_mode][self.action]["speed"]
        self.angle = self.drive_mode[self.default_mode][self.action]["angle"]
        self.action_time_init()

    def lidar_check(self, lidar_array, lidar_increment, left_down, right_up, max_point):
        setting = self.lidar_setting[self.lidar_type]

        if len(lidar_array) != setting[0]:
            return False

        detect_point = 0
        
        for i in range(0, setting[0]):
            radian = i * lidar_increment

            x = lidar_array[i] * setting[1](radian) * setting[3]
            y = lidar_array[i] * setting[2](radian) * setting[4]
            
            if ((x >= left_down["x"]) and \
               (x <= right_up["x"])) and \
               ((y >= left_down["y"]) and \
               (y <= right_up["y"])):
                detect_point += 1

            if detect_point > max_point:
                return True

        return False
        
    def obstacle_chk(self, ultra_array, lidar_array, lidar_increment):
        if len(ultra_array) == 0:
            ultra_array = [140, 140, 140, 140, 140, 140, 140, 140]

        left_down = {"x": -0.25, "y": 0.15}
        right_up  = {"x": 0.25,  "y": 0.35 }

        obstacle = 0

        if self.lidar_check(lidar_array, lidar_increment, left_down, right_up, 3):  
            print("lidar_detect")
            obstacle += 1

        if min(ultra_array[5:8]) < 20:
            obstacle += 2

        return obstacle

    def mode_and_action_checker(self, obstacle):
        current_speed = self.drive_mode[self.mode][self.action]["speed"]
        if current_speed != 0:
            speed_state = self.speed_states[current_speed//abs(current_speed)]
        else:
            speed_state = self.speed_states[0]

        if (self.last_mode != self.mode) or (self.last_action != self.action):
            self.action_time_init()
            self.last_mode = self.mode
            self.last_action = self.action

        if speed_state in self.obstacle_mode[obstacle]:
            remind_time = self.drive_mode[self.mode][self.action]["time"] - (time.time() - self.action_start_time)
            if speed_state == "f":
                self.last_action = self.action
                self.action = 0
                self.last_mode = self.mode
                self.mode = 1
                if self.yolo_detect:
                    self.last_mode = self.mode
                    self.mode = 2
                    return 1

            elif speed_state == "b":
                self.last_action = self.action
                self.action += 1
            
            return remind_time

        if ((time.time() - self.action_start_time) < self.drive_mode[self.mode][self.action]["time"]) and \
           (not self.yolo_detect):

            if not ((self.mode == 2) or (self.mode == 4)):
                return 0

            print("mode", self.mode)
            if (self.mode == 2) or (self.mode == 4):
                self.last_mode = self.mode
                self.mode = self.default_mode
                self.last_action = self.action
                self.action = 0

                return 1

        if (len(self.drive_mode[self.mode]) - 1) != self.action:
            self.last_action = self.action
            self.action += 1
            return 0

        if self.yolo_detect:
            self.last_mode = self.mode
            self.mode = 4
            self.last_action = self.action
            self.action = 0
            return 0

        self.last_mode = self.mode
        self.mode = self.default_mode
        self.last_action = self.action
        self.action = 0

        return 0

    def set_yolo_detect(self, yolo_detect):
        self.yolo_detect = yolo_detect

    def get_angle(self):
        angle = self.drive_mode[self.mode][self.action]["angle"]
        return angle

    def get_speed(self):
        speed = self.drive_mode[self.mode][self.action]["speed"]
        return speed

    def set_yolo_angle(self, angle):
        self.drive_mode[4][0]["angle"] = angle

    def action_time_init(self):
        self.action_start_time = time.time()

