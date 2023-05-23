#!/usr/bin/env python3
# -*- coding: utf-8 -*-

####################################################################
# 프로그램명 : yolo_tracking.py
# 모 델 명 : ALL
# 작 성 자 : 자이트론
# 생 성 일 : 2021년 03월 25일
# 수 정 일 : 2021년 03월 30일
# 검 수 인 : 조 이현
# 본 프로그램은 상업 라이센스에 의해 제공되므로 무단 배포 및 상업적 이용을 금합니다.
####################################################################

import cv2, torch
import numpy as np

class yolo:

    select_object = None
    current_frame = np.array([])

    def boxes_data_reprocess(self, box_datas, detect_label, min_probability):
        return_list = []
        for box in box_datas:
            if (box.Class == detect_label) and (box.probability > min_probability):
                return_list.append(([box.xmin, box.ymin], [box.xmax, box.ymax]))
        return return_list

    def set_frame(self, frame):
        self.current_frame = frame

    def get_select_object(self):
        return self.select_object

    def object_pretreatment(self, left_up, right_down):
        src = self.current_frame[left_up[1]:right_down[1], left_up[0]:right_down[0]]
        dst = cv2.resize(src, dsize=(100, 100), interpolation=cv2.INTER_AREA)
        return cv2.cvtColor(dst, cv2.COLOR_BGR2HSV)

    def set_tracking_objects(self, hsv):
        so = cv2.calcHist([hsv], [0, 1], None, [100, 100], [0, 100, 0, 100])
        cv2.normalize(so, so, 0, 1, cv2.NORM_MINMAX)
        return so

    def set_host_object(self, host):
        self.select_object = host

    def diff_object(self, client, method):
        return cv2.compareHist(self.select_object, client, method)
        
    def calculating_middle_point(self, left_up, right_down):
        x = int((left_up[0] + right_down[0]) / 2)
        y = int((left_up[1] + right_down[1]) / 2)
        return x, y

    def calculating_area(self, left_up, right_down):
        width = right_down[0] - left_up[0]
        height = right_down[1] - left_up[1]
        return int(width * height)

    def calculating_angle(self, x, y, cam_width):
        half_width = float(cam_width / 2.0)
        angle = int(50.0*((half_width - x)/half_width)) * -1
        return angle


