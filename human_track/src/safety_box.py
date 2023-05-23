#!/usr/bin/env python3
# -*- coding: utf-8 -*-

####################################################################
# 프로그램명 : safety_mode.py
# 모 델 명 : B
# 작 성 자 : 자이트론
# 생 성 일 : 2021년 03월 25일
# 수 정 일 : 2021년 03월 30일
# 검 수 인 : 조 이현
# 본 프로그램은 상업 라이센스에 의해 제공되므로 무단 배포 및 상업적 이용을 금합니다.
####################################################################

import time

class safety:

    remind_time = 0
    before_time = 0
    wait = False

    def time_init(self):
        self.before_time = time.time()

    def get_wait(self):
        return self.wait

    def set_angle_and_speed(self, angle, speed):
        self.angle = angle
        self.speed = speed

    def set_safety_wait_time(self, remind_time):
        self.remind_time = max(remind_time, 0)

    def reduce_time_check(self):
        after_time = time.time()
        self.remind_time = max(self.remind_time - (after_time - self.before_time), 0)
        self.before_time = after_time

    def safety_drive(self):
        if (self.remind_time <= 0) or (self.remind_time == float('inf')):
            self.wait = False
            return self.angle, self.speed
        else:
            self.wait = True
            self.reduce_time_check()
            return 0, 0


