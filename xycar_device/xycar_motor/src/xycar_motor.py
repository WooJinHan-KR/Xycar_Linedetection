#!/usr/bin/env python3
# -*- coding: utf-8 -*-

####################################################################
# 프로그램명 : xycar_motor.py
# 모 델 명 : B2 - LIDAR
# 작 성 자 : 자이트론
# 생 성 일 : 2020년 07월 12일
# 수 정 일 : 2021년 03월 16일
# 검 수 인 : 조 이현
# 본 프로그램은 상업 라이센스에 의해 제공되므로 무단 배포 및 상업적 이용을 금합니다.
#############################################################################

import rospy, rospkg
import time
import serial
import threading
from xycar_msgs.msg import xycar_motor
from ackermann_msgs.msg import AckermannDriveStamped

import sys
import os
import signal

def signal_handler(sig, frame):
    import time
    time.sleep(3)
    os.system('killall -9 python3 rosout')
    sys.exit(0)


signal.signal(signal.SIGINT, signal_handler)

class motor:
    def __init__(self):
        self.control = False
        self.ros_init()
        self.set_parameter()
        if self.motor_type == 1:
            self.set_arduino()
        rospy.spin()

    def set_parameter(self):
        motor_controller = [
            [0, 0, 0.0068, 0, 0, 0.08, 0, None, self.auto_drive_vesc], 
            [0, 0, 1, 0, 0, 1, 0, serial.Serial('/dev/ttyMOTOR',115200, timeout=0.2), self.auto_drive_arduino] # /dev/ttyMOTOR
        ]

        self.angle_weight = motor_controller[self.motor_type][2]
        self.angle_bias_1 = motor_controller[self.motor_type][0]
        self.angle_bias_2 = motor_controller[self.motor_type][1]
        self.speed_weight = motor_controller[self.motor_type][5]
        self.speed_bias_1 = motor_controller[self.motor_type][3]
        self.speed_bias_2 = motor_controller[self.motor_type][4]
        self.Angle = motor_controller[self.motor_type][6]
        self.Speed = motor_controller[self.motor_type][6]
        self.seridev = motor_controller[self.motor_type][7]
        self.auto_drive = motor_controller[self.motor_type][8]
        self.angle_offset = 0.0
        self.last_speed = 0.0
        self.g_chk_time = 0
        self.change_vector_term = 0.05

    def set_arduino(self):
        start_time = time.time()
        while self.arduino_wait():
            if (time.time() - start_time) > 2:
                start_time = time.time()
                print("wait")
            continue

        sndAdata = "SA,1000,1550,1950,50,X"
        self.seridev.write(sndAdata)
        while not self.arduino_check_cmd():
            time.sleep(0.1)
            self.seridev.write(sndAdata)
            
        sndMdata = "SM,1000,1497,1950,50,X"
        self.seridev.write(sndMdata)
        while not self.arduino_check_cmd():
            time.sleep(0.1)
            self.seridev.write(sndAdata)
            
        print("Motor Configure Complete.")
        time.sleep(3)

        self.control = True

    def ros_init(self):
        rospy.init_node('xycar_motor')
        rospy.Subscriber("xycar_motor", xycar_motor, self.callback, queue_size=1)

        self.angle_offset = rospy.get_param("~angle_offset")
        self.motor_type = rospy.get_param("~motor_type")

        if self.motor_type == 0:
            self.ack_msg = AckermannDriveStamped()
            self.ack_msg.header.frame_id = ''
            self.ackerm_publisher = rospy.Publisher('ackermann_cmd', AckermannDriveStamped, queue_size=1)

    def auto_drive_vesc(self, steer_val, car_run_speed):
        self.ack_msg.header.stamp = rospy.Time.now()
        self.ack_msg.drive.steering_angle = -steer_val

        if (self.last_speed > 0) and (car_run_speed < 0):
            self.ack_msg.drive.speed = -50
            self.ackerm_publisher.publish(self.ack_msg)
            time.sleep(self.change_vector_term)
            for _ in range(5):
                self.ack_msg.drive.speed = 0
                self.ackerm_publisher.publish(self.ack_msg)
            time.sleep(self.change_vector_term)

        self.ack_msg.drive.speed = car_run_speed
        self.ackerm_publisher.publish(self.ack_msg)
        self.last_speed = car_run_speed

    def auto_drive_arduino(self, angle, speed):
        self.seridev.flushInput()
        if not self.control:
            return

        sndData = "R,{},{},X".format(int(angle), int(speed))
        self.seridev.write(sndData)
        
        time.sleep(0.1)
        self.arduino_check_cmd(int(angle), int(speed))

    def serial_receive(self):
        try:
            serial_value = self.seridev.readline()
        except serial.serialutil.SerialException:
            print("serial.serialutil.SerialException")
            return -1
        return serial_value[:-2]

    def arduino_wait(self):
        serial_string_remove_newline = self.serial_receive()
        if serial_string_remove_newline == -1:
            return True
        if serial_string_remove_newline == "Xycar B2 v1.0.2":
            return False
        return True
        
    def arduino_check_cmd(self, angle=None, speed=None):
        serial_string_remove_newline = self.serial_receive()
        if serial_string_remove_newline == -1:
            return
        if not ("," in serial_string_remove_newline):
            return

        serial_string_split_comma = serial_string_remove_newline.split(",")

        if (serial_string_split_comma[0] == "VR") and (serial_string_split_comma[3] == "X"):
            return True

        if (serial_string_split_comma[0] == "VSA") and (serial_string_split_comma[5] == "X"):
            print("angle_pulse_min :" + str(serial_string_split_comma[1]))
            print("angle_pulse_mid :" + str(serial_string_split_comma[2]))
            print("angle_pulse_max :" + str(serial_string_split_comma[3]))
            print("angle_step_count :" + str(serial_string_split_comma[4]))
            return True

        if (serial_string_split_comma[0] == "VSM") and (serial_string_split_comma[5] == "X"):
            print("speed_pulse_min :" + str(serial_string_split_comma[1]))
            print("speed_pulse_mid :" + str(serial_string_split_comma[2]))
            print("speed_pulse_max :" + str(serial_string_split_comma[3]))
            print("speed_step_count :" + str(serial_string_split_comma[4]))
            return True

        return False
    
    def callback(self, data):
        data.angle = max(-50, min(data.angle, 50))
        data.speed = max(-50, min(data.speed, 50))

        Angle = (((float(data.angle + self.angle_offset) + self.angle_bias_1) * self.angle_weight)) + self.angle_bias_2
        Speed = ((float(data.speed) + self.speed_bias_1) * self.speed_weight) + self.speed_bias_2

        #print(Angle, Speed)

        self.auto_drive(Angle, Speed)

if __name__ == '__main__':
    m = motor()

