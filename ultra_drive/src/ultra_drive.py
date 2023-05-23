#!/usr/bin/env python

# Int32MultiArray, xycar_motor 메시지 사용 준비 
import rospy
import time
from sensor_msgs.msg import Int32MultiArray
from xycar_msgs.msg import xycar_motor
import logging

# setting logger
logger = logging.getLogger(__name__) # logger 생성
logger.setLevel(logging.DEBUG) # DEBUG Level부터 handler에게 전달
formatter = logging.Formatter('%(asctime)s -- [%(levelname)s]\n%(message)s\n') # 로그메시지 format 설정

# setting logger-handler
stream_handler = logging.StreamHandler() # 스트림으로 로그를 출력하는 handler(stream_handler) 생성
stream_handler.setLevel(logging.INFO) # console에서 INFO Level부터 표시
stream_handler.setFormatter(formatter) # 위에서 지정한 formatter 형식으로 handler format 설정
logger.addHandler(stream_handler) # stream_handler 객체 추가

# 초음파 센서 거리 정보를 담을 저장 공간 준비
ultra_msg = None
motor_msg = xycar_motor()

# 모터 메시지를 발행하는 함수
def publish_motor_msg(speed, angle):
	# log msg - info
	logger.info('-- START: publish_motor_msg() --')

	global motor_msg
	motor_msg.speed = speed
	motor_msg.angle = angle
	pub.publish(motor_msg)

# 초음파 센서 토픽이 들어오면 실행되는 콜백 함수 정의
def ultra_callback(data):
	global ultra_msg
	ultra_msg = data.data

def start():
	# log msg - info
	logger.info('-- START: start() --')

	# 주기 설정, 0.15
	rate = rospy.Rate(0.15)

	# 노드 선언, 구독과 발행할 토픽 선언
	rospy.init_node('ultra_driver')
	rospy.Subscriber("xycar_ultrasonic", Int32MultiArray, ultra_callback, queue_size = 1)
	pub = rospy.Publisher('xycar_motor', xycar_motor, queue_size = 1)

	while not rospy.is_shutdown():
		# 전방 초음파 센서가 감지한 거리 정보가 0 < 거리 < 10cm 범위에 있으면 정차, 그보다 크면 후진
		# 거리값 0 -> 무한대, 장애물이 없음을 의미한다. 
		# 1 -> 왼쪽
		# 2	-> 오른쪽
		# 3, 4, 5 -> 뒤 / 3 -> 왼쪽 뒤, 5 -> 오른쪽 뒤 

		# 양 옆 센서를 사용하지 않아도 주행에 문제가 없고, 앞쪽이라서 감지해서 동작하게 되면 이미 늦는 경우가 있어서 일단 주석 처리
		# # 왼쪽 초음파 센서, 왼쪽에 장애물이 있는 경우 
		# if ultra_msg[1] > 0 and ultra_msg[1] < 10:
		# 	drive_stop()
		# 	drive_spin_r()

		# # 오른쪽 초음파 센서, 오른쪽에 장애물이 있는 경우 
		# if ultra_msg[2] > 0 and ultra_msg[2] < 10:
		# 	drive_stop()
		# 	drive_spin_l()

		rate.sleep() 

		# 뒷쪽 초음파 센서, 뒤에 장애물이 있는 경우 
		# 후진 정면 
		if ultra_msg[4] > 0 and ultra_msg[4] < 10:
			publish_motor_msg(0, 0)
			break 
		# 후진 왼쪽
		elif ultra_msg[3] > 0 and ultra_msg[3] < 10:
			publish_motor_msg(0, 0)
			publish_motor_msg(-3, -45)
		# 후진 오른쪽 
		elif ultra_msg[5] > 0 and ultra_msg[5] < 10:
			publish_motor_msg(0, 0)
			publish_motor_msg(-3, 45)
		# 이상 없는 경우 계속 후진, 왼쪽 + 오른쪽에 아무것도 없는 경우 포함
		else:
			publish_motor_msg(5, 0)

# 시작점 지정 
if __name__ == '__main__':
	try:
		start()
	except:
		# log msg - error
		logger.error('SOMETHING WAS WRONG...')