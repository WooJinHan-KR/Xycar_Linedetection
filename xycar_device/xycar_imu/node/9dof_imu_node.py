#!/usr/bin/env python
import rospy
import serial
import math
import sys

from sensor_msgs.msg import Imu
from std_msgs.msg import Header, ColorRGBA
from geometry_msgs.msg import Quaternion, Vector3, Pose
from tf.transformations import euler_from_quaternion 
from visualization_msgs.msg import Marker

rospy.init_node("razor_node")

pub_mode = rospy.get_param('~rviz_mode', 'true')
port = rospy.get_param('~port', '/dev/ttyIMU')
frame_id = rospy.get_param('~frame_id', "imu")
topic = rospy.get_param('~topic', "imu")
queue_size = rospy.get_param('~queue_size', 1)

if pub_mode == "true":
    marker_pub = rospy.Publisher('visualization_marker', Marker, queue_size=1)

rospy.loginfo("Opening %s...", port)
try:
    ser = serial.Serial(port=port, baudrate=115200, timeout=1)
except serial.serialutil.SerialException:
    rospy.logerr("IMU not found at port "+ port + ". Did you specify the correct port in the launch file?")
    sys.exit(0)

pub = rospy.Publisher(topic, Imu, queue_size=queue_size)

imuMsg = Imu()
imuMsg.header.frame_id = frame_id

marker = Marker( \
    color=ColorRGBA( \
        r=1, \
        g=1, \
        b=1, \
        a=1 \
    ), \
    type=9, \
    id=0, \
    scale=Vector3( \
        x=0, \
        y=0, \
        z=0.14 \
    ), \
    pose=Pose( \
        position=Vector3( \
            x=0.5, \
            y=0.5, \
            z=1.45 \
        ), \
        orientation=Quaternion( \
            w=1, \
            x=0, \
            y=0, \
            z=0 \
        ) \
    ), \
    ns="imu" \
)

rospy.loginfo("Giving the razor IMU board 3 seconds to boot...")
rospy.sleep(3)

rospy.loginfo("Flushing first 30 IMU entries...")

for x in range(0, 30):
    line = ser.readline()

rospy.loginfo("Publishing IMU data...")

last_timestamp = rospy.Time.now()
last_roll, last_pitch, last_yaw = 0, 0, 0

imuMsg.orientation_covariance = [-1, 0, 0, 0, 0, 0, 0, 0, 0]
imuMsg.angular_velocity_covariance = [0, 0, 0, 0, 0, 0, 0, 0, 0]
imuMsg.linear_acceleration_covariance = [0, 0, 0, 0, 0, 0, 0, 0, 0]

while not rospy.is_shutdown():
    line = ser.readline()
    sig_chk = str(line).split("#")[1]
    sig_chk = sig_chk.split("=")
    if sig_chk[0] == "XYMU":
        raw_data = str(sig_chk[1]).split(",")
    else:
        continue
        
    raw_data = sig_chk[1].split(",")
    
    acc_x = float(raw_data[0])
    acc_y = float(raw_data[1])
    acc_z = float(raw_data[2])
    gro_x = float(raw_data[3])
    gro_y = float(raw_data[4])
    gro_z = float(raw_data[5])
    mag_x = float(raw_data[6])
    mag_y = float(raw_data[7])
    mag_z = float(raw_data[8])
    qtn_w = float(raw_data[9])
    qtn_x = float(raw_data[10])
    qtn_y = float(raw_data[11])
    qtn_z = float(raw_data[12])
    
    current_roll, current_pitch, current_yaw = euler_from_quaternion([qtn_x, qtn_y, qtn_z, qtn_w])
    
    if pub_mode == "true":
        
        output_txt = "roll : "+str(round(current_roll,2))+", "
        output_txt += "pitch : "+str(round(current_pitch,2))+", "
        output_txt += "yaw : "+str(round(current_yaw,2))

        marker.header = Header(stamp=rospy.Time.now(), frame_id=frame_id)
        marker.text = output_txt

        marker_pub.publish(marker)
        
    imuMsg.orientation.x = qtn_x
    imuMsg.orientation.y = qtn_y
    imuMsg.orientation.z = qtn_z
    imuMsg.orientation.w = qtn_w

    current_timestamp = rospy.Time.now()
    imuMsg.header.stamp = current_timestamp
    
    dt = current_timestamp.to_time() - last_timestamp.to_time()
    
    av_x = float(current_roll - last_roll) / dt
    av_y = float(current_pitch - last_pitch) / dt
    av_z = float(current_yaw - last_yaw) / dt
    
    imuMsg.angular_velocity.x = av_x
    imuMsg.angular_velocity.y = av_y
    imuMsg.angular_velocity.z = av_z

    imuMsg.linear_acceleration.x = acc_x
    imuMsg.linear_acceleration.y = acc_y
    imuMsg.linear_acceleration.z = acc_z

    pub.publish(imuMsg)
    
    last_roll, last_pitch, last_yaw = current_roll, current_pitch, current_yaw
    last_timestamp = current_timestamp
        
ser.close()

