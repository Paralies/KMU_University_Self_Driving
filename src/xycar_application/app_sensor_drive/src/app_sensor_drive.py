#!/usr/bin/env python
# -*- coding: utf-8 -*-
####################################################################
# 프로그램명 : app_sendsor_drive.py
# 작 성 자 : (주)자이트론
# 생 성 일 : 2020년 07월 23일
# 본 프로그램은 상업 라이센스에 의해 제공되므로 무단 배포 및 상업적 이용을 금합니다.
####################################################################

import rospy, time
import numpy as np
from std_msgs.msg import Int32MultiArray
from sensor_msgs.msg import LaserScan
from xycar_msgs.msg import xycar_motor

steering_pub = None
ult_data = None
lidar_msg = None
speed = 4
back_speed = -3
distance = 25

def callback_ultra(msg):
    global ult_data
    ult_data = msg.data
    
def lidar_callback(data):
    global lidar_msg, lidar_ready
    lidar_msg = data.ranges

def init_node():
    global steering_pub
    rospy.init_node('sensor_drive')
    rospy.Subscriber('xycar_ultrasonic', Int32MultiArray, callback_ultra)
    steering_pub = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)
    rospy.Subscriber("/scan", LaserScan, lidar_callback)

def drive(angle, speed):
    global steering_pub
    msg = xycar_motor()
    msg.angle = angle
    msg.speed = speed
    steering_pub.publish(msg)
    
def check_obstacle(dis):
    global lidar_msg
    if lidar_msg == None:
        return False
            
    obstacle = False
        
    mid_arr = np.array([])
    mid_arr = np.append(mid_arr, lidar_msg[491:504])
    mid_arr = np.append(mid_arr, lidar_msg[1:14])
        
    median_1 = np.nanpercentile(lidar_msg[99:126], 50)
    median_2 = np.nanpercentile(lidar_msg[71:98], 50)
    median_3 = np.nanpercentile(lidar_msg[43:70], 50)
    median_4 = np.nanpercentile(lidar_msg[15:42], 50)
    median_5 = np.nanpercentile(mid_arr, 50)
    median_6 = np.nanpercentile(lidar_msg[463:490], 50)
    median_7 = np.nanpercentile(lidar_msg[435:462], 50)
    median_8 = np.nanpercentile(lidar_msg[407:434], 50)
    median_9 = np.nanpercentile(lidar_msg[379:406], 50)
        
    min_value = [median_1, median_2, median_3, median_4, median_5, median_6, median_7, median_8, median_9]
    while 0.0 in min_value:
        min_value.remove(0.0)
    if 0 < min(min_value) < dis:
        return True
    else:
        return False

def forward_drive():
    global speed
    #print("========================")
    #print("forward")
    while True:
        if check_obstacle(0.3):
            break
        else:
            drive(0,speed)
            time.sleep(0.1)

def stop():
    #print("stop")
    for _ in range(10):
        drive(0, 0)
        time.sleep(0.1)

def backward_drive():  
    global speed 
    #print("backward")
    for _ in range(20):
        if (min(ult_data[5],ult_data[6],ult_data[7]) < distance):
            break
        else:
            drive(0,back_speed)
            time.sleep(0.1)

def right_turn_drive():
    global speed
    #print("right turn")
    for _ in range(20):
        if check_obstacle(0.2):
            break
        else:
            drive(50,speed)
            time.sleep(0.1)
    
init_node()
time.sleep(5)

while ult_data == None:
    continue
print("UltraSonic Ready ----------")

while lidar_msg == None:
    continue
print("Lidar Ready ----------")


while not rospy.is_shutdown():
    forward_drive()
    stop()
    backward_drive()
    stop()
    right_turn_drive()
    stop()
    
