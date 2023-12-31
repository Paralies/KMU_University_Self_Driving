#!/usr/bin/env python
# -*- coding: utf-8 -*-

####################################################################
# 프로그램명 : joy_cam.py
# 버 전 : xavier.411.py3.1_0
# 본 프로그램은 상업 라이센스에 의해 제공되므로 무단 배포 및 상업적 이용을 금합니다.
####################################################################

import rospy
import time

from geometry_msgs.msg import Twist
from xycar_msgs.msg import xycar_motor

ack_msg = xycar_motor()
ack_publisher = None

def callback_speed(msg_android_speed):
    global ack_msg
    ack_msg.speed = int(msg_android_speed.linear.x*50)

def callback_steering(msg_android_steering):
    global ack_msg
    ack_msg.angle = int((-msg_android_steering.angular.z)*50)

rospy.init_node("joystick_cam")
rospy.Subscriber("android_motor_speed",Twist, callback_speed)
rospy.Subscriber("android_motor_steering",Twist, callback_steering)
ack_publisher = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)

while not rospy.is_shutdown():
    ack_publisher.publish(ack_msg)
    time.sleep(0.01)
