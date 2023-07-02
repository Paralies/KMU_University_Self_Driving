#!/usr/bin/env python

import rospy, math
from xycar_msgs.msg import xycar_motor
from std_msgs.msg import Int32MultiArray

motor_msg = xycar_motor()
ultrasonicData = None

def callback(msg): 
    global ultrasonicData
    ultrasonicData = msg.data  

rospy.init_node('drive')
motor_pub = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)
rospy.Subscriber('ultrasonic', Int32MultiArray, callback)

while not rospy.is_shutdown():

    while ultrasonicData is None:
        continue

    R = ultrasonicData[3] 
    L = ultrasonicData[1]  
    print(R,L)
    Q = R - L

    angle = 0
    if Q > 0 and abs(Q) > 0.1:           
        angle = Q
    elif Q < 0 and abs(Q) > 0.1:
        angle = Q

    motor_msg.angle = int(angle)
    motor_msg.speed = 10
    
    motor_pub.publish(motor_msg)

