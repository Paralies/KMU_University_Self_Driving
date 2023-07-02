#!/usr/bin/env python

import rospy
import time
import numpy as np
from xycar_msgs.msg import xycar_motor
from sensor_msgs.msg import LaserScan

motor_control = xycar_motor()
lidar_ready = False
lidar_msg = None
speed = 4
distance = 0.3
    
def lidar_callback(data):
    global lidar_msg, lidar_ready
    lidar_msg = data.ranges
    lidar_ready = True

rospy.init_node('motor_driver')
pub = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)
rospy.Subscriber("/scan", LaserScan, lidar_callback)


def motor_pub(angle, speed): 
    global pub
    global motor_control
    
    if check_obstacle() == True:
        speed = 0

    motor_control.angle = angle
    motor_control.speed = speed

    pub.publish(motor_control)

def check_obstacle():
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
    if 0 < min(min_value) < distance:
        return True
    else:
        return False

while lidar_ready == False:
    continue
print("Lidar Ready ----------")
    
while not rospy.is_shutdown():
    angle = -50
    for i in range(30): 
        motor_pub(angle, speed) 
        time.sleep(0.1)

    angle = 0
    for i in range(50):
        motor_pub(angle, speed)
        time.sleep(0.1)

    angle = 50
    for i in range(30):
        motor_pub(angle, speed) 
        time.sleep(0.1)

    angle = 0
    for i in range(50):
        motor_pub(angle, speed) 
        time.sleep(0.1)
 
