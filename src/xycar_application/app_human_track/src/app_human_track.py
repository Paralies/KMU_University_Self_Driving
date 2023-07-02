#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import torch, rospy, cv2, sys, rospkg, time
from cv_bridge import CvBridge
import numpy as np
from xycar_msgs.msg import xycar_motor
from sensor_msgs.msg import Image
from vision.ssd.vgg_ssd import create_vgg_ssd, create_vgg_ssd_predictor
from vision.ssd.mobilenetv1_ssd import create_mobilenetv1_ssd, create_mobilenetv1_ssd_predictor
from vision.utils.misc import Timer
from sensor_msgs.msg import LaserScan

image = np.empty(shape=[0]) 
bridge = CvBridge() 
motor = None 
img_ready = False  
lidar_ready = False
lidar_msg = None 
WIDTH, HEIGHT = 640, 480 
distance = 0.3  

def img_callback(data):
    global image, img_ready
    image = bridge.imgmsg_to_cv2(data, "bgr8")
    img_ready = True

def drive(angle, speed):
    global motor
    if check_obstacle() == True:
        speed = 0
    motor_msg = xycar_motor()
    motor_msg.angle = angle
    motor_msg.speed = speed
    motor.publish(motor_msg)

def lidar_callback(data):
    global lidar_msg, lidar_ready
    lidar_msg = data.ranges
    lidar_ready = True
    
def start():

    global image, img_ready
    global motor, lidar_msg
    nobody_count = 0
    speed  = 0
    angle = 0
    
    rospy.init_node("SSD_driver", anonymous=True)
    motor = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)
    rospy.Subscriber("/usb_cam/image_raw/",Image,img_callback)
    rospy.Subscriber("/scan", LaserScan, lidar_callback)

    net_type = 'mb1-ssd'
    #model_path = rospy.get_param('~model_path', "/home/xytron/xycar_ws/src/ssd_ros/src/models/mobilenet-v1-ssd-mp-0_675.pth")
    #label_path = rospy.get_param('~label_path', "/home/xytron/xycar_ws/src/ssd_ros/src/models/voc-model-labels.txt")
    model_path = rospy.get_param('~model_path')
    label_path = rospy.get_param('~label_path')

    class_names = [name.strip() for name in open(label_path).readlines()]
    num_classes = len(class_names)

    net = create_mobilenetv1_ssd(len(class_names), is_test=True)
    net.load(model_path)

    predictor = create_mobilenetv1_ssd_predictor(net, candidate_size=200)

    print ("----- Xycar SSD driving -----")

    while not image.size == (WIDTH * HEIGHT * 3):
        continue
    print("Camera Ready --------------")
    
    while lidar_msg == None:
        continue
    print("Lidar Ready ----------")

    while not rospy.is_shutdown():

        while img_ready == False:
            continue
        
        img = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

        boxes, labels, probs = predictor.predict(img, 10, 0.2)
        # print('Time: {:.2f}s, Detect Objects: {:d}.'.format(interval, labels.size(0)))

        nobody = True
    
        for i in range(boxes.size(0)):
            box = boxes[i, :]

            Class = class_names[labels[i]]
            probability = float(probs[i])
            ymin = int(box[1])
            xmin = int(box[0])
            ymax = int(box[3])
            xmax = int(box[2])
            id = int(labels[i])
        
            cv2.rectangle(image, (xmin, ymin), (xmax, ymax), (255, 255, 0), 4)
            cv2.putText(image, Class, (xmin+20, ymin+40), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 255), 2) 		
  
            if Class == "person":
                nobody = False
                center = (xmin+xmax)/2
                angle = int(50.0* ((center - 320.0)/320.0))
                speed = 4
                nobody_count = 0
            
        if nobody:
            nobody_count = nobody_count + 1
            if nobody_count > 10:
                angle = 0
                speed = 0
        
        drive(angle,speed)  

        cv2.imshow('SSD Detection', image)
        cv2.waitKey(1)

    cv2.destroyAllWindows()
    

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


if __name__ == '__main__':
    start()


