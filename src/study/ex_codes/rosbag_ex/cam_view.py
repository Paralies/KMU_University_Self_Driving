#!/usr/bin/env python
# -*- coding: utf-8 -*-

import cv2
import rospy
import numpy as np

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

bridge = CvBridge()
cv_image = np.empty(shape=[0])

def img_callback(data):
    global cv_image
    cv_image = bridge.imgmsg_to_cv2(data, "bgr8")

rospy.init_node('cam_view', anonymous=True)
rospy.Subscriber("/usb_cam/image_raw/", Image, img_callback)

while not rospy.is_shutdown():

    if cv_image.size != (640*480*3):
        continue

    cv2.imshow("rosbag camera topic view", cv_image)
    cv2.waitKey(1)


