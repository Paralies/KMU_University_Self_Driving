#!/usr/bin/env python
# -*- coding: utf-8 -*-

import cv2
import numpy as np

image = cv2.imread('line_pic1.png')

blur_img = cv2.GaussianBlur(image,(5, 5), 0)

hsv = cv2.cvtColor(blur_img, cv2.COLOR_BGR2HSV)

upper_white = np.array([255, 255, 255])
lower_white = np.array([0, 0, 150])

threshed_img = cv2.inRange(hsv, lower_white, upper_white)

edge_img = cv2.Canny(np.uint8(threshed_img), 30, 60)

cv2.imshow("blur", blur_img)
cv2.imshow("hsv", hsv)
cv2.imshow("threshed", threshed_img)
cv2.imshow("hsv edge", edge_img)
cv2.waitKey()

