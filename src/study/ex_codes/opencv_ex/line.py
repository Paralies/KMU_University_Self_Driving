import cv2
import numpy as np

image = cv2.imread('sample.png')
hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

upper_white = np.array([255, 255, 255])
lower_white = np.array([0, 0, 70])

mask = cv2.inRange(hsv, lower_white, upper_white)

cv2.imshow('line', mask)

cv2.waitKey(10000)
