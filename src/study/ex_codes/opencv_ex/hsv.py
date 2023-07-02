import cv2
import numpy as np

img = cv2.imread('cars.png')
hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

upper_white = np.array([255,255,255])
lower_white = np.array([0,0,150])

mask = cv2.inRange(hsv, lower_white, upper_white)

cv2.imshow('line', mask)

cv2.waitKey(10000)
