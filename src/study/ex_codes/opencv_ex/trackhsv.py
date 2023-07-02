import cv2
import numpy as np

img = cv2.imread('yellow.png')
hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

def nothing(x):
    pass

cv2.namedWindow('hsv')
cv2.createTrackbar('H low', 'hsv',0,255, nothing)
cv2.createTrackbar('H high','hsv',0,255, nothing)
cv2.createTrackbar('S low', 'hsv',0,255, nothing)
cv2.createTrackbar('S high','hsv',0,255, nothing)
cv2.createTrackbar('V low', 'hsv',0,255, nothing)
cv2.createTrackbar('V high','hsv',0,255, nothing)

while True:
    if cv2.waitKey(1) & 0xFF == 27:
        break
    h_low  = cv2.getTrackbarPos('H low', 'hsv')
    h_high = cv2.getTrackbarPos('H high','hsv')
    s_low  = cv2.getTrackbarPos('S low', 'hsv')
    s_high = cv2.getTrackbarPos('S high','hsv')
    v_low  = cv2.getTrackbarPos('V low', 'hsv')
    v_high = cv2.getTrackbarPos('V high','hsv')

    low_mask = (h_low, s_low, v_low)
    high_mask = (h_high, s_high, v_high)

    hsv_img = cv2.inRange(hsv, low_mask, high_mask)
    cv2.imshow('hsv',hsv_img)

cv2.destroyAllWindows()
