import cv2
import numpy as np

img = cv2.imread('yellow.png')
lab = cv2.cvtColor(img, cv2.COLOR_BGR2LAB)

def nothing(x):
    pass

cv2.namedWindow('lab')
cv2.createTrackbar('L low', 'lab',0,255, nothing)
cv2.createTrackbar('L high','lab',0,255, nothing)
cv2.createTrackbar('A low', 'lab',0,255, nothing)
cv2.createTrackbar('A high','lab',0,255, nothing)
cv2.createTrackbar('B low', 'lab',0,255, nothing)
cv2.createTrackbar('B high','lab',0,255, nothing)

while True:
    if cv2.waitKey(1) & 0xFF == 27:
        break
    l_low  = cv2.getTrackbarPos('L low', 'lab')
    l_high = cv2.getTrackbarPos('L high','lab')
    a_low  = cv2.getTrackbarPos('A low', 'lab')
    a_high = cv2.getTrackbarPos('A high','lab')
    b_low  = cv2.getTrackbarPos('B low', 'lab')
    b_high = cv2.getTrackbarPos('B high','lab')

    low_mask = (l_low, a_low, b_low)
    high_mask = (l_high, a_high, b_high)

    lab_img = cv2.inRange(lab, low_mask, high_mask)
    cv2.imshow('lab',lab_img)

cv2.destroyAllWindows()
