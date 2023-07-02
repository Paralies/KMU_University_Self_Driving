import cv2
import numpy as np

img = cv2.imread('yellow.png')
hls = cv2.cvtColor(img, cv2.COLOR_BGR2HLS)

def nothing(x):
    pass

cv2.namedWindow('hls')
cv2.createTrackbar('H low', 'hls',0,255, nothing)
cv2.createTrackbar('H high','hls',0,255, nothing)
cv2.createTrackbar('L low', 'hls',0,255, nothing)
cv2.createTrackbar('L high','hls',0,255, nothing)
cv2.createTrackbar('S low', 'hls',0,255, nothing)
cv2.createTrackbar('S high','hls',0,255, nothing)

while True:
    if cv2.waitKey(1) & 0xFF == 27:
       break
    h_low  = cv2.getTrackbarPos('H low', 'hls')
    h_high = cv2.getTrackbarPos('H high','hls')
    s_low  = cv2.getTrackbarPos('L low', 'hls')
    s_high = cv2.getTrackbarPos('L high','hls')
    v_low  = cv2.getTrackbarPos('S low', 'hls')
    v_high = cv2.getTrackbarPos('S high','hls')

    low_mask = (h_low, s_low, v_low)
    high_mask = (h_high, s_high, v_high)

    hls_img = cv2.inRange(hls, low_mask, high_mask)
    cv2.imshow('hls',hls_img)

cv2.destroyAllWindows()
