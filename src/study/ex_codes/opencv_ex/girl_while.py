import cv2

while(True):
    img = cv2.imread('girl.png', cv2.IMREAD_COLOR)
    cv2.imshow('My Girl', img)
    cv2.waitKey(1)

