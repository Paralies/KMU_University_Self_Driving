import cv2

img = cv2.imread('cars.png')

cv2.imshow('car', img[120:270, 270:460])

cv2.waitKey(10000)
