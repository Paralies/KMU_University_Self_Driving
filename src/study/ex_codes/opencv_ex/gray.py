import cv2

img = cv2.imread('cars.png')
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

cv2.imshow('color', img)
cv2.imshow('gray', gray)
cv2.waitKey(10000)
