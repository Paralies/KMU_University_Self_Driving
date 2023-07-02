import cv2

img = cv2.imread('girl.png', cv2.IMREAD_GRAYSCALE)

cv2.imshow('My Girl (gray)', img)

cv2.waitKey(10000)
cv2.destroyAllWindows()
