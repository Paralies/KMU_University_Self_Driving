#!/usr/bin/env python
# -*- coding: utf-8 -*-

import cv2
import numpy as np

image = cv2.imread('yellow.png')

gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
blur_img = cv2.GaussianBlur(gray,(5, 5), 0)
edge_img = cv2.Canny(np.uint8(blur_img), 30, 60)
cv2.imshow('edge', edge_img)

blur_img = cv2.GaussianBlur(image,(5, 5), 0)
lab = cv2.cvtColor(blur_img, cv2.COLOR_BGR2LAB)

L, A, B = cv2.split(lab)

_, L_img = cv2.threshold(L, 155, 255, cv2.THRESH_BINARY)
_, A_img = cv2.threshold(A, 155, 255, cv2.THRESH_BINARY)
_, B_img = cv2.threshold(B, 155, 255, cv2.THRESH_BINARY)

cv2.imshow('original', image)
cv2.imshow('LAB', lab)
cv2.imshow('L', L_img)
cv2.imshow('A', A_img)
cv2.imshow('B', B_img)

edge_img = cv2.Canny(np.uint8(L_img), 30, 60)
cv2.imshow('LAB L Canny', edge_img)

edge_img = cv2.Canny(np.uint8(A_img), 30, 60)
cv2.imshow('LAB A Canny', edge_img)

edge_img = cv2.Canny(np.uint8(B_img), 30, 60)
cv2.imshow('LAB B Canny', edge_img)

cv2.waitKey()

