#!/usr/bin/env python
# -*- coding: utf-8 -*-

import cv2
import numpy as np

image = cv2.imread('line_pic1.png')
blur_img = cv2.GaussianBlur(image,(5, 5), 0)

hls = cv2.cvtColor(blur_img, cv2.COLOR_BGR2HLS)
H, L, S = cv2.split(hls)

_, H_img = cv2.threshold(H, 125, 255, cv2.THRESH_BINARY)
_, L_img = cv2.threshold(L, 125, 255, cv2.THRESH_BINARY)
_, S_img = cv2.threshold(S, 125, 255, cv2.THRESH_BINARY)

cv2.imshow('original', image)
cv2.imshow('HLS', hls)
cv2.imshow('H', H_img)
cv2.imshow('L', L_img)
cv2.imshow('S', S_img)

threshed_img = cv2.inRange(L, 125, 255)
edge_img = cv2.Canny(np.uint8(threshed_img), 30, 60)

cv2.imshow('threshed L', threshed_img)
cv2.imshow('HLS Canny', edge_img)
cv2.waitKey()

