#!/usr/bin/env python
# -*- coding: utf-8 -*-

import cv2
import numpy as np

image = cv2.imread('yellow.png')

blur = cv2.GaussianBlur(image, (5,5), 0)
hls = cv2.cvtColor(blur, cv2.COLOR_BGR2HLS)

hls_yellow = cv2.inRange(hls, (0, 100, 40), (40, 200, 200))
hls_yellow_n_white = cv2.inRange(hls, (0, 95, 0), (255, 255, 255))

white_lane_img = cv2.bitwise_xor(hls_yellow, hls_yellow_n_white)

cv2.imshow('original', image)
cv2.imshow('blur', blur)
cv2.imshow('hls_yellow', hls_yellow)
cv2.imshow('hls_yellow_n_white', hls_yellow_n_white)
cv2.imshow('white_lane_img', white_lane_img)

cv2.waitKey()

