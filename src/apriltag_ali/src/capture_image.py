#!/usr/bin/env python3
import cv2
import numpy as np
import math as mt

cam_obj = cv2.VideoCapture(1)
result = True

while result:
	ret,frame = cam_obj.read()
	cv2.imwrite("chess_img.jpg",frame)
	image = cv2.imread("chess_img.jpg")
	gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
	cv2.imwrite("chess_img_gray.jpg",gray)
	result = False

cam_obj.release()
