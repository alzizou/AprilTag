#!/usr/bin/env python3
#import rospy
import cv2
import apriltag
import json
import numpy as np
import math as mt

cam_obj = cv2.VideoCapture(1)
result = True
while (result):
	ret,frame = cam_obj.read()
	cv2.imwrite("/home/ali/ali_ws/AprilTag/image.jpg",frame)
	image = cv2.imread("/home/ali/ali_ws/AprilTag/image.jpg")
	gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
	cv2.imwrite("/home/ali/ali_ws/AprilTag/image_gray.jpg",gray)
	options = apriltag.DetectorOptions(families="tag36h11")
	detector = apriltag.Detector(options)
	results = detector.detect(gray)
	result = False
cam_obj.release()

