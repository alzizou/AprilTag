#!/usr/bin/env python3
#import rospy
import cv2
import apriltag
import json
import numpy as np
import math as mt

camera_pose = [0.0,0.0,0.0,1.0]
tag_rel_pose = [0.0,0.0,0.0,0.0]
cam_param = (100,100,200,200)
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
	if ((not results)==0):
		print(results[0].center)
		print(results[0].corners)
		TF_matrix, err1, err2 = detector.detection_pose(results[0], cam_param, tag_size=1, z_sign=1)
		print(TF_matrix)
		print(err1)
		print(err2)
		for i in range(0,4):
			tag_rel_pose[i] = 0.0
			for j in range(0,4):
				tag_rel_pose[i] = tag_rel_pose[i] + ( TF_matrix[i][j] * camera_pose[j] )
		print(tag_rel_pose)
		result = False
cam_obj.release()

