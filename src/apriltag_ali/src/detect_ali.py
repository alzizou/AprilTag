#!/usr/bin/env python3
import rospy
import cv2
import apriltag
import json
import numpy as np
import math as mt
from std_msgs.msg import String

camera_pose = [0.0,0.0,0.0,1.0]
tag_rel_pose = [0.0,0.0,0.0,0.0]
cam_param = (100,100,200,200)
cam_obj = cv2.VideoCapture(1)
result = True
time_stmp_ROS = 0.0
json_data = String()

def clean_shutdown():
	cam_obj.release()

def detect_ali():
	global time_stmp_ROS. result, cam_obj, tag_rel_pose
	rospy.init_node("detect_ali",anonymous=True)
	rate = rospy.Rate(100)
	rospy.on_shutdown(clean_shutdown)
	pub_apriltag = rospy.Publisher("Data_AprilTag",String,queue_size=1)
	while not rospy.is_shutdown():
		rate.sleep()
		period = float(rospy.get_time()) - time_stmp_ROS
		frequency = 1.0/period
		if (time_stmp_ROS<1.0):
			period = 0.0
		time_stmp_ROS = float(rospy.get_time())
		print("Frequency of AptilTag: %.2f" % frequency)
		#---------------------------------------------------------------------------------------------------
		#(START) Tag detection and pose estimation using AprilTag package
		ret,frame = cam_obj.read()
		cv2.imwrite("image.jpg",frame)
		image = cv2.imread("image.jpg")
		gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
		cv2.imwrite("image_gray.jpg",gray)
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
		#(END) Tag detection and pose estimation using AprilTag package
		#---------------------------------------------------------------------------------------------------
		#(START) Publishing data
		json_data["detection"] = results
		json_data["Transformation_Matrix"] = TF_matrix
		json_data["Relative_Pose"] = tag_rel_pose
		pub_apriltag.publish(json_data.dumps())
		#(END) Publishing data
		#---------------------------------------------------------------------------------------------------
#	rospy.spin()


if __name__ == "__main__":
	try:
		detect_ali()
	except (rospy.ROSInterruptException, rospy.ServiceException, rospy.ROSException):
		pass

