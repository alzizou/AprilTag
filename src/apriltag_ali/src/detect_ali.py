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
publish_data = {}
json_data = {}
TF_matrix = np.zeros([4,4])
Tag_center = np.zeros([1,2])
Tag_corners = np.zeros([4,2])


def clean_shutdown():
	cam_obj.release()

def detect_ali():
	global time_stmp_ROS, result, cam_obj, tag_rel_pose, publish_data, json_data
	global TF_matrix, Tag_center, Tag_corners
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
			Tag_center[0][0] = results[0].center[0]
			Tag_center[0][1] = results[0].center[1]
			april_TF_matrix, err1, err2 = detector.detection_pose(results[0], cam_param, tag_size=1, z_sign=1)
			print(april_TF_matrix)
			print(err1)
			print(err2)
			for i in range(0,4):
				Tag_corners[i][0] = results[0].corners[i][0]
				Tag_corners[i][1] = results[0].corners[i][1]
				tag_rel_pose[i] = 0.0
				for j in range(0,4):
					tag_rel_pose[i] = tag_rel_pose[i] + ( april_TF_matrix[i][j] * camera_pose[j] )
			print(tag_rel_pose)
			#publish_data["detection_center"] = [results[0].center[0],results[0].center[1]]
			#publish_data["detection_corners0"] = [results[0].corners[0][0],results[0].corners[0][1]]
			#publish_data["TF_Matrix"] = TF_matrix
			publish_data["Relative_Pose"] = tag_rel_pose
			result = False
		#(END) Tag detection and pose estimation using AprilTag package
		#---------------------------------------------------------------------------------------------------
		#(START) Publishing data
		json_data = json.dumps(publish_data)
		pub_apriltag.publish(json_data)
		#(END) Publishing data
		#---------------------------------------------------------------------------------------------------
#	rospy.spin()


if __name__ == "__main__":
	try:
		detect_ali()
	except (rospy.ROSInterruptException, rospy.ServiceException, rospy.ROSException):
		pass

