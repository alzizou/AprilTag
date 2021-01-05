#!/usr/bin/env python3
import rospy
import cv2
import apriltag
import json
import numpy as np
import math as mt
from std_msgs.msg import String

camera_pose = [0.0,0.0,0.0]
cam_param = (100,100,200,200)

tag_rel_pose = [0.0,0.0,0.0]
cam_obj = cv2.VideoCapture(1)
result = True
time_stmp_ROS = 0.0
publish_data = {}
json_data = {}
TF_matrix = np.zeros([4,4])
Tag_center = np.zeros([1,2])
Tag_corners = np.zeros([4,2])
Euler_Sol1 = [0.0,0.0,0.0]
Euler_Sol2 = [0.0,0.0,0.0]
Quat = [0.0,0.0,0.0,0.0]
Unit_Vector = [0.0,0.0,0.0]

Results_File_AprilTag_RelPos = open(r"/home/ali/ali_ws/AprilTag/logs/Results_File_AprilTag_RelPos.txt","w+")
Results_File_AprilTag_Euler1 = open(r"/home/ali/ali_ws/AprilTag/logs/Results_File_AprilTag_Euler1.txt","w+")
Results_File_AprilTag_Euler2 = open(r"/home/ali/ali_ws/AprilTag/logs/Results_File_AprilTag_Euler2.txt","w+")
Results_File_AprilTag_Quat = open(r"/home/ali/ali_ws/AprilTag/logs/Results_File_AprilTag_Quat.txt","w+")
Results_File_AprilTag_UnitVector = open(r"/home/ali/ali_ws/AprilTag/logs/Results_File_AprilTag_UnitVector.txt","w+")

def clean_shutdown():
	cam_obj.release()
	Results_File_AprilTag_RelPos.close()
	Results_File_AprilTag_Euler1.close()
	Results_File_AprilTag_Euler2.close()
	Results_File_AprilTag_Quat.close()
	Results_File_AprilTag_UnitVector.close()


def rel_position_estimate(inp_tf,inp_cam_pos):
	global tag_rel_pose
	for i in range(0,3):
		tag_rel_pose[i] = inp_tf[i][3]
		for j in range(0,3):
			tag_rel_pose[i] = tag_rel_pose[i] + ( inp_tf[i][j] * inp_cam_pos[j] )
	print(tag_rel_pose)


def euler_eval(inp_tf):
	# (source: https://www.gregslabaugh.net/publications/euler.pdf)
	global Euler_Sol1, Euler_Sol2
	if (abs(inp_tf[2][0])<1.0):
		Pitch1 = -mt.asin(inp_tf[2][0])
		Pitch2 = mt.pi - Pitch1
		Roll1 = mt.atan2((inp_tf[1][0]/mt.cos(Pitch1)),(inp_tf[0][0]/mt.cos(Pitch1)))
		Roll2 = mt.atan2((inp_tf[1][0]/mt.cos(Pitch2)),(inp_tf[0][0]/mt.cos(Pitch2)))
		Yaw1 = mt.atan2((inp_tf[2][1]/mt.cos(Pitch1)),(inp_tf[2][2]/mt.cos(Pitch1)))
		Yaw2 = mt.atan2((inp_tf[2][1]/mt.cos(Pitch2)),(inp_tf[2][2]/mt.cos(Pitch2)))
	else:
		if(inp_tf[2][0] == 1.0):
			Pitch1 = mt.pi/2.0
			Pitch2 = Pitch1
			Roll1 = 0.0
			Roll2 = Roll1
			Yaw1 = Roll1 + mt.atan2(inp_tf[0][1],inp_tf[0][2])
			Yaw2 = Yaw2
		else:
			Pitch1 = -mt.pi/2.0
			Pitch2 = Pitch1
			Roll1 = 0.0
			Roll2 = Roll1
			Yaw1 = -Roll1 + mt.atan2(-inp_tf[0][1],-inp_tf[0][2])
			Yaw2 = Yaw2
	Euler_Sol1 = [Roll1,Pitch1,Yaw1]
	Euler_Sol2 = [Roll2,Pitch2,Yaw2]
	print("Euler angles (SOL1):\r\n")
	print(Euler_Sol1)
	print("Euler angles (SOL2):\r\n")
	print(Euler_Sol2)


def quat_eval(inp_tf):
	# (source: http://www.euclideanspace.com/maths/geometry/rotations/conversions/matrixToQuaternion/)
	global Quat
	quat_T = inp_tf[0][0] + inp_tf[1][1] + inp_tf[2][2]
	if (quat_T > 0):
		quat_S = mt.sqrt(quat_T + 1.0) * 2
		quat_w = 0.25 * quat_S
		quat_x = (inp_tf[2][1] - inp_tf[1][2]) / quat_S
		quat_y = (inp_tf[0][2] - inp_tf[2][0]) / quat_S
		quat_z = (inp_tf[1][0] - inp_tf[0][1]) / quat_S
	else:
		if ( (inp_tf[0][0]>inp_tf[1][1]) & (inp_tf[0][0]>inp_tf[2][2]) ):
			quat_S = mt.sqrt(1.0 + inp_tf[0][0] - inp_tf[1][1] - inp_tf[2][2]) * 2.0
			quat_w = (inp_tf[2][1] - inp_tf[1][2]) / quat_S
			quat_x = 0.25 * quat_S
			quat_y = (inp_tf[0][1] + inp_tf[1][0]) / quat_S
			quat_z = (inp_tf[0][2] + inp_tf[2][0]) / quat_S
		else:
			if (inp_tf[1][1]>inp_tf[2][2]):
				quat_S = mt.sqrt(1.0 + inp_tf[1][1] - inp_tf[0][0] - inp_tf[2][2]) * 2.0
				quat_w = (inp_tf[0][2] - inp_tf[2][0]) / quat_S
				quat_x = (inp_tf[0][1] + inp_tf[1][0]) / quat_S
				quat_y = 0.25 * quat_S
				quat_z = (inp_tf[1][2] + inp_tf[2][1]) / quat_S
			else:
				quat_S = mt.sqrt(1.0 + inp_tf[2][2] - inp_tf[0][0] - inp_tf[1][1]) * 2.0
				quat_w = (inp_tf[1][0] - inp_tf[0][1]) / quat_S
				quat_x = (inp_tf[0][2] + inp_tf[2][0]) / quat_S
				quat_y = (inp_tf[1][2] + inp_tf[2][1]) / quat_S
				quat_z = 0.25 * quat_S
	Quat = [quat_w,quat_x,quat_y,quat_z]
	print("Quaternions:\r\n")
	print(Quat)


def unit_vector_eval(inp_tf):
	# to compute the unit vector of the link in the drone frame
	global Unit_Vector
	local_vec = [0.0,0.0,-1.0]
	for i in range(0,3):
		Unit_Vector[i] = 0.0
		for j in range(0,3):
			Unit_Vector[i] = Unit_Vector[i] + ( inp_tf[i][j] * local_vec[j] )
	print("Unit vector:\r\n")
	print(Unit_Vector)


def detect_ali():
	global time_stmp_ROS, result, cam_obj, tag_rel_pose, publish_data, json_data
	global TF_matrix, Tag_center, Tag_corners
	global Euler_Sol1, Euler_Sol2, Quat, Unit_Vector
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
			april_TF_matrix, err1, err2 = detector.detection_pose(results[0], cam_param, tag_size=1, z_sign=1)
			print(april_TF_matrix)
			print(err1)
			print(err2)
			#---------------------------------------------------------
			#(START) constructing the variables for publishing
			Tag_center[0][0] = results[0].center[0]
			Tag_center[0][1] = results[0].center[1]
			for i in range(0,4):
				Tag_corners[i][0] = results[0].corners[i][0]
				Tag_corners[i][1] = results[0].corners[i][1]
				for j in range(0,4):
					TF_matrix[i][j] = april_TF_matrix[i][j]
			#(END) constructing the variables for publishing
			#---------------------------------------------------------
			#(START) relative position estimation using the affine transformation
			rel_position_estimate(TF_matrix,camera_pose)
			Results_File_AprilTag_RelPos.write("%.9f %.6f %.6f %.6f\r\n" %
			                (rospy.get_time(),tag_rel_pose[0],tag_rel_pose[1],tag_rel_pose[2]))
			#(END) relative position estimation using the affine transformation
			#---------------------------------------------------------
			#(START) computng the Euler angles
			euler_eval(TF_matrix)
			Results_File_AprilTag_Euler1.write("%.9f %.6f %.6f %.6f\r\n" %
			                (rospy.get_time(),Euler_Sol1[0],Euler_Sol1[1],Euler_Sol1[2]))
			Results_File_AprilTag_Euler2.write("%.9f %.6f %.6f %.6f\r\n" %
			                (rospy.get_time(),Euler_Sol2[0],Euler_Sol2[1],Euler_Sol2[2]))
			#(END) computing the Euler angles
			#---------------------------------------------------------
			#(START) computing the quaternion
			quat_eval(TF_matrix)
			Results_File_AprilTag_Quat.write("%.9f %.6f %.6f %.6f %.6f\r\n" %
			                (rospy.get_time(),Quat[0],Quat[1],Quat[2],Quat[3]))
			#(END) computing the quaternion
			#---------------------------------------------------------
			#(START) computing the unit vector
			unit_vector_eval(TF_matrix)
			Results_File_AprilTag_UnitVector.write("%.9f %.6f %.6f %.6f\r\n" %
			                (rospy.get_time(),Unit_Vector[0],Unit_Vector[1],Unit_Vector[2]))
			#(END) computign the unit vector
			#---------------------------------------------------------
			publish_data["Relative_Pose"] = tag_rel_pose
			publish_data["Euler_Angle1"] = Euler_Sol1
			publish_data["Euler_Angle2"] = Euler_Sol2
			publish_data["Quaternion"] = Quat
			publish_data["Unit_Vector"] = Unit_Vector
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

