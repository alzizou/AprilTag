# AprilTag
This is a local repo for the well-known AprilTag relative 3D position estimation using camera

For calibration use the following command inside the appropriate folder:

python3 calibrate_camera.py filenames "chess_img.jpg" --rows 8 --cols 6 --size 20

Note-1: the last parameter is the size of each square on the chess board.
Note-2: the output of the camera calibration would be the estimated values for (fx, fy, cx, cy) in 		this order!

####################################################

Note: One important parameter to have accurate relative position estimation is the "tag_size" as the third input parameter for "detection_pose" function. This should be calibrated by some priliminary validation tests in the lab/field. 
