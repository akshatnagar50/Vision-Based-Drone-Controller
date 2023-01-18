#!/usr/bin/env python
from plutodrone.srv import *
import rospy
from std_msgs.msg import Float64
import cv2
import cv2.aruco as aruco
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped
import numpy as np
import math

class request_data():
	"""docstring for request_data"""
	def __init__(self):
		rospy.init_node('drone_board_data')
		data = rospy.Service('PlutoService', PlutoPilot, self.access_data)
		
	def access_data(self, req):
		r,p,y = req.roll, req.pitch, req.yaw
		# print ("accx = " + str(req.accX), "accy = " + str(req.accY), "accz = " + str(req.accZ))
		# print ("gyrox = " + str(req.gyroX), "gyroy = " + str(req.gyroY), "gyroz = " + str(req.gyroZ))
		# print ("magx = " + str(req.magX), "magy = " + str(req.magY), "magz = " + str(req.magZ))
		# print ("roll = " + str(req.roll), "pitch = " + str(req.pitch), "yaw = " + str(req.yaw))
		# print ("altitude = " +str(req.alt))
		# print ("battery = " + str(req.battery), "Power Consumed = " + str(req.rssi))

		pub = rospy.Publisher("Detection", PoseStamped, queue_size=10)
		obj = PoseStamped()

		aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_250)
		# cap = cv2.VideoCapture(0)
		marker_size = 0.028  # meters
		fov_degrees = 72
		fov_radians = fov_degrees * (math.pi / 180)
		mtx = np.array([[1.52085585e+03, 0.00000000e+00, 9.56707469e+02],
						[0.00000000e+00, 1.52222433e+03, 5.22968509e+02],
						[0.00000000e+00, 0.00000000e+00, 1.00000000e+00]])

		dist = np.array(
			[[0.05334265,  0.15252761, 0.00372602, 0.00134423, -1.13086175]])

		bad_mtx = np.array([[698.30269162,   0,         482.23369024],
							[0,         699.30531713, 281.24277949],
							[0,           0,           1.]])

		bad_dist = np.array(
			[[-0.14822482,  0.52992297, -0.005417,   -0.00265437, -0.75054646]])

		#rospy.init_node("Detection")
		rate = rospy.Rate(10)
		pub = rospy.Publisher("Detection", PoseStamped, queue_size=10)
		obj = PoseStamped()
		global cap
		ret, frame = cap.read()
		gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
		params = aruco.DetectorParameters_create()
		corners, ids, _ = aruco.detectMarkers(
			gray, aruco_dict, parameters=params)

		if corners:
			aruco.drawDetectedMarkers(frame, corners)
			cv2.imshow('Frame',frame)

		rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
			corners, marker_size, bad_mtx, bad_dist)

		marker_height = -1

		if rvecs is not None and tvecs is not None:
			rvec = rvecs[0]
			rmat, _ = cv2.Rodrigues(rvec)
			tvec = tvecs[0]
			if rmat.shape[0] != tvec.shape[0]:
				tvec = tvec.reshape(rmat.shape[0], 1)
			if rmat.dtype != tvec.dtype:
				tvec = tvec.astype(rmat.dtype)
			pose_mat = cv2.hconcat((rmat, tvec))
			tvec = tvecs[0]
			if tvec is not None:
				marker_height = tvec[0][2]



		if marker_height != -1:
			obj.pose.position.x = tvec[0][0]
			obj.pose.position.y = tvec[0][1]
			obj.pose.position.z = tvec[0][2]
			obj.pose.orientation.x = r
			obj.pose.orientation.y = p
			obj.pose.orientation.z = y
			pub.publish(obj)
			print (obj)
			rospy.loginfo("Publishing")


		return PlutoPilotResponse(rcAUX2 =1500)

cap = cv2.VideoCapture(0)
test = request_data()
rospy.spin()
