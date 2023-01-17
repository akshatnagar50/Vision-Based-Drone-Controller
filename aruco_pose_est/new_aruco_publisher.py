#!/usr/bin/env python

import numpy as np
import cv2
import cv2.aruco as aruco
import math
import rospy
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseStamped
def detect_and_publish():
    aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_250)
    cap = cv2.VideoCapture(0)
    marker_size = 0.028  # meters
    fov_degrees = 72
    fov_radians = fov_degrees * (math.pi / 180)
    mtx = np.array([[1.52085585e+03, 0.00000000e+00, 9.56707469e+02],
 [0.00000000e+00 ,1.52222433e+03 ,5.22968509e+02],
 [0.00000000e+00 ,0.00000000e+00 ,1.00000000e+00]])

    bad_mtx = np.array([[698.30269162,   0,         482.23369024],
        [  0,         699.30531713, 281.24277949],
        [  0,           0,           1.        ]])
    

    bad_dist = np.array([[-0.14822482,  0.52992297, -0.005417,   -0.00265437, -0.75054646]])

    dist = np.array([[0.05334265,  0.15252761 , 0.00372602 , 0.00134423 ,-1.13086175]])
    rospy.init_node("Detection")
    rate=rospy.Rate(10)
    pub = rospy.Publisher("Detection", PoseStamped, queue_size=10)
    obj=PoseStamped()

    while not rospy.is_shutdown():
        ret, frame = cap.read()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        params = aruco.DetectorParameters_create()
        corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters = params)

        if corners:
            aruco.drawDetectedMarkers(frame, corners)
            
        rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners, marker_size, bad_mtx, bad_dist)
        
        r,p,y,marker_height = 0,0,0,-1


        if rvecs is not None and tvecs is not None:
            rvec = rvecs[0]
            rmat, _ = cv2.Rodrigues(rvec)
            tvec = tvecs[0]
            # check if the number of rows of rmat and tvec match
            if rmat.shape[0] != tvec.shape[0]:
                # reshape tvec to match the number of rows of rmat
                tvec = tvec.reshape(rmat.shape[0], 1)
            # check if the data types of rmat and tvec match
            if rmat.dtype != tvec.dtype:
                # convert the data type of tvec to match that of rmat
                tvec = tvec.astype(rmat.dtype)
            # concatenate rmat and tvec horizontally
            pose_mat = cv2.hconcat((rmat, tvec))
            tvec = tvecs[0]
            if tvec is not None:
                marker_height = tvec[0][2]
                marker_size_pixels = cv2.norm(corners[0][0][0] - corners[0][0][1])
                #marker_height = marker_size / (2 * math.tan(fov_radians / 2) * (marker_size_pixels / frame.shape[1]))
            _,_,_,_,_,_,euler_angles = cv2.decomposeProjectionMatrix(pose_mat)
            r,p,y = euler_angles



        print("Roll:" + str(r))
        print("Pitch:" + str(p))
        print("Yaw:" + str(y))
        print("Height:" + str(marker_height))
        print()
        if marker_height!=-1:
            obj.pose.position.x=r
            obj.pose.position.y=p
            obj.pose.position.z=y
            obj.pose.orientation.x=marker_height
            pub.publish(obj)
        cv2.imshow('Frame', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        rate.sleep()
    cap.release()
    cv2.destroyAllWindows()



if __name__ == '__main__':
    try:
        detect_and_publish()
    except rospy.ROSInterruptException:
        pass
