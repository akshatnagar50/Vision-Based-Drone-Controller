#!/usr/bin/env python

import numpy as np
import cv2
import cv2.aruco as aruco

aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_250)
cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners, ids, _ = aruco.detectMarkers(gray, aruco_dict)

    if corners:
        aruco.drawDetectedMarkers(frame, corners)
        print("Number of ArUco markers detected: ", len(corners))
        for i in range(len(corners)):
            print("ID: ", ids[i], " Location: ", corners[i])

    cv2.imshow('Frame', frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()