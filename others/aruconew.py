import numpy as np
import cv2
import cv2.aruco as aruco

aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_250)
cap = cv2.VideoCapture(1
    ,cv2.CAP_DSHOW
    )
print(cap.get(cv2.CAP_PROP_FRAME_WIDTH),cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
width = 1920
height = 1080
cap.set(cv2.CAP_PROP_FRAME_WIDTH,width)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT,height)
print(cap.get(cv2.CAP_PROP_FRAME_WIDTH),cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

if True:
    mtx = np.array([[698.30269162,   0,         482.23369024],
    [  0,         699.30531713, 281.24277949],
    [  0,           0,           1.        ]])
    # change this after calibratioCAP_PROP_FRAME_WIDTHn

    dist = np.array([[-0.14822482,  0.52992297, -0.005417,   -0.00265437, -0.75054646]])
    # change this after calibration



while True:
    ret, frame = cap.read()
    print(frame.shape)
    
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    params = aruco.DetectorParameters()
    # corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters = params)
    detector = aruco.ArucoDetector(aruco_dict, params)
    cv2.imshow('Gray',gray)
    corners, ids, _ = detector.detectMarkers(gray)
    print(corners)

    if corners:
        aruco.drawDetectedMarkers(gray, corners)
      
    # rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners, 0.168656, mtx, dist)
    
    # r,p,y,marker_height = 0,0,0,-1


    # if rvecs is not None and tvecs is not None:
    #     rvec = rvecs[0]
    #     rmat, _ = cv2.Rodrigues(rvec)
    #     tvec = tvecs[0]
    #     if rmat.shape[0] != tvec.shape[0]:
    #         tvec = tvec.reshape(rmat.shape[0], 1)
    #     if rmat.dtype != tvec.dtype:
    #         tvec = tvec.astype(rmat.dtype)
    #     pose_mat = cv2.hconcat((rmat, tvec))
    #     tvec = tvecs[0]
    #     if tvec is not None:
    #         marker_height = tvec[0][2]
    #     _,_,_,_,_,_,euler_angles = cv2.decomposeProjectionMatrix(pose_mat)
    #     r,p,y = euler_angles



    # # print("Roll:" + str(r))
    # # print("Pitch:" + str(p))
    # # print("Yaw:" + str(y))
    # # print("Height:" + str(marker_height))
    # # print("")
    # # print(cap.get(cv2.CAP_PROP_FRAME_WIDTH))


    cv2.imshow('Frame', frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()