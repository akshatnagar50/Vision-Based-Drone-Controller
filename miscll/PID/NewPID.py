'''
Things to do:
(0) Collect data for a step input
(1) Add derivative filtering term(exponential smoothing or any other?) and test
(2) Add anti windup term for integral
(3) Get roll/pitch feedback and add inner PID loop for roll&pitch 
'''

'''Questions:
1. Have u tried running full rcPitch/roll? not good to run on full value? Need a low upper cap? (1400-1600)?
2. Need to enable D&I terms only after some time after the start?
3. Exact mean is 1500 for r,p,y?
4. Need d(e)/dt itself instead of d(feedback)/dt?
5. 
'''


#!/usr/bin/env python3
from cmath import inf
import time
import math
import rospy
from plutodrone.msg import PlutoMsg
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32
from matplotlib import pyplot as plt
import pickle
import pandas as pd
import numpy as np
import cv2
import cv2.aruco as aruco
import sys, time, math
cam_orientation=0
rc_th = []
rc_r  = []
rc_p  = []
rc_y  = []
sen_r = []
sen_p = []
sen_y = []
cam_x = []
cam_y = []
cam_z = []
# import csv

class PID:

    def arm(self,pub):
        obj = PlutoMsg()
        obj.rcPitch    = 1500
        obj.rcRoll     = 1500
        obj.rcYaw      = 1500
        obj.rcAUX4     = 1500
        obj.rcAUX3     = 1500
        obj.rcAUX2     = 1500                
        obj.rcAUX1     = 1500
        obj.rcThrottle = 1000
        t=time.time()
        while(time.time()-t<5):
            pub.publish(obj)
        print('SLept')

    """
    Implements a PID controller.
    """

    def __init__(self,K_z: float, K_roll: float, K_pitch: float, K_yaw: float, dt:float, tau:float, alpha:float,cam_orientation:float) -> None:
        
        self.dt       = dt
        self.tau      = tau
        self.alpha    = alpha  # exponential smoothing factor
        self.memory_thr    = 0
        self.memory_roll   = 0
        self.current_consecutive_frames=0
        self.memory_pitch  = 0
        self.memory_yaw    = 0

        self.rc_th = []
        self.rc_r  = []
        self.rc_p  = []
        self.rc_y  = []
        self.sen_r = []
        self.sen_p = []
        self.sen_y = []
        self.cam=cam_orientation
        self.cam_x = []
        self.cam_y = []
        self.cam_z = []

        self.Kp_z     = K_z[0]
        self.Ki_z     = K_z[1]
        self.Kd_z     = K_z[2]

        self.Kp_roll  = K_roll[0]
        self.Ki_roll  = K_roll[1]
        self.Kd_roll  = K_roll[2]

        self.Kp_pitch = K_pitch[0]
        self.Ki_pitch = K_pitch[1]
        self.Kd_pitch = K_pitch[2]
        
        self.Kp_yaw   = K_yaw[0]
        self.Ki_yaw   = K_yaw[1]
        self.Kd_yaw   = K_yaw[2]

        # self.target = target

        # setting the initial Derivative&Integral term as 0
        self.Pterm_z             = 0
        self.Dterm_z             = 0   
        self.Iterm_z             = 0
        self.Pterm_roll          = 0
        self.Dterm_roll          = 0
        self.Iterm_roll          = 0
        self.Pterm_pitch         = 0
        self.Dterm_pitch         = 0
        self.Iterm_pitch         = 0
        self.Dterm_yaw           = 0
        self.Iterm_yaw           = 0
        
        # setting the initial error 0
        self.last_error_z        = 0 
        self.last_error_roll     = 0
        self.last_error_pitch    = 0
        self.last_error_yaw      = 0

        # setting the initial z,r,p,y feedback = 0
        self.last_feedback_z     = 0 
        self.last_feedback_roll  = 0
        self.last_feedback_pitch = 0
        self.last_feedback_yaw   = 0
        self.last_feedback_z_filtered     = 0
        self.last_feedback_roll_filtered  = 0
        self.last_feedback_pitch_filtered = 0
        self.last_feedback_yaw_filtered   = 0

        # setting the initial PID outputs = 0
        self.last_output_z       = 0 
        self.last_output_roll    = 0
        self.last_output_pitch   = 0
        self.last_output_yaw     = 0

        self.last_time = time.time()   

#         self.set_limits(1000, 2000)
        
        self.plotlist_throttle = []
        self.plotlist_height   = []


        rospy.init_node("PID")
        self.pub = rospy.Publisher("/drone_command",PlutoMsg,queue_size=10)
        self.arm(self.pub)
        # rospy.Subscriber("Detection",PoseStamped,self.controller_out)
        rospy.Subscriber("Kp_z",Float32,self.setKp_z)
        rospy.Subscriber("Kp_roll",Float32,self.setKp_roll)
        rospy.Subscriber("Kp_pitch",Float32,self.setKp_pitch)

        id_to_find  = 0
        marker_size  = 4.3 #- [cm]


        #------------------------------------------------------------------------------
        #------- ROTATIONS https://www.learnopencv.com/rotation-matrix-to-euler-angles/
        #------------------------------------------------------------------------------
        # Checks if a matrix is a valid rotation matrix.
        def isRotationMatrix(R):
            Rt = np.transpose(R)
            shouldBeIdentity = np.dot(Rt, R)
            I = np.identity(3, dtype=R.dtype)
            n = np.linalg.norm(I - shouldBeIdentity)
            return n < 1e-6


        # Calculates rotation matrix to euler angles
        # The result is the same as MATLAB except the order
        # of the euler angles ( x and z are swapped ).
        def rotationMatrixToEulerAngles(R):
            assert (isRotationMatrix(R))

            sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])

            singular = sy < 1e-6

            if not singular:
                x = math.atan2(R[2, 1], R[2, 2])
                y = math.atan2(-R[2, 0], sy)
                z = math.atan2(R[1, 0], R[0, 0])
            else:
                x = math.atan2(-R[1, 2], R[1, 1])
                y = math.atan2(-R[2, 0], sy)
                z = 0

            return np.array([x, y, z])




        #--- Get the camera calibration path
        calib_path  = ""
        camera_matrix   = np.loadtxt(calib_path+'camera_matrix.txt', delimiter=',')
        camera_distortion   = np.loadtxt(calib_path+'camera_distortion.txt', delimiter=',')

        #--- 180 deg rotation matrix around the x axis
        R_flip  = np.zeros((3,3), dtype=np.float32)
        R_flip[0,0] = 1.0
        R_flip[1,1] =-1.0
        R_flip[2,2] =-1.0

        #--- Define the aruco dictionary
        aruco_dict  = aruco.Dictionary_get(aruco.DICT_4X4_250)
        parameters  = aruco.DetectorParameters_create()


        #--- Capture the videocamera (this may also be a video or a picture)
        cap = cv2.VideoCapture(2)
        #-- Set the camera size as the one it was calibrated with
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

        #-- Font for the text in the image
        font = cv2.FONT_HERSHEY_PLAIN

        while True:

            #-- Read the camera frame
            ret, frame = cap.read()

            #-- Convert in gray scale
            gray    = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) #-- remember, OpenCV stores color images in Blue, Green, Red

            #-- Find all the aruco markers in the image
            corners, ids, rejected = aruco.detectMarkers(image=gray, dictionary=aruco_dict, parameters=parameters)
            
            if ids is not None and ids[0] == id_to_find:
                
                #-- ret = [rvec, tvec, ?]
                #-- array of rotation and position of each marker in camera frame
                #-- rvec = [[rvec_1], [rvec_2], ...]    attitude of the marker respect to camera frame
                #-- tvec = [[tvec_1], [tvec_2], ...]    position of the marker in camera frame
                ret = aruco.estimatePoseSingleMarkers(corners, marker_size, camera_matrix, camera_distortion)

                #-- Unpack the output, get only the first
                rvec, tvec = ret[0][0,0,:], ret[1][0,0,:]

                #-- Draw the detected marker and put a reference frame over it
                aruco.drawDetectedMarkers(frame, corners)
                cv2.drawFrameAxes(frame, camera_matrix, camera_distortion, rvec, tvec, 10)

                #-- Print the tag position in camera frame
                str_position = "MARKER Position x=%4.0f  y=%4.0f  z=%4.0f"%(tvec[0], tvec[1], tvec[2])
                cv2.putText(frame, str_position, (0, 100), font, 1, (0, 255, 0), 2, cv2.LINE_AA)

                #-- Obtain the rotation matrix tag->camera
                R_ct    = np.matrix(cv2.Rodrigues(rvec)[0])
                R_tc    = R_ct.T

                #-- Get the attitude in terms of euler 321 (Needs to be flipped first)
                roll_marker, pitch_marker, yaw_marker = rotationMatrixToEulerAngles(R_flip*R_tc)

                #-- Print the marker's attitude respect to camera frame
                str_attitude = "MARKER Attitude r=%4.0f  p=%4.0f  y=%4.0f"%(math.degrees(roll_marker),math.degrees(pitch_marker),
                                    math.degrees(yaw_marker))
                cv2.putText(frame, str_attitude, (0, 150), font, 1, (0, 255, 0), 2, cv2.LINE_AA)


                #-- Now get Position and attitude f the camera respect to the marker
                pos_camera = -R_tc*np.matrix(tvec).T

                str_position = "CAMERA Position x=%4.0f  y=%4.0f  z=%4.0f"%(pos_camera[0], pos_camera[1], pos_camera[2])
                cv2.putText(frame, str_position, (0, 200), font, 1, (0, 255, 0), 2, cv2.LINE_AA)

                #-- Get the attitude of the camera respect to the frame
                roll_camera, pitch_camera, yaw_camera = rotationMatrixToEulerAngles(R_flip*R_tc)
                str_attitude = "CAMERA Attitude r=%4.0f  p=%4.0f  y=%4.0f"%(math.degrees(roll_camera),math.degrees(pitch_camera),
                                    math.degrees(yaw_camera))
                cv2.putText(frame, str_attitude, (0, 250), font, 1, (0, 255, 0), 2, cv2.LINE_AA)


   
#     def set_limits(self, min: float, max: float) -> None:
#         self.max = max
#         self.min = min


    def update_z(self, feedback: float,min,max) -> float:
        
        # output = P + I + D
        output = self.Pterm_z  + self.Iterm_z + self.Dterm_z
        
        error = -(0.6 - feedback)
        
        feedback_z_filtered = self.alpha*feedback + (1-self.alpha)*self.last_feedback_z_filtered
        
        # P term
        self.Pterm_z  = 1600 + self.Kp_z * error
        
        # I term  (With anti-windup)
        if ((output>max and error>0) or (output<min and error<0)):
            self.Iterm_z = self.Iterm_z
        else:
            self.Iterm_z += (error + self.last_error_z) * 0.5 * self.Ki_z * self.dt

        # D term
        self.Dterm_z  = (-2 * self.Kd_z * (feedback - self.last_feedback_z)
                      + (2 * self.tau - self.dt) * self.Dterm_z / (2 * self.tau + self.dt))

        # Dterm with exponential smoothing:
        #self.Dterm_z = -2 * self.Kd_z * (feedback_z_filtered - self.last_feedback_z_filtered) 
        
        
        
        # output limits
        if output < min:
            return min
        if output > max:
            return max

        self.last_output_z   = output
        self.last_error_z    = error
        self.last_feedback_z = feedback
        self.last_feedback_z_filtered = feedback_z_filtered

        return output


    def update_roll(self, feedback: float,min,max) -> float:

        error = (0 - feedback)
        feedback_roll_filtered = self.alpha*feedback + (1-self.alpha)*self.last_feedback_roll_filtered

        # P term
        self.Pterm_roll  = 1500 + self.Kp_roll * error 
        # I term
        self.Iterm_roll += (error + self.last_error_roll) * 0.5 * self.Ki_roll * self.dt
        # D term
        self.Dterm_roll  = (-2 * self.Kd_roll * (feedback - self.last_feedback_roll)
                         + (2 * self.tau - self.dt) * self.Dterm_roll / (2 * self.tau + self.dt))

        # Dterm with exponential smoothing:
        #self.Dterm_roll = -2 * self.Kd_roll * (feedback_roll_filtered - self.last_feedback_roll_filtered)
        
        # output = P + I + D
        output = self.Pterm_roll + self.Iterm_roll + self.Dterm_roll
        
        # output limits
        if output < min:
            return min
        if output > max:
            return max

        self.last_output_roll   = output
        self.last_error_roll    = error
        self.last_feedback_roll = feedback
        self.last_feedback_roll_filtered = feedback_roll_filtered

        return output
        
        
    def update_pitch(self, feedback: float,min,max) -> float: 

        error = (0 - feedback)
        feedback_pitch_filtered = self.alpha*feedback + (1-self.alpha)*self.last_feedback_pitch_filtered
        # P term
        self.Pterm_pitch  = 1500 + self.Kp_pitch * error
        # I term
        self.Iterm_pitch += (error + self.last_error_pitch) * 0.5 * self.Ki_pitch * self.dt
        # D term
        self.Dterm_pitch  = (-2 * self.Kd_pitch * (feedback - self.last_feedback_pitch)
                          + (2 * self.tau - self.dt) * self.Dterm_pitch / (2 * self.tau + self.dt))
        
        # Dterm with exponential smoothing:
        #self.Dterm_pitch = -2 * self.Kd_pitch * (feedback_pitch_filtered - self.last_feedback_pitch_filtered)
        
        # output = P + I + D
        output = self.Pterm_pitch + self.Iterm_pitch + self.Dterm_pitch
        
        # output limits
        if output < min:
            return min
        if output > max:
            return max

        self.last_output_pitch   = output
        self.last_error_pitch    = error
        self.last_feedback_pitch = feedback
        self.last_feedback_pitch_filtered = feedback_pitch_filtered


        return output


    def update_yaw(self, feedback: float,min,max) -> float:
        # if feedback>180:Dt
        #     feedback = feedback -360
        error = (180 - feedback)
        feedback_yaw_filtered = self.alpha*feedback + (1-self.alpha)*self.last_feedback_yaw_filtered
       
        # P term
        self.Pterm_yaw  = 1500 + self.Kp_yaw * error
        # I term
        self.Iterm_yaw += (error + self.last_error_yaw) * 0.5 * self.Ki_yaw * self.dt
        # D term
        self.Dterm_yaw  = (-2 * self.Kd_yaw * (feedback - self.last_feedback_yaw)
                        + (2 * self.tau - self.dt) * self.Dterm_yaw / (2 * self.tau + self.dt))

        # Dterm with exponential smoothing:
        #self.Dterm_yaw = -2 * self.Kd_yaw * (feedback_yaw_filtered - self.last_feedback_yaw_filtered)
            
        # output = P + I + D
        output = self.Pterm_yaw + self.Iterm_yaw + self.Dterm_yaw
        
        # output limits
        if output < min:
            return min
        if output > max:
            return max

        self.last_output_yaw   = output
        self.last_error_yaw    = error
        self.last_feedback_yaw = feedback
        self.last_feedback_yaw_filtered = feedback_yaw_filtered

        return output
        

    def controller_out(self,current_data:PoseStamped):
        # getting feedback (current data)
        current_z   = current_data.pose.position.z
        current_x   = current_data.pose.position.x+0.13
        current_y   = current_data.pose.position.y+0.12
        current_yaw = current_data.pose.orientation.z

    


        #altitude_PID_output = self.update_z(current_z)

        if current_z>0:
            # rc outputs
            self.current_consecutive_frames=0
            altitude_PID_output = self.update_z(current_z,1200,1900)
            self.df = pd.DataFrame()
            obj = PlutoMsg()
            obj.rcThrottle = int(self.update_z(current_z,1200,1900))
            self.memory_thr = obj.rcThrottle
    
            obj.rcPitch    = int(self.update_pitch((current_x*math.sin((-cam_orientation+current_yaw)*math.pi/180)-current_y*math.cos(math.pi/180*(-cam_orientation+current_yaw))),1450,1550))
            self.memory_pitch = obj.rcPitch

            obj.rcRoll     = int(self.update_roll(current_x*math.cos(math.pi/180*(-cam_orientation+current_yaw))+current_y*math.sin(math.pi/180*(-cam_orientation+current_yaw)),1450,1550))
            self.memory_roll = obj.rcRoll

            #obj.rcYaw      = int(self.update_yaw(current_yaw))
            obj.rcYaw      = 1500
            obj.rcAUX4     = 1500
            obj.rcAUX3     = 1500
            obj.rcAUX2     = 1500
            obj.rcAUX1     = 1500

            # storing data
            self.rc_th.append(obj.rcThrottle)
            self.rc_r.append(obj.rcRoll)
            self.rc_p.append(obj.rcPitch)
            self.rc_y.append(obj.rcYaw)
            self.sen_r.append(current_data.pose.orientation.x)
            self.sen_p.append(current_data.pose.orientation.y)
            self.sen_y.append(current_data.pose.orientation.z)
            self.cam_x.append(current_data.pose.position.x)
            self.cam_y.append(current_data.pose.position.y)
            self.cam_z.append(current_data.pose.position.z)
            self.pub.publish(obj)
            self.plotlist_throttle.append(int(altitude_PID_output-1000)/10)
            self.plotlist_height.append(int(current_z*100))
            # plt.plot(self.plotlist_throttle)self.plotlist_height)
            # k=plt.plot([self.plotlist_throttle,self.plotlist_height])
            # file=open('Graph.pickle','wb')
            # pickle.dump(k,file)

            print(time.time())
            print("rcThrottle = ",altitude_PID_output)
            print("rcRoll = ",obj.rcRoll)
            print("rcPitch",obj.rcPitch)
            print(" ")



            self.df['rc_th'] = self.rc_th
            self.df['rc_r '] = self.rc_r 
            self.df['rc_p '] = self.rc_p 
            self.df['rc_y '] = self.rc_y 
            self.df['sen_r'] = self.sen_r
            self.df['sen_p'] = self.sen_p
            self.df['sen_y'] = self.sen_y
            self.df['cam_x'] = self.cam_x
            self.df['cam_y'] = self.cam_y
            self.df['cam_z'] = self.cam_z
            self.df['time'] = time.time()
            self.df.to_csv('Data.csv')
        else: 
            self.current_consecutive_frames+=1
             # When drone is not detected by camera
        if current_z<0 and self.current_consecutive_frames<=20:
            obj=PlutoMsg()
            obj.rcThrottle = self.memory_thr
            obj.rcPitch    = self.memory_pitch
            obj.rcRoll     = self.memory_roll
            obj.rcYaw      = 1500
            obj.rcAUX4     = 1500
            obj.rcAUX3     = 1500
            obj.rcAUX2     = 1500
            obj.rcAUX1     = 1500
            self.pub.publish(obj)
            print("memory_thr = ",self.memory_thr)
            print("memory_Roll = ",self.memory_roll)
            print("memory_Pitch",self.memory_pitch)
            print(" ")
        if current_z<0 and self.current_consecutive_frames>20:
            obj=PlutoMsg()
            obj.rcThrottle = 1000
            obj.rcPitch    = 1500
            obj.rcRoll     = 1500
            obj.rcYaw      = 1500
            obj.rcAUX4     = 1500
            obj.rcAUX3     = 1500
            obj.rcAUX2     = 1500
            obj.rcAUX1     = 1500
            self.pub.publish(obj)
            print("memory_thr = ",1000)
            print("memory_Roll = ",1500)
            print("memory_Pitch",1500)
            print(" ")    
        if current_z<0 and self.current_consecutive_frames>300:
            obj=PlutoMsg()
            obj.rcThrottle = 1000
            obj.rcPitch    = 1500
            obj.rcRoll     = 1500
            obj.rcYaw      = 1500
            obj.rcAUX4     = 1000
            obj.rcAUX3     = 1500
            obj.rcAUX2     = 1500
            obj.rcAUX1     = 1500
            self.pub.publish(obj)
            print("memory_thr = ",1000)
            print("memory_Roll = ",1500)
            print("memory_Pitch",1500)
            print(" ")    
    def setKp_z(self,msg:Float32):
        self.Kp_z = msg.data
        
    def setKp_roll(self,msg:Float32):
        self.Kp_roll  = msg.data
        
    def setKp_pitch(self,msg:Float32):
        self.Kp_pitch = msg.data
        
    def setKp_yaw(self,msg:Float32):
        self.Kp_yaw   = msg.data

    def main(self):
        rate=rospy.Rate(50)
        while not rospy.is_shutdown():
            rate.sleep()
    
if __name__ == '__main__':
    try:
        K_z     = [2000, 5, 300]
        K_roll  = [20 , 0, 0]
        K_pitch = [20, 0, 0]
        K_yaw   = [0, 0, 0]

        pid = PID(K_z,K_roll,K_pitch,K_yaw,dt=0.02,tau=0.012,alpha = 0.5,cam_orientation=0.0)
        pid.main()

    except KeyboardInterrupt:
        print ("keyboarrrdd")
        pass
