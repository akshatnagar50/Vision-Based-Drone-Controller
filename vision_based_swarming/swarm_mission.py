# SWARM 

from plutoMultiWii_with_swarm import *
from threading import Thread
import time
from pynput.keyboard import Key, Listener

TRIM_MAX = 1000
TRIM_MIN = -1000

import numpy as np
import cv2
import cv2.aruco as aruco
import sys, time, math
import pandas as pd
from collections import deque

#-- Font for the text in the image
font = cv2.FONT_HERSHEY_PLAIN
#--- Define Tag
id = [0, 2] # index=0, index=1
marker_size  = 5.5 #- [cm]  

#--- Get the camera calibration path
calib_path  = ""
camera_matrix   = np.load('camera_matrix_new.npy')


camera_distortion   =  np.load('camera_distortion_new.npy')

#--- Define the aruco dictionary
aruco_dict  = aruco.getPredefinedDictionary(aruco.DICT_4X4_250)
parameters  = aruco.DetectorParameters()

#--- Capture the videocamera (this may also be a video or a picture)
cap = cv2.VideoCapture(1, cv2.CAP_DSHOW)
#-- Set the camera size as the one it was calibrated with
cap.set(cv2.CAP_PROP_FPS, 30.0)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter.fourcc('m','j','p','g'))
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter.fourcc('M','J','P','G'))
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
#print(cap.get(cv2.CAP_PROP_FRAME_WIDTH),cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

class USER_RC():
    def __init__(self,index):
        self.rcRoll = 1500
        self.rcPitch = 1500
        self.rcThrottle = 1500
        self.rcYaw = 1500
        self.rcAUX1 = 1500
        self.rcAUX2 = 1500
        #self.rcAUX3 = 1000 #Throttle
        self.rcAUX3 = 1500 #Altitude Hold Mode 
        self.rcAUX4 = 1000
        self.index=index
        self.commandType = 0

        self.current_x = 1000
        self.current_y = 1000
        self.current_z = 1000
        
        ### DRONE ORIENTATION ###
        self.orientation_x = 1000 # ROLL
        self.orientation_y = 1000 # PITCH
        self.orientation_z = 1000 # YAW
        self.orientation_z_aruco = -1000

        self.goto_called = False
        
        self.mode = 'Manual'
        self.battery = -1

        self.trim_pitch = 0
        self.trim_roll = 0



    def set_throttle(self, throttle):
        self.rcThrottle = throttle

    def set_roll(self, roll):
        self.rcRoll = roll

    def set_pitch(self, pitch):
        self.rcPitch = pitch

    def arm(self):
        self.rcRoll = 1500 - self.trim_roll
        self.rcYaw = 1500
        self.rcPitch = 1500 - self.trim_pitch
        self.rcThrottle = 1000
        self.rcAUX4 = 1500
		# self.isAutoPilotOn = 0
    
    def box_arm(self):
        self.rcRoll=1500
        self.rcYaw=1500
        self.rcPitch =1500
        self.rcThrottle = 1500
        self.rcAUX4 =1500
		# self.isAutoPilotOn = 0

    def disarm(self):
        self.rcThrottle = 1300
        self.rcAUX4 = 1200
        
    def forward(self):
        self.rcPitch = 1600

    def backward(self):
        self.rcPitch =1400

    def left(self):
        self.rcRoll =1400

    def right(self):
        self.rcRoll =1600

    def left_yaw(self):
        self.rcYaw = 1200

    def right_yaw(self):
        self.rcYaw = 1800

    def reset(self):
        self.rcRoll =1500 - self.trim_roll
        self.rcThrottle =1500
        self.rcPitch =1500 - self.trim_pitch
        self.rcYaw = 1500
        self.commandType = 0

    def increase_height(self):
        self.rcThrottle = 1800

    def decrease_height(self):
        self.rcThrottle = 1300

    def take_off(self):
        self.disarm()
        self.box_arm()
        self.commandType = 1

    def land(self):
        self.commandType = 2

    def rcValues(self):
        return [self.rcRoll, self.rcPitch, self.rcThrottle, self.rcYaw,
        self.rcAUX1, self.rcAUX2, self.rcAUX3, self.rcAUX4,self.index]


class PID:
    """
    Implements a PID controller.
    """

    def __init__(self,K_z: list, K_roll: list, K_pitch: list, K_yaw: list, dt:float, tau:float, alpha:float,cam_orientation:float,tolerance:float, index:int ,beta:float) -> None:

        self.reached_target = False
        self.tol = tolerance
        self.index = index
        self.beta     = beta
        self.dt       = dt 
        # time step for I_term in PID
        self.tau      = tau 
        # Derivative filter time constant
        self.alpha    = alpha  
        # exponential smoothing factor
        #Memory for out of detection frames
        self.memory_thr        = 0
        self.memory_roll       = 0 
        self.consecutive_undetected_frames = 0
        self.memory_pitch      = 0
        self.memory_yaw        = 0
        self.x_correction      = 0
        self.y_correction      = 0
        self.cam_orientation   = cam_orientation


        self.throttle_upper=1750
        self.throttle_lower=1400 
        self.roll_lower    =1460
        self.pitch_lower   =1460
        self.roll_upper    =1540
        self.pitch_upper   =1540
        

        #Recording the flight data

        self.df = pd.DataFrame(columns=['rc_th','rc_r','rc_p','rc_y','sen_roll','sen_pitch','sen_yaw','relative_yaw',
                                        'cam_x','cam_y','cam_z','estimated_x','estimated_y','drone_error_x','drone_error_y',
                                        'Pterm_z','Iterm_z','Dterm_z',
                                        'Pterm_roll','Iterm_roll','Dterm_roll',
                                        'Pterm_pitch','Iterm_pitch','Dterm_pitch',
                                        'mode','AUX2','AUX3','Battery','time step'])

        self.t=time.time()
        
        #Setting Gains
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

        # setting the initial P,I&D term as 0
        self.Pterm_z             = 0
        self.Pterm_z             = 0 
        self.Dterm_z             = 0   
        self.Iterm_z             = 0
        self.Pterm_roll          = 0
        self.Dterm_roll          = 0
        self.Iterm_roll          = 0
        self.Pterm_pitch         = 0
        self.Dterm_pitch         = 0
        self.Iterm_pitch         = 0
        self.Pterm_yaw           = 0
        self.Dterm_yaw           = 0
        self.Iterm_yaw           = 0

        # setting the initial error 0
        self.last_error_z        = 0 
        self.last_error_roll     = 0
        self.last_error_pitch    = 0
        self.last_error_yaw      = 0

        self.control_last_time_z = time.time()
        self.control_last_time_r = time.time()
        self.control_last_time_p = time.time()

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

           
        self.last_velo_x = 0
        self.last_velo_y = 0

#         self.set_limits(1000, 2000) 

        self.plotlist_throttle = []
        self.plotlist_height   = []

        self.cam_err_x = 1000
        self.cam_err_y = 1000
        self.cam_err_z = 1000
        
        self.drone_err_x = 1000
        self.drone_err_y = 1000


        self.estimated_x = 1000
        self.estimated_y = 1000

        self.velo_x    = 0
        self.velo_y    = 0
        self.timestep  = 0
        self.last_x    = 0
        self.last_y    = 0
        self.last_time = time.time()

        self.q_z = deque()
        self.q_roll = deque()
        self.q_pitch = deque()
        self.q_yaw = deque()

        for _ in range(0,10):
            self.q_z.append(0)
            self.q_roll.append(0)
            self.q_pitch.append(0)
            self.q_yaw.append(0)
            
    def set_target(self, x, y, z):
        """Setting the PID targets"""
        self.reached_target = False
        self.aim_x = x
        self.aim_y = y
        self.aim_z = z

    def update_z(self, feedback: float,min,max) -> float:
        
        # output = P + I + D
        output = self.Pterm_z  + self.Iterm_z + self.Dterm_z
        self.dt= time.time()-self.control_last_time_z
        error = -(0 - feedback)

        # feedback_z_filtered = self.alpha*fSeedback + (1-self.alpha)*self.last_feedback_z_filtered

        # P term
        self.Pterm_z  = 1500 + self.Kp_z * error

        #self.Dterm_pitch = -2 * self.Kd_pitch * (feedback_pitch_filtered - self.last_feedback_pitch_filtered)

        

        # output limits
        # I term  (With anti-windup)
        if ((output>max and error>0) or (output<min and error<0)):
            self.Iterm_z = self.Iterm_z
        else:
            self.Iterm_z += (error + self.last_error_z) * 0.5 * self.Ki_z * self.dt

        


        # output limits
        # self.Dterm_z  = (-2 * self.Kd_z * (feedback - self.last_feedback_z) + (2 * self.tau - self.dt) * self.Dterm_z / (2 * self.tau + self.dt))
        
        self.Dterm_z =  (-2 * self.Kd_z * (feedback - self.last_feedback_z))
        self.q_z.append(self.Dterm_z)
        self.q_z.popleft()
        self.Dterm_z = sum(self.q_z)/10

        # Dterm with exponential smoothing:
        #self.Dterm_z = -2 * self.Kd_z * (feedback_z_filtered - self.last_feedback_z_filtered) 



        # output limits
        if output < min:
            return min
        if output > max:
            return max

        self.last_output_z            = output
        self.last_error_z             = error
        self.last_feedback_z          = feedback
        # self.last_feedback_z_filtered = feedback_z_filtered
        self.control_last_time_z=time.time()
        return output


    def update_roll(self, feedback: float,min,max) -> float:
        self.dt=time.time()-self.control_last_time_r
        error = (0 - feedback)
        feedback_roll_filtered = self.alpha*feedback + (1-self.alpha)*self.last_feedback_roll_filtered

        # P term
        self.Pterm_roll  = 1500 + self.Kp_roll * error 
        # I term
        # Replace 0.1 with self.dt and retune
        self.Iterm_roll += (error + self.last_error_roll) * 0.5 * self.Ki_roll * 0.1
        # D term
        # self.Dterm_roll  = (-2 * self.Kd_roll * (feedback - self.last_feedback_roll)
        #                  + (2 * self.tau - self.dt) * self.Dterm_roll / (2 * self.tau + self.dt))

        # Dterm with exponential smoothing:
        # self.Dterm_roll = -2 * self.Kd_roll * (feedback_roll_filtered - self.last_feedback_roll_filtered)

        self.Dterm_roll =  (-2 * self.Kd_roll * (feedback - self.last_feedback_roll))
        self.q_roll.append(self.Dterm_roll)
        self.q_roll.popleft()
        self.Dterm_roll = sum(self.q_roll)/10

        # output = P + I + D
        output = self.Pterm_roll + self.Iterm_roll + self.Dterm_roll

        # output limits
        if output < min:
            return min
        if output > max:
            return max

        self.last_output_roll            = output
        self.last_error_roll             = error
        self.last_feedback_roll          = feedback
        self.last_feedback_roll_filtered = feedback_roll_filtered
        self.control_last_time_r=time.time()
        return output


    def update_pitch(self, feedback: float,min,max) -> float: 
        self.dt=time.time()-self.control_last_time_p
        error = (0 - feedback)
        feedback_pitch_filtered = self.alpha*feedback + (1-self.alpha)*self.last_feedback_pitch_filtered
        # P term
        self.Pterm_pitch  = 1500 + self.Kp_pitch * error
        # I term
        # Replace 0.1 with self.dt and retune
        self.Iterm_pitch += (error + self.last_error_pitch) * 0.5 * self.Ki_pitch * 0.1
        # D term
        # self.Dterm_pitch  = (-2 * self.Kd_pitch * (feedback - self.last_feedback_pitch)
        #                   + (2 * self.tau - self.dt) * self.Dterm_pitch / (2 * self.tau + self.dt))

        # Dterm with exponential smoothing:
        # self.Dterm_pitch = -2 * self.Kd_pitch * (feedback_pitch_filtered - self.last_feedback_pitch_filtered)

        self.Dterm_pitch =  (-2 * self.Kd_pitch * (feedback - self.last_feedback_pitch))/self.dt
        self.q_pitch.append(self.Dterm_pitch)
        self.q_pitch.popleft()
        self.Dterm_pitch = sum(self.q_pitch)/10


        # output = P + I + D
        output = self.Pterm_pitch + self.Iterm_pitch + self.Dterm_pitch

        # output limits
        if output < min:
            return min
        if output > max:
            return max

        self.last_output_pitch            = output
        self.last_error_pitch             = error
        self.last_feedback_pitch          = feedback
        self.last_feedback_pitch_filtered = feedback_pitch_filtered

        self.control_last_time_p=time.time()
        return output


    def update_yaw(self, feedback: float,min,max) -> float:
        # if feedback>180:Dt
        #     feedback = feedback -360
        error = (180 - feedback)
        feedback_yaw_filtered = self.alpha*feedback + (1-self.alpha)*self.last_feedback_yaw_filtered

        # P term
        self.Pterm_yaw  = 1500 + self.Kp_yaw * error
        # I term
        # Replace 0.1 with self.dt and retune
        self.Iterm_yaw += (error + self.last_error_yaw) * 0.5 * self.Ki_yaw * 0.1
        # D term
        self.Dterm_yaw  = (-2 * self.Kd_yaw * (feedback - self.last_feedback_yaw)
                        + (2 * self.tau - self.dt) * self.Dterm_yaw / (2 * self.tau + self.dt))

        # Dterm with exponential smoothing:
        #self.Dterm_yaw = -2 * self.Kd_yaw * (feedback_yaw_filtered - self.last_feedback_yaw_filtered)
        self.Dterm_yaw =  (-2 * self.Kd_yaw * (feedback - self.last_feedback_yaw))/self.dt
        self.q_yaw.append(self.Dterm_yaw)
        self.q_yaw.popleft()
        self.Dterm_yaw = sum(self.q_yaw)/10
        # output = P + I + D
        output = self.Pterm_yaw + self.Iterm_yaw + self.Dterm_yaw

        # output limits
        if output < min:
            return min
        if output > max:
            return max

        self.last_output_yaw            = output
        self.last_error_yaw             = error
        self.last_feedback_yaw          = feedback
        self.last_feedback_yaw_filtered = feedback_yaw_filtered

        return output


    def controller_out(self):
        
        # Tolerance code
        if userRC[self.index].current_x < self.aim_x + self.tol and userRC[self.index].current_x > self.aim_x - self.tol:
            if userRC[self.index].current_y < self.aim_y + self.tol and userRC[self.index].current_y > self.aim_y - self.tol:
                self.reached_target = True

        # getting feedback (current data)
        self.cam_err_z   = userRC[self.index].current_z-self.aim_z
        self.cam_err_x   = userRC[self.index].current_x-self.aim_x
        self.cam_err_y   = userRC[self.index].current_y-self.aim_y
        current_yaw = userRC[self.index].orientation_z ### GET THE YAW FROM WRAPPER
        relative_yaw = self.cam_orientation - current_yaw
        self.relative_yaw = relative_yaw


        if userRC[self.index].current_z>0 and self.consecutive_undetected_frames==0:
            #This is executed when there is a detection and even previous frame was detected
            self.timestep  =   time.time()-self.last_time
            self.velo_x    =   (userRC[self.index].current_x-self.last_x)/self.timestep
            
            #self.velo_x    =   self.beta*self.velo_x  + (1-self.beta)*self.last_velo_x
            self.velo_y    =   (userRC[self.index].current_y-self.last_y)/self.timestep
            #self.velo_y    =   self.beta*self.velo_y  + (1-self.beta)*self.last_velo_y

            self.last_velo_x = self.velo_x
            self.last_velo_y = self.velo_y
            
        if userRC[self.index].current_z>0: 
            #As long as the drone is below the camera, this value is positive which indicates detection. 
            #This if segment is executed when there is detection
            
            self.consecutive_undetected_frames=  0       
            self.last_time                 =  time.time()
            self.last_x                    =  userRC[self.index].current_x
            self.last_y                    =  userRC[self.index].current_y
            
            ### CONFUSED AF ABOUT NAMING CONVENTION
            # For now, pitch affects drone_y, roll affects drone_x
            # 2D Cartesian coordinates at the drone

            
            self.drone_err_x  = -self.cam_err_x*math.cos(math.pi/180*(relative_yaw))+self.cam_err_y*math.sin(math.pi/180*(relative_yaw))
            self.drone_err_y  = (self.cam_err_x*math.sin((relative_yaw)*math.pi/180)+self.cam_err_y*math.cos(math.pi/180*(relative_yaw)))
            # rc outputs
            userRC[self.index].rcThrottle = int(self.update_z(self.cam_err_z,self.throttle_lower,self.throttle_upper))
            userRC[self.index].rcPitch    = int(self.update_pitch(self.drone_err_y,self.pitch_lower,self.pitch_upper))            
            userRC[self.index].rcRoll     = int(self.update_roll(self.drone_err_x,self.roll_lower,self.roll_upper))            
            #userRC[self.index].rcYaw      = int(self.update_yaw(current_yaw))f
            userRC[self.index].rcYaw      = 1500
            userRC[self.index].rcAUX4     = 1500
            # # userRC[self.index].rcAUX3     = 1000
            # userRC[self.index].rcAUX3     = 1500
            userRC[self.index].rcAUX2     = 1500
            userRC[self.index].rcAUX1     = 1500


            self.memory_thr = userRC[self.index].rcThrottle
            self.memory_pitch = userRC[self.index].rcPitch
            self.memory_roll = userRC[self.index].rcRoll

            userRC[self.index].mode = 'Controller Detection'
            # p
            # rint(time.time())
            # print("rcThrottle = ",userRC[self.index].rcThrottle)
            # print("rcRoll = ",userRC[self.index].rcRoll)
            # print("rcPitch",userRC[self.index].rcPitch)
            # print(" ")

            # Getting flight data        
          
        else: 
            self.consecutive_undetected_frames+=1
             # When drone is not detected by camera

        #When there is no detection for less than 40 consecutive frames
        if userRC[self.index].current_z<0 and self.consecutive_undetected_frames<=80:
            self.estimated_x = self.last_x+self.velo_x*(time.time()-self.last_time)
            self.estimated_y = self.last_y+self.velo_y*(time.time()-self.last_time)
            self.drone_err_x  = -(self.estimated_x)*math.cos(math.pi/180*(relative_yaw))+(self.estimated_y)*math.sin(math.pi/180*(relative_yaw))
            self.drone_err_y  = (self.estimated_x)*math.sin((relative_yaw)*math.pi/180)+(self.estimated_y)*math.cos(math.pi/180*(relative_yaw))
            # rc outputs
            #userRC[self.index].rcThrottle = int(self.update_z(self.cam_err_z,self.throttle_lower,self.throttle_upper))
            userRC[self.index].rcThrottle = 1500
            userRC[self.index].rcPitch    = int(self.update_pitch(self.drone_err_y,self.pitch_lower,self.pitch_upper))            
            userRC[self.index].rcRoll     = int(self.update_roll(self.drone_err_x,self.roll_lower,self.roll_upper))     
            userRC[self.index].rcYaw            =      1500
            userRC[self.index].rcAUX4           =      1500
            # #userRC[self.index].rcAUX3           =      1000
            # userRC[self.index].rcAUX3     = 1500
            userRC[self.index].rcAUX2           =      1500
            userRC[self.index].rcAUX1           =      1500

            userRC[self.index].mode = 'Controller Memory'
            # print("memory_thr = ",self.memory_thr)
            # print("memory_Roll = ",self.memory_roll)
            # print("memory_Pitch",self.memory_pitch)
            # print(" ")
   
            
        #FAIL SAFE 1: When the detection did not happen for 300 frames, reduce throttle
        if userRC[self.index].current_z<0 and self.consecutive_undetected_frames>80 and self.consecutive_undetected_frames<300:
            self.estimated_x = self.last_x+self.velo_x*(time.time()-self.last_time)
            self.estimated_y = self.last_y+self.velo_y*(time.time()-self.last_time)
            self.drone_err_x  = -(self.estimated_x)*math.cos(math.pi/180*(relative_yaw))+(self.estimated_y)*math.sin(math.pi/180*(relative_yaw))
            self.drone_err_y  = (self.estimated_x)*math.sin((relative_yaw)*math.pi/180)+(self.estimated_y)*math.cos(math.pi/180*(relative_yaw))
            userRC[self.index].rcThrottle = 1000
            # userRC[self.index].rcPitch    = int(self.update_pitch(self.drone_err_y,self.pitch_lower,self.pitch_upper))            
            # userRC[self.index].rcRoll     = int(self.update_roll(self.drone_err_x,self.roll_lower,self.roll_upper))  
            userRC[self.index].rcPitch    = 1500
            userRC[self.index].rcRoll     = 1500
            userRC[self.index].rcYaw      = 1500
            userRC[self.index].rcAUX4     = 1500
            # #userRC[self.index].rcAUX3     = 1000
            # userRC[self.index].rcAUX3     = 1500
            userRC[self.index].rcAUX2     = 1500
            userRC[self.index].rcAUX1     = 1500
 
            userRC[self.index].mode = 'Controller Default'
            # print("default_thr = ",1000)
            # print("default_Roll = ",1500)
            # print("default_Pitch",1500)
            # print(" ")  
              

        #FAIL SAFE 2: When detection did not happen for more than 300 frames, disarm
        if userRC[self.index].current_z<0 and self.consecutive_undetected_frames>=300:
            userRC[self.index].rcThrottle = 1200
            userRC[self.index].rcPitch    = 1500
            userRC[self.index].rcRoll     = 1500
            userRC[self.index].rcYaw      = 1500
            userRC[self.index].rcAUX4     = 1000
            # userRC[self.index].rcAUX3     = 1000
            # userRC[self.index].rcAUX3     = 1500
            userRC[self.index].rcAUX2     = 1500
            userRC[self.index].rcAUX1     = 1500


            userRC[self.index].mode = 'Controller Disarm'
            # print("disarm_thr = ",1500)
            # print("disarm_Roll = ",1500)
            # print("disarm_Pitch",1500)
            # print(" ") 
        lst=[userRC[self.index].rcThrottle,
             userRC[self.index].rcRoll ,
             userRC[self.index].rcPitch ,
             userRC[self.index].rcYaw ,
             userRC[self.index].orientation_x,
             userRC[self.index].orientation_y,
             userRC[self.index].orientation_z,
             relative_yaw,
             userRC[self.index].current_x,
             userRC[self.index].current_y,
             userRC[self.index].current_z,
             self.estimated_x,
             self.estimated_y,
             self.drone_err_x,
             self.drone_err_y,
             self.Pterm_z,
             self.Iterm_z,
             self.Dterm_z,
             self.Pterm_roll,
             self.Iterm_roll,
             self.Dterm_roll,
             self.Pterm_pitch,
             self.Iterm_pitch,
             self.Dterm_pitch,
             userRC[self.index].mode,
             userRC[self.index].rcAUX2,
             userRC[self.index].rcAUX3,
             userRC[self.index].battery,
             time.time()-self.t]
        self.df.loc[len(self.df.self.index)]=lst
        self.df.to_csv('Data'+str(self.t)+"_Kpp_"+str(self.Kp_pitch)+"_Kpr_"+str(self.Kp_roll)+'_Kpt_'+str(self.Kp_z)+'.csv') 
       


def aruco_detect():
    
    print("Cap opened:",cap.isOpened())
    while cap.isOpened():

        #-- Read the camera frame
        ret, frame = cap.read()
        #-- Convert in gray scale
        gray    = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) #-- remember, OpenCV stores color images in Blue, Green, Red

        detector = aruco.ArucoDetector(aruco_dict,parameters)
        #-- Find all the aruco markers in the image
        corners, ids, rejected = detector.detectMarkers(gray)
        #cv2.imshow('Orginal Feed',frame)
        #print(ids)
        for index in range(len(id)):
            if ids is not None and ids[0] == id[index]:
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
                

                x1 =  tvec[0]
                y1 =  tvec[1]
                z1 = tvec[2]
                yaw = rvec[2]
    
                
            else:
                x1 = -1000
                y1 = -1000
                z1 = -1000
                yaw = -1000
                
            userRC[index].current_x = x1
            userRC[index].current_y = y1
            userRC[index].current_z = z1
            userRC[index].orientation_z_aruco = yaw
            
            if userRC[index].goto_called == True:
                plutoPID[index].controller_out()

        cv2.waitKey(1)

        #--- Display the frame
        frame_resized = cv2.resize(frame.copy(),None,fx=0.3,fy=0.3)
        cv2.imshow('frame', frame_resized)


# userRCAP = [1500,1500,1500,1500]

droneRC = [1500,1500,1500,1500,1000,1000,1000,1000,0]

NONE_COMMAND = 0
TAKE_OFF = 1
LAND = 2

        
def writeFunction():
    # Function to write to the drone
    requests = list()
    # requests.append(MSP_RC)
    requests.append(MSP_ATTITUDE)
    # requests.append(MSP_RAW_IMU)
    # requests.append(MSP_ALTITUDE)
    requests.append(MSP_ANALOG)

    # sendRequestMSP_ACC_TRIM()
  
    while True:
        droneRC[:] = userRC[drone_shift[0]].rcValues() # copy the updated userRC values to droneRC

        sendMulRequestMSP_SET_RAW_RC(droneRC) # send the updated droneRC values to the drone
        print("Drone {0:2}".format(droneRC[8]), end=' ')
        print('R:{}, P:{}, T:{}, Y:{} '.format(droneRC[0],droneRC[1],droneRC[2],droneRC[3]),end=' ')
        #print('A1:{}, A2:{}, A3:{}, A4:{}'.format(droneRC[4],droneRC[5],droneRC[6],droneRC[7]),end='  ')
        print('x:{}, y:{}, z:{} '.format(round(userRC[droneRC[8]].current_x),round(userRC[droneRC[8]].current_y),round(userRC[droneRC[8]].current_z)),end=' ')
        print('Roll: {}, Pitch: {}, Yaw: {}'.format(round(userRC[droneRC[8]].orientation_x),round(userRC[droneRC[8]].orientation_y),round(userRC[droneRC[8]].orientation_z)),end=' ')
        #print('Estimated_x: {}, Estimated_y: {}'.format(round(plutoPID[droneRC[8]].estimated_x),round(plutoPID[droneRC[8]].estimated_y)),end=' ')
        print('drone_err_x:{}, drone_err_y:{}, drone_err_z:{}, mode:{}, battery:{}'.format(round(plutoPID[droneRC[8]].drone_err_x),round(plutoPID[droneRC[8]].drone_err_y),round(plutoPID[droneRC[8]].cam_err_z),userRC[droneRC[8]].mode,userRC[droneRC[8]].battery),end=' ')
        print('Setpoint Number: {}, Setpoint:{}'.format(rec_mission.setpoint_number+1,rec_mission.present_setpoint))
        sendMulRequestMSP_GET_DEBUG(requests,droneRC[8])

                    
        time.sleep(0.022)

def readFunction(index):
    def twos_complement(binary_string): 
        result = ""
        # Invert all bits in the binary string
        for bit in binary_string: 
            if bit == '0': 
                result += '1'
            else: 
                result += '0'
        
    # Add 1 to the result 
        carry = True
        for i in range(len(result)-1, -1, -1): 
            if result[i] == '1' and carry: 
                result = result[:i] + '0' + result[i+1:]
                carry = True
            elif result[i] == '0' and carry: 
                result = result[:i] + '1' + result[i+1:]
                carry = False
    
        return result

    def twos_complement_inverse(binary_string):
        if binary_string[0]=='0':
            return int(binary_string,2)
        elif binary_string[0]=='1':
            return -int(twos_complement(binary_string),2)
        else:
            print('non binary bit received: ',binary_string[0])

    def get_RPY(segment):
        #binary representation of the little endian hexadecimal 2 byte value
        roll_str = (str(bin(segment[6]))[2:].zfill(8)+str(bin(segment[5])[2:].zfill(8)))
        roll = twos_complement_inverse(roll_str)/10

        pitch_str = str(bin(segment[8]))[2:].zfill(8) + str(bin(segment[7]))[2:].zfill(8)
        pitch = twos_complement_inverse(pitch_str)/10

        yaw = segment[9] + 256*segment[10]

        return roll,pitch,yaw 

    while True:
        data = mul_clients[index].recv(10000)
        
        segments = data.split(b'$M>')
        for segment in segments:
            segment = b'$M>' + segment

            if len(segment)==12: #checking if the length of the packet is 12
                if segment[4]==108: #checking if the type of the packet is MSP_ATTITUDE
                    roll,pitch,yaw = get_RPY(segment)
                    userRC[index].orientation_x = roll
                    userRC[index].orientation_y = pitch
                    userRC[index].orientation_z = yaw
 
            if len(segment)==13:
                userRC[index].battery = segment[5]/10
        time.sleep(0.02)

def on_press(key):
    # Create a function to handle key presses
    print('{0} pressed'.format(key))

    if key == Key.shift:
        drone_shift[0] = 0
        #print(drone_shift[0])
    if key == Key.shift_r:
        drone_shift[0] = 1
        #print(drone_shift[0])
    
    i = drone_shift[0]
    if key == Key.space: #ARM
        if droneRC[7] == 1500: # if armed
            userRC[i].disarm()
        else:
            userRC[i].arm()

    if key == Key.up:
        userRC[i].forward()
    if key == Key.down:
        userRC[i].backward()
    if key == Key.left:
        userRC[i].left()
    if key == Key.right:
        userRC[i].right()

    if 'char' in dir(key):
        if key.char == 'q':
            userRC[i].take_off()

        if key.char == 'e':
            userRC[i].land()

        if key.char == 'w':
            userRC[i].increase_height()
        if key.char == 's':
            userRC[i].decrease_height()

        if key.char == 'a':
            userRC[i].left_yaw()
        if key.char == 'd':
            userRC[i].right_yaw()
        
        if key.char == 'f':
            #----------------- PID START --------------------#
            userRC[i].goto_called = True
        
        if key.char == 'h':
            #----------------- PID HALT --------------------#
            userRC[i].goto_called = False

    
def on_release(key):
    #print('{0} release'.format(key))
    # Create a function to handle key releases
    if key in [Key.up, Key.down, Key.left, Key.right]:
        userRC[i].reset()
    elif 'char' in dir(key):
        if  key.char in ['w', 'a', 's', 'd']:
            userRC[i].reset()

def key_handling():
    with Listener(
        on_press=on_press, on_release=on_release) as listener:
        listener.join()

class Rectangle_Mission():
    def __init__(self,setpoints):
        self.setpoints = setpoints
        self.present_setpoint = self.setpoints[0]
        self.prev_setpoint = None
        self.setpoint_number = 0
        
def rectangle():
    
    # RECTANGLE CODE
    print("RECTANGLE MISSION STARTED....")
    print("RECTANGLE MISSION STARTED....")
    print("RECTANGLE MISSION STARTED....")
    print("RECTANGLE MISSION STARTED....")

    for i in range(len(rec_mission.setpoints)):
        rec_mission.setpoint_number = i
        rec_mission.present_setpoint = rec_mission.setpoints[i]
        

        plutoPID[0].set_target(rec_mission.present_setpoint[0],rec_mission.present_setpoint[1],rec_mission.present_setpoint[2])
        
        # # PLUTO 2 CODE
        # if i != 0:
        #     rec_mission.prev_setpoint = rec_mission.setpoints[i-1]
        #     plutoPID[1].set_target(rec_mission.prev_setpoint[0],rec_mission.prev_setpoint[1],rec_mission.prev_setpoint[2])
        while not plutoPID[0].reached_target:
            #print('Going to setpoint Number: , ',i,setpoint)
            time.sleep(0.022)
            pass

        plutoPID[0].Iterm_pitch=0
        plutoPID[0].Iterm_roll=0
        for i in range(0,10):
            plutoPID[0].q_roll[i]=0
            plutoPID[0].q_pitch[i]=0
            plutoPID[0].q_yaw[i]=0    
        print("Reached target:", i+1, plutoPID[0].reached_target)

        # PLUTO 2 CODE
        while not plutoPID[1].reached_target:
            #print('Going to setpoint Number: , ',i,setpoint)
            time.sleep(0.022)
            pass

        plutoPID[1].Iterm_pitch=0
        plutoPID[1].Iterm_roll=0
        for i in range(0,10):
            plutoPID[1].q_roll[i]=0
            plutoPID[1].q_pitch[i]=0
            plutoPID[1].q_yaw[i]=0    
        print("Reached target :", i, plutoPID[1].reached_target)
    
    userRC[0].decrease_height()
    time.sleep(0.5)
    userRC[0].disarm()

    # PLUTO 2 CODE
    i += 1
    rec_mission.prev_setpoint = rec_mission.setpoints[i-1]
    plutoPID[1].set_target(rec_mission.prev_setpoint[0],rec_mission.prev_setpoint[1],rec_mission.prev_setpoint[2])

    while not plutoPID[1].reached_target:
        #print('Going to setpoint Number: , ',i,setpoint)
        time.sleep(0.022)
        pass

    plutoPID[1].Iterm_pitch=0
    plutoPID[1].Iterm_roll=0
    for i in range(0,10):
        plutoPID[1].q_roll[i]=0
        plutoPID[1].q_pitch[i]=0
        plutoPID[1].q_yaw[i]=0    
    print("Reached target :", i-1, plutoPID[1].reached_target)
    userRC[1].decrease_height()
    time.sleep(0.5)
    userRC[1].disarm()



if __name__ == '__main__':  

    K_z     = [3.5, 0, 0]
    K_pitch = [0.4 , 0.01 , 0.1] 
    K_roll  = [0.4 , 0.01 , 0.1] 
    K_yaw   = [0, 0, 0]
    
    hover_height = 150
    setpoints = np.array([[0, 0, 200]], [50,100,hover_height],[50,0,hover_height],[50,-100,hover_height],[0,-100,hover_height],[-50,-100,hover_height],[-50,0,hover_height],[-50,100,hover_height],[0,0,300])
    setpoints = setpoints

    # Creating an instance of the PID class to call the constructor
    plutoPID = list()
    userRC = list()
    drone_shift = [0] 


    thread_write = list()
    thread_key = list()
    thread_read = list()
    thread_camera = list()
      
    for i in range(len(mul_clients)):

        print("Connecting to ", mul_clients[i])

        userRC.append(USER_RC(i))
        plutoPID.append(PID(K_z,K_roll,K_pitch,K_yaw,dt=0.1,tau=0.06,alpha = 0.5,cam_orientation= 196.0, tolerance=15, index = i, beta=0.5))
            
        
        thread_write.append(Thread(target=writeFunction))
        thread_write[i].start()

        thread_key.append(Thread(target=key_handling))
        thread_key[i].start()

        
        thread_read.append(Thread(target=readFunction, args=(i,)))
        thread_read[i].start()

    
    thread_camera = Thread(target=aruco_detect)
    thread_camera.start()

    rec_mission = Rectangle_Mission(setpoints)
    
# end of code

