from plutoMultiwii import *
from threading import Thread
import time
from pynput.keyboard import Key, Listener

TRIM_MAX = 1000
TRIM_MIN = -1000

# userRC = [1500,1500,1500,1500,1000,1000,1000,1000]
# userRC = {'rcRoll': 1500, 'rcPitch': 1500, 'rcThrottle': 1500, 'rcYaw': 1500,
# 'rcAUX1':1000, 'rcAUX2':1000, 'rcAUX3':1000, 'rcAUX4':1000}


import numpy as np
import cv2
import cv2.aruco as aruco
import sys, time, math
import pandas as pd

#--- Define Tag
id_to_find  = 0
marker_size  = 4.3 #- [cm]

#--- Get the camera calibration path
calib_path  = ""
camera_matrix   = np.array([[708.03220678,   0.        , 314.11308886],
       [  0.        , 706.23511415, 246.16059333],
       [  0.        ,   0.        ,   1.        ]])
camera_distortion   =  np.array(
[[-1.07268339e-01,  2.65555312e+00,  1.06981415e-02,
         1.89814985e-03, -1.14050523e+01]])

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


class USER_RC():
    def __init__(self):
        self.rcRoll = 1500
        self.rcPitch = 1500
        self.rcThrottle = 1500
        self.rcYaw = 1500
        self.rcAUX1 = 1500
        self.rcAUX2 = 1500
        self.rcAUX3 = 1500
        self.rcAUX4 = 1000
        self.commandType = 0

        self.current_x = None
        self.current_y = None
        self.current_z = None
        
        ### DRONE ORIENTATION ###
        self.orientation_x = None # ROLL
        self.orientation_y = None # PITCH
        self.orientation_z = None # YAW

        self.goto_called = False

    # def goto(self):
    #     # Example usage
    #     self.goto_called = True
    #     while True:
    #         current_height = userRC.current_height # Replace with code to get current height
    #         output = height_controller.update(current_height)
    #         self.set_throttle(int(output)) # Replace with code to set throttle

    def set_throttle(self, throttle):
        self.rcThrottle = throttle

    def set_roll(self, roll):
        self.rcRoll = roll

    def set_pitch(self, pitch):
        self.rcPitch = pitch

    def arm(self):
        self.rcRoll = 1500
        self.rcYaw = 1500
        self.rcPitch = 1500
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
        self.rcRoll =1500
        self.rcThrottle =1500
        self.rcPitch =1500
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
        self.rcAUX1, self.rcAUX2, self.rcAUX3, self.rcAUX4]


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
    """
    Implements a PID controller.
    """

    def __init__(self,K_z: list, K_roll: list, K_pitch: list, K_yaw: list, dt:float, tau:float, alpha:float,cam_orientation:float,XT:float,YT:float,ZT:float) -> None:

        self.dt       = dt # time step for I_term in PID
        self.tau      = tau
        self.alpha    = alpha  # exponential smoothing factor
        self.memory_thr    = 0
        self.memory_roll   = 0
        self.current_consecutive_frames = 0
        self.memory_pitch  = 0
        self.memory_yaw    = 0
        self.x_correction  = 0
        self.y_correction  = 0

        self.rc_th = []
        self.rc_r  = []
        self.rc_p  = []
        self.rc_y  = []
        self.fake_x = []
        self.sen_r = []
        self.fake_y = []
        self.sen_p = []
        self.sen_y = []
        self.cam   = cam_orientation
        self.cam_x = []
        self.cam_y = []
        self.cam_z = []

        self.aim_y =  YT
        self.aim_x =  XT
        self.aim_z =  ZT

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
        rospy.Subscriber("Detection",PoseStamped,self.controller_out)
        #rospy.Subscriber("Kp_z",Float32,self.setKp_z)
        #rospy.Subscriber("Kp_roll",Float32,self.setKp_roll)
        #rospy.Subscriber("Kp_pitch",Float32,self.setKp_pitch)


#     def set_limits(self, min: float, max: float) -> None:
#         self.max = max
#         self.min = min


    def update_z(self, feedback: float,min,max) -> float:

        # output = P + I + D
        output = self.Pterm_z  + self.Iterm_z + self.Dterm_z

        error = -(0 - feedback)

        # feedback_z_filtered = self.alpha*feedback + (1-self.alpha)*self.last_feedback_z_filtered

        # P term
        self.Pterm_z  = 1600 + self.Kp_z * error

        #self.Dterm_pitch = -2 * self.Kd_pitch * (feedback_pitch_filtered - self.last_feedback_pitch_filtered)

        

        # output limits
        # I term  (With anti-windup)
        if ((output>max and error>0) or (output<min and error<0)):
            self.Iterm_z = self.Iterm_z
        else:
            self.Iterm_z += (error + self.last_error_z) * 0.5 * self.Ki_z * self.dt

        # D term
        #self.Dterm_pitch = -2 * self.Kd_pitch * (feedback_pitch_filtered - self.last_feedback_pitch_filtered)

        # output = P + I + D
        output = self.Pterm_pitch + self.Iterm_pitch + self.Dterm_pitch

        # output limits
        self.Dterm_z  = (-2 * self.Kd_z * (feedback - self.last_feedback_z)
                      + (2 * self.tau - self.dt) * self.Dterm_z / (2 * self.tau + self.dt))

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

        self.last_output_roll            = output
        self.last_error_roll             = error
        self.last_feedback_roll          = feedback
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

        self.last_output_pitch            = output
        self.last_error_pitch             = error
        self.last_feedback_pitch          = feedback
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

        self.last_output_yaw            = output
        self.last_error_yaw             = error
        self.last_feedback_yaw          = feedback
        self.last_feedback_yaw_filtered = feedback_yaw_filtered

        return output


    def controller_out(self):
        # getting feedback (current data)
        cam_err_z   = userRC.current_z-self.aim_z
        cam_err_x   = userRC.current_x-self.aim_x
        cam_err_y   = userRC.current_y-self.aim_y
        current_yaw = userRC.orientation_z ### GET THE YAW FROM WRAPPER


        if userRC.current_z>0:
            
            self.current_consecutive_frames=0         
            self.df = pd.DataFrame()
            
            ### CONFUSED AF ABOUT NAMING CONVENTION
            # For now, pitch affects drone_y, roll affects drone_x
            # 2D Cartesian coordinates at the drone

            drone_err_y = (cam_err_x*math.sin((-cam_orientation+current_yaw)*math.pi/180)-cam_err_y*math.cos(math.pi/180*(-cam_orientation+current_yaw)))
            drone_err_x = cam_err_x*math.cos(math.pi/180*(-cam_orientation+current_yaw))+cam_err_y*math.sin(math.pi/180*(-cam_orientation+current_yaw))
            # rc outputs
            userRC.rcThrottle = int(self.update_z(cam_err_z,1200,1900))
            userRC.rcPitch    = int(self.update_pitch(drone_err_y,1000,2000))            
            userRC.rcRoll     = int(self.update_roll(drone_err_x,1000,2000))            
            #userRC.rcYaw      = int(self.update_yaw(current_yaw))
            userRC.rcYaw      = 1500
            userRC.rcAUX4     = 1500
            userRC.rcAUX3     = 1500
            userRC.rcAUX2     = 1500
            userRC.rcAUX1     = 1500


            self.memory_thr = userRC.rcThrottle
            self.memory_pitch = userRC.rcPitch
            self.memory_roll = userRC.rcRoll
            
            print(time.time())
            print("rcThrottle = ",userRC.rcThrottle)
            print("rcRoll = ",userRC.rcRoll)
            print("rcPitch",userRC.rcPitch)
            print(" ")
            
            # storing data
            self.df = pd.DataFrame()
            self.rc_th.append(userRC.rcThrottle)
            self.rc_r.append(userRC.rcRoll)
            self.rc_p.append(userRC.rcPitch)
            self.rc_y.append(userRC.rcYaw)
            self.sen_r.append(userRC.orientation_x)
            self.sen_p.append(userRC.orientation_y)
            self.sen_y.append(userRC.orientation_z)
            self.cam_x.append(userRC.current_x)
            self.cam_y.append(userRC.current_y)
            self.cam_z.append(userRC.current_z)
            
            # UNCERTAIN: logic is correct, naming convention is not
            self.fake_x.append(drone_err_y)
            self.fake_y.append(drone_err_x)
            

            self.df['rc_th']     =      self.rc_th
            self.df['rc_r ']     =      self.rc_r 
            self.df['rc_p ']     =      self.rc_p 
            self.df['rc_y ']     =      self.rc_y 
            self.df['sen_r']     =      self.sen_r
            self.df['sen_p']     =      self.sen_p
            self.df['sen_y']     =      self.sen_y
            self.df['cam_x']     =      self.cam_x
            self.df['cam_y']     =      self.cam_y
            self.df['cam_z']     =      self.cam_z
            self.df['fake_x']    =      self.fake_y
            self.df['fake_y']    =      self.fake_x
            self.df['time']      =      time.time()
            self.df.to_csv('Data.csv')
        else: 
            self.current_consecutive_frames+=1
             # When drone is not detected by camera

        #When there is no detection for less than 40 consecutive frames
        if userRC.current_z<0 and self.current_consecutive_frames<=40:
            userRC.rcThrottle       =      self.memory_thr
            userRC.rcPitch          =      self.memory_pitch
            userRC.rcRoll           =      self.memory_roll
            userRC.rcYaw            =      1500
            userRC.rcAUX4           =      1500
            userRC.rcAUX3           =      1500
            userRC.rcAUX2           =      1500
            userRC.rcAUX1           =      1500

            print("memory_thr = ",self.memory_thr)
            print("memory_Roll = ",self.memory_roll)
            print("memory_Pitch",self.memory_pitch)
            print(" ")
            self.df = pd.DataFrame()
            self.rc_th.append(userRC.rcThrottle)
            self.rc_r.append(userRC.rcRoll)
            self.rc_p.append(userRC.rcPitch)
            self.rc_y.append(userRC.rcYaw)
            self.sen_r.append(userRC.orientation_x)
            self.sen_p.append(userRC.orientation_y)
            self.sen_y.append(userRC.orientation_z)
            self.cam_x.append(userRC.current_x)
            self.cam_y.append(userRC.current_y)
            self.cam_z.append(userRC.current_z)
            self.fake_x.append(drone_err_y)
            self.fake_y.append(drone_err_x)


            self.df['rc_th']         =    self.rc_th
            self.df['rc_r ']         =    self.rc_r 
            self.df['rc_p ']         =    self.rc_p 
            self.df['rc_y ']         =    self.rc_y 
            self.df['sen_r']         =    self.sen_r
            self.df['sen_p']         =    self.sen_p
            self.df['sen_y']         =    self.sen_y
            self.df['cam_x']         =    self.cam_x
            self.df['cam_y']         =    self.cam_y
            self.df['cam_z']         =    self.cam_z
            self.df['fake_x']        =    self.fake_y
            self.df['fake_y']        =    self.fake_x

            self.df['time'] = time.time()
            self.df.to_csv('Data.csv')
            
        #FAIL SAFE 1: When the detection did not happen for 300 frames, reduce throttle
        if userRC.current_z<0 and self.current_consecutive_frames>40 and self.current_consecutive_frames<300:
            userRC.rcThrottle = 1000
            userRC.rcPitch    = 1500
            userRC.rcRoll     = 1500
            userRC.rcYaw      = 1500
            userRC.rcAUX4     = 1500
            userRC.rcAUX3     = 1500
            userRC.rcAUX2     = 1500
            userRC.rcAUX1     = 1500

            print("default_thr = ",1000)
            print("default_Roll = ",1500)
            print("default_Pitch",1500)
            print(" ") 
            self.df = pd.DataFrame()
            self.rc_th.append(userRC.rcThrottle)
            self.rc_r.append(userRC.rcRoll)
            self.rc_p.append(userRC.rcPitch)
            self.rc_y.append(userRC.rcYaw)
            self.sen_r.append(userRC.orientation_x)
            self.sen_p.append(userRC.orientation_y)
            self.sen_y.append(userRC.orientation_z)
            self.cam_x.append(userRC.current_x)
            self.cam_y.append(userRC.current_y)
            self.cam_z.append(userRC.current_z)
            self.fake_x.append(drone_err_y)
            self.fake_y.append(drone_err_x)
            
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
            self.df['fake_x']= self.fake_y
            self.df['fake_y']= self.fake_x
            self.df['time'] = time.time()
            self.df.to_csv('Data.csv')   

        #FAIL SAFE 2: When detection did not happen for more than 300 frames, disarm
        if userRC.current_z<0 and self.current_consecutive_frames>=300:
            userRC.rcThrottle = 1500
            userRC.rcPitch    = 1500
            userRC.rcRoll     = 1500
            userRC.rcYaw      = 1500
            userRC.rcAUX4     = 1000
            userRC.rcAUX3     = 1500
            userRC.rcAUX2     = 1500
            userRC.rcAUX1     = 1500

            print("disarm_thr = ",1500)
            print("disarm_Roll = ",1500)
            print("disarm_Pitch",1500)
            print(" ") 
            self.df = pd.DataFrame()
            self.rc_th.append(userRC.rcThrottle)
            self.rc_r.append(userRC.rcRoll)
            self.rc_p.append(userRC.rcPitch)
            self.rc_y.append(userRC.rcYaw)
            self.sen_r.append(userRC.orientation_x)
            self.sen_p.append(userRC.orientation_y)
            self.sen_y.append(userRC.orientation_z)
            self.cam_x.append(userRC.current_x)
            self.cam_y.append(userRC.current_y)
            self.cam_z.append(userRC.current_z)
            self.fake_x.append(drone_err_y)
            self.fake_y.append(drone_err_x)
            
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
            self.df['fake_x']= self.fake_y
            self.df['fake_y']= self.fake_x
            self.df['time'] = time.time()
            self.df.to_csv('Data.csv')               

    # def setKp_z(self,msg:Float32):
    #     self.Kp_z = msg.data

    # def setKp_roll(self,msg:Float32):
    #     self.Kp_roll  = msg.data

    # def setKp_pitch(self,msg:Float32):
    #     self.Kp_pitch = msg.data

    # def setKp_yaw(self,msg:Float32):
    #     self.Kp_yaw   = msg.data


    ### 
    # def main(self):
    #     rate=rospy.Rate(50)
    #     while not rospy.is_shutdown():
    #         rate.sleep()

class HeightController:
    def __init__(self, setpoint, kp, ki, kd):
        self.setpoint = setpoint
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.last_error_z = 0
        # self.integral = 0
        self.dt = 0.02
        self.tau = 0.012
        self.min = 1100
        self.max = 1900

        self.Dterm_z             = 0   
        self.Iterm_z             = 0
        self.Pterm_z             = 0
        self.last_feedback_z     = 0 
        self.last_output_z       = 0
        self.last_error_z = 0
        
    # def update(self, current_height):
    #     error = self.setpoint - current_height
    #     self.integral += error * self.dt
    #     derivative = (error - self.previous_error) / self.dt
    #     self.previous_error = error
    #     output = 1500 + self.kp * error + self.ki * self.integral + self.kd * derivative

    #     # output limits
    #     if output < self.min:
    #         return self.min
    #     if output > self.max:
    #         return self.max

    #     return output
    
    def update(self, feedback): # feedback is current dist below camera
        
        # output = P + I + D
        output = self.Pterm_z  + self.Iterm_z + self.Dterm_z
        
        error = -(0.8 - feedback)
        
        # feedback_z_filtered = self.alpha*feedback + (1-self.alpha)*self.last_feedback_z_filtered
        
        # P term
        self.Pterm_z  = 1600 + self.kp * error
        
        # I term  (With anti-windup)
        if ((output>self.max and error>0) or (output<self.min and error<0)):
            self.Iterm_z = self.Iterm_z
        else:
            self.Iterm_z += (error + self.last_error_z) * 0.5 * self.ki * self.dt

        # D term
        self.Dterm_z  = (-2 * self.kd * (feedback - self.last_feedback_z)
                      + (2 * self.tau - self.dt) * self.Dterm_z / (2 * self.tau + self.dt))

        # Dterm with exponential smoothing:
        #self.Dterm_z = -2 * self.kd * (feedback_z_filtered - self.last_feedback_z_filtered) 
        
    
        # output limits
        if output < self.min:
            return self.min
        if output > self.max:
            return self.max

        self.last_output_z   = output
        self.last_error_z    = error
        self.last_feedback_z = feedback
        # self.last_feedback_z_filtered = feedback_z_filtered

        return output

def aruco_detect():
    while True:

        #-- Read the camera frame
        ret, frame = cap.read()

        #-- Convert in gray scale
        gray    = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) #-- remember, OpenCV stores color images in Blue, Green, Red

        #-- Find all the aruco markers in the image
        corners, ids, rejected = aruco.detectMarkers(image=gray, dictionary=aruco_dict, parameters=parameters)
        #cv2.imshow('Orginal Feed',frame)

        if ids is not None and ids[0] == id_to_find:
            
            #-- ret = [rvec, tvec, ?]
            #-- array of rotation and position of each marker in camera frame
            #-- rvec = [[rvec_1], [rvec_2], ...]    attitude of the marker respect to camera frame
            #-- tvec = [[tvec_1], [tvec_2], ...]    position of the marker in camera frame
            ret = aruco.estimatePoseSingleMarkers(corners, marker_size, camera_matrix, camera_distortion)

            #-- Unpack the output, get only the first
            rvec, tvec = ret[0][0,0,:], ret[1][0,0,:]

            #-- Draw the detected marker and put a reference frame over it
            #aruco.drawDetectedMarkers(frame, corners)
            #cv2.drawFrameAxes(frame, camera_matrix, camera_distortion, rvec, tvec, 10)

            #-- Print the tag position in camera frame
            #str_position = "MARKER Position x=%.0f  y=%.0f  z=%.0f"%(tvec[0], tvec[1], tvec[2])
            #cv2.putText(frame, str_position, (0, 100), font, 1, (0, 255, 0), 2, cv2.LINE_AA)
            
            userRC.current_x = tvec[0]/100
            userRC.current_y = tvec[1]/100
            userRC.current_z = tvec[2]/100
            
        else:
            userRC.current_x = -1
            userRC.current_y = -1
            userRC.current_z = -1
        
        if userRC.goto_called == True:
            plutoPID.controller_out()

        time.sleep(0.02) # WILL HAVE TO CHECK THIS
        #
        #cv2.circle(frame, (int(280/2), int(620/2)), 10, (0, 0, 255), 2, cv2.LINE_AA)
        #--- Display the frame
        #cv2.imshow('frame', frame)


# userRCAP = [1500,1500,1500,1500]

droneRC = [1500,1500,1500,1500,1000,1000,1000,1000]

NONE_COMMAND = 0
TAKE_OFF = 1
LAND = 2

# commandType = 0

def writeFunction():
    requests = list()
    requests.append(MSP_RC)
    requests.append(MSP_ATTITUDE)
    requests.append(MSP_RAW_IMU)
    requests.append(MSP_ALTITUDE)
    requests.append(MSP_ANALOG)

    sendRequestMSP_ACC_TRIM()

    while True:
        # global commandType
        droneRC[:] = userRC.rcValues()

        sendRequestMSP_SET_RAW_RC(droneRC)
        print(droneRC)
        sendRequestMSP_GET_DEBUG(requests)

        if (userRC.commandType != NONE_COMMAND):
            sendRequestMSP_SET_COMMAND(userRC.commandType)
            userRC.commandType = NONE_COMMAND

        time.sleep(0.022)

def on_press(key):
    print('{0} pressed'.format(key))

    if key == Key.space: #ARM
        if droneRC[7] == 1500: # if armed
            userRC.disarm()
        else:
            userRC.arm()

    if key == Key.up:
        userRC.forward()
    if key == Key.down:
        userRC.backward()
    if key == Key.left:
        userRC.left()
    if key == Key.right:
        userRC.right()

    if 'char' in dir(key):
        if key.char == 'q':
            userRC.take_off()

        if key.char == 'e':
            userRC.land()

        if key.char == 'w':
            userRC.increase_height()
        if key.char == 's':
            userRC.decrease_height()

        if key.char == 'a':
            userRC.left_yaw()
        if key.char == 'd':
            userRC.right_yaw()
        
        if key.char == 'f':
            #----------------- MISSION --------------------#
            userRC.goto_called = True
        
        if key.char == 'h':
            #----------------- MISSION HALT--------------------#
            userRC.goto_called = False

    
def on_release(key):
    print('{0} release'.format(key))
    
    if key in [Key.up, Key.down, Key.left, Key.right]:
        userRC.reset()
    elif 'char' in dir(key):
        if  key.char in ['w', 'a', 's', 'd']:
            userRC.reset()

def key_handling():
    with Listener(
        on_press=on_press, on_release=on_release) as listener:
        listener.join()


if __name__ == '__main__':
    print("Connecting to ", client)

    userRC = USER_RC()

    # PID Constants
    K_z     = [2500, 0, 50]
    K_pitch = [0, 0, 0]
    K_roll  = [0 , 0, 0]
    K_yaw   = [0, 0, 0]

    plutoPID = PID(K_z,K_roll,K_pitch,K_yaw,dt=0.1,tau=0.06,alpha = 0.5,cam_orientation=180.0,XT=0,YT=0,ZT=0.35)

    thread = Thread(target=writeFunction)
    thread.start()
    thread_key = Thread(target=key_handling)
    thread_key.start()
    thread_camera = Thread(target=aruco_detect)
    thread_camera.start()

