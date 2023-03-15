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


#-- Font for the text in the image
font = cv2.FONT_HERSHEY_PLAIN
#--- Define Tag
id_to_find  = 0
marker_size  = 4.3 #- [cm]  

#--- Get the camera calibration path
calib_path  = ""
camera_matrix   = np.load('camera_matrix_new.npy')


camera_distortion   =  np.load('camera_distortion_new.npy')

#--- Define the aruco dictionary
aruco_dict  = aruco.getPredefinedDictionary(aruco.DICT_4X4_250)
parameters  = aruco.DetectorParameters()

#--- Capture the videocamera (this may also be a video or a picture)
cap = cv2.VideoCapture(0,cv2.CAP_DSHOW)
#-- Set the camera size as the one it was calibrated with
cap.set(cv2.CAP_PROP_FPS, 30.0)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter.fourcc('m','j','p','g'))
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter.fourcc('M','J','P','G'))
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
#print(cap.get(cv2.CAP_PROP_FRAME_WIDTH),cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

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

        self.current_x = 1000
        self.current_y = 1000
        self.current_z = 1000
        
        ### DRONE ORIENTATION ###
        self.orientation_x = None # ROLL
        self.orientation_y = None # PITCH
        self.orientation_z = 1000 # YAW
        self.orientation_z_aruco = -1000

        self.goto_called = False
        
        self.mode = 'Manual'

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
        self.rcThrottle = 1950

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


# cam_orientation=0
# rc_th = []
# rc_r  = []
# rc_p  = []
# rc_y  = []
# sen_r = []
# sen_p = []
# sen_y = []
# cam_x = []
# cam_y = []
# cam_z = []
# # import csv

class PID:
    """
    Implements a PID controller.
    """

    def __init__(self,K_z: list, K_roll: list, K_pitch: list, K_yaw: list, dt:float, tau:float, alpha:float,cam_orientation:float,XT:float,YT:float,ZT:float) -> None:

        self.dt       = dt 
        # time step for I_term in PID
        self.tau      = tau 
        # Derivative filter time constant
        self.alpha    = alpha  
        # exponential smoothing factor
        #Memory for out of detection frames
        self.memory_thr    = 0
        self.memory_roll   = 0 
        self.current_consecutive_frames = 0
        self.memory_pitch  = 0
        self.memory_yaw    = 0
        self.x_correction  = 0
        self.y_correction  = 0

        #Recording the flight data
        self.rc_th = []
        self.rc_r  = []
        self.rc_p  = []
        self.rc_y  = []
        self.fake_x = []
        self.sen_r = []
        self.fake_y = []
        self.sen_p = []
        self.sen_y = []
        self.cam_orientation   = cam_orientation
        self.cam_x = []
        self.cam_y = []
        self.cam_z = []

        #Setting the PID targets
        self.aim_y =  YT
        self.aim_x =  XT
        self.aim_z =  ZT
        
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

        self.cam_err_x = 1000
        self.cam_err_y = 1000
        self.cam_err_z = 1000
        
        self.drone_err_x = 1000
        self.drone_err_y = 1000

        # Commented out ROS commands

        # rospy.init_node("PID")
        # self.pub = rospy.Publisher("/drone_command",PlutoMsg,queue_size=10)
        # self.arm(self.pub)
        # rospy.Subscriber("Detection",PoseStamped,self.controller_out)
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
        self.Pterm_z  = 1700 + self.Kp_z * error

        #self.Dterm_pitch = -2 * self.Kd_pitch * (feedback_pitch_filtered - self.last_feedback_pitch_filtered)

        
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

        self.last_output_z            = output
        self.last_error_z             = error
        self.last_feedback_z          = feedback
        # self.last_feedback_z_filtered = feedback_z_filtered

        return output


    def update_roll(self, feedback: float,min,max) -> float:
		
	# output = P + I + D
        output = self.Pterm_roll + self.Iterm_roll + self.Dterm_roll

        error = (0 - feedback)
        # feedback_roll_filtered = self.alpha*feedback + (1-self.alpha)*self.last_feedback_roll_filtered

        # P term
        self.Pterm_roll  = 1500 + self.Kp_roll * error 
	
        # I term  (With anti-windup)
        if ((output>max and error>0) or (output<min and error<0)):
            self.Iterm_roll = self.Iterm_roll
        else:
            self.Iterm_roll += (error + self.last_error_roll) * 0.5 * self.Ki_roll * self.dt
	
        # D term
        self.Dterm_roll  = (-2 * self.Kd_roll * (feedback - self.last_feedback_roll)
                         + (2 * self.tau - self.dt) * self.Dterm_roll / (2 * self.tau + self.dt))

        # Dterm with exponential smoothing:
        #self.Dterm_roll = -2 * self.Kd_roll * (feedback_roll_filtered - self.last_feedback_roll_filtered)

        

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
	
	# output = P + I + D
        output = self.Pterm_pitch + self.Iterm_pitch + self.Dterm_pitch
	
        error = (0 - feedback)
        #feedback_pitch_filtered = self.alpha*feedback + (1-self.alpha)*self.last_feedback_pitch_filtered
        # P term
        self.Pterm_pitch  = 1500 + self.Kp_pitch * error
	
        # I term  (With anti-windup)
        if ((output>max and error>0) or (output<min and error<0)):
            self.Iterm_pitch = self.Iterm_pitch
        else:
            self.Iterm_pitch += (error + self.last_error_pitch) * 0.5 * self.Ki_pitch * self.dt
	
        # D term
        self.Dterm_pitch  = (-2 * self.Kd_pitch * (feedback - self.last_feedback_pitch)
                          + (2 * self.tau - self.dt) * self.Dterm_pitch / (2 * self.tau + self.dt))

        # Dterm with exponential smoothing:
        #self.Dterm_pitch = -2 * self.Kd_pitch * (feedback_pitch_filtered - self.last_feedback_pitch_filtered)


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
		
	# output = P + I + D
        output = self.Pterm_yaw + self.Iterm_yaw + self.Dterm_yaw
	
        # if feedback>180:Dt
        #     feedback = feedback -360
        error = (180 - feedback)
	# feedback_yaw_filtered = self.alpha*feedback + (1-self.alpha)*self.last_feedback_yaw_filtered

        # P term
        self.Pterm_yaw  = 1500 + self.Kp_yaw * error
	
        # I term  (With anti-windup)
        if ((output>max and error>0) or (output<min and error<0)):
            self.Iterm_yaw = self.Iterm_yaw
        else:
            self.Iterm_yaw += (error + self.last_error_yaw) * 0.5 * self.Ki_yaw * self.dt
	
        # D term
        self.Dterm_yaw  = (-2 * self.Kd_yaw * (feedback - self.last_feedback_yaw)
                        + (2 * self.tau - self.dt) * self.Dterm_yaw / (2 * self.tau + self.dt))

        # Dterm with exponential smoothing:
        #self.Dterm_yaw = -2 * self.Kd_yaw * (feedback_yaw_filtered - self.last_feedback_yaw_filtered)

        

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
        self.cam_err_z   = userRC.current_z-self.aim_z
        self.cam_err_x   = userRC.current_x-self.aim_x
        self.cam_err_y   = userRC.current_y-self.aim_y
        current_yaw = userRC.orientation_z ### GET THE YAW FROM WRAPPER


        if userRC.current_z>0: 
            #As long as the drone is below the camera, this value is positive which indicates detection. 
            #This if segment is executed when there is detectionfh
            
            self.current_consecutive_frames=0       

            
            ### CONFUSED AF ABOUT NAMING CONVENTION
            # For now, pitch affects drone_y, roll affects drone_x
            # 2D Cartesian coordinates at the drone

            relative_yaw = self.cam_orientation - current_yaw
            
            self.drone_err_x = -self.cam_err_x*math.cos(math.pi/180*(relative_yaw))+self.cam_err_y*math.sin(math.pi/180*(relative_yaw))
            self.drone_err_y = (self.cam_err_x*math.sin((relative_yaw)*math.pi/180)+self.cam_err_y*math.cos(math.pi/180*(relative_yaw)))
            # rc outputs
            userRC.rcThrottle = int(self.update_z(self.cam_err_z,1200,1900))
            userRC.rcPitch    = int(self.update_pitch(self.drone_err_y,1000,2000))            
            userRC.rcRoll     = int(self.update_roll(self.drone_err_x,1000,2000))            
            #userRC.rcYaw      = int(self.update_yaw(current_yaw))f
            userRC.rcYaw      = 1500
            userRC.rcAUX4     = 1500
            userRC.rcAUX3     = 1500
            userRC.rcAUX2     = 1500
            userRC.rcAUX1     = 1500


            self.memory_thr = userRC.rcThrottle
            self.memory_pitch = userRC.rcPitch
            self.memory_roll = userRC.rcRoll

            userRC.mode = 'Controller Detection'
            # p
            # rint(time.time())
            # print("rcThrottle = ",userRC.rcThrottle)
            # print("rcRoll = ",userRC.rcRoll)
            # print("rcPitch",userRC.rcPitch)
            # print(" ")

            # Getting flight data

            # self.df = pd.DataFrame()
            # self.rc_th.append(userRC.rcThrottle)
            # self.rc_r.append(userRC.rcRoll)
            # self.rc_p.append(userRC.rcPitch)
            # self.rc_y.append(userRC.rcYaw)
            # self.sen_r.append(current_data.pose.orientation.x)
            # self.sen_p.append(current_data.pose.orientation.y)
            # self.sen_y.append(current_data.pose.orientation.z)
            # self.cam_x.append(current_data.pose.position.x)
            # self.cam_y.append(current_data.pose.position.y)
            # self.cam_z.append(current_data.pose.position.z)
            # self.fake_x.append(-self.cam_err_x*math.cos(math.pi/180*(relative_yaw))+self.cam_err_y*math.sin(math.pi/180*(relative_yaw)))
            # self.fake_y.append((self.cam_err_x*math.sin((relative_yaw)*math.pi/180)+self.cam_err_y*math.cos(math.pi/180*(relative_yaw))))
            # self.df['rc_th']     =      self.rc_th
            # self.df['rc_r ']     =      self.rc_r 
            # self.df['rc_p ']     =      self.rc_p 
            # self.df['rc_y ']     =      self.rc_y 
            # self.df['sen_r']     =      self.sen_r
            # self.df['sen_p']     =      self.sen_p
            # self.df['sen_y']     =      self.sen_y
            # self.df['cam_x']     =      self.cam_x
            # self.df['cam_y']     =      self.cam_y
            # self.df['cam_z']     =      self.cam_z
            # self.df['fake_x']    =      self.fake_x
            # self.df['fake_y']    =      self.fake_y
            # self.df['time']      =      time.time()
            # self.df.to_csv('Data.csv')           
          
        else: 
            self.current_consecutive_frames+=1
             # When drone is not detected by camera

        #When there is no detection for less than 40 consecutive frames
        if userRC.current_z<0 and self.current_consecutive_frames<=100:
            userRC.rcThrottle       =      self.memory_thr
            userRC.rcPitch          =      self.memory_pitch
            userRC.rcRoll           =      self.memory_roll
            userRC.rcYaw            =      1500
            userRC.rcAUX4           =      1500
            userRC.rcAUX3           =      1500
            userRC.rcAUX2           =      1500
            userRC.rcAUX1           =      1500

            userRC.mode = 'Controller Memory'
            # print("memory_thr = ",self.memory_thr)
            # print("memory_Roll = ",self.memory_roll)
            # print("memory_Pitch",self.memory_pitch)
            # print(" ")

            # Uncomment to get flight data

            # self.df = pd.DataFrame()
            # self.rc_th.append(userRC.rcThrottle)
            # self.rc_r.append(userRC.rcRoll)
            # self.rc_p.append(userRC.rcPitch)
            # self.rc_y.append(userRC.rcYaw)
            # self.sen_r.append(current_data.pose.orientation.x)
            # self.sen_p.append(current_data.pose.orientation.y)
            # self.sen_y.append(current_data.pose.orientation.z)
            # self.cam_x.append(current_data.pose.position.x)
            # self.cam_y.append(current_data.pose.position.y)
            # self.cam_z.append(current_data.pose.position.z)
            # self.fake_x.append(-self.cam_err_x*math.cos(math.pi/180*(relative_yaw))+self.cam_err_y*math.sin(math.pi/180*(relative_yaw)))
            # self.fake_y.append((self.cam_err_x*math.sin((relative_yaw)*math.pi/180)+self.cam_err_y*math.cos(math.pi/180*(relative_yaw))))
            # self.df['rc_th']     =      self.rc_th
            # self.df['rc_r ']     =      self.rc_r 
            # self.df['rc_p ']     =      self.rc_p 
            # self.df['rc_y ']     =      self.rc_y 
            # self.df['sen_r']     =      self.sen_r
            # self.df['sen_p']     =      self.sen_p
            # self.df['sen_y']     =      self.sen_y
            # self.df['cam_x']     =      self.cam_x
            # self.df['cam_y']     =      self.cam_y
            # self.df['cam_z']     =      self.cam_z
            # self.df['fake_x']    =      self.fake_x
            # self.df['fake_y']    =      self.fake_y
            # self.df['time']      =      time.time()
            # self.df.to_csv('Data.csv')    
            
        #FAIL SAFE 1: When the detection did not happen for 300 frames, reduce throttle
        if userRC.current_z<0 and self.current_consecutive_frames>100 and self.current_consecutive_frames<300:
            userRC.rcThrottle = 1000
            userRC.rcPitch    = 1500
            userRC.rcRoll     = 1500
            userRC.rcYaw      = 1500
            userRC.rcAUX4     = 1500
            userRC.rcAUX3     = 1500
            userRC.rcAUX2     = 1500
            userRC.rcAUX1     = 1500

            userRC.mode = 'Controller Default'
            # print("default_thr = ",1000)
            # print("default_Roll = ",1500)
            # print("default_Pitch",1500)
            # print(" ") 

            # Uncomment to get flight data
            
            # self.df = pd.DataFrame()
            # self.rc_th.append(userRC.rcThrottle)
            # self.rc_r.append(userRC.rcRoll)
            # self.rc_p.append(userRC.rcPitch)
            # self.rc_y.append(userRC.rcYaw)
            # self.sen_r.append(current_data.pose.orientation.x)
            # self.sen_p.append(current_data.pose.orientation.y)
            # self.sen_y.append(current_data.pose.orientation.z)
            # self.cam_x.append(current_data.pose.position.x)
            # self.cam_y.append(current_data.pose.position.y)
            # self.cam_z.append(current_data.pose.position.z)
            # self.fake_x.append(-self.cam_err_x*math.cos(math.pi/180*(relative_yaw))+self.cam_err_y*math.sin(math.pi/180*(relative_yaw)))
            # self.fake_y.append((self.cam_err_x*math.sin((relative_yaw)*math.pi/180)+self.cam_err_y*math.cos(math.pi/180*(relative_yaw))))
            # self.df['rc_th']     =      self.rc_th
            # self.df['rc_r ']     =      self.rc_r 
            # self.df['rc_p ']     =      self.rc_p 
            # self.df['rc_y ']     =      self.rc_y 
            # self.df['sen_r']     =      self.sen_r
            # self.df['sen_p']     =      self.sen_p
            # self.df['sen_y']     =      self.sen_y
            # self.df['cam_x']     =      self.cam_x
            # self.df['cam_y']     =      self.cam_y
            # self.df['cam_z']     =      self.cam_z
            # self.df['fake_x']    =      self.fake_x
            # self.df['fake_y']    =      self.fake_y
            # self.df['time']      =      time.time()
            # self.df.to_csv('Data.csv')    
              

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


            userRC.mode = 'Controller Disarm'
            # print("disarm_thr = ",1500)
            # print("disarm_Roll = ",1500)
            # print("disarm_Pitch",1500)
            # print(" ") 

            # Uncomment to get flight data

            # self.df = pd.DataFrame()
            # self.rc_th.append(userRC.rcThrottle)
            # self.rc_r.append(userRC.rcRoll)
            # self.rc_p.append(userRC.rcPitch)
            # self.rc_y.append(userRC.rcYaw)
            # self.sen_r.append(current_data.pose.orientation.x)
            # self.sen_p.append(current_data.pose.orientation.y)
            # self.sen_y.append(current_data.pose.orientation.z)
            # self.cam_x.append(current_data.pose.position.x)
            # self.cam_y.append(current_data.pose.position.y)
            # self.cam_z.append(current_data.pose.position.z)
            # self.fake_x.append(-self.cam_err_x*math.cos(math.pi/180*(relative_yaw))+self.cam_err_y*math.sin(math.pi/180*(relative_yaw)))
            # self.fake_y.append((self.cam_err_x*math.sin((relative_yaw)*math.pi/180)+self.cam_err_y*math.cos(math.pi/180*(relative_yaw))))
            # self.df['rc_th']     =      self.rc_th
            # self.df['rc_r ']     =      self.rc_r 
            # self.df['rc_p ']     =      self.rc_p 
            # self.df['rc_y ']     =      self.rc_y 
            # self.df['sen_r']     =      self.sen_r
            # self.df['sen_p']     =      self.sen_p
            # self.df['sen_y']     =      self.sen_y
            # self.df['cam_x']     =      self.cam_x
            # self.df['cam_y']     =      self.cam_y
            # self.df['cam_z']     =      self.cam_z
            # self.df['fake_x']    =      self.fake_x
            # self.df['fake_y']    =      self.fake_y
            # self.df['time']      =      time.time()
            # self.df.to_csv('Data.csv')    
           

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
            
            x1 =  tvec[0]
            y1 =  tvec[1]
            z1 = tvec[2]
            yaw = rvec[2]
 
            
        else:
            x1 = -1000
            y1 = -1000
            z1 = -1000
            yaw = -1000
            
        userRC.current_x = x1
        userRC.current_y = y1
        userRC.current_z = z1
        userRC.orientation_z_aruco = yaw
        
        #print(round(x1),round(y1),round(z1))
        if userRC.goto_called == True:
            plutoPID.controller_out()

        cv2.waitKey(1)

        #--- Display the frame
        frame_resized = cv2.resize(frame.copy(),None,fx=0.3,fy=0.3)
        cv2.imshow('frame', frame_resized)


# userRCAP = [1500,1500,1500,1500]

droneRC = [1500,1500,1500,1500,1000,1000,1000,1000]

NONE_COMMAND = 0
TAKE_OFF = 1
LAND = 2

        
def writeFunction():
    # Function to write to the drone
    requests = list()
    requests.append(MSP_RC)
    requests.append(MSP_ATTITUDE)
    requests.append(MSP_RAW_IMU)
    requests.append(MSP_ALTITUDE)
    requests.append(MSP_ANALOG)

    sendRequestMSP_ACC_TRIM()
  
    while True:
        droneRC[:] = userRC.rcValues() # copy the updated userRC values to droneRC

        sendRequestMSP_SET_RAW_RC(droneRC) # send the updated droneRC values to the drone
        print('R:{}, P:{}, T:{}, Y:{} '.format(droneRC[0],droneRC[1],droneRC[2],droneRC[3]),end='  ')
        print('x:{}, y:{}, z:{}, yaw:{},yaw_aruco:{}    '.format(round(userRC.current_x),round(userRC.current_y),round(userRC.current_z),userRC.orientation_z,round(userRC.orientation_z_aruco*180/np.pi)),end='  ')
        print('drone_err_x:{}, drone_err_y:{}, drone_err_z:{}, mode:{}'.format(round(plutoPID.drone_err_x),round(plutoPID.drone_err_y),round(plutoPID.cam_err_z),userRC.mode))
        sendRequestMSP_GET_DEBUG(requests)
        if (userRC.commandType != NONE_COMMAND):
            sendRequestMSP_SET_COMMAND(userRC.commandType)
            userRC.commandType = NONE_COMMAND


        #-------- Reading Yaw Value from IMU ----------#
        data = client.recv(1024)
        segments = data.split(b'$M>')
        
        for segment in segments:
            segment = b'$M>'+segment
            if len(segment) == 12:
                if segment[4] == 108:
#                     roll = segment[5:7]  
#                     pitch = segment[7:9]
#                     yaw = segment[9:11]
                    yaw = segment[9]+256*segment[10] # Little Endian Representation
                    userRC.orientation_z = yaw
                    
        time.sleep(0.022)

def on_press(key):
    # Create a function to handle key presses
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
            #----------------- PID START --------------------#
            userRC.goto_called = True
        
        if key.char == 'h':
            #----------------- PID HALT --------------------#
            userRC.goto_called = False

    
def on_release(key):
    #print('{0} release'.format(key))
    # Create a function to handle key releases
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
    print("CONNECTED!")

    userRC = USER_RC()

    # PID Constants
    K_z     = [2.5, 0.01,  0]
    K_pitch = [-4.5, 0, 0] 
    K_roll  = [-4.5, 0, 0] 
    K_yaw   = [0, 0, 0]
    # Creating an instance of the PID class to call the constructor
    plutoPID = PID(K_z,K_roll,K_pitch,K_yaw,dt=0.1,tau=0.06,alpha = 0.5,cam_orientation= 90.0,XT=0,YT=0,ZT=200)
    # ZT is 200 cm from the camera	
	
    # Threading simultaneously running functions
    thread = Thread(target=writeFunction)
    print('Write Thread Started')
    thread.start()
    
    thread_key = Thread(target=key_handling)
    print('Key Handling Thread Started')
    thread_key.start()
    
    thread_camera = Thread(target=aruco_detect)
    print('Aruco Detect Thread Started')
    thread_camera.start()
