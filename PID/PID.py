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
import time
import math
import rospy
from plutodrone.msg import PlutoMsg
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32
from matplotlib import pyplot as plt
import pickle
import pandas as pd
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

    def __init__(self,K_z: float, K_roll: float, K_pitch: float, K_yaw: float, dt:float, tau:float, alpha:float,cam_orientation:float,XT:float,YT:float,ZT:float) -> None:

        self.dt       = dt
        self.tau      = tau
        self.alpha    = alpha  # exponential smoothing factor
        self.memory_thr    = 0
        self.memory_roll   = 0
        self.current_consecutive_frames=0
        self.memory_pitch  = 0
        self.memory_yaw    = 0
        self.x_correction  = 0
        self.y_correction  = 0

        self.rc_th = []
        self.rc_r  = []
        self.rc_p  = []
        self.rc_y  = []
        self.fake_x= []
        self.sen_r = []
        self.fake_y= []
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


    def controller_out(self,current_data:PoseStamped):
        # getting feedback (current data)
        current_z   = current_data.pose.position.z-self.aim_z
        current_x   = current_data.pose.position.x-self.aim_x
        current_y   = current_data.pose.position.y-self.aim_y
        current_yaw = current_data.pose.orientation.z


        if current_z>0:
            
            self.current_consecutive_frames=0         
            self.df = pd.DataFrame()
            
            # rc outputs
            obj = PlutoMsg()
            obj.rcThrottle = int(self.update_z(current_z,1200,1900))
            obj.rcPitch    = int(self.update_pitch((current_x*math.sin((-cam_orientation+current_yaw)*math.pi/180)-current_y*math.cos(math.pi/180*(-cam_orientation+current_yaw))),1000,2000))            
            obj.rcRoll     = int(self.update_roll(current_x*math.cos(math.pi/180*(-cam_orientation+current_yaw))+current_y*math.sin(math.pi/180*(-cam_orientation+current_yaw)),1000,2000))            
            #obj.rcYaw      = int(self.update_yaw(current_yaw))
            obj.rcYaw      = 1500
            obj.rcAUX4     = 1500
            obj.rcAUX3     = 1500
            obj.rcAUX2     = 1500
            obj.rcAUX1     = 1500
            self.pub.publish(obj)

            self.memory_thr = obj.rcThrottle
            self.memory_pitch = obj.rcPitch
            self.memory_roll = obj.rcRoll
            
            print(time.time())
            print("rcThrottle = ",obj.rcThrottle)
            print("rcRoll = ",obj.rcRoll)
            print("rcPitch",obj.rcPitch)
            print(" ")
            
            # storing data
            self.df = pd.DataFrame()
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
            self.fake_x.append((current_x*math.sin((-cam_orientation+current_yaw)*math.pi/180)-current_y*math.cos(math.pi/180*(-cam_orientation+current_yaw))))
            self.fake_y.append(current_x*math.cos(math.pi/180*(-cam_orientation+current_yaw))+current_y*math.sin(math.pi/180*(-cam_orientation+current_yaw)))
            

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
        if current_z<0 and self.current_consecutive_frames<=40:
            obj=PlutoMsg()
            obj.rcThrottle       =      self.memory_thr
            obj.rcPitch          =      self.memory_pitch
            obj.rcRoll           =      self.memory_roll
            obj.rcYaw            =      1500
            obj.rcAUX4           =      1500
            obj.rcAUX3           =      1500
            obj.rcAUX2           =      1500
            obj.rcAUX1           =      1500
            self.pub.publish(obj)
            print("memory_thr = ",self.memory_thr)
            print("memory_Roll = ",self.memory_roll)
            print("memory_Pitch",self.memory_pitch)
            print(" ")
            self.df = pd.DataFrame()
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
            self.fake_x.append((current_x*math.sin((-cam_orientation+current_yaw)*math.pi/180)-current_y*math.cos(math.pi/180*(-cam_orientation+current_yaw))))
            self.fake_y.append(current_x*math.cos(math.pi/180*(-cam_orientation+current_yaw))+current_y*math.sin(math.pi/180*(-cam_orientation+current_yaw)))
           


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
            
        if current_z<0 and self.current_consecutive_frames>40 and self.current_consecutive_frames<300:
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
            print("default_thr = ",1000)
            print("default_Roll = ",1500)
            print("default_Pitch",1500)
            print(" ") 
            self.df = pd.DataFrame()
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
            self.fake_x.append((current_x*math.sin((-cam_orientation+current_yaw)*math.pi/180)-current_y*math.cos(math.pi/180*(-cam_orientation+current_yaw))))
            self.fake_y.append(current_x*math.cos(math.pi/180*(-cam_orientation+current_yaw))+current_y*math.sin(math.pi/180*(-cam_orientation+current_yaw)))
            
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
        if current_z<0 and self.current_consecutive_frames>=300:
            obj=PlutoMsg()
            obj.rcThrottle = 1500
            obj.rcPitch    = 1500
            obj.rcRoll     = 1500
            obj.rcYaw      = 1500
            obj.rcAUX4     = 1000
            obj.rcAUX3     = 1500
            obj.rcAUX2     = 1500
            obj.rcAUX1     = 1500
            self.pub.publish(obj)
            print("disarm_thr = ",1500)
            print("disarm_Roll = ",1500)
            print("disarm_Pitch",1500)
            print(" ") 
            self.df = pd.DataFrame()
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
            self.fake_x.append((current_x*math.sin((-cam_orientation+current_yaw)*math.pi/180)-current_y*math.cos(math.pi/180*(-cam_orientation+current_yaw))))
            self.fake_y.append(current_x*math.cos(math.pi/180*(-cam_orientation+current_yaw))+current_y*math.sin(math.pi/180*(-cam_orientation+current_yaw)))
            
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

    def main(self):
        rate=rospy.Rate(50)
        while not rospy.is_shutdown():
            rate.sleep()

if __name__ == '__main__':
    try:
        K_z     = [2500, 0, 50]
        K_pitch = [0, 0, 0]
        K_roll  = [0 , 0, 0]
        K_yaw   = [0, 0, 0]

        pid = PID(K_z,K_roll,K_pitch,K_yaw,dt=0.1,tau=0.06,alpha = 0.5,cam_orientation=180.0,XT=0,YT=0,ZT=0.35)
        pid.main()

    except KeyboardInterrupt:
        print ("keyboarrrdd")
        pass
