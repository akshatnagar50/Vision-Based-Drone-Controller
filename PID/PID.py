#!/usr/bin/env python3
from cmath import inf
import time
import rospy
from plutodrone.msg import PlutoMsg
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32
from matplotlib import pyplot as plt
import pickle
import pandas as pd
rc_th=[]
rc_r =[]
rc_p =[]
rc_y =[]
sen_r =[]
sen_p =[]
sen_y  =[]
cam_x = []
cam_y =[]
cam_z =[]
# import csv

class PID:

    def arm(self,pub):
        obj=PlutoMsg()
        obj.rcPitch=1500
        obj.rcRoll=1500
        obj.rcYaw=1500
        obj.rcAUX4=1500
        obj.rcAUX3=1500
        obj.rcAUX2=1500                
        obj.rcAUX1=1500
        obj.rcThrottle=1000
        t=time.time()
        while(time.time()-t<5):
            pub.publish(obj)
        print('SLept')
    """
    Implements a PID controller.
    """

    def __init__(self, Kp_roll: float, Kp_pitch: float, Kp: float,Kp_yaw: float) -> None:
        
        self.Kp = Kp
        self.Kp_roll=Kp_roll
        self.Kp_pitch=Kp_pitch
        self.Kp_yaw=Kp_yaw
        # self.Ki = Ki
        # self.Kd = Kd
        # self.tau = tau
        # self.target = target

        self.Dterm = 0
        self.df=pd.DataFrame()
        self.Iterm = 0
        self.last_error = 0
        self.last_time = time.time()    
        self.last_feedback = 0
        self.plotlist_throttle=[]
        self.plotlist_height=[]
        self.last_output = 0
        self.set_limits(1000, 1875, -inf, inf)
        rospy.init_node("PID")
        self.pub=rospy.Publisher("/drone_command",PlutoMsg,queue_size=10)
        self.arm(self.pub)
        rospy.Subscriber("Detection",PoseStamped,self.controller_out)
        rospy.Subscriber("Kp",Float32,self.setKp)
        rospy.Subscriber("Kp_roll",Float32,self.setKp_roll)
        rospy.Subscriber("Kp_pitch",Float32,self.setKp_pitch)

   

    def set_limits(self, min: float, max: float, min_int: float, max_int: float) -> None:
        self.max = max
        # self.max_int = max_int
        self.min = min
        # self.min_int = min_int

    def update(self, feedback: float) -> float:
        error = -(0.75 - feedback)
        self.Pterm = 1700 + self.Kp * error
        output = self.Pterm  #+ self.Iterm + self.Dterm
        if output < self.min:
            return self.min
        if output > self.max:
            return self.max
        self.last_output = output
        return output
        
    def update_yaw(self, feedback: float) -> float:
        # if feedback>180:
        #     feedback = feedback -360
        error = (180 - feedback)
        self.Pterm_yaw = 1500 + self.Kp_yaw * error
        output = self.Pterm_yaw
        if output < self.min:
            return self.min
        if output > self.max:
            return self.max
        self.last_output = output
        return output
        
    def update_roll(self, feedback: float) -> float:
        error = (0 - feedback)
        self.Pterm_roll = 1500 + self.Kp_roll * error
        output = self.Pterm_roll
        if output < self.min:
            return self.min
        if output > self.max:
            return self.max
        self.last_output = output
        return output
        
    def update_pitch(self, feedback: float) -> float: 
        error = (0 - feedback)
        self.Pterm_pitch = 1500 + self.Kp_pitch * error
        output = self.Pterm_pitch
        if output < self.min:
            return self.min
        if output > self.max:
            return self.max
        self.last_output = output
        return output

    def controller_out(self,current_data:PoseStamped):
        global rc_th
        global rc_p
        global rc_y
        global rc_r
        global sen_r 
        global sen_p 
        global sen_y 
        global cam_x 
        global cam_y 
        global cam_z 

        current_z = current_data.pose.position.z
        current_x = current_data.pose.position.x
        current_y = current_data.pose.position.y
        current_yaw = current_data.pose.orientation.z
        altitude_PID_output = self.update(current_z)
        if current_z!=-1:
            obj=PlutoMsg()
            obj.rcThrottle=int(self.update(current_z))
            obj.rcPitch=int(self.update_pitch(current_x))
            obj.rcRoll=int(self.update_roll(current_y))
            obj.rcYaw=int(self.update_yaw(current_yaw))
            obj.rcAUX4=1500
            obj.rcAUX3=1500
            obj.rcAUX2=1500
            obj.rcAUX1=1500
            rc_th.append(obj.rcThrottle)
            rc_r.append(obj.rcRoll)
            rc_p.append(obj.rcPitch)
            rc_y.append(obj.rcYaw)
            sen_r.append(current_data.pose.orientation.x)
            sen_p.append(current_data.pose.orientation.y)
            sen_y.append(current_data.pose.orientation.z)
            cam_x.append(current_data.pose.position.x)
            cam_y.append(current_data.pose.position.y)
            cam_z.append(current_data.pose.position.z)
            self.pub.publish(obj)
            self.plotlist_throttle.append(int(altitude_PID_output-1000)/10)
            self.plotlist_height.append(int(current_z*100))
            # plt.plot(self.plotlist_throttle)self.plotlist_height)
            k=plt.plot([self.plotlist_throttle,self.plotlist_height])
            file=open('Graph.pickle','wb')
            pickle.dump(k,file)
            print("rcThrottle = ",altitude_PID_output)
            self.df['rc_th']=rc_th
            self.df['rc_th']=rc_th
            self.df['rc_r ']=rc_r 
            self.df['rc_p ']=rc_p 
            self.df['rc_y ']=rc_y 
            self.df['sen_r']=sen_r
            self.df['sen_p']=sen_p
            self.df['sen_y']=sen_y
            self.df['cam_x']=cam_x
            self.df['cam_y']=cam_y
            self.df['cam_z']=cam_z
            self.df.to_csv('Data.csv')
        else:
            obj=PlutoMsg()
            obj.rcPitch=1500
            obj.rcRoll=1500
            obj.rcYaw=1500
            obj.rcAUX4=1500
            obj.rcAUX3=1500
            obj.rcAUX2=1500
            obj.rcAUX1=1500
            obj.rcThrottle=1800 
            self.pub.publish(obj)
            print("rcThrottle = 1800")

    def setKp(self,msg):
        self.Kp=msg.data
        
    def setKp_roll(self,msg):
        self.Kp_roll=msg.data
        
    def setKp_pitch(self,msg):
        self.Kp_pitch=msg.data
        
    def setKp_yaw(self,msg):
        self.Kp_yaw = msg.data

    def main(self):
        rate=rospy.Rate(10)
        while not rospy.is_shutdown():
            rate.sleep()
    
if __name__ == '__main__':
    try:
        altitude_Kp = 2750
        roll_Kp=200
        pitch_Kp=200
        yaw_Kp = 50
        # altitude_Ki = 0
        # altitude_Kd = 0
        # target_z  = 0.75
        altitude_PID = PID(roll_Kp,pitch_Kp,altitude_Kp,yaw_Kp)
        altitude_PID.main()
    except KeyboardInterrupt:
        print ("keyboarrrdd")
        #df=pd.DataFrame()
        #df['rc_th']=rc_th
        #df['rc_th']=rc_th
        #df['rc_r ']=rc_r 
        #df['rc_p ']=rc_p 
        #df['rc_y ']=rc_y 
        #df['sen_r']=sen_r
        #df['sen_p']=sen_p
        #df['sen_y']=sen_y
        #df['cam_x']=cam_x
        #df['cam_y']=cam_y
        #df['cam_z']=cam_z
        #df.to_csv('Data.csv')
        pass

