#!/usr/bin/env python3
from cmath import inf
import time
import rospy
from plutodrone.msg import PlutoMsg
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32
from matplotlib import pyplot as plt
import pickle

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

    def __init__(self, Kp_roll: float, Kp_pitch: float, Kp: float, Ki: float, Kd: float, target: float, tau: float) -> None:
        
        self.Kp = Kp
        self.Kp_roll=Kp_roll
        self.Kp_pitch=Kp_pitch
        self.Ki = Ki
        self.Kd = Kd
        self.tau = tau
        self.target = target

        self.Dterm = 0
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
        rospy.Subscriber("Detection",PoseStamped,self.getZ)
        rospy.Subscriber("Kp",Float32,self.setKp)
        rospy.Subscriber("Kp_roll",Float32,self.setKp_roll)
        rospy.Subscriber("Kp_pitch",Float32,self.setKp_pitch)


    def update_yaw(self, feedback: float) -> float:
        error = -(self.target - feedback)
        self.Pterm = 1500 + self.Kp * error
        output = self.Pterm
        if output < self.min:
            return self.min
        if output > self.max:
            return self.max
        self.last_output = output
        return output
    def set_limits(self, min: float, max: float, min_int: float, max_int: float) -> None:
        self.max = max
        # self.max_int = max_int
        self.min = min
        # self.min_int = min_int

    def update(self, feedback: float) -> float:
        error = -(self.target - feedback)
        self.Pterm = 1700 + self.Kp * error
        output = self.Pterm  #+ self.Iterm + self.Dterm
        if output < self.min:
            return self.min
        if output > self.max:
            return self.max
        self.last_output = output
        return output
        
    def update_yaw(self, feedback: float) -> float:
        error = -(self.target - feedback)
        self.Pterm = 1500 + self.Kp * error
        output = self.Pterm
        if output < self.min:
            return self.min
        if output > self.max:
            return self.max
        self.last_output = output
        return output
        
    def update_roll(self, feedback: float) -> float:
        error = -(self.target - feedback)
        self.Pterm = 1500 + self.Kp * error
        output = self.Pterm
        if output < self.min:
            return self.min
        if output > self.max:
            return self.max
        self.last_output = output
        return output
        
    def update_pitch(self, feedback: float) -> float: 
        error = -(self.target - feedback)
        self.Pterm = 1500 + self.Kp * error
        output = self.Pterm
        if output < self.min:
            return self.min
        if output > self.max:
            return self.max
        self.last_output = output
        return output

    def getZ(self,z_value):
        current_z = z_value.pose.position.z
        current_x = z_value.pose.position.x
        current_y = z_value.pose.position.y
        if current_z!=-1:
            altitude_PID_output = self.update(current_z)

            obj=PlutoMsg()
            obj.rcPitch=self.update_pitch(current_x)
            obj.rcRoll=self.update_roll(current_y)
            obj.rcYaw=1500
            obj.rcAUX4=1500
            obj.rcAUX3=1500
            obj.rcAUX2=1500
            obj.rcAUX1=1500
            obj.rcThrottle=int(altitude_PID_output)
            self.pub.publish(obj)
            self.plotlist_throttle.append(int(altitude_PID_output-1000)/10)
            self.plotlist_height.append(int(current_z*100))
            # plt.plot(self.plotlist_throttle)self.plotlist_height)
            k=plt.plot([self.plotlist_throttle,self.plotlist_height])
            file=open('Graph.pickle','wb')
            pickle.dump(k,file)
            print("rcThrottle = ",altitude_PID_output)
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
        

    def main(self):
        rate=rospy.Rate(10)
        while not rospy.is_shutdown():
            rate.sleep()
    
if __name__ == '__main__':
    altitude_Kp = 2750
    x_Kp=500
    y_Kp=500
    altitude_Ki = 0
    altitude_Kd = 0
    target_z  = 0.75
    altitude_PID = PID(x_Kp,y_Kp,altitude_Kp,altitude_Ki,altitude_Kd,target_z,0)
    altitude_PID.main()
