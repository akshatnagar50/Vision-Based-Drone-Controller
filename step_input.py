#!/usr/bin/env python
import rospy
import time
from plutodrone.msg import PlutoMsg
def throttle_step(pub):
    obj=PlutoMsg()
    obj.rcPitch=1500
    obj.rcRoll=1500
    obj.rcYaw=1500
    obj.rcAUX4=1500
    obj.rcThrottle=1825
    pub.publish(obj)
    time.sleep(5)
    obj.rcThrottle=1875
    pub.publish(obj)
    time.sleep(5)
def pitch_step(pub):
    obj=PlutoMsg()
    obj.rcPitch=1500
    obj.rcRoll=1500
    obj.rcYaw=1500
    obj.rcAUX4=1500
    obj.rcThrottle=1825
    pub.publish(obj)
    time.sleep(5)    
    obj.rcPitch=1510
    pub.publish(obj)
    time.sleep(5)

def roll_step(pub):
    obj=PlutoMsg()
    obj.rcPitch=1500
    obj.rcRoll=1500
    obj.rcYaw=1500
    obj.rcAUX4=1500
    obj.rcThrottle=1825
    pub.publish(obj)
    time.sleep(5)    
    obj.rcRoll=1510
    pub.publish(obj)
    time.sleep(5)

def yaw_step(pub):
    obj=PlutoMsg()
    obj.rcPitch=1500
    obj.rcRoll=1500
    obj.rcYaw=1500
    obj.rcAUX4=1500
    obj.rcThrottle=1825
    pub.publish(obj)
    time.sleep(5)    
    obj.rcYaw=1510
    pub.publish(obj)
    time.sleep(5)  

def land(pub):
    obj=PlutoMsg()
    obj.rcPitch=1500
    obj.rcRoll=1500
    obj.rcYaw=1500
    obj.rcAUX4=1000
    obj.rcAUX3=1500
    obj.rcAUX2=1500
    obj.rcAUX1=1500
    obj.rcThrottle=1825
    pub.publish(obj)
    time.sleep(5)      


if __name__=='__main__':
    try:
        rospy.init_node('step',anonymous=True)  
        pub=rospy.Publisher("/drone_command",PlutoMsg,queue_size=10)
        rate=rospy.Rate(10)
        throttle_step(pub)
        land(pub)
        a=input("HI")
        pitch_step(pub)
        land(pub)
        a=input("HI")
        roll_step(pub)
        land(pub)
        a=input("HI")
        yaw_step(pub)
        land(pub)
        a=input("HI")
        land(pub)
    except rospy.ROSInterruptException:
        pass
