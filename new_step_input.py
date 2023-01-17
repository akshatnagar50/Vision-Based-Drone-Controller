#!/usr/bin/env python
import rospy
import time
from plutodrone.msg import PlutoMsg
def arm(pub,t):
    obj=PlutoMsg()
    obj.rcPitch=1500
    obj.rcRoll=1500
    obj.rcYaw=1500
    obj.rcAUX4=1500
    obj.rcAUX3=1500
    obj.rcAUX2=1500
    obj.rcAUX1=1500
    obj.rcThrottle=1000
    t2=0
    while(t2<t+5):
        t2=time.time()
        pub.publish(obj)

def throttle_step(pub,t):
    obj=PlutoMsg()
    obj.rcPitch=1500
    obj.rcRoll=1500
    obj.rcYaw=1500
    obj.rcAUX4=1500
    obj.rcThrottle=1825
    t2=0
    while(t2<t+5):
        t2=time.time()
        pub.publish(obj)   
    obj.rcThrottle=1875
    t2=0
    t=time.time()
    while(t2<t+5):
        t2=time.time()
        pub.publish(obj) 
def pitch_step(pub,t):
    obj=PlutoMsg()
    obj.rcPitch=1500
    obj.rcRoll=1500
    obj.rcYaw=1500
    obj.rcAUX4=1500
    obj.rcThrottle=1825
    t2=0
    while(t2<t+5):
        t2=time.time()
        pub.publish(obj)     
    obj.rcPitch=1510
    t2=0
    t=time.time()
    while(t2<t+5):
        t2=time.time()
        pub.publish(obj) 

def roll_step(pub,t):
    obj=PlutoMsg()
    obj.rcPitch=1500
    obj.rcRoll=1500
    obj.rcYaw=1500
    obj.rcAUX4=1500
    obj.rcThrottle=1825
    t2=0
    while(t2<t+5):
        t2=time.time()
        pub.publish(obj)  
    obj.rcRoll=1510
    t2=0
    t=time.time()
    while(t2<t+5):
        t2=time.time()
        pub.publish(obj) 

def yaw_step(pub,t):
    obj=PlutoMsg()
    obj.rcPitch=1500
    obj.rcRoll=1500
    obj.rcYaw=1500
    obj.rcAUX4=1500
    obj.rcThrottle=1825
    t2=0
    while(t2<t+5):
        t2=time.time()
        pub.publish(obj)  
    obj.rcYaw=1510
    t2=0
    t=time.time()
    while(t2<t+5):
        t2=time.time()
        pub.publish(obj)  

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
    rospy.init_node('step',anonymous=True)  
    pub=rospy.Publisher("/drone_command",PlutoMsg,queue_size=10)
    rate=rospy.Rate(10)
    try:
        t=time.time()
        arm(pub,t)
        t=time.time()
        throttle_step(pub,t)
        land(pub)
        time.sleep(10)
        t=time.time()
        arm(pub,t)
        t=time.time()
        pitch_step(pub,t)
        land(pub)
        time.sleep(10)
        t=time.time()
        arm(pub,t)
        t=time.time()
        roll_step(pub,t)
        t=time.time()
        land(pub)
        time.sleep(10)
        t=time.time()
        arm(pub,t)
        t=time.time()
        yaw_step(pub,t)
        t=time.time()
        land(pub)
        time.sleep(10)
    except rospy.ROSInterruptException:
        land(pub)
