#!/usr/bin/env python
import rospy
import time
from plutodrone.msg import PlutoMsg
def throttle_step():
    pub=rospy.publisher("/drone_command",PlutoMsg,queue_size=10)
    rate=rospy.Rate(10)
    obj=PlutoMsg()
    obj.rcThrottle=1825
    pub.publish(obj)
    time.sleep(5)
    obj.reThrottle=1875
    pub.publish(obj)
    time.sleep(5)
def pitch_step():
    pub=rospy.publisher("/drone_command",PlutoMsg,queue_size=10)
    rate=rospy.Rate(10)
    obj=PlutoMsg()
    obj.rcThrottle=1825
    pub.publish(obj)
    time.sleep(5)    
    obj.rcPitch=1510
    pub.publish(obj)
    time.sleep(5)

def roll_step():
    pub=rospy.publisher("/drone_command",PlutoMsg,queue_size=10)
    rate=rospy.Rate(10)
    obj=PlutoMsg()
    obj.rcThrottle=1825
    pub.publish(obj)
    time.sleep(5)    
    obj.rcRoll=1510
    pub.publish(obj)
    time.sleep(5)

def yaw_step():
    pub=rospy.publisher("/drone_command",PlutoMsg,queue_size=10)
    rate=rospy.Rate(10)
    obj=PlutoMsg()
    obj.rcThrottle=1825
    pub.publish(obj)
    time.sleep(5)    
    obj.rcYaw=1510
    pub.publish(obj)
    time.sleep(5)  

if __name__=='__main__':
    try:
        rospy.init_node('step',anonymous=True)  
        throttle_step()
        pitch_step()
        roll_step()
        yaw_step()
    except rospy.ROSInterruptException:
        pass
