from cmath import inf
import time
import rospy
from plutodrone.msg import PlutoMsg
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32


class PID:
    """
    Implements a PID controller.
    """

    def __init__(self, Kp: float, Ki: float, Kd: float, target: float, tau: float) -> None:
        
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.tau = tau
        self.target = target

        self.Dterm = 0
        self.Iterm = 0
        self.last_error = 0
        self.last_time = time.time()
        self.last_feedback = 0
        self.last_output = 0
        self.set_limits(1000, 1875, -inf, inf)
        rospy.init_node("PID")
        rospy.Subscriber("Detected",PoseStamped,self.getZ)
        self.pub=rospy.Publisher("/drone_command",PlutoMsg,queue_size=10)
        rospy.Subscriber("Kp",Float32,self.setKp)


    def set_limits(self, min: float, max: float, min_int: float, max_int: float) -> None:
        self.max = max
        # self.max_int = max_int
        self.min = min
        # self.min_int = min_int

    def update(self, feedback: float) -> float:
        error = self.target - feedback
        self.Pterm = 1500 + self.Kp * error
        output = self.Pterm  #+ self.Iterm + self.Dterm
        if output < self.min:
            return self.min
        if output > self.max:
            return self.max
        self.last_output = output
        return output


    def getZ(self,z_value):
        current_z = z_value.pose.orientation.x
        if current_z!=-1:
            altitude_PID_output = self.update(current_z)
            obj=PlutoMsg()
            obj.rcPitch=1500
            obj.rcRoll=1500
            obj.rcYaw=1500
            obj.rcAUX4=1500
            obj.rcAUX3=1500
            obj.rcAUX2=1500
            obj.rcAUX1=1500
            obj.rcThrottle=altitude_PID_output
            self.pub.publish(obj)
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

    def main(self):
        rate=rospy.Rate(10)
        while not rospy.is_shutdown():
            rate.sleep()
    
if __name__ == '__main__':
    altitude_Kp = 200
    altitude_Ki = 0
    altitude_Kd = 0
    target_z  = 1.0
    altitude_PID = PID(altitude_Kp,altitude_Ki,altitude_Kd,target_z,0)
    altitude_PID.main()
