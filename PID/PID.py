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
        """

        Parameters
        ----------
        Kp : float
            Proportional gain.
        Ki : float
            Integration gain.
        Kd : float
            Derivative gain.
        tau : float
            Low pass filter time constant.
                    target : float
            Target value.
        """
        
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
        self.set_limits(1000, 2000, -inf, inf)
        rospy.init_node("PID")
        rospy.Subscriber("Detected",PoseStamped,self.getZ)
        self.pub=rospy.Publisher("/drone_command",PlutoMsg,queue_size=10)
        rospy.Subscriber("Kp",Float32,self.setKp)


    def set_limits(self, min: float, max: float, min_int: float, max_int: float) -> None:
        """
        Output limits.

        Parameters
        ----------
        min : float
            Minimum output.
        max : float
            Maximum output.
        """
        self.max = max
        # self.max_int = max_int
        self.min = min
        # self.min_int = min_int
    def update(self, feedback: float) -> float:
        """
        Calculate the PID output value.

        Parameters
        ----------
        feedback : float
            Value to be compared to the target.

        Returns
                -------
        float
            Output of the PID controller.
        """
        # print(f"P: {self.Pterm}")  #, I: {self.Iterm}, f: {feedback}
        
        # current_time = time.time()
        # delta_time = 0.001
        # if delta_time == 0:
        #     return self.last_output

        # self.Iterm += (error + self.last_error) * 0.5 * self.Ki * delta_time
        # self.Dterm = (-2 * self.Kd * (feedback - self.last_feedback)
        #               + (2 * self.tau - delta_time) * self.Dterm / (2 * self.tau + delta_time))

        # if self.Iterm > self.max_int:
        #     self.Iterm = self.max_int
        # elif self.Iterm < self.min_int:
        #     self.Iterm = self.min_int

        # self.last_time = current_time
        # self.last_error = error
        # self.last_feedback = feedback


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
       altitude_PID_output = self.update(current_z)
       obj=PlutoMsg()
       obj.rcPitch=1500
       obj.rcRoll=1500
       obj.rcYaw=1500
       obj.rcAUX4=1500
       obj.rcThrottle=altitude_PID_output
       self.pub.publish(obj)

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
        
    # altitude_PID.set_limits(1000,2000,0,0)
    
