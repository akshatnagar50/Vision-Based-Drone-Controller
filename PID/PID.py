from cmath import inf
import time
import rospy
from plutodrone.msg import PlutoMsg
from geometry_msgs.msg import PoseStamped
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

        self.Dterm = 1000
        self.Iterm = 0
        self.last_error = 0
        self.last_time = time.time()
        self.last_feedback = 0
        self.last_output = 0
        self.set_limits(0, inf, -inf, inf)

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
        error = self.target - feedback
        
        # current_time = time.time()
        # delta_time = 0.001
        # if delta_time == 0:
        #     return self.last_output

        self.Pterm = 1500 + self.Kp * error
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

        print(f"P: {self.Pterm}")  #, I: {self.Iterm}, f: {feedback}

        output = self.Pterm  #+ self.Iterm + self.Dterm
        if output < self.min:
            return self.min
        if output > self.max:
            return self.max
        self.last_output = output
        return output





def getZ(z_value):
    current_z = z_value.pose.orientation.x
    altitude_PID_output = altitude_PID.update(current_z)
    obj=PlutoMsg()
    obj.rcPitch=1500
    obj.rcRoll=1500
    obj.rcYaw=1500
    obj.rcAUX4=1500
    obj.rcThrottle=altitude_PID_output
    pub.publish(obj)
    
if __name__ == '__main__':
    altitude_Kp = 200
    altitude_Ki = 0
    altitude_Kd = 0
    target_z  = 1.0
    altitude_PID = PID(altitude_Kp,altitude_Ki,altitude_Kd,target_z,0)
    altitude_PID.set_limits(1000,2000,0,0)
    rospy.init_node("PID")
    rospy.Subscriber("Detected",PoseStamped,callback=getZ)
    pub=rospy.Publisher("/drone_command",PlutoMsg,queue_size=10)

