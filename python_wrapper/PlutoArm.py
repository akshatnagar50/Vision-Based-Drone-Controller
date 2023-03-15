from plutoMultiwii import *
from threading import Thread
import time
from pynput.keyboard import Key, Listener

TRIM_MAX = 1000
TRIM_MIN = -1000

# userRC = [1500,1500,1500,1500,1000,1000,1000,1000]
# userRC = {'rcRoll': 1500, 'rcPitch': 1500, 'rcThrottle': 1500, 'rcYaw': 1500,
# 'rcAUX1':1000, 'rcAUX2':1000, 'rcAUX3':1000, 'rcAUX4':1000}

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
        self.rcThrottle = 1800

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

# userRCAP = [1500,1500,1500,1500]

droneRC = [1500,1500,1500,1500,1000,1000,1000,1000]

NONE_COMMAND = 0
TAKE_OFF = 1
LAND = 2

# commandType = 0

def writeFunction():
    requests = list()
    requests.append(MSP_RC)
    requests.append(MSP_ATTITUDE)
    requests.append(MSP_RAW_IMU)
    requests.append(MSP_ALTITUDE)
    requests.append(MSP_ANALOG)

    sendRequestMSP_ACC_TRIM()

    while True:
        # global commandType
        droneRC[:] = userRC.rcValues()

        sendRequestMSP_SET_RAW_RC(droneRC)
        print(droneRC)
        sendRequestMSP_GET_DEBUG(requests)

        if (userRC.commandType != NONE_COMMAND):
            sendRequestMSP_SET_COMMAND(userRC.commandType)
            userRC.commandType = NONE_COMMAND

        time.sleep(0.022)


def on_press(key):
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

    
def on_release(key):
    print('{0} release'.format(key))
    
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

    userRC = USER_RC()

    thread = Thread(target=writeFunction)
    thread.start()
    thread_key = Thread(target=key_handling)
    thread_key.start()

