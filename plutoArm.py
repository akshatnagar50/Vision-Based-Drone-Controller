from plutoMultiwii import *
from threading import Thread
import time
import keyboard

TRIM_MAX = 1000
TRIM_MIN = -1000

NONE_COMMAND = 0
TAKE_OFF = 1
LAND = 2

commandType = 0

def writeFunction():
    requests = list()
    requests.append(MSP_RC)
    requests.append(MSP_ATTITUDE)
    requests.append(MSP_RAW_IMU)
    requests.append(MSP_ALTITUDE)
    requests.append(MSP_ANALOG)

    sendRequestMSP_ACC_TRIM()

    while True:
        
        droneRC = userRC

        sendRequestMSP_SET_RAW_RC(droneRC)
        sendRequestMSP_GET_DEBUG(requests)

        if (commandType != NONE_COMMAND):
            sendRequestMSP_SET_COMMAND(commandType)
            commandType = NONE_COMMAND

        time.sleep(0.022)
    

if __name__ == '__main__':
    pass


