import subprocess

import numpy as np
import os
#automating the entire process to arming of drone
#connect to pluto wifi
def connect_to_wifi(ssid, password):
    command = "nmcli dev wifi connect {0} password {1}".format(ssid, password)
    process = subprocess.Popen(command.split(), stdout=subprocess.PIPE)
    output, error = process.communicate()
    return output
#update pluto hostname ssid and password below
# output = connect_to_wifi("OnePlusAayush", "aayush030202")

command = "cd catkin_ws; catkin_make; source devel/setup.bash"

ret = subprocess.run(command, capture_output=True, shell=True)

# before Python 3.7:
# ret = subprocess.run(command, stdout=subprocess.PIPE, shell=True)

# print(ret.stdout.decode())
command="xterm -e roscore &"
ret = subprocess.run(command, capture_output=False, shell=True)
command="xterm -e rostopic echo /rosout &"
ret = subprocess.run(command, capture_output=True, shell=True)
