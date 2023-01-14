import sockets
import numpy as np

PORT=23
IP_ADDRESS="192.168.4.1"
CAMERA_PORT=9060
CAMERA_IP_ADDRESS="192.168.0.1"

#protocol pro to be added

indx=0
len=0
checksum=0
command=0
payload_size=0


socketSyckLock=0
socketOpStarted=0
checksumIndex=0
recbuf=np.zeros(1024)
c_s