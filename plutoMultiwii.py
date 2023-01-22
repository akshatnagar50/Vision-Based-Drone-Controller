import socket

TRIM_MAX = 1000
TRIM_MIN = -1000

isAutoPilotOn = 0

# Communication com()
# Protocol pro;
# ros::ServiceClient serviceClient;
# plutodrone::PlutoPilot service;

userRC = [1500,1500,1500,1500,1000,1000,1000,1000]

userRCAP = [1500,1500,1500,1500]

droneRC = [1500,1500,1500,1500,1000,1000,1000,1000]
    
MSP_HEADER = "244d3c"# "$M<"
TCP_IP = '192.168.4.1'
TCP_PORT = 23

MSP_FC_VERSION=3
MSP_RAW_IMU=102
MSP_RC = 105
MSP_ATTITUDE=108
MSP_ALTITUDE=109
MSP_ANALOG=110
MSP_SET_RAW_RC=200
MSP_ACC_CALIBRATION=205
MSP_MAG_CALIBRATION=206
MSP_SET_MOTOR=214
MSP_SET_ACC_TRIM=239
MSP_ACC_TRIM=240
MSP_EEPROM_WRITE = 250
MSP_SET_POS= 216
MSP_SET_COMMAND = 217


client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
client.connect((TCP_IP, TCP_PORT))

def sendRequestMSP(data):
    '''
    client: client object returned by com()
    data: string created by createPacketMSP()
    '''
    client.send(bytes.fromhex(data))

def createPacketMSP(msp, payload):
    '''
    msp (UINT8): message type 
    payload (list): Contains the rc values in case of 
    sendRequestMSP_SET_RAW_RC
    '''
    bf = ""
    bf += MSP_HEADER

    checksum = 0
    pl_size = len(payload)
    checksum ^= pl_size
    checksum ^= msp
    for k in payload:
        bf += '{:02x}'.format(k & 0xFF)
        bf += '{:02x}'.format((k >> 8) & 0xFF)
        # In Protocol.cpp, the above is done while sending the payload itself
        # 
    bf += '{:02x}'.format(checksum)

    return bf

def sendRequestMSP_SET_RAW_RC(channels):
    '''
    channels: list of 8 RC channel values
    '''
    sendRequestMSP(createPacketMSP(channels))

def sendRequestMSP_SET_COMMAND(commandType):
    sendRequestMSP(createPacketMSP(MSP_SET_COMMAND, [commandType]))

def sendRequestMSP_GET_DEBUG(requests):
    
    for i in range(len(requests)):
        sendRequestMSP(createPacketMSP(requests[i], []))

def sendRequestMSP_SET_ACC_TRIM(trim_roll, trim_pitch):

    sendRequestMSP(createPacketMSP(MSP_ACC_TRIM, [trim_roll, trim_pitch]))

def sendRequestMSP_ACC_TRIM():
    sendRequestMSP(createPacketMSP(MSP_ACC_TRIM, []))

def sendRequestMSP_EEPROM_WRITE():
    sendRequestMSP(createPacketMSP(MSP_EEPROM_WRITE, []))