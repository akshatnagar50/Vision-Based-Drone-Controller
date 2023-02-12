import socket
from threading import Thread
TRIM_MAX = 1000
TRIM_MIN = -1000

isAutoPilotOn = 0

# Communication com()
# Protocol pro;
# ros::ServiceClient serviceClient;
# plutodrone::PlutoPilot service;
    
MSP_HEADER = "244d3c"# "$M<"
MSP_HEADER_1="244d3e"# "$M>"
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


client1 = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
client1.connect(("192.168.117.252", TCP_PORT)) 


client2=socket.socket(socket.AF_INET, socket.SOCK_STREAM)
client2.connect(("192.168.117.206",TCP_PORT))

mul_clients=[]
mul_clients.append(client1)
mul_clients.append(client2)
#add ip addresses here
# def sendRequestMSP(data):
#     '''
#     client: client object returned by com()
#     data: string created by createPacketMSP()
#     '''
#     client.send(bytes.fromhex(data))

def sendMulRequestMSP(data, i):
    mul_clients[i].send(bytes.fromhex(data))

def createPacketMSP(msp, payload):
    '''
    msp (UINT8): message type 
    payload (list): Contains the rc values in case of 
    sendRequestMSP_SET_RAW_RC
    '''
    bf = ""
    bf += MSP_HEADER

    checksum = 0
    if (msp == MSP_SET_COMMAND):
        pl_size = 1
    else:
        pl_size = len(payload) * 2 # No.of bytes

    bf += '{:02x}'.format(pl_size & 0xFF)
    checksum ^= pl_size

    bf += '{:02x}'.format(msp & 0xFF)
    checksum ^= msp

    for k in payload:
        if (msp == MSP_SET_COMMAND):
            bf += '{:02x}'.format(k & 0xFF)
            checksum ^= k & 0xFF

        else:
            bf += '{:02x}'.format(k & 0xFF)
            checksum ^= k & 0xFF
            bf += '{:02x}'.format((k >> 8) & 0xFF)
            checksum ^= (k >> 8) & 0xFF
        # In Protocol.cpp, the above is done while sending the payload itself
        # 
    bf += '{:02x}'.format(checksum)

    return bf


# def sendRequestMSP_SET_RAW_RC(channels):
#     '''
#     channels: list of 8 RC channel values
#     '''
#     sendRequestMSP(createPacketMSP(MSP_SET_RAW_RC, channels))

def sendMulRequestMSP_SET_RAW_RC(channels):
    """
    channels: list of 8 RC channel values + 
    """
    sendMulRequestMSP(createPacketMSP(MSP_SET_RAW_RC, channels),channels[8])

# def sendRequestMSP_SET_COMMAND(commandType):
#     sendRequestMSP(createPacketMSP(MSP_SET_COMMAND, [commandType]))

# def sendRequestMSP_GET_DEBUG(requests):
#     for i in range(len(requests)):
#         sendRequestMSP(createPacketMSP(requests[i], []))
        
def sendMulRequestMSP_GET_DEBUG(requests,index):
    for i in range(len(requests)):
        sendMulRequestMSP(createPacketMSP(requests[i],[]),index)

# def sendRequestMSP_SET_ACC_TRIM(trim_roll, trim_pitch):

#     sendRequestMSP(createPacketMSP(MSP_ACC_TRIM, [trim_roll, trim_pitch]))

# def sendRequestMSP_ACC_TRIM():
#     sendRequestMSP(createPacketMSP(MSP_ACC_TRIM, []))

# def sendRequestMSP_EEPROM_WRITE():
#     sendRequestMSP(createPacketMSP(MSP_EEPROM_WRITE, [])) 
