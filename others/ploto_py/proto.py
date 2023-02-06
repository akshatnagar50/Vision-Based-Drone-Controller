import numpy

inputBuffer = numpy.zeros(1024)
bufferIndex= 0
#extern NSMutableArray* requests;
MSP_HEADER="$M<"
MSP_FC_VERSION=3;
MSP_RAW_IMU=102;
MSP_RC = 105;
MSP_ATTITUDE=108;
MSP_ALTITUDE=109;
MSP_ANALOG=110;
MSP_SET_RAW_RC=200;
MSP_ACC_CALIBRATION=205;
MSP_MAG_CALIBRATION=206;
MSP_SET_MOTOR=214;
MSP_SET_ACC_TRIM=239;
MSP_ACC_TRIM=240;
MSP_EEPROM_WRITE = 250;
MSP_SET_POS= 216;
MSP_SET_COMMAND = 217;


IDLE = 0
HEADER_START = 1
HEADER_M = 2
HEADER_ARROW = 3
HEADER_SIZE = 4
HEADER_CMD = 5
HEADER_ERR = 6;

roll = 0
pitch = 0
yaw = 0
battery = 0
rssi = 0

accX = 0
accY = 0
accZ = 0

gyroX = 0
gyroY = 0
gyroZ = 0

magX = 0
magY = 0
magZ = 0

alt = 0

FC_versionMajor = 0
FC_versionMinor = 0
FC_versionPatchLevel = 0

trim_roll = 0
trim_pitch = 0


rcThrottle = 1500
rcRoll = 1500
rcPitch = 1500
rcYaw = 1500
rcAUX1  = 1500
rcAUX2 = 1500
rcAUX3 = 1500
rcAUX4 = 1500

NONE_COMMAND = 0
TAKE_OFF = 1
LAND = 2 

class Protocol:
    def read8(self):
        x = bufferIndex
        bufferIndex += 1
        return inputBuffer[x] & 0xff
    
    def read16(self):
        x=bufferIndex
        bufferIndex+=2
        add_1=(inputBuffer[x] & 0xff) 
        add_2=((inputBuffer[x+1]) << 8) & 0xff
        return add_1+add_2
    
    def read32(self):
        x = bufferIndex;
        bufferIndex += 4
        add_1 = (inputBuffer[x] & 0xff)
        add_2 = ((inputBuffer[x+1]) << 8) & 0xff
        add_3 = ((inputBuffer[x+2]) << 16) & 0xff
        add_4 = ((inputBuffer[x+3]) << 24) & 0xff

        return add_1 + add_2 + add_3 + add_4

    def evaluateCommand(self,command):
        if command==MSP_FC_VERSION:
            FC_versionMajor=self.read8()
            FC_versionMinor=self.read8()
            FC_versionPatchLevel=self.read8()

        elif command==MSP_RAW_IMU:
            global accX=self.read16()
            global accY=self.read16()
            global accZ=self.read16()
            global gyroX=self.read16()/8
            global gyroY=self.read16()/8
            global gyroZ=self.read16()/8
            global magX=self.read16()/3
            global magY=self.read16()/3
            global magZ=self.read16()/3
        elif command==MSP_ATTITUDE:
            roll=self.read16()/10
            pitch=self.read16()/10
            yaw=self.read16()

        elif command==MSP_ALTITUDE:
             alt=(self.read32()/10)-0
        elif command==MSP_ANALOG:
            battery=(self.read8()/10.0)
            rssi=self.read16()
        elif command==MSP_ACC_TRIM:
            trim_pitch = self.read16()
            trim_roll = self.read16()

        elif command == MSP_RC :
            rcRoll = self.read16()
            rcPitch = self.read16()
            rcYaw = self.read16()
            rcThrottle = self.read16()
            rcAUX1 = self.read16()
            rcAUX2 = self.read16()
            rcAUX3 = self.read16()
            rcAUX4 = self.read16()

    # def sendMulReq -> rewquires com

    def createMSP(int msp, payload):
        bf=[]
        for s in MSP_HEADER:
            bf.append(int(s & 0xFF))
        checksum=0
        if payload==None:
            pl_size=0
        else:
            pl_size=len(payload)
        bf.append(pl_size)
        checksum ^=(pl_size & 0xFF)
        bf.append(int(msp & 0xFF))
        checksum ^= (msp & 0xFF)
        if payload:
            for p in payload:
                int k=p
                bf.append(int(k & 0xFF))
                checksum ^=(k & 0xFF)

        bf.append(checksum)
        return bf

    


