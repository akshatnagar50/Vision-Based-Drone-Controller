from plutoMultiwii import *

def twos_complement(binary_string): 
    result = ""
    # Invert all bits in the binary string
    for bit in binary_string: 
        if bit == '0': 
            result += '1'
        else: 
            result += '0'
      
  # Add 1 to the result 
    carry = True
    for i in range(len(result)-1, -1, -1): 
        if result[i] == '1' and carry: 
            result = result[:i] + '0' + result[i+1:]
            carry = True
        elif result[i] == '0' and carry: 
            result = result[:i] + '1' + result[i+1:]
            carry = False
  
    return result

def twos_complement_inverse(binary_string):
    if binary_string[0]=='0':
        return int(binary_string,2)
    elif binary_string[0]=='1':
        return -int(twos_complement(binary_string),2)
    else:
        print('non binary bit received: ',binary_string[0])

def get_RPY(segment):
    #binary representation of the little endian hexadecimal 2 byte value
    roll_str = (str(bin(segment[6]))[2:].zfill(8)+str(bin(segment[5])[2:].zfill(8)))
    roll = twos_complement_inverse(roll_str)/10

    pitch_str = str(bin(segment[8]))[2:].zfill(8) + str(bin(segment[7]))[2:].zfill(8)
    pitch = twos_complement_inverse(pitch_str)/10

    yaw = segment[9] + 256*segment[10]

    return roll,pitch,yaw

file = open('received_bytes_armed.txt','wb')

while True:
    data = client.recv(1024)
    file.write((data))
    
    segments = data.split(b'$M>')
    for segment in segments:
        segment = b'$M>' + segment

        if len(segment)==12: #checking if the length of the packet is 12
            if segment[4]==108: #checking if the type of the packet is MSP_ATTITUDE
                roll,pitch,yaw = get_RPY(segment)



file.close()