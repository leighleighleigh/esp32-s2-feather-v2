#!/usr/bin/python3
# Attempts to open a serial connecton on /dev/ttyUSB0
# And will print valid BM83 data packets (starting with hex 0xAA)

import sys
import serial
import io
from bm83_uart import BM83uart
import time

# Specify our driver inteface and give it the serial connection
driver = BM83uart()



def print_msg(msg):
    global msgsRecieved
    if(msg != None):
        msgsRecieved += 1
        print([hex(x) for x in msg])

### Create a list of message to send
# And send one off after the init messages (3) recieved
msgsRecieved = 0
msgsRecievedSendThresh = 0
cmds_to_send = []

### Turn on commands
print(driver.assemble_message(0x02,[0x00,0x53]))
cmds_to_send.append(driver.assemble_message(0x02,[0x00,0x53]))
print(driver.assemble_message(0x02,[0x00,0x54]))
cmds_to_send.append(driver.assemble_message(0x02,[0x00,0x54]))
cmds_to_send.append([3]) # Delay of 3 sec
print(driver.assemble_message(0x02,[0x00,0x51]))
cmds_to_send.append(driver.assemble_message(0x02,[0x00,0x51]))
print(driver.assemble_message(0x02,[0x00,0x52]))
cmds_to_send.append(driver.assemble_message(0x02,[0x00,0x52]))

### Link status
# cmds_to_send.append(driver.assemble_message(0x0D,[0x0]))

### Play music
# cmds_to_send.append([5])
print(driver.assemble_message(0x02,[0x00,0x32]))
# cmds_to_send.append(driver.assemble_message(0x02,[0x00,0x32]))

### Skip track
# cmds_to_send.append([3])
# cmds_to_send.append(driver.assemble_message(0x02,[0x00,0x34]))

### Proper play/pause toggle
print(driver.assemble_message(0x04,[0x00,0x7]))

### Read BTM Version
# cmds_to_send.append(driver.assemble_message(0x08,[0x0]))



# Open serial connection
ser = serial.Serial(
    port='/dev/ttyUSB0',\
    baudrate=115200,\
    parity=serial.PARITY_NONE,\
    stopbits=serial.STOPBITS_ONE,\
    bytesize=serial.EIGHTBITS,\
        timeout=0)

print("connected to: " + ser.portstr)




while True:
    for c in ser.read():
        # Print it
        # print(hex(c))
        # Immediately pass to the BM83uart
        driver.add_data(c)
        
        # Parse messages
        result = driver.parse_data()
        print_msg(result)

    # Send a message
    if(msgsRecieved >= msgsRecievedSendThresh and len(cmds_to_send) != 0):
        # Pop next message of queue
        msgToSend = cmds_to_send.pop(0)
        
        # If len == 1, delay for X seconds
        if(len(msgToSend) == 1):
            time.sleep(msgToSend[0])
        else:
            # Send it
            print("TX: {}".format(str([hex(x) for x in msgToSend])))
            ser.write(bytes(msgToSend))    
            # Increase thresh
            msgsRecievedSendThresh+=1

ser.close()
