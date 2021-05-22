#!/usr/bin/python3

class BM83uart():
    def __init__(self):
        self.input_buffer = []
        self.start_byte = 0xAA
        
    def add_data(self,data):
        """
        Add data to the input buffer
        """
        self.input_buffer.append(data)

    def calc_checksum(self,bytes):
        clampedsum = sum(bytes) & 0b11111111
        return 1 + (0xFF-clampedsum)

    def assemble_message(self,opcode,parameters):
        """
        Assembles a message together and returns it with start byte and checksum.
        """
        dataList = [0xAA]
        dataLen = 1 + len(parameters)
        dataLenH = dataLen & (0xFF << 8)
        dataLenL = dataLen & 0xFF
        dataList.append(dataLenH)
        dataList.append(dataLenL)
        dataList.append(opcode)
        dataList.extend(parameters)
        dataList.append(self.calc_checksum(dataList[1:]))
        return dataList

    def parse_data(self):
        """
        Go through the input_buffer and find 0xAA.
        Then attempt to decode it!

        Many different exit conditions here.
        Any found messages are returned (and respective data removed off stack).
        0xAA,DLC_MSB,DLC,LSB,CMD,PARAM,PARAM-N,CHECKSUM
        """
        dataByteIndex = 0
        startIndex = -1

        # Find start byte
        for dataByte in self.input_buffer:
            if(dataByte == 0xAA):
                startIndex = dataByteIndex
                break

            dataByteIndex += 1

        # Check if start byte found?
        if(startIndex != -1):
            # Extract data after this start byte
            messageData = self.input_buffer[startIndex:65535]

            # Check for a 2 byte length code
            if(len(messageData) > 3):
                # Extract DLC
                DLC_upper = messageData[1]
                DLC_lower = messageData[2]
                
                # Extract total data length
                DLC = (DLC_upper << 8) + DLC_lower
                
                # Check if message data is longer than DLC+1 (+1 for checksum)
                if(len(messageData) >= DLC+4):
                    # Lets extract all the other bytes
                    OPCODE = messageData[3]
                    PARAMS = messageData[4:4+(DLC-1)]
                    CHECKSUM = messageData[DLC+3]

                    # print("MSG LEN: {}".format(DLC))
                    # print("OPCODE: {}".format(hex(OPCODE)))
                    # print("PARAMS: {}".format(str([hex(x) for x in PARAMS])))
                    # print("CHK: {}".format(hex(CHECKSUM)))
                    checksumDataInput = [DLC_upper,DLC_lower,OPCODE]
                    checksumDataInput.extend(PARAMS)
                    chkCalc = self.calc_checksum(checksumDataInput)
                    # print("CHK clc: {}".format(hex()))
                    
                    # Return if checksum is good
                    messageData = [OPCODE]
                    messageData.extend(PARAMS)

                    # Remove this data from input buffer
                    self.input_buffer = self.input_buffer[startIndex+(DLC+4):]

                    if(CHECKSUM == chkCalc):
                        return messageData
                    else:
                        return None

        return None

        
    