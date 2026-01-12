##### TODO ######
### 1. Serial Port setup
### 2. Set up packet class 
### 3. set up functions to tx and rx packets and store in buffer

import asyncio
import serial_asyncio

# constants #
PACKET_LENGTH_BYTES = 1
PACKET_DATA_BYTES = 16
PACKET_CRC_BYTES  = 1
PACKET_ACK_DATA   = 0x15
PACKET_RETX_DATA  = 0x19

def crc8(data):
    crc = 0

    for byte in data:
        crc = (crc ^ byte) & 0xff

        for i in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ 0x07) & 0xff
            else:
                crc = (crc << 1) & 0xff
    
    return crc


#### PACKET CLASS ####
class Packet:
    def __init__(self, length, data, crc=None):
        self.length = length
        self.data = data

        BYTES_TO_PAD = PACKET_DATA_BYTES - self.length
        PADDING = [0xFF] * BYTES_TO_PAD
        
        ### Add padding to list ###
        self.data += PADDING

        if crc is None:
            self.crc = crc8(self.length + self.data)
        else:
            self.crc = crc

    def toBuffer(self):
        buffer = self.length + self.data + self.crc
        return buffer
    
    def isSingleBytePacket(self, byte):
        if this.length != 1:
            return False
        
        if this.data[0] != byte:
            return False
        
        for byte in this.data[1:]:
            if byte != 0xff:
                return False
        
        return True

    def isAck(self):
        return isSingleBytePacket(PACKET_ACK_DATA)
    
    def isRetx(self):
        return isSingleBytePacket(PACKET_RETX_DATA)

#buffer to store packets
packet_buffer = []

#buffer to store rx
rx_buffer = []

# Need to create function to recieve data and reconstruct packet
# Once packet is received, write packet back to MCU
class UARTProtocol(asyncio.Protocol):
    def __init__(self, rx_event: asyncio.Event):
        self.rx_event = rx_event

    def connection_made(self, transport):
        self.transport = transport
        print("port opened: ", transport)

    # callback function when data is found on rx line (state machine)
    def data_received(self, data):
        print("RX: ", data)
        self.rx_event.set() #signal RX occured

        # #add the data to rxbuffer
        # rx_buffer.append(data)

        # #check if a packet can be built
        # if(rx_buffer.length >= RX_)




async def main():

    rx_event = asyncio.Event()
    loop = asyncio.get_running_loop()

    transport, protocol = await serial_asyncio.create_serial_connection(loop, lambda: UARTProtocol(rx_event), '/dev/ttyACM0', baudrate=115200)

    print("Waiting for packet: ")
    await rx_event.wait()

    transport.write(b"ACK")


asyncio.run(main())










