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
PACKET_LENGTH = PACKET_LENGTH_BYTES + PACKET_DATA_BYTES + PACKET_CRC_BYTES
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

        BYTES_TO_PAD = PACKET_DATA_BYTES - len(self.data)
        PADDING = [0xFF] * BYTES_TO_PAD
        
        ### Add padding to list ###
        self.data += PADDING

        if crc is None:
            data_buf = [self.length]
            self.crc = crc8(data_buf + self.data)
        else:
            self.crc = crc

    def toBuffer(self):
        buffer = [self.length] + self.data + [self.crc]
        return buffer
    
    def isSingleBytePacket(self, byte):
        if self.length != 1:
            return False
        
        if self.data[0] != byte:
            return False
        
        for byte in self.data[1:]:
            if byte != 0xff:
                return False
        
        return True

    def isAck(self):
        return self.isSingleBytePacket(PACKET_ACK_DATA)
    
    def isRetx(self):
        return self.isSingleBytePacket(PACKET_RETX_DATA)

    def computeCrc(self):
        length_data_bytes = [self.length] + self.data
        return crc8(length_data_bytes)

def consumeBufferData(buffer, bytes):
    consumed = buffer[0:bytes]

    # remove consumed bytes from buffer
    buffer = buffer[bytes: ]

    return consumed 

#buffer to store packets
packet_buffer = []

#buffer to store rx
rx_buffer = []

#retransmit and ack packets
retx = Packet(1, [PACKET_RETX_DATA, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF])
ack = Packet(1, [PACKET_ACK_DATA, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF])
last_packet = Packet(1, [0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF])

# Need to create function to recieve data and reconstruct packet
# Once packet is received, write packet back to MCU
class UARTProtocol(asyncio.Protocol):
    def __init__(self, rx_event: asyncio.Event):
        self.rx_event = rx_event

    def connection_made(self, transport):
        self.transport = transport
        print("port opened: ", transport)


    def write_packet(self, packet):
        print("Writing packet to STM")
        self.transport.write(bytes(packet.toBuffer()))

    # callback function when data is found on rx line (state machine)
    def data_received(self, data):
        print("RX: ", data.hex())

        #add the data to rx-buffer
        rx_buffer.extend(data)

        #check if a packet can be built
        if (len(rx_buffer) >= PACKET_LENGTH):
            packet_data = consumeBufferData(rx_buffer, PACKET_LENGTH)

            print("Comsumed Data: ", packet_data)

            #create packet to store in buffer
            temp_packet = Packet(packet_data[0], packet_data[1:PACKET_DATA_BYTES + 1], packet_data[PACKET_DATA_BYTES + 1])
            print("Packet Length: ", temp_packet.length)
            print("Packet Data: ", temp_packet.data)
            print("Packet CRC: ", temp_packet.crc)

            computedCrc = temp_packet.computeCrc()
            print("Computed CRC: ", computedCrc)

            #check if retransmission is required
            if(computedCrc != temp_packet.crc):
                print("Sending retx packet")
                self.write_packet(retx)

            elif(temp_packet.isRetx()):
                print("Sending last sent packet")
                self.write_packet(last_packet)

            elif(temp_packet.isAck()):
                print("Ack packet sent, do nothing")
            
            else:
                #store packet in buffer
                packet_buffer.append(temp_packet)
                print("Sending ack packet")
                self.write_packet(ack)
            
            self.rx_event.set() #signal RX occured
                

async def main():

    rx_event = asyncio.Event()
    loop = asyncio.get_running_loop()

    transport, protocol = await serial_asyncio.create_serial_connection(loop, lambda: UARTProtocol(rx_event), '/dev/ttyACM0', baudrate=115200)

    fw_packet = Packet(5, [0x1, 0x2, 0x3, 0x4, 0x5, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF])
    
    print("Sending packet")
    #data = [0x1]
    transport.write(bytes(fw_packet.toBuffer()))
    #await asyncio.sleep(0.5)
    # data_two = [0x2]
    # transport.write(bytearray(data_two))

    print("Waiting for ACK packet: ")
    await rx_event.wait()

    #transport.write(b"ACK")


asyncio.run(main())










