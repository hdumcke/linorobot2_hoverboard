import serial
import time
import struct

def crc16(data : bytearray, offset , length):
    if data is None or offset < 0 or offset > len(data)- 1 and offset+length > len(data):
        return 0
    #crc = 0xFFFF
    crc = 0x0000
    for i in range(0, length):
        crc ^= data[offset + i] << 8
        for j in range(0,8):
            if (crc & 0x8000) > 0:
                crc =(crc << 1) ^ 0x1021
            else:
                crc = crc << 1
    return crc & 0xFFFF



ser = serial.Serial('COM4',19200)  # open serial port
print(ser.name) 
if ser.is_open:
	print("Open...\n")
while True:
	in_str = ser.readline()
	print(in_str.hex())
	packet_rx = bytearray()
	packet_rx.extend(in_str)
	#print(packet_rx)
	rx_crc=crc16(packet_rx,0,len(packet_rx)-3)
	#print( hex((rx_crc>> 8)  & 0xFF) )
	#print( hex(rx_crc & 0x00FF) )

	if(len(packet_rx)>23):
		right_current = struct.unpack('!f', packet_rx[5:9])[0]
		right_speed = struct.unpack('!f', packet_rx[9:13])[0]
		left_current = struct.unpack('!f', packet_rx[13:17])[0]
		left_speed = struct.unpack('!f', packet_rx[17:21])[0]
		print(str(round(right_current,3))+"A "+str(round(right_speed,3))+"m/s "+str(round(left_current,3))+"A "+str(round(left_speed,3))+"m/s")

	packet = bytearray()
	packet.append(0x2F)
	packet.append(0)
	packet.append(160)
	packet.append(0)
	packet.append(160)
	crc=crc16(packet,0,5)
	packet.append( (crc >> 8)  & 0xFF )
	packet.append(crc & 0xFF)
	packet.append(0x0A)

	ser.write(packet)
	ser.flushOutput()
	print(packet.hex())
	#tx_crc=crc16(packet,0,len(packet)-3)
	#print( hex((tx_crc>> 8)  & 0xFF) )
	#print( hex(tx_crc & 0x00FF) )

	time.sleep(0.01)


ser.close()