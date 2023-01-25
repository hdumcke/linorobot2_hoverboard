import serial
from ctypes import *
import os
import sys
import pdb

speed = 100
steer = 0

so_file = os.path.dirname(os.path.realpath(sys.argv[0])) + "/comms.so"
comms = CDLL(so_file)

ser = serial.Serial('/dev/ttyAMA1', 19200)

while True:
    data = ser.readline()
    if data:
        pdb.set_trace()
        if data[0] == ord('/'):
            buff = '/'
            buff += chr(speed >> 8 & 0xFF)
            buff += chr(speed & 0xFF)
            buff += chr(steer >> 8 & 0xFF)
            buff += chr(steer & 0xFF)
            enc_array = []
            for i in range(len(buff)):
                enc_array.append(ord(buff[i]))
            enc_buff = bytes(enc_array)
            crc = comms.CalcCRC(enc_buff, len(buff)) & 0xFFFF
            buff += chr(crc >> 8 & 0xFF)
            buff += chr(crc & 0xFF)
            buff += "\n"
            enc_array = []
            for i in range(len(buff)):
                enc_array.append(ord(buff[i]))
            enc_buff = bytes(enc_array)
            pdb.set_trace()
            print(enc_array)
            ser.write(enc_buff)
