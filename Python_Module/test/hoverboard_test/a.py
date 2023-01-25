from ctypes import *
import os
import sys
import pdb

so_file = os.path.dirname(os.path.realpath(sys.argv[0])) + "/comms.so"
comms = CDLL(so_file)
speed = 300
steer = 0

buff = '/'
buff += chr(speed >> 8 & 0xFF)
buff += chr(speed & 0xFF)
buff += chr(steer >> 8 & 0xFF)
buff += chr(steer & 0xFF)
crc = comms.CalcCRC(buff.encode(), len(buff))
buff += chr(crc >> 8 & 0xFF)
buff += chr(crc & 0xFF)
buff += "\n"

enc_buff = buff.encode()
enc_buff = bytes(buff, 'UTF-8')

enc_array = []
for i in range(len(buff)):
    print("%s: %s %s" % (i, buff[i], ord(buff[i])))
    enc_array.append(ord(buff[i]))

enc_buff = bytes(enc_array)
for i in range(len(enc_buff)):
    print("%s: %s %s" % (i, enc_buff[i], hex(enc_buff[i])))
pdb.set_trace()
print(enc_buffer)
