import socket
import errno
from struct import pack, unpack


class HoverboardInterface:
    """HoverboardInterface"""

    def __init__(self):
        self.connect()

    def connect(self):
        # Connect the socket to the port where the server is listening
        server_address = "/tmp/hoverboard-proxy.socket"
        self.sock = socket.socket(socket.AF_UNIX, socket.SOCK_SEQPACKET)
        try:
            self.sock.connect(server_address)
        except Exception as e:
            print("%s" % e)

    def close(self):
        try:
            self.sock.close()
        except Exception as e:
            print("%s" % e)

    def set_speed(self, speed, steer):
        try:
            self.sock.sendall(pack("BBhh", 6, 1, speed, steer))
            data = self.sock.recv(2)
        except Exception as e:
            if e.errno == errno.EPIPE or e.errno == errno.ENOTCONN or e.errno == errno.EBADF:
                self.close()
                self.connect()
            else:
                print("%s" % e)
            return

        if data != pack("BB", 2, 1):
            print("Invalid Ack")
            self.close()
            return

    def get_battery(self):
        try:
            self.sock.sendall(pack("BB", 2, 2))
            data = self.sock.recv(6)
        except Exception as e:
            if e.errno == errno.EPIPE or e.errno == errno.ENOTCONN or e.errno == errno.EBADF:
                self.close()
                self.connect()
            else:
                print("%s" % e)
            return None

        if data[0:2] != pack("BB", 6, 2):
            print("Invalid Ack")
            self.close()
            return None

        battery = list(unpack("<f", data[2:]))[0]
        return battery

    def get_current(self):
        try:
            self.sock.sendall(pack("BB", 2, 3))
            data = self.sock.recv(10)
        except Exception as e:
            if e.errno == errno.EPIPE or e.errno == errno.ENOTCONN or e.errno == errno.EBADF:
                self.close()
                self.connect()
            else:
                print("%s" % e)
            return None

        if data[0:2] != pack("BB", 10, 3):
            print("Invalid Ack")
            self.close()
            return None

        current = list(unpack("<2f", data[2:]))
        return current

    def get_speed(self):
        try:
            self.sock.sendall(pack("BB", 2, 4))
            data = self.sock.recv(10)
        except Exception as e:
            if e.errno == errno.EPIPE or e.errno == errno.ENOTCONN or e.errno == errno.EBADF:
                self.close()
                self.connect()
            else:
                print("%s" % e)
            return None

        if data[0:2] != pack("BB", 10, 4):
            print("Invalid Ack")
            self.close()
            return None

        speed = list(unpack("<2f", data[2:]))
        return speed
