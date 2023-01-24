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
            self.sock.sendall(pack("BBHH", 6, 1, *speed, *steer))
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
