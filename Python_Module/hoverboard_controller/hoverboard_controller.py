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

    def set_speed(self, speed_l, speed_r):
        try:
            self.sock.sendall(pack("BBhh", 6, 1, speed_l, speed_r))
            data = self.sock.recv(2)
        except Exception as e:
            if (errno in e) and (e.errno == errno.EPIPE or e.errno == errno.ENOTCONN or e.errno == errno.EBADF):
                self.close()
                self.connect()
            else:
                print("%s" % e)
            return

        if data != pack("BB", 2, 1):
            print("Invalid Ack")
            self.close()
            return

    def set_pid(self, pid_p, pid_i, pid_d):
        try:
            self.sock.sendall(pack("BB3h", 8, 2, pid_p, pid_i, pid_d))
            data = self.sock.recv(2)
        except Exception as e:
            if (errno in e) and (e.errno == errno.EPIPE or e.errno == errno.ENOTCONN or e.errno == errno.EBADF):
                self.close()
                self.connect()
            else:
                print("%s" % e)
            return

        if data != pack("BB", 2, 2):
            print("Invalid Ack")
            self.close()
            return

    def set_led(self, led_l, led_r):
        try:
            self.sock.sendall(pack("BBhh", 6, 3, led_l, led_r))
            data = self.sock.recv(2)
        except Exception as e:
            if (errno in e) and (e.errno == errno.EPIPE or e.errno == errno.ENOTCONN or e.errno == errno.EBADF):
                self.close()
                self.connect()
            else:
                print("%s" % e)
            return

        if data != pack("BB", 2, 3):
            print("Invalid Ack")
            self.close()
            return

    def set_backled(self, backled_l, backled_r):
        try:
            self.sock.sendall(pack("BBhh", 6, 4, backled_l, backled_r))
            data = self.sock.recv(2)
        except Exception as e:
            if (errno in e) and (e.errno == errno.EPIPE or e.errno == errno.ENOTCONN or e.errno == errno.EBADF):
                self.close()
                self.connect()
            else:
                print("%s" % e)
            return

        if data != pack("BB", 2, 4):
            print("Invalid Ack")
            self.close()
            return

    def set_buzzer(self, buzzer):
        try:
            self.sock.sendall(pack("BBh", 4, 5, buzzer))
            data = self.sock.recv(2)
        except Exception as e:
            if (errno in e) and (e.errno == errno.EPIPE or e.errno == errno.ENOTCONN or e.errno == errno.EBADF):
                self.close()
                self.connect()
            else:
                print("%s" % e)
            return

        if data != pack("BB", 2, 5):
            print("Invalid Ack")
            self.close()
            return

    def get_enc(self):
        try:
            self.sock.sendall(pack("BB", 2, 6))
            data = self.sock.recv(18)
        except Exception as e:
            if (errno in e) and (e.errno == errno.EPIPE or e.errno == errno.ENOTCONN or e.errno == errno.EBADF):
                self.close()
                self.connect()
            else:
                print("%s" % e)
            return None

        if data[0:2] != pack("BB", 18, 6):
            print("Invalid Ack")
            self.close()
            return None

        enc = {}
        (enc['encM'],  enc['encS']) = unpack("<i4xi4x", data[2:])
        return [enc['encS'], enc['encM']]

    def get_battery(self):
        try:
            self.sock.sendall(pack("BB", 2, 7))
            data = self.sock.recv(4)
        except Exception as e:
            if (errno in e) and (e.errno == errno.EPIPE or e.errno == errno.ENOTCONN or e.errno == errno.EBADF):
                self.close()
                self.connect()
            else:
                print("%s" % e)
            return None

        if data[0:2] != pack("BB", 4, 7):
            print("Invalid Ack")
            self.close()
            return None

        battery = list(unpack("<h", data[2:]))[0]
        return battery / 100

    def get_current(self):
        try:
            self.sock.sendall(pack("BB", 2, 8))
            data = self.sock.recv(6)
        except Exception as e:
            if (errno in e) and (e.errno == errno.EPIPE or e.errno == errno.ENOTCONN or e.errno == errno.EBADF):
                self.close()
                self.connect()
            else:
                print("%s" % e)
            return None

        if data[0:2] != pack("BB", 6, 8):
            print("Invalid Ack")
            self.close()
            return None

        current = list(unpack("<2h", data[2:]))
        return current

    def get_speed(self):
        try:
            self.sock.sendall(pack("BB", 2, 9))
            data = self.sock.recv(6)
        except Exception as e:
            if (errno in e) and (e.errno == errno.EPIPE or e.errno == errno.ENOTCONN or e.errno == errno.EBADF):
                self.close()
                self.connect()
            else:
                print("%s" % e)
            return None

        if data[0:2] != pack("BB", 6, 9):
            print("Invalid Ack")
            self.close()
            return None

        speed = list(unpack("<2h", data[2:]))
        return speed

    def get_controlblock(self):
        try:
            self.sock.sendall(pack("BB", 2, 10))
            data = self.sock.recv(50)
        except Exception as e:
            if (errno in e) and (e.errno == errno.EPIPE or e.errno == errno.ENOTCONN or e.errno == errno.EBADF):
                self.close()
                self.connect()
            else:
                print("%s" % e)
            return None

        if data[0:2] != pack("BB", 50, 10):
            print("Invalid Ack")
            self.close()
            return None

        cb = {}
        (cb['set_speed_left'], cb['set_speed_right'], cb['pid_p'], cb['pid_i'], cb['pid_d'], cb['led_l'], cb['led_r'],
         cb['back_led_l'], cb['back_led_r'], cb['buzzer'], cb['responseId'], cb['encM'],  cb['encS'], cb['battery'],
         cb['currentMaster'],  cb['speedMaster'], cb['currentSlave'], cb['speedSlave']) = unpack("<10hHi4xi4x5h", data[2:])
        return cb
