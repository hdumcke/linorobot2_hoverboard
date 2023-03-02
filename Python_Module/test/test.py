#!/usr/bin/python3

from hoverboard_controller.hoverboard_controller import HoverboardInterface
import sys
import time

def clamp(num, min_value, max_value):
   return max(min(num, max_value), min_value)

class PID(object):
    def __init__(self):
        self.last_error = 0.0
        self.Kp = -1. # Proportial
        self.Ki = -1. # Integral
        self.Kd = -1. # Differential
        self.Iterm = 0. # Remember Integral of error
        self.output_guard = 1000.

    def updatePID(self, desired_val, measured_val, dt):
        #print("desired_val:"+str(desired_val)+" measured_val:"+str(measured_val)+" dt:"+str(dt))

        if (self.Kp == -1. or self.Ki == -1. or self.Kp == -1.):
            # PID values not initialized
            return 0.0

        error = desired_val - measured_val
        delta_error = (error - self.last_error) / dt
        self.Iterm += error * dt

        # Prevent I from getting too large
        if (self.Iterm < - self.output_guard):
            self.Iterm = - self.output_guard
        elif (self.Iterm > self.output_guard):
                self.Iterm = self.output_guard

        # Remember stuff for next calculation
        self.last_error = error;

        # Compute actual PWM output
        output = self.Kp * error + self.Ki * self.Iterm + self.Kd * delta_error
        print("updatePID Kp: %.2f Ki: %.2f Kd: %.2f desired: %.2f measured: %.2f output: %.2f error: %.2f delta_error: %.2f" % (self.Kp, self.Ki, self.Kd, desired_val, measured_val, output, error, delta_error))

        return clamp(output,-self.output_guard,self.output_guard) # it is better to clamp the ouput inside the PID

if len(sys.argv) != 1:
    print("usage: %s" % sys.argv[0])
    sys.exit(1)

ct = HoverboardInterface()
pid = PID()

while True:
    cb = ct.get_controlblock()
    pid.Kp = cb['pid_p'] / 100
    pid.Ki = cb['pid_i'] / 100
    pid.Kd = cb['pid_d'] / 100
    error = cb['set_speed_right'] * 100 - cb['speedMaster']
    pid.updatePID(cb['set_speed_right'], cb['speedMaster'] / 100, 0.01)
    #print("%s %s %s %.2f %.2f %.2f %s" % (cb['pid_p'], cb['pid_i'], cb['pid_d'], cb['set_speed_right'], cb['speedMaster'] / 100, error / 100, int(pid.updatePID(cb['set_speed_right'], cb['speedMaster'], 0.01))))
    time.sleep(1 / 100)  # 100 Hz
