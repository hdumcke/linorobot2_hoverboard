#!/usr/bin/python3

from hoverboard_controller.hoverboard_controller import HoverboardInterface
import time

ct = HoverboardInterface()
time_start = time.time()
speed = 100

ct.set_speed(0, 0)
Kp = 50
Ki = 50
Kd = 50
ct.set_pid(Kp, Ki, Kd)
time_point1 = 1
time_point2 = 5
time_point3 = 7

fd = open('/tmp/test-firmware_v2.log', 'w')
while True:
    delta_time = time.time() - time_start
    if delta_time >= time_point1 and delta_time < time_point2:
        ct.set_speed(speed, speed)
    if delta_time >= time_point2:
        ct.set_speed(0, 0)
    if delta_time >= time_point3:
        break
    cb = ct.get_controlblock()
    fd.write("%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\n" % (Kp, Ki, Kd,
                                                           cb['responseId'], 
                                                           cb['set_speed_left'], cb['set_speed_right'],
                                                           cb['encM'], cb['encS'],
                                                           cb['speedMaster'], cb['speedSlave']))
    time.sleep(1 / 10)

fd.close()
