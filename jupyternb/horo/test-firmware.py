#!/usr/bin/python3

from hoverboard_controller.hoverboard_controller import HoverboardInterface
import time

ct = HoverboardInterface()
# range from 0 to 1000 step 100
step = 100
start = 0
# range from 50 to 160 step 10
#step = 10
#start = 50


def get_feedback(speed_r, speed_l):
    battery = ct.get_battery()
    current = ct.get_current()
    speed = ct.get_speed()
    print("%i\t%i\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f" % (speed_r,
                                                    speed_l,
                                                    battery,
                                                    current[0],
                                                    current[1],
                                                    speed[0],
                                                    speed[1]))


def run_a_while(speed_r, speed_l):
    # let the speed stabilize
    time.sleep(1)
    for i in range(60 * 20):
        get_feedback(speed_r, speed_l)
        time.sleep(1 / 20)  # 20 Hz


for i in range(11):
    speed_r = i * step + start
    speed_l = i * step + start
    ct.set_speed(speed_r, speed_l)
    run_a_while(speed_r, speed_l)

for i in range(11):
    speed_r = (10 - i) * step + start
    speed_l = (10 - i) * step + start
    ct.set_speed(speed_r, speed_l)
    run_a_while(speed_r, speed_l)

ct.set_speed(0, 0)
