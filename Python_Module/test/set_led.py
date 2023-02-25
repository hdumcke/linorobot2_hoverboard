#!/usr/bin/python3

import sys
from hoverboard_controller.hoverboard_controller import HoverboardInterface

if len(sys.argv) != 3:
    print("usage: %s <led_l> <led_r>" % sys.argv[0])
    sys.exit(1)

ct = HoverboardInterface()
ct.set_led(int(sys.argv[1]), int(sys.argv[2]))
