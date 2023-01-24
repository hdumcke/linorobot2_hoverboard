#!/usr/bin/python3

import sys
from hoverboard_controller import HoverboardInterface

if len(sys.argv) != 2:
    print("usage: set_speed <speep> <steer>")


cd = HoverboardInterface()
cd.set_speed(int(sys.argv[0]), int(sys.argv[1]))
