#!/usr/bin/python3

import sys
from hoverboard_controller.hoverboard_controller import HoverboardInterface

if len(sys.argv) != 2:
    print("usage: %s <buzzer>" % sys.argv[0])
    sys.exit(1)

ct = HoverboardInterface()
ct.set_buzzer(int(sys.argv[1]))
