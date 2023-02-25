#!/usr/bin/python3

import sys
from hoverboard_controller.hoverboard_controller import HoverboardInterface

if len(sys.argv) != 4:
    print("usage: %s <pid_p> <pid_i> <pid_d>" % sys.argv[0])
    sys.exit(1)

ct = HoverboardInterface()
ct.set_pid(int(sys.argv[1]), int(sys.argv[2]), int(sys.argv[1]))
