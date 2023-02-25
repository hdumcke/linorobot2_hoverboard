#!/usr/bin/python3

from hoverboard_controller.hoverboard_controller import HoverboardInterface
import sys
import time

if len(sys.argv) != 1:
    print("usage: %s" % sys.argv[0])
    sys.exit(1)

ct = HoverboardInterface()

while True:
    print(ct.get_current())
    time.sleep(1 / 10)  # 10 Hz
