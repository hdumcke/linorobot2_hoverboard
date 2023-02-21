#!/usr/bin/python3

from hoverboard_controller.hoverboard_controller import HoverboardInterface
import sys
import time

if len(sys.argv) != 1:
    print("usage: %s" % sys.argv[0])
    sys.exit(1)

ct = HoverboardInterface()

first_responseId = None
last_responseId = None
time_start = None
counter = None
count_until = 10

while True:
    cb = ct.get_controlblock()
    if last_responseId is None:
        last_responseId = cb['responseId']
        first_responseId = cb['responseId']
        time_start = time.time()
        counter = 1
        continue

    if last_responseId == cb['responseId']:
        #print("I am faster than firmware response")
        continue

    if last_responseId + 1 < cb['responseId']:
        print("I missed a firmware response")
        sys.exit(1)

    last_responseId = cb['responseId']
    counter += 1
    if counter == count_until:
        print("Reponses: %s\n" % (1 + last_responseId - first_responseId))
        print("Time: %.2f sec\n" % (time.time() - time_start))
        print("Frequency: %.2f Hz\n" % (count_until / (time.time() - time_start)))
        sys.exit(0)
