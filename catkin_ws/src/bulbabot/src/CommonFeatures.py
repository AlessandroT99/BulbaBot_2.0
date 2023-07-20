#!/usr/bin/env python3

from time import sleep
from sys import stdout

#Connection variables and utilities ------------------------------------
ROS_COMMUNICATION_FREQUENCY = 1
NUMBER_OF_CONNECTION_TRIES = 4
CONNECTION_TIMEOUT = 5

#Node IDs for connection
MAIN_PROGRAM_ID = 2
CONTROL_LOOP_ID = 3

#Parser utilities ------------------------------------------------------
def waitingPoints(sentence, sleepTime = 0.5):
    print(sentence, end = '', flush = True)
    for _ in range(3):
        sleep(sleepTime)
        print(".", end = '', flush = True)
    print()
    