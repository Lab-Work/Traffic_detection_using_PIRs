"""
This class contains all functions for operating the status LEDs on the board.
"""

# Imports

import RPi.GPIO as GPIO
import time

# Variables

pinDict = {1:33,2:36,3:35,4:38}
statDict = {}

# Class Declaration

class status_leds:
    def __init__(self):
        # set initial state to Off
        for key in pinDict:
            statDict[key] = "Off"

        # initialize GPIO
        GPIO.setmode(GPIO.BOARD)
        for key, value in pinDict.iteritems():
        GPIO.setup(13, GPIO.OUT)
