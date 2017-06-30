"""
This class contains all of the classes to handle GPS operations, namely datetime synchronization and location retrieval.
"""

# Imports

import pynmea2
import serial
import RPi.GPIO as GPIO
import time

# Variables

use_Dir = False # The solder jumper must be soldered for this to work!

Dir_pin = 37 # Set this to the pin that DIR is connected to. For board V1 this is PIN37/BCM26
TX_pin = 8  # Set this to the RPi onboard TX pin
RX_pin = 10 # Set this to the RPi onboard RX pin

UART_0 = 18 # Set this to S0 for the UART multiplexer
UART_1 = 22 # Set this to S1 for the UART multiplexer

UART_num = 0 # Set this to the input multiplexer port

# Class Definition

class em506:
    def __init__(self):
        GPIO.setmode(GPIO.BOARD) # Initialize GPIO
        GPIO.setup(UART_0, GPIO.OUT) # S0
        GPIO.setup(UART_1, GPIO.OUT) # S1
        if use_Dir:
            GPIO.setup(Dir_pin, GPIO.IN)

    def location(self):
        """ Returns the location data from the GPS. (WIP) """
        self.switch_to_GPS()
        return

    def time(self):
        """ Returns the current timestamp to as accurate as possible """
        self.switch_to_GPS()
        return

    def set_system_time(self):
        """ Sets the current system time to GPS data after confirming there is a fix """
        self.switch_to_GPS()
        # if self.is_fix():
            # settime = Popen("sudo date -u", shell=True).wait()
            # return True
        return False


    def is_fix(self):
        """ Returns True if the GPS has a fix. Will not work if Dir pin is not connected! Requires at least one second to poll."""
        self.switch_to_GPS()
        fi = GPIO.input(Dir_pin) # Get current state
        time.sleep(1) # Wait one second
        return fi ^ GPIO.input(Dir_pin) # Return True if this switches

    def switch_to_GPS(self):
        """ Switches the GPS Multiplexer to the correct device. Run before working with GPS """
        GPIO.output(UART_0, False)
        GPIO.output(UART_1, False)
