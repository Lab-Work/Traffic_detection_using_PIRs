"""
This is a script to live test the operation of the ultrasonic sensor
"""

# imports
import sys
sys.path.insert(1, '../../../lib/')
from ads1015 import ads1015
import RPi.GPIO as GPIO
import time

# initalize status LEDs
GPIO.setmode(GPIO.BOARD)
GPIO.setup(29,GPIO.OUT)
GPIO.setup(31,GPIO.OUT)
GPIO.setup(33, GPIO.OUT)
GPIO.output(29,True)
GPIO.output(31,False)

GPIO.output(33, True)

# create file and ult instance
ult = ads1015(400000)
f = open('ult_data','a')
f.write("\n")
f.close()

# main collection loop
try:
    while True:
        f = open('ult_data','a')
        time1=time.time()
        raw = ult.read(0,6144,300)
        time2=time.time()
        print time2-time1
        val = raw/1.442438
        # prints a truncated float and asterisks to visualize
        print "{0:.2f}".format(val), int(val*2)*'#'
        f.close()
except KeyboardInterrupt:
    GPIO.output(33,False)
