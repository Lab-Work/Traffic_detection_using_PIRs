"""
This script is built for the collection of raw data from the 3x pir sensors. 16 hz collection is ideal
"""

# imports
import time
import numpy as np
from datetime import datetime
from sensor_classes import *
import RPi.GPIO as GPIO

# LED pinout initialize
GPIO.setmode(GPIO.BOARD)
GPIO.setup(29,GPIO.OUT)
GPIO.setup(31,GPIO.OUT)
GPIO.output(29,True)
GPIO.output(31,False)

savename = "sdtest1"

FREQ = 64
SECTIME = 1800

pir = PIR_Array(400000)
ult = ADC(400000)
dat_array=[]
pir.set_frequency(FREQ)
i=0
scale = 4.33/0.984

data = file("60fov128hzt1.txt","a")

# most data collection scripts are the same:
# set a loop that stops after a time counter,
# wait until 1/framerate has passed,
# run the loop again, and break if the time is up

try:
    while True:
        time1=time.time()
        #np.savetxt(data,[pir.read_raw(),ult.read(sps=3300)*scale])
        #dat_array.append([datetime.now(),pir.read_raw(),ult.read(sps=3300)*scale])
        X = pir.read()
        dat_array.append([datetime.now(),X])
        print X
        while time.time()-time1<(1.0/FREQ):
            pass
        i=i+1
        if i==SECTIME*FREQ:
            break

finally:
    x = np.array(dat_array)
    np.save(savename, x)
