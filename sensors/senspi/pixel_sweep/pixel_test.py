"""
This script samples individual pixels - specified in the config variables, and saves it to a numpy array. The purpose is to understand the RAM refresh of the sensor.
"""

#Variables

pixels = [[0,0],[3,15]] # list of pixels
sensor_rate = 128 # refresh rate of the sensor (1,2,4,8,16,32,64,128,256,512)
read_rate = 0 # sampling rate from the RAM (0 for no delay)
file_name = "sweep_data" #filename to save as
duration = 30 #sample duration. press C-c at any time to save current data and exit

#Imports

import sys
sys.path.insert(0, '../../lib/')
import time
import os #os.path.isfile()
import numpy as np
from mlx90621 import PIRSensor
import datetime



#Script
def main():
    # handle filenames
    if os.path.isfile(file_name):
        flag = False
        while not flag:
            r = raw_input("$s: Filename exists. Overwrite file? (y/n): " , file_name)
            if r == 'y':
                break
            elif r == 'n':
                return
            else:
                continue
            
    # initialize sensor and array
    pir = PIRSensor(1,400000)
    pir.mlx_set_frequency(sensor_rate)
    data = []
    
    # begin collection
    try:
        stime = time.time()
        if read_rate == 0:            
            while True:
                read_vals = []
                for pixel in pixels:
                    read_vals.append(pir.mlx_ir_read_pixel(pixel[0],pixel[1]))
                data.append([
                    datetime.datetime.now(),read_vals])
                if time.time()-stime > duration:
                    break                
        else:
            while True:
                time1=time.time()
                read_vals = []
                for pixel in pixels:
                    read_vals.append(pir.mlx_ir_read_pixel(pixel[0],pixel[1]))
                data.append(datetime.datetime.now(),read_vals)            
                while time.time()-time1<(1.0/FREQ):
                    pass                
                if time.time()-stime > duration:
                    break
    finally:
        x = np.array(data)
        np.save(file_name, x)

main()
