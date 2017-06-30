"""
This script is used to collect multiple datasets to test various settings of the sensor. Parameters can be changed through the config.ini file located in the same folder.
"""

# Imports

import sys
sys.path.insert(0, '../../../lib/')
import time
import numpy as np
import ConfigParser
from datetime import datetime
from mlx90621 import PIRArray
import RPi.GPIO as gp

# Variables
## initialize default params

calc_temps = True
start_hz = 32
end_hz = 128
duration = 180
delay_duration = 0.003
num_sets = 10
save_dir = "data/"
sensor1 = False
sensor2 = False

# load from config?
load_config = True

# load the config file
if load_config:
    cfg = ConfigParser.ConfigParser()
    cfg.read("config.ini")
    section1 = cfg.sections()[0]
    # load into variables
    sensor1 = bool(int(cfg.get(section1,"sensor1")))
    sensor2 = bool(int(cfg.get(section1,"sensor2")))

    calc_temps = bool(int(cfg.get(section1, "calc_temps")))
    start_hz = int(cfg.get(section1, "start_hz"))
    end_hz = int(cfg.get(section1, "end_hz"))
    duration = int(cfg.get(section1, "duration"))
    delay_duration = float(cfg.get(section1, "delay_duration"))
    num_sets = int(cfg.get(section1, "num_sets"))
    save_dir = cfg.get(section1, "save_dir")

# initialize variables and sensors

## list of frequencies, do not touch
freq_list = [1,2,4,8,16,32,64,128,256,512]

pir = PIRArray(400000)

#initializes status LED
gp.setmode(gp.BOARD)
gp.setup(32,gp.OUT)
gp.output(32, True)

#begin collection
for x in range(num_sets):
    for frequency in freq_list[freq_list.index(start_hz):freq_list.index(end_hz)+1]:
        # Both sensors
        if sensor1 and sensor2:
            pir.set_frequency(frequency)
            data = []
            wait_time = (1.0/frequency)-delay_duration
            start_time = time.time()
            timestamp = datetime.now()
            filename = save_dir+str(frequency)+"_"+str(duration)+"_"+str(x)+"_"+timestamp.strftime("%H%M%S")
            print filename
            while True:
                time1=time.time()
                print time1
                if calc_temps:
                    n = pir.read()
                else:
                    n = pir.read_raw()
                data.append([datetime.now(),n])
                while time.time()-time1<wait_time:
                    pass
                if time.time()-start_time>duration:
                    break
            p=np.array(data)
            np.save(filename, p)
        # sensor1
        elif sensor1:
            pir.set_frequency(frequency)
            data = []
            wait_time = (1.0/frequency)-delay_duration
            start_time = time.time()
            filename = save_dir+str(frequency)+"_"+str(duration)+"_"+str(x)+"_s1"
            print filename
            while True:
                time1=time.time()
                if calc_temps:
                    n = pir.pir1.calculate_4x16_np(pir.pir1.mlx_cshape(pir.pir1.mlx_ir_read()),pir.pir1.mlx_ptat(),pir.pir1.mlx_cp())
                else:
                    n = [pir.pir1.mlx_ir_read(),pir.pir1.mlx_ptat(),pir.pir1.mlx_cp()]
                data.append([datetime.now(),n])
                while time.time()-time1<wait_time:
                    pass
                if time.time()-start_time>duration:
                    break
            p=np.array(data)
            np.save(filename, p)
        #sensor2
        elif sensor2:
            pir.set_frequency(frequency)
            data = []
            wait_time = (1.0/frequency)-delay_duration
            start_time = time.time()
            timestamp = datetime.now()
            filename = save_dir+str(frequency)+"_"+str(duration)+"_"+str(x)+"_s2_"+timestamp.strftime("%H%M%S")
            print filename
            while True:
                time1=time.time()
                if calc_temps:
                    n = pir.pir2.calculate_4x16_np(pir.pir2.mlx_cshape(pir.pir2.mlx_ir_read()),pir.pir2.mlx_ptat(),pir.pir2.mlx_cp())
                else:
                    n = [pir.pir2.mlx_ir_read(),pir.pir2.mlx_ptat(),pir.pir2.mlx_cp()]
                data.append([datetime.now(),n])
                while time.time()-time1<wait_time:
                    pass
                if time.time()-start_time>duration:
                    break
            p=np.array(data)
            np.save(filename, p)

#make sure the light shuts down before quitting!
#in theory the pinnumber should be a variable, maybe in future revisions
gp.output(32, False)
