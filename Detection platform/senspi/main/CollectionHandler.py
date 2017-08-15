"""
This is the threading subclass that handles all data collection.
The instance will update the passed variable with data collected
from the sensors from this class
"""

# Imports

import threading
import collections
import sys
sys.path.insert(1, '../../lib/')
import time
from datetime import datetime
import numpy as np
from mlx90621 import PIRArray
from ads1015 import ads1015

# Class Declaration

class CollectionHandler(threading.Thread):
    def __init__(self, queue, frequency):
        threading.Thread.__init__(self)
        self.queue = queue
        self.pir = PIRArray(400000)
        self.ult = ads1015(400000)
        self.frequency = frequency
        self.pir.set_frequency(frequency)
        self.run_flag = 0
        self.exit_flag = 0

    def run(self):
        while True:
            if self.exit_flag:
                break
            elif not self.run_flag:
                time.sleep(0.02)
            else:
                time1=time.time()
                now = datetime.now()
                pirdata = self.pir.read()
                ultdata = self.ult.read(sps=3300)*4.33/0.984
                self.queue.appendleft([now,pirdata,ultdata, self.pir.pir1.Tambient, self.pir.pir2.Tambient])
                # print str(self.pir.pir1.Tambient) + "\t" + str(self.pir.pir2.Tambient) + "\t" +  str(ultdata) + "\t" + int(ultdata*10)*"*"
                while time.time()-time1 < 1.0/self.frequency:
                    pass
        print "Exiting CollectionHandler"

    def start_flag(self):
        self.run_flag = 1

    def stop_flag(self):
        self.run_flag = 0

    def exit(self):
        self.exit_flag = 1
