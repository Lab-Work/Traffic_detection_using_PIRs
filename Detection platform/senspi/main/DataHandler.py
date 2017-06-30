"""
This is the threading subclass that handles all data storage.
The instance will pop values from the queue and store them in a file.
"""

# Imports

import threading
import collections
import sys
sys.path.insert(1, '../../lib/')
import time
from datetime import datetime
import numpy as np

# Class Declaration

class DataHandler(threading.Thread):
    def __init__(self, queue, directory):
        threading.Thread.__init__(self)
        self.queue = queue
        self.run_flag = 1
        self.exit_flag = 0
        self.directory = directory
        self.fref = None

    def run(self):
        count = 0
        while True:
            if self.exit_flag:
                break
            elif not self.run_flag:
                if not self.fref.closed:
                    self.fref.close()
                time.sleep(0.02)
            else:
                if self.fref == None or self.fref.closed:
                    atime = datetime.now().strftime("%d%m-%H%M%S")
                    self.fname = "%s.txt" % atime
                    self.fref = file(self.directory + "/" + self.fname,"w")
                if count > 1000:
                    self.fref.flush()
                    os.fsync()
                else:
                    if self.queue:
                        line = self.queue.pop()
                        sen1 = ','.join(','.join('%f' %x for x in y) for y in line[1])
                        #sen2 = ','.join(','.join('%f' %x for x in y) for y in line[1][1])
                        written = str(line[0]) + "|" + sen1 + "|" + str(line[2]) + "|" + str(line[3]) + "|" + str(line[4]) + "\n"
                        self.fref.write(written)
                    else:
                        time.sleep(0.01)
                        pass
        print "Exiting DataHandler"

    def start_flag(self):
        self.run_flag = 1

    def stop_flag(self):
        self.run_flag = 0

    def exit(self):
        self.exit_flag = 1
