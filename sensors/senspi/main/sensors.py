"""
This file contains the multithreading classes and main function
to handle data collection, file operation, status operation, and calculations.
"""

# Imports
import os,sys
sys.path.insert(1, '../../lib/')
import time
import threading
import collections
from datetime import datetime
from CollectionHandler import CollectionHandler
from DataHandler import DataHandler
from IOHandler import IOHandler

# Variables
dat = collections.deque()

inpq = collections.deque()
outpq = collections.deque()

datafolder = "data"
frequency = 64

# Let's make sure the data directory exists
if not os.path.isdir("./data"):
    os.system("mkdir data")

# Threading Instances

collectionhandler = CollectionHandler(dat, frequency)
datahandler = DataHandler(dat, datafolder)
iohandler = IOHandler(inpq,outpq)

datahandler.start()
collectionhandler.start()
iohandler.start()

outpq.appendleft(["RDY","On"])

try:
    while True:
        time.sleep(0.5)
        if inpq:
            pep = inpq.pop()
            if pep == "START" and not collectionhandler.run_flag:
                collectionhandler.start_flag()
                datahandler.start_flag()
                outpq.appendleft(["RUN","On"])
            elif pep == "START" and collectionhandler.run_flag:
                collectionhandler.stop_flag()
                datahandler.stop_flag()
                outpq.appendleft(["RUN","Off"])
            else:
                pass
# try:
#     time1 = time.time()
#     collectionhandler.start_flag()
#     print "starting"
#     while time.time() - time1 < 300:
#         time.sleep(60)
#         print "ping"
#     collectionhandler.stop_flag()
#     print "stopping"

finally:
    collectionhandler.exit()
    datahandler.exit()
    iohandler.exit()
