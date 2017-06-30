"""
This script handles the video collection. Similar to the senspi script, multiple
threads are used to handle collection and button input.
"""

# Imports

import collections
import time
from ValidationHandler import ValidationHandler
from IOHandler import IOHandler

# Global queues

outpq = collections.deque()
inpq = collections.deque()

# Global variables

directory = "data"

# Handler declarations

iohandler = IOHandler(inpq, outpq)
vhandler = ValidationHandler(outpq, directory)

iohandler.start()
vhandler.start()

outpq.appendleft(["RDY","On"])

try:
    while True:
        time.sleep(0.5)
        if inpq:
            pep = inpq.pop()
            print pep
            print vhandler.status
            if pep == "EXIT":
                break
            elif pep == "START" and not vhandler.status:
                vhandler.start_rec()
            elif pep == "START" and vhandler.status:
                vhandler.stop_rec()
            else:
                pass

finally:
    vhandler.exit()
    iohandler.exit()
