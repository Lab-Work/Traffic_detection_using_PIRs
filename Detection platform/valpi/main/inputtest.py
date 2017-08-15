from IOHandler import IOHandler
import collections
import time

y = collections.deque()
z = collections.deque()

x = IOHandler(y,z)
x.start()
time.sleep(1)
z.appendleft(["RDY","On"])
time.sleep(1)
z.appendleft(["RUN","On"])
# z.appendleft(["UTIL","On"])
# z.appendleft(["CON","On"])

time.sleep(4)
# time.sleep(1)
# z.appendleft(["RUN","On"])
# time.sleep(1)
# z.appendleft(["CON","On"])
# time.sleep(1)
# z.appendleft(["RDY","Off"])

z.appendleft(["RDY","BlinkSlow"])
z.appendleft(["RUN","BlinkSlow"])
# z.appendleft(["UTIL","BlinkSlow"])
# z.appendleft(["CON","BlinkSlow"])

# z.appendleft(["RUN","Off"])
# z.appendleft(["CON","Off"])
# z.appendleft(["UTIL","Off"])
time.sleep(5)
x.exit()
