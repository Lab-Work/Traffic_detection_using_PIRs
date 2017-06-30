import time
import picamera
from LRF import LRF
import datetime
import numpy as np

#rf = LRF()
capture_time = 900
fps = 30
dat_array = []
count=0

with picamera.PiCamera() as camera:
    camera.rotation = 180
    camera.resolution = (640,480)
    camera.framerate = fps
    camera.start_recording('test2.h264')
    #time1=time.time()
    # while time.time()-time1<capture_time:
    #     rd = rf.read()
    #     dat_array.append([datetime.datetime.now(),rd])
    #     print str(rd) + int(rd*2/100)*'#'
    time.sleep(capture_time)
    camera.stop_recording()
    #np.save('dat_array', dat_array)

    # try:
    #     for i, filename in enumerate(camera.capture_continuous('data/image{counter}.jpg')):
    #         time1 = time.time()
    #         dat_array.append([datetime.datetime.now(),rf.read(),filename])
    #         print i
    #         while time.time()-time1<(1.0/fps):
    #             pass
    #         if i == fps*capture_time:
    #             break
    # finally:
    #     np.save('data', dat_array)
