"""
This thread handles all video recording and processing
"""
import time
import picamera
from datetime import datetime
from PtsOutput import PtsOutput
import numpy as np
import RPi.GPIO as GPIO
import os
import threading
import collections

class ValidationHandler(threading.Thread):
    def __init__(self, outpq, directory):
        threading.Thread.__init__(self)
        self.outpq = outpq
        self.run_flag = 0
        self.exit_flag = 0
        self.directory = directory
        self.status = 0

    def run(self):
        while True:
            if self.exit_flag:
                break
            elif not self.run_flag:
                time.sleep(0.1)
            else:
                time1 = datetime.now().strftime("%d%m-%H%M%S")
                fname = self.directory + "/" + time1
                print "Creating camera and recording"
                camera = picamera.PiCamera()
                print "created"
                camera.rotation = 180
                camera.resolution = (640,480)
                camera.framerate = 60
                camera.start_recording(PtsOutput(camera, '%s.h264'%fname, '%s.txt'%fname),format='h264')
                print '%s.h264'%fname
                while True:
                    camera.wait_recording(1)
                    if not self.run_flag:
                        break
                print "Stop recording"
                camera.stop_recording()
                camera.close()
        print "Exiting ValidationHandler"

    def start_rec(self):
        print "start"
        self.outpq.appendleft(["RUN","On"])
        self.run_flag = 1
        self.status = 1

    def stop_rec(self):
        print "stop"
        self.outpq.appendleft(["RUN","Off"])
        self.run_flag = 0
        self.status = 0

    def exit(self):
        self.exit_flag = 1
