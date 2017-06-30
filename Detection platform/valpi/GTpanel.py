## imports

import time
import picamera
import datetime
import numpy as np
import RPi.GPIO as GPIO
import os

#initialize constants

capture_time = 900
fps = 60
dat_array = []
count=0

#set up GPIO. It allows the LEDs and buttons to work properly

GPIO.setmode(GPIO.BOARD)
GPIO.setup(32,GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(33,GPIO.OUT)
GPIO.setup(36,GPIO.OUT)
GPIO.output(33,True)

def main():

    #place outer block in try/finally. This makes sure the file closes and the lights turn off when exited
    
    try:

        #loop allows us to collect as many sets as we need before shutting down
        
        while True:
            print "waiting for start"
            while True:
                time.sleep(0.1)
                input_state = GPIO.input(32)
                if input_state == False:
                    break
            print "making directory"
            dirs = "data/"+str(datetime.datetime.now())
            os.system("mkdir \"%s\"" % dirs)
            print "opening file"
            f = open("%s/rfdata.txt" % dirs,"w")
            print "starting recording"
            with picamera.PiCamera() as camera:
                camera.resolution = (640,480)
                camera.framerate = fps
                GPIO.output(36,True)
                camera.start_recording('%s/video.h264'%dirs,format='h264')
                count = 0
                while True:
                    #rd = rf.read()
                    #f.write(str(datetime.datetime.now())+"#"+str(rd)+"\n")
                    #print str(rd) + int(rd*2/100)*'#'
                    input_state = GPIO.input(32)
                    #count = count+1
                    #if count == 1000:
                    #    count = 0
                    #    f.flush()
                    #    os.fsync(f.fileno())
                    time.sleep(1)
                    if input_state == False:
                        break
                print "stopping recording"
                time.sleep(1)
                GPIO.output(36,False)
                camera.stop_recording()
                f.close()
    finally:
        GPIO.output(33,False)
        GPIO.output(36,False)
        GPIO.cleanup()

main()
