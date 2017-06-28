"""
This thread handles input and status led indication, and wireless communication
"""


# Imports

import threading
import collections
import time
from datetime import datetime
import numpy as np
import RPi.GPIO as GPIO
import serial
from xbee import XBee

# Varible Declaration

ledDict = {1:33,2:36,3:35,4:38}
lednameDict = {"RDY":1,"RUN":2,"CON":3,"UTIL":4}
inDict = {32:"INIT",40:"START"}
#innameDict = {1:"INIT",2:"START"}

slowHz = 2
fastHz = 5



# Class Declaration

class IOHandler(threading.Thread):
    def __init__(self, inpq, outpq):
        threading.Thread.__init__(self)
        # Common variables
        self.inpq = inpq
        self.outpq = outpq
        self.run_flag = 1
        self.exit_flag = 0

        # We need a timer to keep track of led blinking
        self.blinkFastTimer = time.time()
        self.blinkSlowTimer = time.time()
        # This timer will switch this boolean
        self.blinkFastState = False
        self.blinkSlowState = False
        # We need registers to debounce button presses
        self.prevIn = {}

        #status dictionaries
        self.statDict = {}

        # Status init
        for key,value in ledDict.iteritems():
            # Off On BlinkFast BlinkSlow
            self.statDict[value] = "Off"

        # BCM init
        GPIO.setmode(GPIO.BOARD)
        for key, value in ledDict.iteritems():
            GPIO.setup(value, GPIO.OUT)
            GPIO.output(value,False)
        for key, value in inDict.iteritems():
            GPIO.setup(key,GPIO.IN, pull_up_down=GPIO.PUD_UP)
            self.prevIn[key] = True

        # Wireless init:

        UART_0=18
        UART_1=22

        GPIO.setmode(GPIO.BOARD) # Initialize GPIO
        GPIO.setup(UART_0, GPIO.OUT) # S0
        GPIO.setup(UART_1, GPIO.OUT)
        GPIO.output(UART_0, True)
        GPIO.output(UART_1, False)

        self.serial_port = serial.Serial('/dev/ttyS0', 9600)

        self.xbee = XBee(self.serial_port)
        self.xbee.send('at', frame_id='A', command='NI')
        x = self.xbee.wait_read_frame()
        print x
        self.name = x['parameter']
        self.xbee = XBee(self.serial_port, callback=self.receive_data)

    def run(self):
        while True:
            if self.exit_flag:
                GPIO.cleanup()
                self.xbee.halt()
                self.serial_port.close()
                break
            elif not self.run_flag:
                pass
            else:
                time.sleep(0.01)
                # Check for button input
                for key,value in inDict.iteritems():
                    inst = GPIO.input(key)
                    if not inst and self.prevIn[key]:
                        self.inpq.appendleft(value)
                        self.prevIn[key]=False
                    elif inst:
                        self.prevIn[key]=True

                # Check for new status
                if self.outpq:
                    nm, stat = self.outpq.pop()
                    print nm + " " + stat
                    self.statDict[ledDict[lednameDict[nm]]] = stat
                    self.status_tx()

                # Update timer and blink states
                cTime = time.time()
                if cTime-self.blinkFastTimer>1.0/fastHz:
                    self.blinkFastState = not self.blinkFastState
                    self.blinkFastTimer = cTime
                if cTime-self.blinkSlowTimer>1.0/slowHz:
                    self.blinkSlowState = not self.blinkSlowState
                    self.blinkSlowTimer = cTime

                # Update GPIO loop
                for pin in ledDict.values():
                    if self.statDict[pin] == "BlinkFast":
                        GPIO.output(pin,self.blinkFastState)
                    elif self.statDict[pin] == "BlinkSlow":
                        GPIO.output(pin,self.blinkSlowState)
                    elif self.statDict[pin] == "On":
                        GPIO.output(pin,True)
                    elif self.statDict[pin] == "Off":
                        GPIO.output(pin,False)

        print "Exiting InputHandler"

    def receive_data(self,data):
        """
        This is the callback function that handles all wireless inputs as they
        are received.
        """
        print data['rf_data']
        #{'rf_data': 'START', 'rssi': '-', 'source_addr': '\x00\x99', 'id': 'rx', 'options': '\x00'}
        # run if a command is sent
        if data['rf_data'] == "START" or data['rf_data'] == "INIT" or data['rf_data'] == "EXIT":
            self.inpq.appendleft(data['rf_data'])
            #self.xbee.tx(dest_addr='\x00\x99', data='%s: %s'%(self.name,data['rf_data']))
        elif data['rf_data'] == "STAT":
            self.status_tx()
            #self.xbee.tx(dest_addr='\x00\x99', data='%s: %s'%(self.name,data['rf_data']))
        else:
            return

    def status_tx(self):
        """
        This function returns the current LED status
        """
        #self.statDict = {}
        strlist = []
        for key,value in lednameDict.iteritems():
            strlist.append(key)
            strlist.append(self.statDict[ledDict[value]])
        dat = self.name + "-" + ' '.join([str(x) for x in strlist])
        self.xbee.tx(dest_addr='\x00\x99', data=dat)

    # These three function definitions are obsolete, this thread should be
    # continuously running
    def start_flag(self):
        self.run_flag = 1

    def stop_flag(self):
        self.run_flag = 0

    def exit(self):
        self.exit_flag = 1
