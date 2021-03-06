import serial
from xbee import XBee
from datetime import datetime
import time
import os

serial_port = serial.Serial('/dev/tty.usbserial-DN018V89', 9600)

pilist = {"V1_1":"",
          "V1_2":"",
          "V2_1":"",
          "V2_2":"",
          "V3_1":"",
          "V3_2":"",
          "S1_0":"",
          "S2_0":"",
          "S3_0":""}

piaddrlist = {"V1_1":"\x12\x12",
              "V1_2":"\x00\x12",
              "V2_1":"\x00\x21",
              "V2_2":"\x00\x22",
              "V3_1":"\x00\x31",
              "V3_2":"\x00\x32",
              "S1_0":"\x00\x10",
              "S2_0":"\x00\x20",
              "S3_0":"\x00\x30"}

def reset():
    for key in pilist.keys():
        pilist[key]="no value"

def refresh():
    os.system('clear')
    for key,value in pilist.iteritems():
        print key + " : " + value

def print_data(data): 
    """
    This method is called whenever data is received
    from the associated XBee device. Its first and
    only argument is the data contained within the
    frame.
    """
    n = data['rf_data'].split("-")
    rssi = data['rssi']
    pilist[n[0]] = n[1] + " " + str(ord(rssi))
    refresh()


xbee = XBee(serial_port, callback=print_data)

# print "i'm here"

# raw_input()

# print "sending start signal"
# xbee.tx(dest_addr='\x00\x32',data='START')

# raw_input()

# print "sending stop signal"
# xbee.tx(dest_addr='\x00\x32',data='START')

# time.sleep(10)
reset()
refresh()

try:
    while True:
        x = raw_input()
        command = x.split(" ")
        if command[0] == "help":
            print "time set *"
            print "startstop *"
            print "status"
        elif command[0] == "startstop":
            if command[1] == "all":
                print "sending %s to %s"%(command[0],command[1])
                xbee.tx(dest_addr='\xFF\xFF',data='START')
            elif command[1] in piaddrlist:
                print "sending %s to %s"%(command[0],command[1])
                xbee.tx(dest_addr=piaddrlist[command[1]],data='START')
        elif command[0] == "status":
            reset()
            refresh()
            for key,value in piaddrlist.iteritems():
                print key
                xbee.tx(dest_addr=value,data='STAT')
                time.sleep(0.05)
            #xbee.tx(dest_addr='\xFF\xFF',data='STAT')
finally:
    xbee.halt()
    serial_port.close()
