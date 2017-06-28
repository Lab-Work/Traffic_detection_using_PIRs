import serial
from xbee import XBee
import RPi.GPIO as GPIO
import time

UART_0=18
UART_1=22


GPIO.setmode(GPIO.BOARD) # Initialize GPIO
GPIO.setup(UART_0, GPIO.OUT) # S0
GPIO.setup(UART_1, GPIO.OUT)
GPIO.output(UART_0, True)
GPIO.output(UART_1, False)

serial_port = serial.Serial('/dev/ttyS0', 9600)

def return_data(data):
    """
    This method is called whenever data is received
    from the associated XBee device. Its first and
    only argument is the data contained within the
    frame.
    """
    print data
    xbee.tx(dest_addr='\x00\x99', data=data['rf_data'])

xbee = XBee(serial_port, callback=return_data)

#for i in range(10):
#    xbee.tx(dest_addr='\x00\x99', data='%i'%i)

while True:
    try:
        time.sleep(0.001)
    except KeyboardInterrupt:
        break

xbee.halt()
serial_port.close()
    


