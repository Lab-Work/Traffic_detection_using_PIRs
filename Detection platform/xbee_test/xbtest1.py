import serial
import time
import RPi.GPIO as GPIO
from xbee import XBee

UART_0=18
UART_1=22


GPIO.setmode(GPIO.BOARD) # Initialize GPIO
GPIO.setup(UART_0, GPIO.OUT) # S0
GPIO.setup(UART_1, GPIO.OUT)
GPIO.output(UART_0, True)
GPIO.output(UART_1, False)

serial_port = serial.Serial('/dev/ttyS0', 9600)

def print_data(data):
    """
    This method is called whenever data is received
    from the associated XBee device. Its first and
    only argument is the data contained within the
    frame.
    """
    print data

xbee = XBee(serial_port, callback=print_data)

while True:
    try:
        time.sleep(0.20)
        xbee.tx(dest_addr='\x00\x99', data='test')
    except KeyboardInterrupt:
        break
    
xbee.halt()
serial_port.close()
GPIO.cleanup()
