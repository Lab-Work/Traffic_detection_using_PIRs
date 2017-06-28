#/src

All libraries related to the sensor project will be located in this folder. This includes libraries for


##mlx90621.py

This file contains two classes for the PIR sensors.

###PIRSensor is a class meant for the operation of a single sensor. This contains low level code for i2c communication and single sensor initialization, collection, and calculation.

###PIRArray contains two instances of PIRSensor and is meant for managing the operation of two sensors together. This includes synchronizing the frequency, read order, and status.

##status_leds.py (WIP)

$status format: create instance of class with list of names. [power, status, collect]. status instance will be assigned in order of LEDs. Two functions - .on() and .off() can be used to control the lights$

This class is used for initializing and using the onboard status LEDs.

##mb7076.py

This class initializes and allows for the collection of ultrasonic data.

##em506.py

This classs initializes and sets up the GPS. Once a lock has been found, the timestamp can be synced with em506.set_system_time().

##ads1015.py

This is a simple helper class that allows the ultrasonic sensor to read data through the ADS1015 ADC.
