#  /src

All libraries related to the sensor project will be located in this folder. This includes libraries for:


## mlx90621.py

This file contains two classes for the PIR sensors.

### PIRSensor

is a class meant for the operation of a single sensor. This contains low level code for i2c communication and single sensor initialization, collection, and calculation.

### PIRArray

contains two instances of PIRSensor and is meant for managing the operation of two sensors together. This includes synchronizing the frequency, read order, and status.

## em506.py (WIP)

This class initializes and sets up the GPS. Once a lock has been found, the timestamp can be synced with em506.set_system_time().

## ads1015.py

This class provides the interface for the ADS1015 to be used to read ultrasonic data.

## sensor_classes.py (deprecated)

This class previously contained all sensor interface classes.
