##Sensor Collection and Validation

This repo contains all the files necessary to install and set up a Raspberry Pi from scratch for use with sensor data collection and camera data validation. The folder breakdown is as follows:

## Theory

The sensor platform is currently set up using two 60 degree MLX90621 thermal cameras and one MB7076 Maxbotic ultrasonic sensor.

## File/Folder Breakdown

### /coordinator

This folder contains scripts to allow XBee control from a base computer.

### /documentation

This folder contains important documentation for software theory, hardware design and setup, and sensor operation.  

### /src

This folder contains Mike M's BCM2835 libraries, and the PyBCM2835 module that needs installing. Run the install script to automatically install these two libraries and set up the library link.

### /lib

This folder contains the classes needed for sensor operation.

### /senspi

This folder contains scripts for data collection and testing for a sensor platform.

### /valpi

This folder contains scripts for data collection and testing for a validation platform.