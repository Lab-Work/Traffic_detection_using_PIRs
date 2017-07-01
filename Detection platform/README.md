#Sensor Collection and Validation

This repo contains all the files necessary to install and set up a Raspberry Pi from scratch for use with sensor data collection and camera data validation. The folder breakdown is as follows:

## Theory

The sensor platform is currently set up using two 60 degree MLX90621 thermal cameras and one MB7076 Maxbotic ultrasonic sensor.

## File/Folder Breakdown
### install.sh (WIP)

This script allows for the setup of a Raspberry Pi. To begin the installation, run

```
./install.sh sensor
```

to install this Pi as a sensor cone, and

```
./install.sh validation
```

as a camera validation cone.

### /src

This folder contains Mike M's BCM2835 libraries, and the PyBCM2835 module that needs installing. Run the install script to automatically install these two libraries and set up the library link.

### /lib (WIP)

This folder contains the classes needed for sensor operation.

### /senspi (WIP)

This folder contains scripts for data collection and testing for a sensor platform.

### /valpi (WIP)

This folder contains scripts for data collection and testing for a validation platform.

### /dsens (WIP)

This folder contains the data that is written from a sensor platform.

### /dval (WIP)

This folder contains the data that is written from a validation platform.
