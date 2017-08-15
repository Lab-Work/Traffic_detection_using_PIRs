#!/usr/bin/env python

# ==============================================================================
#
# Import statements
#
# ==============================================================================

import numpy as np
import serial
import time
import PyBCM2835 as bcm
import math
import RPi.GPIO as GPIO
import smbus
import re
from math import sqrt
from datetime import datetime

# ==============================================================================
#
# ADS1015 constants
#
# ==============================================================================
# These values are adapted from Adafruit's included code for ADS1x15.
# ==============================================================================

# Pointer Register
ADS1015_REG_POINTER_MASK = 0x03
ADS1015_REG_POINTER_CONVERT = 0x00
ADS1015_REG_POINTER_CONFIG = 0x01
ADS1015_REG_POINTER_LOWTHRESH = 0x02
ADS1015_REG_POINTER_HITHRESH = 0x03

# Config Register
ADS1015_REG_CONFIG_OS_MASK = 0x8000
ADS1015_REG_CONFIG_OS_SINGLE = 0x8000  # Write: Set to start a single-conversion
ADS1015_REG_CONFIG_OS_BUSY = 0x0000  # Read: Bit = 0 when conversion is in progress
ADS1015_REG_CONFIG_OS_NOTBUSY = 0x8000  # Read: Bit = 1 when device is not performing a conversion

ADS1015_REG_CONFIG_MUX_MASK = 0x7000
ADS1015_REG_CONFIG_MUX_DIFF_0_1 = 0x0000  # Differential P = AIN0, N = AIN1 (default)
ADS1015_REG_CONFIG_MUX_DIFF_0_3 = 0x1000  # Differential P = AIN0, N = AIN3
ADS1015_REG_CONFIG_MUX_DIFF_1_3 = 0x2000  # Differential P = AIN1, N = AIN3
ADS1015_REG_CONFIG_MUX_DIFF_2_3 = 0x3000  # Differential P = AIN2, N = AIN3
ADS1015_REG_CONFIG_MUX_SINGLE_0 = 0x4000  # Single-ended AIN0
ADS1015_REG_CONFIG_MUX_SINGLE_1 = 0x5000  # Single-ended AIN1
ADS1015_REG_CONFIG_MUX_SINGLE_2 = 0x6000  # Single-ended AIN2
ADS1015_REG_CONFIG_MUX_SINGLE_3 = 0x7000  # Single-ended AIN3

ADS1015_REG_CONFIG_PGA_MASK = 0x0E00
ADS1015_REG_CONFIG_PGA_6_144V = 0x0000  # +/-6.144V range
ADS1015_REG_CONFIG_PGA_4_096V = 0x0200  # +/-4.096V range
ADS1015_REG_CONFIG_PGA_2_048V = 0x0400  # +/-2.048V range (default)
ADS1015_REG_CONFIG_PGA_1_024V = 0x0600  # +/-1.024V range
ADS1015_REG_CONFIG_PGA_0_512V = 0x0800  # +/-0.512V range
ADS1015_REG_CONFIG_PGA_0_256V = 0x0A00  # +/-0.256V range

ADS1015_REG_CONFIG_MODE_MASK = 0x0100
ADS1015_REG_CONFIG_MODE_CONTIN = 0x0000  # Continuous conversion mode
ADS1015_REG_CONFIG_MODE_SINGLE = 0x0100  # Power-down single-shot mode (default)

ADS1015_REG_CONFIG_DR_MASK = 0x00E0
ADS1015_REG_CONFIG_DR_128SPS = 0x0000  # 128 samples per second
ADS1015_REG_CONFIG_DR_250SPS = 0x0020  # 250 samples per second
ADS1015_REG_CONFIG_DR_490SPS = 0x0040  # 490 samples per second
ADS1015_REG_CONFIG_DR_920SPS = 0x0060  # 920 samples per second
ADS1015_REG_CONFIG_DR_1600SPS = 0x0080  # 1600 samples per second (default)
ADS1015_REG_CONFIG_DR_2400SPS = 0x00A0  # 2400 samples per second
ADS1015_REG_CONFIG_DR_3300SPS = 0x00C0  # 3300 samples per second (also 0x00E0)

ADS1015_REG_CONFIG_CMODE_MASK = 0x0010
ADS1015_REG_CONFIG_CMODE_TRAD = 0x0000  # Traditional comparator with hysteresis (default)
ADS1015_REG_CONFIG_CMODE_WINDOW = 0x0010  # Window comparator

ADS1015_REG_CONFIG_CPOL_MASK = 0x0008
ADS1015_REG_CONFIG_CPOL_ACTVLOW = 0x0000  # ALERT/RDY pin is low when active (default)
ADS1015_REG_CONFIG_CPOL_ACTVHI = 0x0008  # ALERT/RDY pin is high when active

ADS1015_REG_CONFIG_CLAT_MASK = 0x0004  # Determines if ALERT/RDY pin latches once asserted
ADS1015_REG_CONFIG_CLAT_NONLAT = 0x0000  # Non-latching comparator (default)
ADS1015_REG_CONFIG_CLAT_LATCH = 0x0004  # Latching comparator

ADS1015_REG_CONFIG_CQUE_MASK = 0x0003
ADS1015_REG_CONFIG_CQUE_1CONV = 0x0000  # Assert ALERT/RDY after one conversions
ADS1015_REG_CONFIG_CQUE_2CONV = 0x0001  # Assert ALERT/RDY after two conversions
ADS1015_REG_CONFIG_CQUE_4CONV = 0x0002  # Assert ALERT/RDY after four conversions
ADS1015_REG_CONFIG_CQUE_NONE = 0x0003  # Disable the comparator and put ALERT/RDY in high state (default)


# Dictionaries with the sampling speed values
spsADS1015 = {
    128: ADS1015_REG_CONFIG_DR_128SPS,
    250: ADS1015_REG_CONFIG_DR_250SPS,
    490: ADS1015_REG_CONFIG_DR_490SPS,
    920: ADS1015_REG_CONFIG_DR_920SPS,
    1600: ADS1015_REG_CONFIG_DR_1600SPS,
    2400: ADS1015_REG_CONFIG_DR_2400SPS,
    3300: ADS1015_REG_CONFIG_DR_3300SPS
}
# Dictionary with the programable gains (PGA)
pgaADS1x15 = {
    6144: ADS1015_REG_CONFIG_PGA_6_144V,
    4096: ADS1015_REG_CONFIG_PGA_4_096V,
    2048: ADS1015_REG_CONFIG_PGA_2_048V,
    1024: ADS1015_REG_CONFIG_PGA_1_024V,
    512: ADS1015_REG_CONFIG_PGA_0_512V,
    256: ADS1015_REG_CONFIG_PGA_0_256V
}

# ==============================================================================
#
# ADC Class
#
# ==============================================================================
# This class is specifically made for reading ultrasonic sensor values.
# The class was adapted from Adafruit's ADS1x15 class, however modified as follows:
#   The read parameters are set on initialization. This saves a configuration
# write each time we want to read from the sensor.
#   This class uses Mike M's BCM2835 i2c library to ensure compatability with
# the other classes in this module. It allows for hardware level communcation
# instead of kernel level communication.
# ==============================================================================
class ads1015:
    def __init__(self, baudrate):
        #initialize bcm
        if not bcm.init():
            return 0
        #begin i2c, set baudrate
        bcm.i2c_begin()
        bcm.i2c_setBaudrate(baudrate)


    def read(self,channel=0,pga=6144,sps=1600):
        self.write_single(channel,pga,sps)
        return self.read_single(pga)

    def write_single(self, channel, pga, sps):
        # With invalid channel return -1
        if channel > 3:
            if self.debug:
                print "ADS1x15: Invalid channel specified: %d" % channel
            return -1

        # Disable comparator, Non-latching, Alert/Rdy active low
        # traditional comparator, single-shot mode
        config = ADS1015_REG_CONFIG_CQUE_NONE | \
                 ADS1015_REG_CONFIG_CLAT_NONLAT | \
                 ADS1015_REG_CONFIG_CPOL_ACTVLOW | \
                 ADS1015_REG_CONFIG_CMODE_TRAD | \
                 ADS1015_REG_CONFIG_MODE_SINGLE

        config |= spsADS1015.setdefault(sps, ADS1015_REG_CONFIG_DR_1600SPS)
        config |= pgaADS1x15.setdefault(pga, ADS1015_REG_CONFIG_PGA_6_144V)
        self.pga = pga

        # Set the channel to be converted
        if channel == 3:
            config |= ADS1015_REG_CONFIG_MUX_SINGLE_3
        elif channel == 2:
            config |= ADS1015_REG_CONFIG_MUX_SINGLE_2
        elif channel == 1:
            config |= ADS1015_REG_CONFIG_MUX_SINGLE_1
        else:
            config |= ADS1015_REG_CONFIG_MUX_SINGLE_0

        # Set 'start single-conversion' bit
        config |= ADS1015_REG_CONFIG_OS_SINGLE

        # write config value, use bytearray and buffers
        config_array = bytearray(3)
        config_buf = buffer(config_array, 0, 3)
        # set commands and values into array
        config_array[0] = ADS1015_REG_POINTER_CONFIG
        config_array[1] = (config >> 8) & 0xFF
        config_array[2] = config & 0xFF
        # write config value
        bcm.i2c_begin()
        bcm.i2c_setSlaveAddress(0x48)
        if bcm.i2c_write(config_buf, 3) == 0:
            return 1
        else:
            return 0

    def read_single(self,pga):
        # read conversion value, use bytearray and buffers
        point_array = bytearray(1)
        point_buf = buffer(point_array, 0, 1)
        conv_array = bytearray(2)
        conv_buf = buffer(conv_array, 0, 2)
        # set commands and values into array
        point_array[0] = ADS1015_REG_POINTER_CONVERT #conversion register
        # read cp value
        bcm.i2c_begin()
        bcm.i2c_setSlaveAddress(0x48)
        bcm.i2c_read_register_rs(point_buf,conv_buf,2)
        convread = (conv_array[0] << 4) | (conv_array[1] >> 4)
        if convread > 2047:
            convread = convread - 4096
        return (convread/2048.0)*pga/1000

