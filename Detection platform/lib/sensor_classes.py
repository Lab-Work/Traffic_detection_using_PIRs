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
# PIR constants and register values
#
# ==============================================================================


# Yes, really, it's not 0x60, the EEPROM has its own address (I've never seen this before)
MLX90620_EEPROM_WRITE = 0xA0
MLX90620_EEPROM_READ = 0xA1

# Ta registers
VTH_L = 0xDA
VTH_H = 0xDB
KT1_L = 0xDC
KT1_H = 0xDD
KT2_L = 0xDE
KT2_H = 0xDF
KT_SCALE = 0xD2

#To registers

##start registers for array
delta_Ai = 0x00
Bi = 0x40
delta_alphaij = 0x80
##end registers for array

KS_SCALE = 0xC0
KS4_EE = 0xC4
ACOMMON_L = 0xD0
ACOMMON_H = 0xD1

ACP_L = 0xD3
ACP_H = 0xD4

BCP = 0xD5
alphaCP_L = 0xD6
alphaCP_H = 0xD7
TGC = 0xD8
ABI_SCALE = 0xD9

alpha0_L = 0xE0
alpha0_H = 0xE1
alpha0_SCALE = 0xE2
delta_alpha_SCALE = 0xE3
EMIS_L = 0xE4
EMIS_H = 0xE5
KSTA_L = 0xE6
KSTA_H = 0xE7



OSC_TRIM_VALUE = 0xF7

# Bits within configuration register 0x92

POR_TEST = 10

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
# PIR Classes
#
# ==============================================================================

class PIR_Array:
    """
    This is the 3 PIR 4x16 sensor array class, which includes three instances of PIR4x16
    """
    def __init__(self, baud_rate):

        self.baud_rate = baud_rate

        # parse each strings or byte buffers into those matrix and array
        # HEX values
        self.temp_pir1_4x16 = np.zeros((4, 16))
        self.temp_pir2_4x16 = np.zeros((4, 16))

        self.ptat_pir1 = 0
        self.ptat_pir2 = 0

        self.cpix_pir1 = 0
        self.cpix_pir2 = 0

        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(11, GPIO.OUT)
        GPIO.setup(13, GPIO.OUT)

        # initialize the three pir sensors
        GPIO.output(11,False)
        GPIO.output(13,False)
        self.pir1 = PIRSensor(1, self.baud_rate)
        time.sleep(0.2)
        GPIO.output(11,True)
        GPIO.output(13,False)
        self.pir2 = PIRSensor(2, self.baud_rate)
        time.sleep(0.2)

    def set_frequency(self,freq):
        # sensor collect
        GPIO.output(11,False)
        GPIO.output(13,False)
        self.pir1.mlx_set_frequency(freq)
        GPIO.output(11,True)
        GPIO.output(13,False)
        self.pir2.mlx_set_frequency(freq)

    def read(self):
        # sensor collect
        GPIO.output(11,False)
        GPIO.output(13,False)
        s1 = self.pir1.mlx_ir_read()
        GPIO.output(11,True)
        GPIO.output(13,False)
        s2 = self.pir2.mlx_ir_read()
        s1c = self.pir1.calculate_4x16_np(self.pir1.mlx_cshape(s1),self.pir1.mlx_ptat(), self.pir1.mlx_cp())
        s2c = self.pir2.calculate_4x16_np(self.pir2.mlx_cshape(s2),self.pir2.mlx_ptat(), self.pir2.mlx_cp())
        return np.hstack([s1c,s2c])

    def read_raw(self):
        tamb = np.empty(2)
        cp = np.empty(2)
        # sensor collect
        # sensor 1
        GPIO.output(11,False)
        GPIO.output(13,False)
        s1 = self.pir1.mlx_cshape(self.pir1.mlx_ir_read())
        tamb[0]=self.pir1.mlx_ptat()
        cp[0]=self.pir1.mlx_cp()
        # sensor 2
        GPIO.output(11,True)
        GPIO.output(13,False)
        s2 = self.pir2.mlx_cshape(self.pir2.mlx_ir_read())
        tamb[1]=self.pir2.mlx_ptat()
        cp[1]=self.pir2.mlx_cp()
        return np.array([s1,s2,tamb,cp])


    # convert hex to signed int
    def hex_val(self, val):
        if val > 32767:
            return val - 65536
        else:
            return val

    # convert hex to signed byte
    def sign_byte(self, val):
        if val > 127:
            return val - 256
        else:
            return val


# this is the library for PIR90620 sensor.
# construct object with EEPROM constants,
# then given the IRraw data, it should compute the temperature matrix in C
class PIRSensor:
    def __init__(self, pir_id, baudrate):

        # Variables for Ta calculation
        self.v_th = 0
        self.k_t1 = 0
        self.k_t2 = 0
        self.k_t1_scale = 0
        self.k_t2_scale = 0

        #other niit
        self.pir_id = pir_id
        self.baudrate = baudrate
        self.eeprom = bytearray(256)
        self.config = bytearray(2)

        #alphaij calculations
        self.alpha_ij = np.zeros(64)
        self.alpha_ij_np = np.zeros((4,16))

        #to values
        self.a_common = 0
        self.a_cp = 0
        self.b_cp = 0
        self.tgc = 0
        self.delta_ai_scale = 0
        self.b_i_scale = 0
        self.emissivity = 0
        self.ksta = 0
        self.ks4 = 0
        self.alpha0_scale = 0

        self.alpha_cp = 0
        
        # self.b_i_scale_calc = 0 #calculated value

        self.a_ij = np.zeros(64)  # 64 array
        self.b_ij = np.zeros(64)

        self.a_ij_np = np.zeros((4,16))
        self.b_ij_np = np.zeros((4,16))

        self.res = 0

        print "init arrays"

        # raw data to be received

        self.temperatures = np.zeros((4, 16))
        self.Tambient = 0
        
        # begin initializing sensor
        print "init"
        for i in range(10):
            print i
            self.mlx_init()
        time.sleep(0.5)
        
        print "const"
        self.const_init()

        # constants needs to be first computed and then used


    # initialize the constant parameters
    def const_init(self):
        # =============================================
        # read and calculate constants for PIR instance
        # =============================================

        # global constants

        self.res = 3
        self.k_t1_scale = (self.eeprom[KT_SCALE]&0xF0)>>4
        self.k_t2_scale = (self.eeprom[KT_SCALE]&0x0F)
        
        # Ta constants

        self.v_th = self.signed_int16(self.eeprom[VTH_H]<<8|self.eeprom[VTH_L])/(2.0**(3-self.res))
        self.k_t1 = self.signed_int16(self.eeprom[KT1_H]<<8|self.eeprom[KT1_L])/(2.0**(self.k_t1_scale+3-self.res))
        print self.signed_int16(self.eeprom[KT2_H]<<8|self.eeprom[KT2_L])
        self.k_t2 = self.signed_int16(self.eeprom[KT2_H]<<8|self.eeprom[KT2_L])/(2.0**(self.k_t2_scale+10+3-self.res))
        print self.k_t2

        self.a_common = self.signed_int16(self.eeprom[ACOMMON_H]<<8|self.eeprom[ACOMMON_L])
                                                                                 
        self.emissivity = (self.eeprom[EMIS_H]<<8|self.eeprom[EMIS_L]) / 32768.0

        self.delta_ai_scale = (self.eeprom[ABI_SCALE]&0xF0)>>4
        self.b_i_scale = (self.eeprom[ABI_SCALE]&0x0F)
        
        self.a_cp = self.signed_int16(self.eeprom[ACP_H]<<8|self.eeprom[ACP_L])/(2.0**(3-self.res))
        self.b_cp = self.signed_int8(self.eeprom[BCP])/(2.0**(self.b_i_scale+3-self.res))

        self.alpha_cp = (self.eeprom[alphaCP_H]<<8|self.eeprom[alphaCP_L])/(2.0**(self.eeprom[alpha0_SCALE]+3-self.res))
        
        self.tgc = self.signed_int8(self.eeprom[TGC])


        self.ksta = self.signed_int16(self.eeprom[KSTA_H]<<8|self.eeprom[KSTA_L])/1048576.0
        self.ks4 = self.signed_int8(self.eeprom[KS4_EE])/(2.0**(9+8))
        
        # self.b_i_scale_calc = np.power(2,self.b_i_scale)

        # =============================================
        # read and calculate constants for PIR instance
        # =============================================
        for i in range(0, 64):
            # Read the individual pixel offsets
            self.a_ij[i] = (self.a_common+self.eeprom[delta_Ai+i]*(2.0**(self.delta_ai_scale)))/(2.0**(3-self.res))
            self.b_ij[i] = self.signed_int8(self.eeprom[Bi + i])/(2.0**(self.b_i_scale+3-self.res))  # Bi(i,j) begins 64 bytes into EEPROM at 0x40

            
        self.a_ij_np = self.a_ij.reshape(4,16,order="F")
        self.b_ij_np = self.b_ij.reshape(4,16,order="F")
        self.alpha_ij_np = self.alpha_ij.reshape(4,16,order="F")

        time.sleep(1)
        self.__calculate_alpha()
        self.__calculate_TA(self.mlx_ptat())

    # calculate TA
    def __calculate_TA(self, ptat):
        self.Tambient = (-self.k_t1 + np.sqrt(self.k_t1*self.k_t1 - (4 * self.k_t2 * (self.v_th - ptat)))) / (2 * self.k_t2) + 25

    def __calculate_alpha(self):
        aij = []
        for i in range(0x80, 0xC0):
            a0 = (self.eeprom[alpha0_H]<<8|self.eeprom[alpha0_L])/(2.0**self.eeprom[alpha0_SCALE])
            aij.append((a0+(self.eeprom[i]/(2.0**self.eeprom[delta_alpha_SCALE])))/(2.0**(3-self.res)))
        self.alpha_ij = np.copy(aij).reshape(4,16,order="F")

    
    def calculate_4x16_temperature(self, ptat, irData, cpix):
        """
        This function computes the temperature for the entire 4x16 frame.
        It first the ambient temperature and then compute the object temperature using irData
        :param ptat: the raw data for ambient temperature from sensor
        :param irData: the raw temperature data from sensor
        :param cpix: adjustment parameter read from sensor
        :return: computed data saved in self.temperature in 4x16 np.array in Celcius
        """

        self.__calculate_TA(ptat)

        # Calculate the offset compensation for the one compensation pixel
        #This is a constant in the TO calculation, so calculate it here.
        v_cp_off_comp = cpix - (self.a_cp + (self.b_cp/ np.power(2, self.b_i_scale)) * (self.Tambient - 25))

        for col in range(0, 16):
            for row in range(0, 4):
                i = col * 4 + row

                v_ir_off_comp = irData[row, col] - (self.a_ij[i] + (self.b_ij[i]) * (self.Tambient - 25))  #1: Calculate Offset Compensation
                v_ir_tgc_comp = v_ir_off_comp - ((self.tgc / 32.0) * v_cp_off_comp)  #2: Calculate Thermal Gradien Compensation (TGC)

                v_ir_comp = v_ir_tgc_comp / self.emissivity  #3: Calculate Emissivity Compensation

                self.temperatures[row, col] = np.sqrt(np.sqrt(
                  (v_ir_comp / self.alpha_ij[i]) + np.power(self.Tambient + 273.15, 4)
                  )) - 273.15

                # save into all_temperatures
                self.all_temperatures[i].append(self.temperatures[row, col])


    def calculate_4x16_np(self, irData, ptat, cp):
        """
        This function computes the temperature for the entire 4x16 frame.
        It first the ambient temperature and then compute the object temperature using irData
        :param ptat: the raw data for ambient temperature from sensor
        :param irData: the raw temperature data from sensor
        :param cpix: adjustment parameter read from sensor
        :return: computed data saved in self.temperature in 4x16 np.array in Celcius
        """
        #self.__calculate_TA(self.mlx_ptat())
        self.__calculate_TA(ptat)

        # Calculate the offset compensation for the one compensation pixel
        #This is a constant in the TO calculation, so calculate it here.
        #v_cp_off_comp = self.signed_int16(self.mlx_cp())-(self.a_cp+(self.b_cp*(self.Tambient-25)))
        v_cp_off_comp = self.signed_int16(cp)-(self.a_cp+(self.b_cp*(self.Tambient-25)))

        v_ir_off_comp = irData-(self.a_ij_np+(self.b_ij_np*(self.Tambient-25))) # offset compensation

        v_ir_tgc_comp = v_ir_off_comp - ((self.tgc/32.0) * v_cp_off_comp)

        v_ir_comp = v_ir_tgc_comp / self.emissivity

        a_comp = (1+self.ksta*(self.Tambient-25))*(self.alpha_ij-(self.tgc/32.0)*self.alpha_cp)

        tak4 = (self.Tambient + 273.15)**4.0

        #sx = (self.ks4*np.power((np.power(a_comp, 3)*v_ir_comp)+(np.power(a_comp,4)*tak4),0.25))
        #return np.power(((v_ir_comp)/(a_comp*(1-self.ks4*273.15)+sx))+tak4,0.25)-273.15
        return np.power(((v_ir_comp)/(a_comp*(1-self.ks4*273.15)))+tak4,0.25)-273.15    

        # self.temperatures = np.sqrt(np.sqrt((v_ir_comp / self.alpha_ij_np) + np.power(self.Tambient + 273.15, 4))) - 273.15

    # begin direct i2c commands
    # initialize mlx
    def mlx_init(self):
        #initialize bcm
        if not bcm.init():
            return 0
        #begin i2c, set baudrate
        bcm.i2c_begin()
        bcm.i2c_setBaudrate(self.baudrate)
        self.eeprom = self.mlx_read_eeprom()
        self.mlx_write_trim(self.eeprom[0xF7])
        self.mlx_write_config(self.eeprom[0xF5], self.eeprom[0xF6])
        self.mlx_set_frequency(128)
        self.config = self.mlx_read_config()


    # read EEPROM and return
    def mlx_read_eeprom(self):
        # for the mlx90621, eeprom needs to be enabled first
        cmsb, clsb = self.mlx_read_config()
        self.mlx_write_config(cmsb | 0b00010000,clsb)
        # command for reading eeprom
        command_array = bytearray(1)
        command_buf = buffer(command_array, 0, 1)
        command_array[0]=0x00
        bcm.i2c_begin()
        bcm.i2c_setSlaveAddress(0x50)
        eeprom_array = bytearray(256)
        eeprom_buf = buffer(eeprom_array, 0, 256)
        # here we do not need to check if the command worked,
        # the function will return a 0 array if it didn't
        bcm.i2c_write_read_rs(command_buf,1,eeprom_buf)
        return eeprom_array

    # write trim value and return success. trim must be in range(255)
    def mlx_write_trim(self, trim):
        # write trim value, use bytearray and buffers
        trim_array = bytearray(5)
        trim_buf = buffer(trim_array, 0, 5)
        # set commands and values into array
        trim_array[0] = 0x04
        trim_array[1] = trim
        trim_array[2] = self.hex_ubyte(trim)
        trim_array[3] = 0x56
        trim_array[4] = 0x00
        # write trim value
        bcm.i2c_begin()
        bcm.i2c_setSlaveAddress(0x60)
        if bcm.i2c_write(trim_buf, 5) == 0:
            return 1
        else:
            return 0

    # read trim value and return two byte array containing msb and lsb .
    def mlx_read_trim(self):
        # read trim value, use bytearray and buffers
        command_array = bytearray(4)
        command_buf = buffer(command_array, 0, 4)
        trim_array = bytearray(2)
        trim_buf = buffer(trim_array, 0, 2)
        # set commands and values into array
        command_array[0] = 0x02 #command
        command_array[1] = 0x93 #start address
        command_array[2] = 0x00 #address step
        command_array[3] = 0x01 #number of reads
        # read config value
        bcm.i2c_begin()
        bcm.i2c_setSlaveAddress(0x60)
        bcm.i2c_write_read_rs(command_buf,4,trim_buf,2)
        return trim_array # will return 0 if not read

    # write config value and return success. trim must be in range(255)
    def mlx_write_config(self, lsb, msb):
        # write config value, use bytearray and buffers
        config_array = bytearray(5)
        config_buf = buffer(config_array, 0, 5)
        # set commands and values into array
        config_array[0] = 0x03 #command
        config_array[1] = self.hex_ubyte(lsb-0x55) #check byte
        config_array[2] = lsb # lsb
        config_array[3] = self.hex_ubyte(msb-0x55) #check byte
        config_array[4] = msb # msb
        # write config value
        bcm.i2c_begin()
        bcm.i2c_setSlaveAddress(0x60)
        if bcm.i2c_write(config_buf, 5) == 0:
            return 1
        else:
            return 0

    # read config value and return two byte array containing msb and lsb .
    def mlx_read_config(self):
        # read config value, use bytearray and buffers
        command_array = bytearray(4)
        command_buf = buffer(command_array, 0, 4)
        config_array = bytearray(2)
        config_buf = buffer(config_array, 0, 2)
        # set commands and values into array
        command_array[0] = 0x02 #command
        command_array[1] = 0x92 #start address
        command_array[2] = 0x00 #address step
        command_array[3] = 0x01 #number of reads
        # read config value
        bcm.i2c_begin()
        bcm.i2c_setSlaveAddress(0x60)
        bcm.i2c_write_read_rs(command_buf,4,config_buf)
        return config_array # will return 0 if not read

    # set the refresh frequency by changing the config bits
    def mlx_set_frequency(self, frequency):
        frequencies = {
            0: 0b1111,
            1: 0b1110,
            2: 0b1101,
            4: 0b1100,
            8: 0b1011,
            16: 0b1010,
            32: 0b1001,
            64: 0b1000,
            128: 0b0111,
            256: 0b0110,
            512: 0b0000
            }
        config = self.mlx_read_config()
        if self.mlx_write_config(frequencies.get(frequency, 0b1110)|0b110000,self.mlx_read_config()[1]):
            return 1
        else:
            return 0

    # get the TA from the sensor
    def mlx_ta(self, ptat):
        return self.__calculate_TA(ptat)

    # get the ptat from the sensor
    def mlx_ptat(self):
        # read ptat value, use bytearray and buffers
        command_array = bytearray(4)
        command_buf = buffer(command_array, 0, 4)
        ptat_array = bytearray(2)
        ptat_buf = buffer(ptat_array, 0, 2)
        # set commands and values into array
        command_array[0] = 0x02 #command
        command_array[1] = 0x40 #start address
        command_array[2] = 0x00 #address step
        command_array[3] = 0x01 #number of reads
        # read ptat value
        bcm.i2c_begin()
        bcm.i2c_setSlaveAddress(0x60)
        bcm.i2c_write_read_rs(command_buf,4,ptat_buf)
        return ptat_array[1]<<8|ptat_array[0] # return an integer

    # get the compensation pixel from the sensor
    def mlx_cp(self):
        # read ptat value, use bytearray and buffers
        command_array = bytearray(4)
        command_buf = buffer(command_array, 0, 4)
        cp_array = bytearray(2)
        cp_buf = buffer(cp_array, 0, 2)
        # set commands and values into array
        command_array[0] = 0x02 #command
        command_array[1] = 0x41 #start address
        command_array[2] = 0x00 #address step
        command_array[3] = 0x01 #number of reads
        # read cp value
        bcm.i2c_begin()
        bcm.i2c_setSlaveAddress(0x60)
        bcm.i2c_write_read_rs(command_buf,4,cp_buf)
        return self.int_tbyte(cp_array[1]<<8|cp_array[0]) # return an integer

    # return entire pixel frame from sensor
    def mlx_ir_read(self):
        # read ir frame values, use bytearray and buffers
        command_array = bytearray(4)
        command_buf = buffer(command_array, 0, 4)
        ir_array = bytearray(128)
        ir_buf = buffer(ir_array, 0, 128)
        # set commands and values into array
        command_array[0] = 0x02 #command
        command_array[1] = 0x00 #start address
        command_array[2] = 0x01 #address step
        command_array[3] = 0x40 #number of reads
        # read config value
        bcm.i2c_begin()
        bcm.i2c_setSlaveAddress(0x60)
        bcm.i2c_write_read_rs(command_buf,4,ir_buf)
        return ir_array # will return 0 if not read

    # switch operating mode bit to BIT
    def mlx_set_meas_mode(self,bit):
        if self.mlx_write_config(((self.mlx_read_config()[0])&0b10111111)|bit<<6,self.mlx_read_config()[1]):
            return 1
        else:
            return 0

    # sends a read command in step mode. will fail if not in correct mode.
    def mlx_start_read(self):
        # initialize buffer
        read_array = bytearray(2)
        read_buf = buffer(read_array, 0, 2)
        # set commands and values into array
        read_array[0] = 0x01 #opcode msb
        read_array[1] = 0x08 #opcode lsb
        # write config value
        bcm.i2c_begin()
        bcm.i2c_setSlaveAddress(0x60)
        if bcm.i2c_write(read_buf, 2) == 0:
            return 1
        else:
            return 0

    # return 0x09 set
    def mlx_meas_done(self):
        return not self.mlx_read_config()[1]&0x01==0x01

    # reshape the raw data. combines data into two bytes and reshapes.
    def mlx_cshape(self, rdata):
        com_data = np.zeros(64)
        for i in range(0,128,2):
            com_data[i/2]=self.int_tbyte((rdata[i+1]<<8|rdata[i]))
        return com_data.reshape(4,16, order='F')


    # signed int
    def signed_int8(self, num):
        if num > 127:
            return num - 256
        else:
            return num

    def signed_int16(self, num):
        if num > 32767:
            return num - 65536
        else:
            return num
        
    # two bytes into signed int
    def int_tbyte(self,tbyte):
        if tbyte > 32767:
            return tbyte - 65536
        else:
            return tbyte


    # make sure byte is in range(255) (not negative)
    def hex_ubyte(self, byte):
        if byte >= 0:
            return byte
        else:
            return self.hex_ubyte(byte+256)
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
class ADC:
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

