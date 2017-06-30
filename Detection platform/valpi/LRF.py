import time
import PyBCM2835 as bcm


class LRF:

    def __init__(self):
        bcm.init()
        bcm.i2c_begin()
        bcm.i2c_setBaudrate(400000)

    def read(self):
        bcm.i2c_setSlaveAddress(0x62)

        command_array = bytearray(2)
        command_buf = buffer(command_array,0,2)
        command_array[0] = 0x00 #regvalue
        command_array[1] = 0x04 #measvalue

        reg_array = bytearray(1)
        reg_buf = buffer(reg_array,0,1)
        reg_array[0] = 0x8F

        out1_array = bytearray(2)
        out1_buf = buffer(out1_array,0,2)

        bcm.i2c_write(command_buf, 2)
        time.sleep(0.008)

        bcm.i2c_read_register_rs(reg_buf, out1_buf, 2)
        return out1_array[0]<<8|out1_array[1]


