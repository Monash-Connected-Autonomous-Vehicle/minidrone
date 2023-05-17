import time
from periphery import I2C 
import struct
from data_dump import data_dump
import ctypes

#test_msgs = ["L500R4567S123C12340\n","L-500R4567S123C12340\n","L0000R4567S123C12340\n"]
test_msgs = ["L-450R000C0E\n"]

i2c_address = 0x50

i2c = I2C("/dev/i2c-1")

class TestStruct(ctypes.Structure):
    # _fields_ = [("QDES_temp", ctypes.c_float),
    #             ("drive_mode", ctypes.c_char * 2),
    #             ("wheel_speed", ctypes.c_float * 2)]
    _fields_ = [("QDES_temp", ctypes.c_float),
                ("drive_mode", ctypes.c_char),
                ("wheel_speed", ctypes.c_float * 1)]



test_struct = TestStruct()
#Sample message "L1234R4567S123C1234"
# Do not send a message without over 20 chars as it may crash the arduinos recieve buffer
# L1234 - Left motor flag followed by up to 4 digits 
# R1234 - Right motor flag followed by up to 4 digits 
# C1234 - Checksum equation flag (message will be dropped if this is inconsistent)

# Two sequential failed checksums will stop the motors
# >100ms without a message will stop the motors

for msg in test_msgs:
    encoded = msg.encode('utf-8')
    array = bytearray(encoded)

    # i2c.transfer(i2c_address, [I2C.Message(array)])
    input = [I2C.Message(array, read = True)]
    output = i2c.transfer(i2c_address, input)

    print(input[0].data)
    print("input data length: ", len(input[0].data))
    print("ctypes struct length: ", ctypes.sizeof(TestStruct))

    # print(input[0].data.decode('utf-8', 'ignore'))
    # QDES_temp, drive_mode, wheel_speed = unpack('<10sHHb', input[0].data)

    my_struct = TestStruct.from_buffer_copy(input[0].data[0:16])

    print("temp: ", my_struct.QDES_temp)
    print("drive_modes: ", my_struct.drive_mode)
    print("wheel_speeds: ", my_struct.wheel_speed[0])


    time.sleep(1)
    
i2c.close()