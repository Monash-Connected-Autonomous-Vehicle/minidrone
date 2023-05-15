import time
from periphery import I2C 
import struct
from data_dump import data_dump

#test_msgs = ["L500R4567S123C12340\n","L-500R4567S123C12340\n","L0000R4567S123C12340\n"]
test_msgs = ["L-450R000C0E\n"]

i2c_address = 0x50

i2c = I2C("/dev/i2c-1")


# fmt = 'f2s'
# fmt2 = '2f'
fmt2 = '2s'

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



    print("input data (raw): ", input[0].data)
    print("input data raw length: ", len(input[0].data))


    unpacked_data = struct.unpack(fmt2, input[0].data[0:2])
    # unpacked_data2 = struct.unpack(fmt2, input[0].data[6:14])
    # qdes_temp, drive_modes = unpacked_data
    wheel_speeds = unpacked_data



    my_struct = {"wheel_speeds": wheel_speeds}

    # print(my_struct["QDES_temp"])
    # print(my_struct["drive_modes"])
    print(my_struct["wheel_speeds"])


    time.sleep(1)
    
i2c.close()