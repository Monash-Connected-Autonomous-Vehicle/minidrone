# For protobuf for python: https://grpc.io/docs/languages/python/quickstart/

import time
from periphery import I2C 
import minidrone_pb2

i2c_address = 0x50

i2c = I2C("/dev/i2c-1")

# msg = "1234567"

# encoded = msg.encode('utf-8')
# array = bytearray(encoded)

# i2c.transfer(i2c_address, [I2C.Message(array)])
# input = [I2C.Message(array, read = True)]
# msgWritePointer = I2C.Message
# i2c.transfer(i2c_address, input)

while input("Request? ") != "n":
    msg = I2C.Message(bytearray(9), read=True)
    i2c.transfer(i2c_address, [msg])

    print("input data raw length: ", len(msg.data))
    print("input data (raw): ", msg.data)

    message = minidrone_pb2.QDES()
    message.ParseFromString(msg.data)
    print("TEMP: " + str(message.computeUnit.temp))
    print("FAN SPEED: " + str(message.computeUnit.fan_speed))

i2c.close()