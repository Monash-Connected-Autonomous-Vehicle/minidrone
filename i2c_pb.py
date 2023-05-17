# For protobuf for python: https://grpc.io/docs/languages/python/quickstart/

import time
from periphery import I2C 
import minidrone_pb2

i2c_address = 0x50

i2c = I2C("/dev/i2c-1")

msg = "test"

encoded = msg.encode('utf-8')
array = bytearray(encoded)

# i2c.transfer(i2c_address, [I2C.Message(array)])
input = [I2C.Message(array, read = True)]
output = i2c.transfer(i2c_address, input)

print("input data (raw): ", input[0].data)
print("input data raw length: ", len(input[0].data))

# decoding
# y = minidrone_pb2.SHCoefficients()
# minidrone_pb2.
message = minidrone_pb2.MinidroneMessage()

message.ParseFromString(bytes(input[0].data))
# print(message.ParseFromString(b'\x08\x01\x12\x03\x74\x65\x73\x22\x05\x48\x65\x6c\x6c\x6f'))


i2c.close()