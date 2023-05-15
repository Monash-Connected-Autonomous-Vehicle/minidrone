import time
from periphery import I2C 

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

i2c.close()