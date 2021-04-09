#!/usr/bin/env python

import smbus 
import time 
import rospy

# Nvidia Jetson Nano i2c Bus 0
bus = smbus.SMBus(0)

address = 0x40


while True:
    data = bus.read_byte(address)
    print(data)


    