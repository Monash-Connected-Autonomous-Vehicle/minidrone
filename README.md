# MiniDrone 

This repository contains the software for the MCAV MiniDrone project.

## mcav_box_sensor_publisher
This package reads sensor data over Serial and republishes it as the corresponding ROS message.

Currently supports:
- Imu
- Magnetic Field
- IMU Calibration Data
- GPS (Latitude, Longitude, Altitude)

## control
This package controls the vehicle's motors based on the desired ROS Twist message.

Supports
- PS4 controller

## Installation

Setup Jetson to run ROS and Arduino IDE
- See https://docs.google.com/document/u/1/d/16hTuHI0GkWB7Fz8jkYoRb6zCtv_vSotQT3AZyo3yxcA/edit?usp=drive_web&ouid=101896295474612094790

Install python requirements.
`pip3 install -r requirements`

Upload code to arduino
- See box_sensor_raw/box_sensor_raw.ino for details and pin layout

## Getting started

To run the node:
` python run_box_republisher.py `

Options:
- -port: select the arduino serial port name (default: /dev/ttyACM0)
- -baud: select the arduino serial baud rate (default: 115200)

Publishes:
- /imu: raw imu data including fused orientation, linear_acceleration and angular_velocity
- /imu/mag: raw imu magnetometer data
- /imu/calibration: status information indicating the state of the onboard calibration
- /gps: gps fix information including latitude, longitude, and altitude


## Tests

Automated sensor data tests (needs roscore)
` pytest test_box.py -v `

Sensor data synchronisation:
` python3 src/control/src/tests/test_time_synch.py `

Motor control tests:
` python3 src/control/src/tests/test_motor_control.py `
