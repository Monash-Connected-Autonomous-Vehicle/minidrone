# MiniDrone 

This repository contains the software for the MCAV MiniDrone project.

## Installation

- Setup Jetson to run ROS and Arduino IDE

  See [MiniDrone Software Setup](https://docs.google.com/document/u/1/d/16hTuHI0GkWB7Fz8jkYoRb6zCtv_vSotQT3AZyo3yxcA/edit?usp=drive_web&ouid=101896295474612094790)

- Install MiniDrone requirements

  ``` ./install.sh ```

- Upload code to arduino

  See box_republisher/box_sensor_raw/box_sensor_raw.ino for details and pin layout

## Getting started

### mcav_box_sensor_republisher
This package reads sensor data over Serial and republishes it as the corresponding ROS message.

Currently supports:
- Imu
- Magnetic Field
- IMU Calibration Data
- GPS (Latitude, Longitude, Altitude)

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


### control
This package controls the vehicle's motors based on the desired ROS Twist message.
This package listens for ROS Twist messages and sets the motor speed and servo position accordingly. It also provides a mapping from a connected joy stick controller to Twist messages.

Supports
- PS4 controller

To run the node the control node by itself:

`rosrun control control.py`

To run the control node as well as the joy stick controller twist publisher:

`roslaunch control dualshock.launch`

For more commands, see [Useful Commands](/useful_cmds.md)

## Tests

Automated sensor data tests (needs roscore):

` pytest test_box.py -v `

Sensor data synchronisation:

` python3 control/src/tests/test_time_synch.py `

Motor control tests:

` python3 control/src/tests/test_motor_control.py `
