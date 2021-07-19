# MiniDrone 

This repository contains the software for the MCAV MiniDrone project.

## Installation

- Setup Jetson to run ROS and Arduino IDE

  See [MiniDrone Software Setup](https://docs.google.com/document/u/1/d/16hTuHI0GkWB7Fz8jkYoRb6zCtv_vSotQT3AZyo3yxcA/edit?usp=drive_web&ouid=101896295474612094790)

- Install MiniDrone requirements

  ``` ./install.sh ```

- Upload code to arduino

  See box_republisher/box_sensor_raw/box_sensor_raw.ino for details and pin layout

## Packages

### mcav_box_sensor_republisher
This package reads sensor data over Serial and republishes it as the corresponding ROS message.

Currently supports IMU, Magnetic Field, IMU Calibration Data, GPS (Latitude, Longitude, Altitude)

### control
This package controls the vehicle's motors based on the desired ROS Twist message.
It listens for ROS Twist messages and sets the motor speed and servo position accordingly. It also provides a mapping from a connected joy stick controller to Twist messages.

### comms
An alternative to the box_republisher, this package uses i2c to communicate with the sensors directly, rather than needing an Arduino in-between.

### emergency_brake
Prevents the MiniDrone from being driven into a wall or object. It filters the pointcloud from the RPLidar scanner and publishes 0 twist commands if a collision is imminent.

### imitation_learning
Imitation learning pipeline so that the MiniDrone can be trained directly on camera data and joystick control inputs.

### mini_tools

For more commands, see [Useful Commands](/useful_cmds.md)

## Tests

Automated sensor data tests (needs roscore):

` pytest test_box.py -v `

Sensor data synchronisation:

` python3 control/src/tests/test_time_synch.py `

Motor control tests:

` python3 control/src/tests/test_motor_control.py `
