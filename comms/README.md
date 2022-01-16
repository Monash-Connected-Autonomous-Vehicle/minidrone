# Communications

Once the installation and setup below has been completed, you can run the communications package using:
```
$ roslaunch comms comms.launch
```

## IMU

Please follow the setup guide here:
- [JetsonHacks: Using i2c](https://www.jetsonhacks.com/2019/07/22/jetson-nano-using-i2c/)


### i2c Permissions

Enable the i2c permissions using:

```
$ sudo usermod -aG i2c $USER
```

See here for reference:
- [Nvidia Forums: i2c permissionerror](https://forums.developer.nvidia.com/t/jetson-nano-i2c-error-permissionerror-errno-13-permission-denied/83383/5)


### Adafruit BNO055 Library

Install the adafruit bno0555 python library:
```
$ pip3 install adafruit-circuitpython-bno055
```
Run the ROS node using:

```
$ rosrun comms comms_to_ros
```


## GPS

Please follow the setup guide here:
- [Adafruit Ultimate GPS on Jetson Nano: UART-Serial](https://learn.adafruit.com/circuitpython-libraries-on-linux-and-the-nvidia-jetson-nano/uart-serial)

### /dev/ttyTHS1 Permissions
Once setup you will need to enable permissions on the uart port (ttyTHS1)
```
$ sudo usermod -a -G tty $USER
$ ls -l /dev/ttyTHS1
$ sudo systemctl stop nvgetty.service
$ sudo systemctl disable nvgetty.service
$ sudo reboot
```
See here for reference: [Nvidia Forum: Read/Write Permission ttyTHS1](https://forums.developer.nvidia.com/t/read-write-permission-ttyths1/81623)


#### ROS GPS Driver
Install the nmea_navsat_driver for ros-melodic:

```
$ sudo apt-get install ros-melodic-nmea-navsat-driver
```

Run the nmea_navsat_driver with the following parameters
```
$ rosrun nmea_navsat_driver nmea_serial_driver _port:=/dev/ttyTHS1 _baud:=9600
```
GPS data should now be published on the /fix topic.

Reference:
- [Platypus Boats: Jetson Peripheral GPS](https://platypus-boats.readthedocs.io/en/latest/source/jetson/peripheral/gps.html) 
- [nmea_navsat_driver](https://wiki.ros.org/nmea_navsat_driver) 


