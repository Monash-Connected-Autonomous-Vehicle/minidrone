# MiniDrone Useful Commands

## run roscore
roscore

## run jetbot camera
roslaunch jetbot_ros jetbot_camera_compressed.launch

## run imu/gps republisher
cd mcav/minidrone

python run_box_republisher.py

## rviz with config
rviz -d /home/jetson/mcav/minidrone/cam_imu.rviz

## record data
rosbag record /gps /imu /imu/calibration /imu/mag /jetbot_camera/compressed 

## run motor control tests
python3 src/control/src/test_motor_control.py

## run ps4 control
connect ps4 controller via bluetooth
roslaunch control dualshock.launch







# ROS deep learning demos

## imagenet
roslaunch ros_deep_learning imagenet.ros1.launch input:=csi://0 output:=display://0

## detectnet
roslaunch ros_deep_learning detectnet.ros1.launch input:=csi://0 output:=display://0


## segnet
roslaunch ros_deep_learning segnet.ros1.launch input:=csi://0 output:=display://0




