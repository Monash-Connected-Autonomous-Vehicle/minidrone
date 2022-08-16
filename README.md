# 🏎️ MiniDrone 

This repository contains the software for the [MCAV MiniDrone project](https://sites.google.com/student.monash.edu/minidrone).

## Requirements
It is recommended to install and run this project using docker. See [Docker Instructions](./docker/README.md).

## Installation
`git clone https://github.com/Monash-Connected-Autonomous-Vehicle/minidrone.git`

## How to start system
1. `cd minidrone`

2. `docker/run.sh`

Can start the ros 1 bridge by going to tmux window 2. (See [Tmux Instructions](./docker/tmux_instructions.md))

## Runing lane following
1. Lane detection + lane following: `ros2 launch md_lane_detection lane_following.launch.xml`
2. (Optional): Run gazebo to simulate: 
```
. /usr/share/gazebo/setup.sh
ros2 launch minidrone_gazebo gazebo.launch.py 
```
