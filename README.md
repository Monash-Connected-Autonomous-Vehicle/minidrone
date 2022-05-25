# 🏎️ MiniDrone 

This repository contains the software for the [MCAV MiniDrone project](https://sites.google.com/student.monash.edu/minidrone).

## Requirements
It is recommended to install and run this project using docker. See [Docker Instructions](./docker/README.md).

OctoMap (https://octomap.github.io/) is a mapping library that can be used to generate probabilistic occupancy grids and heightmaps. To setup OctoMap for use with ROS2, the following can be done:
1. Install and build OctoMap libraries: https://github.com/OctoMap/octomap/wiki/Compilation-and-Installation-of-OctoMap
    * NOTE: package libqglviewer-dev-qt4 is unavailable for Ubuntu 20.04. Can be replaced with libqglviewer-dev-qt5.
2. Install ROS2 server in appropriate workspace: https://github.com/iKrishneel/octomap_server2

## Installation
`git clone https://github.com/Monash-Connected-Autonomous-Vehicle/minidrone.git`

## How to run
1. `cd minidrone`

2. `docker/run.sh`

Can start the ros 1 bridge by going to tmux window 2. (See [Tmux Instructions](./docker/tmux_instructions.md))
