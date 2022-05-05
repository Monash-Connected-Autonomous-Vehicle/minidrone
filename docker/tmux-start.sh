#!/bin/sh

# This is only here to fix some errors that were happening with sudo and networking
# The warning was "sudo: unable to resolve host : Name or service not known"
# Put dynamic hostname into /etc/hosts to remove sudo warnings:
sudo sh -c "echo \"127.0.0.1	`hostname`.localdomain	`hostname`\" >> /etc/hosts" 2> /dev/null


# Start a tmux session
session="mcav"
tmux new-session -d -s $session
tmux send-keys 'source env/bin/activate'
tmux send-keys Enter C-l # press enter and clear screen
tmux new-window
tmux send-keys 'source /opt/ros/noetic/setup.bash && source /opt/ros/foxy/setup.bash && ros2 run ros1_bridge dynamic_bridge' C-l
tmux split-window -h
tmux send-keys 'source /opt/ros/noetic/setup.bash && roscore' C-l
tmux select-window -t 1
tmux attach-session -d -t $session
