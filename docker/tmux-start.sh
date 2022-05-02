#!/bin/sh
session="mcav"
tmux new-session -d -s $session
tmux send-keys 'source env/bin/activate'
tmux send-keys Enter C-l
tmux new-window
tmux send-keys 'source /opt/ros/noetic/setup.bash && source /opt/ros/foxy/setup.bash && ros2 run ros1_bridge dynamic_bridge' C-l
tmux split-window -h
tmux send-keys 'source /opt/ros/noetic/setup.bash && roscore' C-l
tmux select-window -t 1
tmux attach-session -d -t $session
