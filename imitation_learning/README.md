# Imitation Learning

# Running in Simulation

Start CARLA: `/opt/carla-simulator/CarlaUE4.sh`
Spawn a vehicle using `python3 /opt/carla-simulator/PythonAPI/examples/manual_control.py`

Make sure to have sourced the workspace.
`roslaunch imitation_learning carla.launch`

# Structure

`imitation_learning.py` is for recording data or running the model

`train.py` is for training the model