# Mini Tools Package

This package contains a collection of tools for working with and using the MiniDrone.

## Mini UI (mini_ui)
This node is a ui for some basic functions. It contains
- Map View
- Camera View
- System Monitor
- System Controls (Start / Stop)
- Data Recording Controls 

```roslaunch mini_tools mini_ui.launch```

## Recorder
This node contains functionality for recording ROSBAGs via the ui, or programmatically.

The Recorder listens to a topic ```/recorder/recording```, and will record data in a rosbag when this topic is pulled high.

The Recorder creates a single bag each session (life of the node). If the recording is stopped and then resumed, the same bagfile will be appended to rather than a new bag created for each. 

``` rosrun mini_tools recorder.py```

The Recorder is also launched when the ui is launched using the launchfile.

## Extractor
This class extracts all the data from a ROSBAG and merges it into a single dataframe (saved as a .csv).

The class currently supports:
- CompressedImages
- Imu
- NavSatFix (GPS)
More data streams can be easily added to the script.

```python3 extractor.py -b ../data/2021-06-04-12-45-05.bag -d ../data/ ```

-b: path to rosbag 
-d: path to directory to save final dataframe


## GPS
Not Yet Implmented

This node subscribe to a gnss fix topic and publishes live data to a mapview. 
