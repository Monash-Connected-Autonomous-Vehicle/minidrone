#!/usr/bin/env python3

import re

import roslaunch
import rospy
import rosnode

def get_camera_urls():
    """ return the url for viewing compressed image stream over http
    assumes compressed image stream
    
    """

    published_topics = rospy.get_published_topics()
    published_topic_names = [x[0] for x in published_topics]
    published_topic_types = [x[1] for x in published_topics]
    #\/jetbot_camera\/.*
    r = re.compile(".*\/compressed$")
    compressed_camera_topics = list(filter(r.match, published_topic_names))

    cam_urls = []
    for cam_topic in compressed_camera_topics:

        cam_topic = "/".join(cam_topic.split("/")[:-1])
        cam_url = f"http://localhost:8080/stream?topic={cam_topic}&type=ros_compressed&width=360&height=480"

        cam_urls.append(cam_url)

    return cam_urls

def launch_node(launch_file):
    # roslaunch
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    launch = roslaunch.parent.ROSLaunchParent(uuid, [launch_file])
    launch.start()

    return launch