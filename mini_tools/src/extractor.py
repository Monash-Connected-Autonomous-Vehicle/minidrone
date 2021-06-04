#!/usr/bin/env python3

import argparse
import os
import glob
from pprint import pprint

import cv_bridge
import matplotlib.pyplot as plt
import pandas as pd
import PIL
import rosbag
import rospy
import yaml
from sensor_msgs.msg import CompressedImage, Imu, NavSatFix
from tqdm import tqdm

if __name__ == "__main__":

    print("Extractor runnning")

    parser = argparse.ArgumentParser(description="Extract image data from rosbag")
    parser.add_argument(
        "-b",
        "--bagfile",
        dest="bagfile",
        action="store",
        required=True,
        type=str,
        help="ROSBAG to extract data from",
    )
    parser.add_argument(
        "-d",
        "--destination",
        dest="data_path",
        action="store",
        required=True,
        type=str,
        help="Directory to save data in",
    )
    args = parser.parse_args()
    # TODO: set a folder for each rosbag data

    filename = args.bagfile

    # create folder for bag data
    basename = filename.split("/")[-1].split(".")[0]  # basename
    base_dir = args.data_path + basename + "/"
    os.makedirs(base_dir, exist_ok=True)

    # load bag
    bag = rosbag.Bag(filename, mode="r")
    bag_info = yaml.safe_load(bag._get_yaml_info())
    print("ROSBAG Info:")
    pprint(bag_info)

    # cv2 bridge
    bridge = cv_bridge.CvBridge()

    for topic in bag_info["topics"]:

        topic_name = topic["topic"]
        n_messages = topic["messages"]
        topic_type = topic["type"]

        print(f"Extracting {topic_name} ({topic_type})")
        if topic_type == "sensor_msgs/CompressedImage":

            for t, m, a in tqdm(bag.read_messages(topics=topic_name), total=n_messages):

                fname = str(m.header.stamp) + ".png"

                img = bridge.compressed_imgmsg_to_cv2(m)

                img = img[:, :, ::-1]  # bgr to rgb # TODO: check for alpha channel?

                # save image (fname = timestamp)
                # PIL.Image.fromarray(img).save(base_dir + fname)

        else:
            # extract other topics to csv using cmd line tools
            topic_base_name = "".join(topic_name.split("/")[1:])
            csv_filename = base_dir + topic_base_name + ".csv"
            ret = os.system(
                f"rostopic echo -b {filename} -p {topic_name} > {csv_filename}"
            )
    



    # convert img to list of fname and timestamps
    img_filenames = glob.glob(base_dir + "*.png")
    img_filenames = [fname.split("/")[-1] for fname in img_filenames] # remove the base_dir from path
    img_timestamp = [int(fname.split(".")[0]) for fname in img_filenames] # remove file ext from timestamp


    # prepare pandas dataframe for each data type
    df_img = pd.DataFrame(list(zip(img_timestamp, img_filenames)), columns =['%time', 'img'])
    df_imu = pd.read_csv(base_dir + "imu.csv")
    df_gps = pd.read_csv(base_dir + "gps.csv")

    # concatenate each data source together
    df_cat = pd.DataFrame()
    df_cat = pd.concat([df_img, df_imu, df_gps])
    
    df_cat['time'] = pd.to_datetime(df_cat['%time'])
    df_cat_sorted = df_cat.sort_values(by=["time"]) # sort according to timestamp
    df_cat_sorted.index = df_cat_sorted['time']
    del df_cat_sorted["time"]


    # TODO: interpolate the values in between
    # df_cat_sorted = df_cat_sorted.interpolate(method="pad") # TODO: might have to handle this as time series?  # interpolate (fill forward)


    print(df_cat_sorted.columns)
    print(df_cat_sorted.head(10))

    # save full dataset back to csv
    df_cat_sorted.to_csv(base_dir + "data.csv")


# TODO: add support for multiple cameras (prefix img with cam?)
# TODO: add support for multiple vehicles (prefix img with vno?)
