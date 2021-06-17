#!/usr/bin/env python3

import argparse
import os
import glob
from pprint import pprint

import cv_bridge
import pandas as pd
from PIL import Image as PILImage
import rosbag
import yaml
from sensor_msgs.msg import CompressedImage, Imu, NavSatFix
from tqdm import tqdm


def interpolate_data(df, method="pad"):
    """Helper for interpolating data for pandas dataframe"""

    # interpolate missing data (forward fill pad)
    interpolated_columns = [col for col in df.columns if col not in ["field.header.stamp", "time"]]

    # TODO: there has got to be a better way to do this at the dataframe level
    for col in interpolated_columns:
        df[col] = df[col].interpolate(method=method)

    return df


def merge_data_into_single_dataframe(base_dir):
    """Merge all data into a single dataframe with interpolatation between timestamps for missing data"""

    # convert img to list of fname and timestamps
    img_filenames = glob.glob(base_dir + "*.png")
    img_filenames = [
        fname.split("/")[-1] for fname in img_filenames
    ]  # remove the base_dir from path
    img_timestamp = [
        int(fname.split(".")[0]) for fname in img_filenames
    ]  # remove file ext from timestamp

    # prepare pandas dataframe for each data type
    df_img = pd.DataFrame(
        list(zip(img_timestamp, img_filenames)), columns=["field.header.stamp", "img"]
    )


    df_to_concat = [df_img]

    # loop through each .csv file in directory to aggregate
    for fname in glob.glob(base_dir + "*.csv"):
        if "data.csv" not in fname: # dont include the aggregate if already saved    
            print(f"Appending from: {fname}")
            df = pd.read_csv(fname)
            df_to_concat.append(df)

    # concatenate each data source together
    df_concat = pd.concat(df_to_concat)

    df_concat["time"] = pd.to_datetime(
        df_concat["field.header.stamp"]
    )  # reindex frame according to timestamp
    df_concat_sorted = df_concat.sort_values(by=["time"])  # sort according to timestamp
    df_concat_sorted.index = df_concat_sorted["time"]
    del df_concat_sorted["time"]

    # interpolate missing data (forward fill pad)
    df_concat_sorted = interpolate_data(df_concat_sorted)

    # NOTE: Should NANs be filled with zero?
    # Choosing not to, and letting down stream handle it.
    # they might want to consider nan = zero, or nan = no data. Leaving the choice to them
    # TODO: manual_override column should be dealt with here.

    # save full dataset back to csv
    df_concat_sorted.to_csv(base_dir + "data.csv")
    print(f"DataFrame saved as: {base_dir}data.csv ({len(df_concat_sorted)} rows)")

    return df_concat_sorted


def extract_data_from_rosbag(bag, config):
    """ Extract all topics from a ROSBAG into images and csv files"""

    # topics to extract from the config file
    extraction_topics = config["extract"]["topics"]

    # load the bag info
    bag_info = yaml.safe_load(bag._get_yaml_info())
    print("ROSBAG Info:")
    pprint(bag_info)
    print("\n")

    # cv2 bridge
    bridge = cv_bridge.CvBridge()

    # extract each topic in the bag
    for topic in bag_info["topics"]:

        topic_name = topic["topic"]
        n_messages = topic["messages"]
        topic_type = topic["type"]

        print(f"Extracting {topic_name} ({topic_type})")
        if topic_type == "sensor_msgs/CompressedImage":

            for t, msg, a in tqdm(
                bag.read_messages(topics=topic_name), total=n_messages
            ):

                # convert from rosmsg to np.ndarray
                img = bridge.compressed_imgmsg_to_cv2(msg)

                img = img[:, :, ::-1]  # bgr to rgb # TODO: check for alpha channel?

                # save image (fname = timestamp)
                fname = str(msg.header.stamp) + ".png"
                PILImage.fromarray(img).save(base_dir + fname)

        else:

            # extract other topics to csv using cmd line tools
            topic_base_name = "".join(topic_name.split("/")[1:])
            csv_filename = base_dir + topic_base_name + ".csv"
            ret = os.system(
                f"rostopic echo -b {filename} -p {topic_name} > {csv_filename}"
            )


if __name__ == "__main__":

    print("Extractor runnning")

    # parase cmd line arguments
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
    parser.add_argument(
        "-c",
        "--config",
        dest="config",
        action="store",
        default="config.yaml",
        type=str,
        help="Configuration File",
    )    
    args = parser.parse_args()

    filename = args.bagfile
    config = yaml.safe_load(open(args.config))

    # create folder for bag data
    basename = filename.split("/")[-1].split(".")[0]  # basename
    base_dir = args.data_path + basename + "/"
    os.makedirs(base_dir, exist_ok=True)

    # load bag
    bag = rosbag.Bag(filename, mode="r")

    # extract topic data from rosbag into filesystem
    extract_data_from_rosbag(bag, config)

    # Consolidate data into a single dataset for each ROSBAG
    df_concat_sorted = merge_data_into_single_dataframe(base_dir=base_dir)


# TODO: allow selection of topics / etc from a config.yaml file
# TODO: change the data consolidate to a merge? Requires data to be synchronised well / coordinated
# TODO: add support for multiple cameras (prefix img with cam?)
# TODO: add support for multiple vehicles (prefix img with vno?)
