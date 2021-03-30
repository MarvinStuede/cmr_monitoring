
#Copyright (c) 2021, Institute of Mechatronic Systems, Leibniz University Hannover
#All rights reserved.

#This source code is licensed under the BSD-style license found in the
#LICENSE file in the root directory of this source tree.
#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Util script to extract the aggregated monitored values from a bag file
# Creates separate topics for each key with error code and value
from __future__ import print_function

import rospy

from tqdm import tqdm
import sys
import rosbag
import signal
import argparse
import platform
import pathlib
import os
from copy import copy
from std_msgs.msg import Float32
from shutil import copyfile


def remove_slash(s):
    # Helper to remove leading slash for topic names
    if (s[0] == "/"):
        return s[1:]
    else:
        return (s)

def isfloat(value):
    # Helper to check if string can be converted to float
    try:
        float(value)
        return True
    except ValueError:
        return False

def main(argv):
    myargv = rospy.myargv(argv=argv)

    parser = argparse.ArgumentParser(description='Path to bagfile')
    parser.add_argument('path', metavar='ABS_PATH', type=pathlib.Path, nargs='+',
                        help='path to the bag file')
    args = parser.parse_args()

    # Get the absolute path
    strpath=str(args.path[0].resolve())

    # Create a backup
    copyfile(strpath, strpath[:-4]+".orig.bag")

    strpath_temp = strpath+".temp"
    bag = rosbag.Bag(strpath)
    # Create a temporary bag file for extracted messages
    bag_out = rosbag.Bag(strpath_temp, 'w')
    # Define a prefix for error level topics
    topic_pre = "/monitoring/"
    msg_o = Float32()

    # Get total number of messages for progress bar
    info = bag.get_type_and_topic_info(['/monitoring'])
    try:
        pbar = tqdm(total=info.topics['/monitoring'].message_count)
    except KeyError:
        print("Monitoring topic not in bag, quitting...")
        exit(0)

    # Write the error levels to separate topics
    for topic, msg, t in bag.read_messages(topics=['/monitoring']):
        for info_el in msg.info:
            for value in info_el.values:
                # Create a topic for each key
                topic_err = topic_pre + remove_slash(value.key) + "/error_code"
                msg_o.data = value.errorlevel
                bag_out.write(topic_err, msg_o, t)

                if isfloat(value.value):
                    topic_value = topic_pre + remove_slash(value.key) + "/value"
                    msg_o.data = float(value.value)
                    bag_out.write(topic_value, msg_o, t)
        pbar.update(1)
    bag_out.close()
    bag.close()
    pbar.close()
    print("Messages extracted")
    print("Saving bags...")
    # Reopen bags to recopy the messages to original bag file
    bag_out = rosbag.Bag(strpath, 'a')
    bag = rosbag.Bag(strpath_temp)
    # Recopy messages
    for topic, msg, t in bag.read_messages():
        bag_out.write(topic, msg, t)

    bag_out.close()
    bag.close()
    # Remove the temporary bag file
    os.remove(strpath_temp)
    print("Bag saved")



if __name__ == "__main__":
    main(sys.argv)
