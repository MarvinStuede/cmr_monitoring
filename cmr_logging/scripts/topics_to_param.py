#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Read throttled topics and topics and create to param list, containing all topics
import rospy
import yaml
import rospkg
import os
import argparse
import pathlib

r = rospkg.RosPack()
ppath = r.get_path('cmr_logging')
list = rospy.get_param("unthrottled_topics")

parser = argparse.ArgumentParser(description='Path to throttled topics yaml.')
parser.add_argument('path', metavar='ABS_PATH', type=pathlib.Path, nargs='+',
                    help='path to the yaml file with throttled topics')
args = parser.parse_args()

with args.path[0].open() as file:
    fyaml = yaml.safe_load(file)

    def modify_names(field):
        for freq in fyaml[field]:
            # Iterate through topics and append throttle suffix
            list_thr = [topic_thr + '_throttle' for topic_thr in fyaml[field][freq]['topics']]
            list.extend(list_thr)

    modify_names('throttled_topics')
    modify_names('latched_topics')
rospy.set_param("log_topics", list)
