#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Script to launch multiple throttling nodes, which can be used for logging.
# Reads the log_topics_throttled.yaml, given in cfg folder
import rospy
import roslaunch
import yaml
import rospkg
import os
import argparse
import pathlib

list = []
r = rospkg.RosPack()
ppath = r.get_path('cmr_logging')

processes = []


def start_nodes(fyaml, field, launch):
    for freq in fyaml[field]:
        frequency = fyaml[field][freq]['frequency']
        topics = fyaml[field][freq]['topics']
        for topic in topics:
            if field == 'throttled_topics':
                package = 'topic_tools'
                executable = 'throttle'
                node_name = 'throttle_' + topic[1:].replace("/", "_")
                args = "messages " + topic + " " + str(frequency)
                rospy.set_param("/" + node_name + "/lazy", True)
            elif field == 'latched_topics':
                package = 'cmr_logging'
                executable = 'latch_republisher'
                node_name = 'repub_latch_' + topic[1:].replace("/", "_")
                args = topic + " " + str(frequency)
            else:
                rospy.logfatal("Wrong field key")
                return
            node = roslaunch.core.Node(package, executable, name=node_name, args=args,
                                       output="screen", namespace="db_logging")
            launch.launch(node)


def run(path):
    rospy.init_node('log_throttlers', anonymous=True)
    launch = roslaunch.scriptapi.ROSLaunch()
    launch.start()
    # Read yaml file with topics to throttle
    with path.open() as file:
        fyaml = yaml.safe_load(file)
        # Each block contains a frequency and topics
        start_nodes(fyaml, 'throttled_topics', launch)
        start_nodes(fyaml, 'latched_topics', launch)

    rospy.spin()
    launch.stop()


if __name__ == '__main__':
    
    parser = argparse.ArgumentParser(description='Path to throttled topics yaml.')
    parser.add_argument('path', metavar='ABS_PATH', type=pathlib.Path, nargs='+',
                            help='path to the yaml file with throttled topics')
    args = parser.parse_args()
    run(args.path[0])
