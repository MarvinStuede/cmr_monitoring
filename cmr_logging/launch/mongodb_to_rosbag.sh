#!/bin/bash
rosrun mongodb_store mongodb_to_rosbag.py --mongodb-name=$1 -s "$2" -e "$3" $4