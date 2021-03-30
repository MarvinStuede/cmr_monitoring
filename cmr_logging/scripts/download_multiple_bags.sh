#!/bin/bash
# Download logging data to bag files per day
running=false
session=db_logging

declare -a dates=("01/03/21" "02/03/21" "03/03/21" "04/03/21" "05/03/21" "08/03/21" "09/03/21")


for date in "${dates[@]}"; do
	roslaunch cmr_logging mongodb_to_rosbag.launch start:="$date 0:00" end:="$date 23:59" name_prefix:="lta_eval"
done
exit 0
