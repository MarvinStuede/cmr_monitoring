/*
Copyright (c) 2021, Institute of Mechatronic Systems, Leibniz University Hannover
All rights reserved.

This source code is licensed under the BSD-style license found in the
LICENSE file in the root directory of this source tree.
*/
/**
 * @file   monitoring_bt.h
 * @author Marvin Stüde (Marvin.Stuede@imes.uni-hannover.de)
 * @date   01/2021
 *
 * @brief  The Monitoring BT is the implementation of the monitoring interface
 * as behavior tree. It allows us to detect software anomalies in
 * a ROS-System and try to solve each problem with a recovery strategy.
 */

#pragma once

#include "ros/ros.h"
#include <ros/package.h>
#include "std_msgs/String.h"
#include "std_srvs/Empty.h"

#include <iostream>
#include <vector>


class MonitoringBT
{
public:
    MonitoringBT(ros::NodeHandle &node_handle);

private:
    // node handle
    ros::NodeHandle *node_;

};


