/*
Copyright (c) 2021, Institute of Mechatronic Systems, Leibniz University Hannover
All rights reserved.

This source code is licensed under the BSD-style license found in the
LICENSE file in the root directory of this source tree.
*/
/**
 * @file   launch_tools.h
 * @author Mohamed Amine Sassi (sassimedamine96@gmail.com)
 * @date   12/2020
 *
 * @brief  LaunchTools
 * contains tools that are used to kill, start and restart nodes
 */


#ifndef LAUNCH_TOOLS_H
#define LAUNCH_TOOLS_H

#include "ros/ros.h"
#include <ros/package.h>
#include "monitoring_msgs/MonitoringArray.h"
#include <tf/transform_datatypes.h>
#include <stdlib.h>
#include <stdio.h>
#include <memory>
#include <string>
#include <iostream>
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"
#include "std_srvs/Empty.h"
#include <fstream>
#include <unistd.h>
#include "roslaunch_axserver/launchAction.h"
#include <roslaunch_axserver/launchGoal.h>
#include "actionlib/client/simple_action_client.h"
#include "actionlib/client/action_client.h"
#include "actionlib/client/terminal_state.h"

/**
 * @brief exec
 * Execute a system command
 * @param cmd command to execute
 * @return stdcout as string
 */
std::string exec(std::string cmd);
/**
 * @brief killnode
 * Kill a ROS node
 * @param node name as string
 * @return true if node was killed, false if not not killed or not running
 */
bool killnode(std::string node);
/**
 * @brief getBaseNodeName
 * Get the uppermost namespace (e.g. sobi in /sobi/speech/goal)
 * @param full_name
 * @return base name
 */
std::string getBaseNodeName(std::string full_name);
/**
 * @brief complyString
 * Convert a String to ros namespace format by replacing "-" and adding a leading "/" if not there
 * @param str
 * @return
 */
std::string complyString(std::string str);


#endif
