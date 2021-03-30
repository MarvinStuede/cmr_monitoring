/*
Copyright (c) 2021, Institute of Mechatronic Systems, Leibniz University Hannover
All rights reserved.

This source code is licensed under the BSD-style license found in the
LICENSE file in the root directory of this source tree.
*/
/**
 * @file   movebase_monitor.h
 * @author Marvin Stüde (Marvin.Stuede@imes.uni-hannover.de)
 * @date   02/2021
 */

#pragma once
#include <ros/ros.h>
#include <monitoring_core/monitor.h>
#include <std_msgs/String.h>
#include <rosgraph_msgs/Log.h>
#include <memory>
#include "geometry_msgs/Twist.h"
#include "move_base_msgs/MoveBaseActionFeedback.h"
/**
 * @brief The MoveBaseMonitor class
 * Monitors move base console output for "has not been updated since"... output which occasionally occurs although all necessary topics
 * are published
 */
class MoveBaseMonitor
{
public:
    MoveBaseMonitor(ros::NodeHandle &node_handle);

private:
    // node handle
    ros::NodeHandle *node_;

    // ros communication
    ros::Subscriber sub_log_;
    ros::Subscriber sub_cmdvel_;
    ros::Subscriber sub_mbfeedback_;

    ros::Timer tim_mon_;
    std::unique_ptr<Monitor> monitor_;

    int warn_dur_not_updated_, err_dur_not_updated_, warn_dur_no_feedback_, err_dur_no_feedback_;
    std::pair<bool,int> not_updated_;
    ros::Time time_last_vel_  = ros::Time(0);
    ros::Time time_last_mbfb_  = ros::Time(0);

    // callbacks
    void logCallback(const rosgraph_msgs::LogConstPtr &log);
    void cmdvelCallback(const geometry_msgs::TwistConstPtr &vel);
    void mbfeedbackCallback(const move_base_msgs::MoveBaseActionFeedbackConstPtr &mbfeedback);
    void timerCallback(const ros::TimerEvent &evt);

    double getErrorValNotUpdated(const std::pair<bool, int> &not_updated);
    double getErrorValNoFeedback(const ros::Time &time_last_fb);


    /**
     * @brief getTimeNotUpdated
     * Reads the time from the log message
     * @param log message
     * @return time as int
     */
    int getTimeNotUpdated(const rosgraph_msgs::LogConstPtr &log);

};
