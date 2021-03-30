/*
Copyright (c) 2021, Institute of Mechatronic Systems, Leibniz University Hannover
All rights reserved.

This source code is licensed under the BSD-style license found in the
LICENSE file in the root directory of this source tree.
*/
/**
 * @file   rtabmap_monitor.h
 * @author Marvin Stüde (Marvin.Stuede@imes.uni-hannover.de)
 * @date   02/2021
 */

#pragma once
#include <ros/ros.h>
#include <monitoring_core/monitor.h>
#include <std_msgs/String.h>
#include <rtabmap_ros/Info.h>
#include <memory>
/**
 * @brief The RTABMAPMonitor class
 * Checks the RTABMAP Info topic for loop id. Errorneous localizations usually lead to a loop ID = 0 which can be used to indicate a localization error
 *
 */
class RTABMAPMonitor
{
public:
    RTABMAPMonitor(ros::NodeHandle &node_handle);

private:
    // node handle
    ros::NodeHandle *node_;

    // ros communication
    ros::Subscriber sub_rt_info_;
    ros::Timer tim_mon_;
    float warn_dur_, err_dur_, loop_id_ = -1;
    std::unique_ptr<Monitor> monitor_;

    // callbacks
    void rtInfoSubCallback(const rtabmap_ros::InfoConstPtr &info_msg);
    void timerCallback(const ros::TimerEvent &evt);
    std::pair<bool,ros::Time> countdown;

};
