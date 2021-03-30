/*
Copyright (c) 2021, Institute of Mechatronic Systems, Leibniz University Hannover
All rights reserved.

This source code is licensed under the BSD-style license found in the
LICENSE file in the root directory of this source tree.
*/
/**
 * @file   monitoring_type_conditions.h
 * @author Maevin Stüde, Mohamed Amine Sassi (sassimedamine96@gmail.com)
 * @date   12/2020, 02/2021
 *
 * @brief
 *  Contains basic (simple) condition nodes to check if a certain Monitor is present, given the description element from the
 * monitoring message
 */

#pragma once

#include "behaviortree_cpp_v3/behavior_tree.h"
#include <behaviortree_cpp_v3/condition_node.h>
#include <behaviortree_cpp_v3/action_node.h>
#include "cmr_monitoring/bt_nodes/start_stop_nodes.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "cmr_bt_generic/general.h"


namespace MonitoringNodes
{

/**
 * @brief IsMonitor
 * Function which compares the description from monitoring message with a given monitor type
 * @param self
 * @param mon_type Monitor type as string
 * @return SUCCESS if equal, FAILURE if not
 */
BT::NodeStatus IsMonitor(BT::TreeNode &self, const std::string &mon_type){
  auto description = self.getInput<std::string>("description");

  if(!description.has_value()){
    throw BT::RuntimeError("error reading port [description]:", description.error());
  }
  if (description.value()==mon_type) {
    return BT::NodeStatus::SUCCESS;
  }
  else {
    return BT::NodeStatus::FAILURE;
  }
}
/**
 * @brief NodeMonitor
 * checks if the the monitoring message contains informations from the node monitor
 * @return true if key is node monitor
 */
BT::NodeStatus NodeMonitor(BT::TreeNode &self){
  return IsMonitor(self, "nodemonitor");
}
/**
 * @brief TopicMonitor
 * checks if the the monitoring message contains informations from the topic monitor
 * @return true if key is topic monitor
 */
BT::NodeStatus TopicMonitor(BT::TreeNode &self){
  return IsMonitor(self, "topic_monitor");
}
/**
 * @brief StatMonitor
 * checks if the the monitoring message contains informations from the statistics monitor
 * @return true if key is statistics monitor
 */
BT::NodeStatus StatMonitor(BT::TreeNode &self){
  return IsMonitor(self, "Statistics for ROS Topics");
}
/**
 * @brief NTPMonitor
 * checks if the the monitoring message contains informations from the ntp monitor
 * @return true if key is ntp monitor
 */
BT::NodeStatus NTPMonitor(BT::TreeNode &self){
  return IsMonitor(self, "ntp_monitor");
}
/**
 * @brief RamMonitor
 * checks if the the monitoring message contains informations from the ram monitor
 * @return true if key is ram monitor
 */
BT::NodeStatus RamMonitor(BT::TreeNode &self){
  return IsMonitor(self, "RAM-Monitor");
}
/**
 * @brief CpuMonitor
 * checks if the the monitoring message contains informations from the cpu monitor
 * @return true if key is cpu monitor
 */
BT::NodeStatus CpuMonitor(BT::TreeNode &self){
  return IsMonitor(self, "A CPU-Monitor");
}
/**
 * @brief NwMonitor
 * checks if the the monitoring message contains informations from the network monitor
 * @return true if key is network monitor
 */
BT::NodeStatus NwMonitor(BT::TreeNode &self){
  return IsMonitor(self, "Network-Monitor");
}
/**
 * @brief RTABMapMonitor
 * checks if the the monitoring message contains information from the RTABMAP Monitor
 * @param self
 * @return
 */
BT::NodeStatus RTABMapMonitor(BT::TreeNode &self){
  return IsMonitor(self, "RTABMAP-Monitor");
}
/**
 * @brief RTABMapMonitor
 * checks if the the monitoring message contains information from the MoveBase Monitor
 * @param self
 * @return
 */
BT::NodeStatus MoveBaseMonitor(BT::TreeNode &self){
  return IsMonitor(self, "MoveBase-Monitor");
}

BT::NodeStatus Status(BT::TreeNode &self, std::function<bool(double, std::string, std::string)> err_fun){
  auto error = self.getInput<double>("error_level");
  auto key = self.getInput<std::string>("key");
  auto description = self.getInput<std::string>("description");

  if(!error.has_value()){
    throw BT::RuntimeError("error reading port [target]:", error.error());
  }
  if (err_fun(error.value(), description.value(), key.value())) {
    return BT::NodeStatus::SUCCESS;
  }
  else {
    return BT::NodeStatus::FAILURE;}
}

BT::NodeStatus StatusInfo(BT::TreeNode &self){
  auto err_fun = [](double error, std::string desc, std::string key) -> bool {
    bool is_err = error<0.01;
    return is_err;
  };
  return Status(self, err_fun);
}

BT::NodeStatus StatusWarn(BT::TreeNode &self){
  auto err_fun = [](double error, std::string desc, std::string key) -> bool {
    bool is_err = error>=0.299 && error<=0.501;
    if(is_err) ROS_WARN("Warning detected. %s: %s ", desc.c_str(),key.c_str());
    return is_err;
  };
  return Status(self, err_fun);
}


BT::NodeStatus StatusError(BT::TreeNode &self){
  auto err_fun = [](double error, std::string desc, std::string key) -> bool {
    bool is_err = error>=0.699;
    if(is_err) ROS_ERROR("Error detected. %s: %s ", desc.c_str(),key.c_str());
    return is_err;
  };
  return Status(self,err_fun);
}

/**
 * @brief registerConditions
 * Register the conditions with the Behavior Tree Factory
 * @param factory
 */
inline void registerConditions(BT::BehaviorTreeFactory &factory)
{

  factory.registerSimpleCondition("NodeMonitor", NodeMonitor, {BT::InputPort<std::string>("description")});
  factory.registerSimpleCondition("TopicMonitor", TopicMonitor, {BT::InputPort<std::string>("description")});
  factory.registerSimpleCondition("StatMonitor", StatMonitor, {BT::InputPort<std::string>("description")});
  factory.registerSimpleCondition("NTPMonitor", NTPMonitor, {BT::InputPort<std::string>("description")});
  factory.registerSimpleCondition("RamMonitor", RamMonitor, {BT::InputPort<std::string>("description")});
  factory.registerSimpleCondition("CpuMonitor", CpuMonitor, {BT::InputPort<std::string>("description")});
  factory.registerSimpleCondition("NwMonitor", NwMonitor, {BT::InputPort<std::string>("description")});
  factory.registerSimpleCondition("RTABMapMonitor", RTABMapMonitor, {BT::InputPort<std::string>("description")});
  factory.registerSimpleCondition("MoveBaseMonitor", MoveBaseMonitor, {BT::InputPort<std::string>("description")});

  factory.registerSimpleCondition("StatusInfo", StatusInfo, {BT::InputPort<double>("error_level"),BT::InputPort<std::string>("key"),BT::InputPort<std::string>("description")});
  factory.registerSimpleCondition("StatusWarn", StatusWarn, {BT::InputPort<double>("error_level"),BT::InputPort<std::string>("key"),BT::InputPort<std::string>("description")});
  factory.registerSimpleCondition("StatusError", StatusError, {BT::InputPort<double>("error_level"),BT::InputPort<std::string>("key"),BT::InputPort<std::string>("description")});
}
}

