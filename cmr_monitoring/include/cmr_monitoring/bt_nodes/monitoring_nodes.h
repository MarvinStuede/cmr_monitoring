/*
Copyright (c) 2021, Institute of Mechatronic Systems, Leibniz University Hannover
All rights reserved.

This source code is licensed under the BSD-style license found in the
LICENSE file in the root directory of this source tree.
*/
/**
 * @file   monitoring_nodes.h
 * @author Marvin Stüde (Marvin.Stuede@imes.uni-hannover.de)
 * @date   02/2021
 *
 * @brief  MonitoringNodes
 * contains the implementation of condition and action nodes used to monitor a ROS system
 * based on the information provided by the monitoring package. These Nodes are used to
 * identify a monitoring system error and to process a recovery for each detected problem.
 */

#pragma once

#include "behaviortree_cpp_v3/behavior_tree.h"
#include <behaviortree_cpp_v3/condition_node.h>
#include <behaviortree_cpp_v3/action_node.h>
#include "cmr_monitoring/monitoring_bt.h"
#include <tf/transform_datatypes.h>
#include <stdlib.h>
#include <stdio.h>
#include <memory>
#include <string>
#include <iostream>
#include <algorithm>
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"
#include <fstream>
#include <unistd.h>
#include "roslaunch_axserver/launchAction.h"
#include <roslaunch_axserver/launchGoal.h>
#include "actionlib/client/simple_action_client.h"
#include "actionlib/client/action_client.h"
#include "actionlib/client/terminal_state.h"
#include <actionlib_tutorials/FibonacciAction.h>
#include "cmr_monitoring/launch_tools.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "cmr_monitoring/bt_nodes/start_stop_nodes.h"
#include "monitoring_msgs/SetParameters.h"
#include "cmr_monitoring/bt_nodes/monitoring_conditions.h"
#include "cmr_bt_generic/general.h"
#include <stack>

namespace MonitoringNodes
{
/**
 * @brief The Monitoring class
 * Subscribes to the monitoring topic and selects one KeyValue which it sends to the Blackboard
 * Currently, messages with larger error level are prefered
 * To avoid reprocessing the same error within short time, a map blocked_groups defines which groups are blocked for further processing
 */
class Monitoring : public BT::StatefulActionNode
{
public:
  /**
   * @brief Monitoring constructor
   * @param name
   * @param config
   */
  Monitoring(const std::string& name, const BT::NodeConfiguration& config):
    BT::StatefulActionNode(name, config)
  {
    _node = std::make_shared<ros::NodeHandle>("~");
    _pub = _node->advertise<std_msgs::String>("cmr_monitoring/activity", 10);
    //Structure with mappings between node names, topics, groups, launch file names etc.
    _maps = std::make_unique<StartStopNodes::GroupMaps>(_node);
    subscribe();
  }

  static BT::PortsList providedPorts(){
    return {
          BT::OutputPort<double>("error_level"),
          BT::OutputPort<std::string>("key"),
          BT::OutputPort<std::string>("description"),
          BT::BidirectionalPort<StartStopNodes::stt>("blocked_groups")};
  }

  BT::NodeStatus onStart() override {
    _t_start = ros::Time::now();
    return BT::NodeStatus::RUNNING;
  }
  /**
   * @brief onRunning
   * @return BT::NodeStatus::RUNNING if waiting for monitoring message,
   * BT::NodeStatus::SUCCESS if monitoring message is received
   */
  BT::NodeStatus onRunning() override;
  /**
   * @brief setBlockedFilter
   * Retrieves a map from blackboard containing node names and time stamps.
   * Stamps are compared with delay after a launch file to filter out errors corresponding to groups, which are blocked
   * This is used to avoid redecting of errors, when a launch file was started recently
   */
  void setBlockedFilter();
  void onHalted() override{
  }

private:
  struct MonitoringValues{
    std::string description;
    monitoring_msgs::KeyValue msg;
  };

  bool _status_found = false, _reset_done = false;
  ros::Time _t_start;
  std::deque<MonitoringValues> _mon_aggregated;
  std::mutex _agg_mutex;


  std::string _description;

  std::string _key;
  StartStopNodes::stt _blocked_groups;
  std::string value;
  std::string unit;
  double _level;
  std::shared_ptr<ros::NodeHandle> _node;
  ros::Subscriber _sub;
  ros::Publisher _pub;
  std::unique_ptr<StartStopNodes::GroupMaps> _maps;

  void subscribe(){
    _sub = _node->subscribe("/monitoring",10, &Monitoring::subCallback,this);
  }
  void unsubscribe(){
    _sub.shutdown();
  }
  void reset();

  MonitoringValues getMostRelevantMonitoringVal();

  void subCallback(const monitoring_msgs::MonitoringArray::ConstPtr& msg);
};

/**
 * @brief The NodeParamBase class
 * Base class of a BT::SyncActionNode with a node handle
 */
class NodeParamBase : public BT::SyncActionNode
{
public:
  NodeParamBase(const std::string& name, const BT::NodeConfiguration& config):
    BT::SyncActionNode(name, config){
    _node = std::make_shared<ros::NodeHandle>("~");
  }
protected:
  std::shared_ptr<ros::NodeHandle> _node;
  std::string _param_key, _param_stamp, _param_desc;

  void stringsFromKey(std::string key){
    _param_key= "keys"+key;
    _param_stamp = _param_key + "/stamp";
    _param_desc = _param_key + "/desc";
  }

};

/**
 * @brief The StatusToParamServer class
 * Node to write a Monitoring description with time stamp to parameter server
 * Can be used to check when an error occurred
 */
class StatusToParamServer : public NodeParamBase
{
public:

  StatusToParamServer(const std::string& name, const BT::NodeConfiguration& config):
    NodeParamBase(name, config)
  {}
  static BT::PortsList providedPorts(){
    return {
      BT::InputPort<bool>("rewrite_stamp"),
          BT::InputPort<std::string>("key"),
          BT::InputPort<std::string>("description")
    };
  }

private:
  BT::NodeStatus tick() override;

};

/**
 * @brief The DeleteStatus class
 * Delete a status from parameter server based on a key
 */
class DeleteStatus : public NodeParamBase
{
public:

  DeleteStatus(const std::string& name, const BT::NodeConfiguration& config):
    NodeParamBase(name, config)
  {}
  static BT::PortsList providedPorts(){
    return {
      BT::InputPort<std::string>("key"),
    };
  }

private:
  BT::NodeStatus tick() override;

};
/**
 * @brief The ErrorIsNew class
 * COndition to check if an error is new (ie. not older than a defined parameter).
 * Can be used in combination with StatusToParamServer
 *
 */
class ErrorIsNew : public NodeParamBase
{
public:

  ErrorIsNew(const std::string& name, const BT::NodeConfiguration& config):
    NodeParamBase(name, config)
  {}
  static BT::PortsList providedPorts(){
    return {
      BT::InputPort<std::string>("key"),
          BT::InputPort<std::string>("description")
    };
  }

private:
  BT::NodeStatus tick() override;

};
/**
 * @brief The ReconfigureMonitoring class
 * Node to reconfigure parameters of the Monitoring package based on a parameter set.
 * Each parameter set indicates e.g. nodes that should be monitored
 * Currently implemented:
 * OPERATING: Robot is in normal operation, with sensors, localization, navigation, etc.
 * CHARGING: Robot is charging. Only basic nodes for charging are needed (e.g. base, battery monitoring)
 */
class ReconfigureMonitoring : public NodeParamBase
{
public:

  ReconfigureMonitoring(const std::string& name, const BT::NodeConfiguration& config):
    NodeParamBase(name, config)
  {
    _srv_client_node = _node->serviceClient<monitoring_msgs::SetParameters>("/monitors/ros/node_monitor/set_parameters");
  }
  static BT::PortsList providedPorts(){
    return {
      BT::InputPort<std::string>("parameter_set"), //Parameter set input (e.g. written in Groot)
          BT::OutputPort<std::string>("parameter_set_out"),
    };
  }

private:
  BT::NodeStatus tick() override;
  ros::ServiceClient _srv_client_node;


};

/**
 * @brief The SetKillGroups class
 * Based on the parameter set, define which groups should be killed.
 * This can be used to kill nodes that were present in a different parameter set but are not needed
 * in a new parameter set
 */
class SetKillGroups : public BT::CoroActionNode
{
public:

  SetKillGroups(const std::string& name, const BT::NodeConfiguration& config):
    BT::CoroActionNode(name, config)
  {
    _node = std::make_shared<ros::NodeHandle>("~");
    _maps  = std::make_unique<StartStopNodes::GroupMaps>(_node);

  }

  static BT::PortsList providedPorts(){
    return { BT::InputPort<std::string>("parameter_set"),
          BT::OutputPort<std::string>("curr_kill_group")};
  }

  void halt() override{};
  BT::NodeStatus tick() override;

private:

  std::set<std::string> groupSetFromNodes(const std::vector<std::string> &nodes);
  std::unique_ptr<StartStopNodes::GroupMaps> _maps;
  std::shared_ptr<ros::NodeHandle> _node;
  std::set<std::string> _kill_groups;
  bool _configured = false;

};
/**
 * @brief The MonitoringReconfigured class
 * Condition to check if monitors were reconfigured already.
 * This is indicated by a parameter on the parameter server which is written as soon as a reconfiguation was made
 */
class MonitoringReconfigured : public BT::ConditionNode
{

public:
  MonitoringReconfigured(const std::string& name)
    : MonitoringReconfigured::ConditionNode(name, {})
  {
    node_ = std::make_unique<ros::NodeHandle>();
  }

  BT::NodeStatus tick() override{
    std::string param = "/monitors/ros/node_monitor/params_set";
    if(!node_->hasParam(param)){
      ROS_ERROR_STREAM("Parameter '"<<param<<"' does not exist.");
      return BT::NodeStatus::FAILURE;
    }
    bool param_set = false;
    node_->getParam(param, param_set);
    if(param_set) return BT::NodeStatus::SUCCESS;
    else return BT::NodeStatus::FAILURE;
  };

private:
  std::unique_ptr<ros::NodeHandle> node_;

};
/**
 * @brief RotateToRelocalize
 * publishes a messages that is received by monitored navigation
 * Monitored navigation will then rotate the robot to detect a loop closure
 */
class RotateToRelocalize : public cmr_bt::PublishMsg<std_msgs::Bool>
{
public:
  RotateToRelocalize(const std::string &name)
    : cmr_bt::PublishMsg<std_msgs::Bool>(name, "/monitor/lost_localization")
  {
    _msg.data = false;
  }
  private:
  BT::NodeStatus tick() override{
    _msg.data = true;
    publish(_msg);
    return BT::NodeStatus::SUCCESS;
  };
    std_msgs::Bool _msg;
};

/**
 * @brief registerNodes
 * Register all monitoring nodes and conditions with the Behavior Tree Factory
 * @param factory
 */
inline void registerNodes(BT::BehaviorTreeFactory &factory)
{
  factory.registerNodeType<Monitoring>("Monitoring");
  factory.registerNodeType<StatusToParamServer>("StatusToParamServer");
  factory.registerNodeType<ErrorIsNew>("ErrorIsNew");
  factory.registerNodeType<ReconfigureMonitoring>("ReconfigureMonitoring");
  factory.registerNodeType<DeleteStatus>("DeleteStatus");
  factory.registerNodeType<SetKillGroups>("SetKillGroups");
  factory.registerNodeType<MonitoringReconfigured>("MonitoringReconfigured");
  factory.registerNodeType<RotateToRelocalize>("RotateToRelocalize");
}
}
