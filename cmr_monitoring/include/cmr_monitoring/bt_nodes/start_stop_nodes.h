/*
Copyright (c) 2021, Institute of Mechatronic Systems, Leibniz University Hannover
All rights reserved.

This source code is licensed under the BSD-style license found in the
LICENSE file in the root directory of this source tree.
*/
/**
 * @file   system_nodes.h
 * @author Marvin Stüde (Marvin.Stuede@imes.uni-hannover.de)
 * @date   02/2021
 *
 * @brief  StartStopNodes
 * Contains conditions and action nodes to start and kill nodes as well as relations between node names, launch file names etc.
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
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"
#include <fstream>
#include <unistd.h>
#include "roslaunch_axserver/launchAction.h"
#include <roslaunch_axserver/launchGoal.h>
#include "actionlib/client/simple_action_client.h"
#include "actionlib/client/action_client.h"
#include "actionlib/client/terminal_state.h"
#include "cmr_monitoring/launch_tools.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "cmr_bt_generic/general.h"

namespace StartStopNodes{
//Typedefinitions for maps to map topic names, node names, launch file names etc.
using sts_map = std::map<std::string,std::string>;
using sti_map = std::map<std::string,int>;
using sts = std::pair<std::string,std::string>;
using sti = std::pair<std::string,int>;
using ntl_map = std::map<std::string,sts>;
using ntl = std::pair<std::string,sts>;
using stt = std::map<std::string, ros::Time>;



/**
 * @brief The GroupMaps struct
 * Contains maps which define relationships of topics, nodes, launch files etc
 * Most important property is a "group" which mostly corresponds to a launch file that can contain multiple nodes
 * Therefore there are the following relationships:
 * - Each node maps to one, and only one, group
 * - Each topic maps to one, and only one, group
 * - Each group maps to one, and only one, kill node
 * - Each group maps to one, and only one, launch file
 * - Each launch file maps to one, and only one, delay (in seconds) which determines the time before errors of this group are handled again.
 * A group can contain multiple nodes. As soon as one of these nodes does not respond, the launch file corresponding to the group is killed and then relaunched
 */
struct GroupMaps{
  GroupMaps(const std::shared_ptr<ros::NodeHandle> node){
    this->node = node;

    ros::Rate r(ros::Duration(0.1));
    while(ros::ok() && !node->hasParam("map_topic_to_group"))
      r.sleep();

    this->top_to_group = params_to_map<sts_map,sts>("map_topic_to_group",
                                                   [](XmlRpc::XmlRpcValue v) -> sts {return std::make_pair(v["topic"],v["group"]);});
    this->group_to_kill = params_to_map<sts_map,sts>("map_group_to_kill",
                                                    [](XmlRpc::XmlRpcValue v) -> sts {return std::make_pair(v["group"],v["kill_node"]);});
    this->nsnode_to_group = params_to_map<sts_map,sts>("map_node_to_group",
                                                    [](XmlRpc::XmlRpcValue v) -> sts {return std::make_pair(v["node"],v["group"]);});
    this->launch_to_delay = params_to_map<sti_map,sti>("map_launch_to_delay",
                                                    [](XmlRpc::XmlRpcValue v) -> sti {return std::make_pair(v["launch_file"],v["delay"]);});
    this->group_to_launch = params_to_map<ntl_map,ntl>("map_group_to_launch",
                                                      [](XmlRpc::XmlRpcValue v) -> ntl {return std::make_pair(v["group"],std::make_pair(v["pkg"], v["launch_file"]));});

    //Iterate through map. Groups with same name as node can be given as "_" in config file.
    //Here, the '_' is replaced by the node name without leading '/'
    for(auto iter = nsnode_to_group.begin(); iter != nsnode_to_group.end(); ++iter)
    {
      auto key =  iter->first;
      if(iter->second == "_"){
        iter->second = key.substr(1,key.length()-1);
      }

    }
  }
  /**
   *@brief params_to_map function
   * Used to read the mappings as lists from parameter server
   * @param param_name is the name on the ros paramter server
   * @param list_fun is a function which returns a pair that is describes the maps key/value relationship
   */
  template<typename Map, typename Pair> Map params_to_map(const std::string &param_name,
                                                          std::function<Pair(XmlRpc::XmlRpcValue)> list_fun){

    XmlRpc::XmlRpcValue param_list;
    Map map;

    if(!node->hasParam(param_name)) throw std::runtime_error("Param " + param_name + " not set");

    node->getParam(param_name, param_list);

    ROS_ASSERT(param_list.getType() == XmlRpc::XmlRpcValue::TypeArray);

    for (int32_t i = 0; i < param_list.size(); ++i)
    {
      //ROS_ASSERT(launch_file_list[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
      XmlRpc::XmlRpcValue sublist = param_list[i];
      map.insert(list_fun(sublist));

    }
    return map;
  }
  /**
   * @brief groupByKey
   * Get the group name by a key value
   * @param key
   * @return group name as string
   */
  std::string groupByKey(const std::string &key){
    if(this->nsnode_to_group.find(key) == this->nsnode_to_group.end()){
      std::stringstream ss;
      ss<<"No mapping to group found for key '"<<key<<"'";
      throw BT::RuntimeError(ss.str());
    }
    else return this->nsnode_to_group.at(key);
  }
  sts_map top_to_group;
  sts_map group_to_kill;
  sts_map nsnode_to_group;
  sti_map launch_to_delay;
  ntl_map group_to_launch;

  std::shared_ptr<ros::NodeHandle> node;
};
/**
 * @brief The StartLaunchFile class
 * Action to start a launch file given by "pkg" and "launch_file" Blackboard values
 * Additionally, the group is provided via "group_block" so that the group os added to the "blocked_groups" when the launch file was started
 */
class StartLaunchFile : public BT::CoroActionNode
{
public:

  StartLaunchFile(const std::string& name, const BT::NodeConfiguration& config):
    BT::CoroActionNode(name, config),
    _client("/roslaunch_server", true)
  {
    _node = std::make_shared<ros::NodeHandle>();
  }

  static BT::PortsList providedPorts(){
    return {
      BT::InputPort<std::string>("pkg"),
          BT::InputPort<std::string>("launch_file"),
          BT::InputPort<std::string>("group_block"),
          BT::BidirectionalPort<stt>("blocked_groups")
    };
  }

private:


  BT::NodeStatus tick() override;

  void halt() override
  {
    _aborted = true;
    CoroActionNode::halt();
  }

  bool _aborted = false, _sent = false;
  std::shared_ptr<ros::NodeHandle> _node;
  actionlib::SimpleActionClient<roslaunch_axserver::launchAction> _client;
  roslaunch_axserver::launchGoal _goal;
  roslaunch_axserver::launchResultConstPtr _result;
};

/**
 * @brief KillROSNode
 * SimpleActionNode to kill a ros node given by the inputport "node"
 * @param self
 * @return
 */
BT::NodeStatus KillROSNode(BT::TreeNode &self);

/**
 * @brief The NodeMapBase class
 * Base class with a ROS node handle and the maps
 */
class NodeMapBase : public BT::SyncActionNode
{
public:
  NodeMapBase(const std::string& name, const BT::NodeConfiguration& config):
  BT::SyncActionNode(name, config){
    _node = std::make_shared<ros::NodeHandle>("~");
    _maps = std::make_unique<GroupMaps>(_node);
  }
protected:
  std::shared_ptr<ros::NodeHandle> _node;
  std::unique_ptr<GroupMaps> _maps;

  std::string receiveGroupName();

  //virtual BT::NodeStatus tick() override;


};

/**
 * @brief The LaunchFileFromGroup class
 * Based on the group, get the corresponding launch file
 */
class LaunchFileFromGroup :  public NodeMapBase
{
public:

  LaunchFileFromGroup(const std::string& name, const BT::NodeConfiguration& config):
    NodeMapBase(name, config)
  {}
  static BT::PortsList providedPorts(){
    return {
      BT::InputPort<std::string>("group"),
          BT::OutputPort<std::string>("pkg"),
          BT::OutputPort<std::string>("launch_file")
    };
  }

private:
  BT::NodeStatus tick() override;
};

/**
 * @brief The GroupFromTopic class
 * Get the group, where a topic belongs to
 */
class GroupFromTopic : public NodeMapBase
{
public:

  GroupFromTopic(const std::string& name, const BT::NodeConfiguration& config):
    NodeMapBase(name, config)
  {}
  static BT::PortsList providedPorts(){
    return {
          BT::InputPort<std::string>("topic"),
          BT::OutputPort<std::string>("group")
    };
  }

private:
  BT::NodeStatus tick() override;
};
/**
 * @brief The KillNodeFromGroup class
 * Get the kill node of a specific group
 */
class KillNodeFromGroup : public NodeMapBase
{
public:

  KillNodeFromGroup(const std::string& name, const BT::NodeConfiguration& config):
    NodeMapBase(name, config)
  {}
  static BT::PortsList providedPorts(){
    return {
          BT::InputPort<std::string>("group"),
          BT::OutputPort<std::string>("kill_node")
    };
  }

private:
  BT::NodeStatus tick() override;
};
/**
 * @brief The GroupFromKey class
 * From a Monitoring Key get the group
 */
class GroupFromKey : public NodeMapBase
{
public:

  GroupFromKey(const std::string& name, const BT::NodeConfiguration& config):
    NodeMapBase(name, config)
  {}
  static BT::PortsList providedPorts(){
    return {
          BT::InputPort<std::string>("key"),
          BT::OutputPort<std::string>("group")
    };
  }

private:
  BT::NodeStatus tick() override;
};

class SetDelayAfterLaunch : public NodeMapBase
{
public:

  SetDelayAfterLaunch(const std::string& name, const BT::NodeConfiguration& config):
    NodeMapBase(name, config)
  {}
  static BT::PortsList providedPorts(){
    return {
          BT::InputPort<std::string>("launch_file"),
          BT::OutputPort<unsigned int>("delay")
    };
  }

private:
  BT::NodeStatus tick() override;
};
/**
 * @brief The GroupNotBlocked class
 * Condition to check if a group is currently blocked
 */
class GroupNotBlocked : public BT::ConditionNode
{

public:
  GroupNotBlocked(const std::string& name, const BT::NodeConfiguration& config)
    : GroupNotBlocked::ConditionNode(name, config)
  {
    node_ = std::make_shared<ros::NodeHandle>("~");
    maps_ = std::make_unique<GroupMaps>(node_);
  }

  static BT::PortsList providedPorts(){
    return {
          BT::InputPort<std::string>("group"),
          BT::InputPort<stt>("blocked_groups")
    };
  }
  BT::NodeStatus tick() override;

private:
  std::shared_ptr<ros::NodeHandle> node_;
  std::unique_ptr<GroupMaps> maps_;

};
/**
 * @brief registerSystem
 * Register the StartStopNodes
 * @param factory
 */
inline void registerStartStop(BT::BehaviorTreeFactory& factory)
{

  factory.registerNodeType<StartLaunchFile>("StartLaunchFile");
  factory.registerNodeType<LaunchFileFromGroup>("LaunchFileFromGroup");
  factory.registerNodeType<KillNodeFromGroup>("KillNodeFromGroup");
  factory.registerNodeType<GroupFromTopic>("GroupFromTopic");
  factory.registerNodeType<GroupFromKey>("GroupFromKey");
  factory.registerNodeType<GroupNotBlocked>("GroupNotBlocked");
  factory.registerSimpleAction("KillROSNode", StartStopNodes::KillROSNode,{BT::InputPort<std::string>("node")});
}

}

