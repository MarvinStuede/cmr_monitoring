#include "cmr_monitoring/bt_nodes/monitoring_nodes.h"

BT_REGISTER_NODES(factory)
{
  MonitoringNodes::registerNodes(factory);
}

namespace MonitoringNodes
{

BT::NodeStatus Monitoring::onRunning()
{
  //Multithreaded operation: Lock aggregated values
  std::lock_guard<std::mutex> lock(_agg_mutex);

  //If no value received yet, return RUNNING until we have some values
  if(_mon_aggregated.empty())
    return BT::NodeStatus::RUNNING;

  //Update the groups that are currently blocked
  setBlockedFilter();

  //Get the value with largest error level. Prefer younger values over older values
  auto monitoring_value = getMostRelevantMonitoringVal();

  //Write the values to the blackboard
  setOutput("error_level", double(monitoring_value.msg.errorlevel));
  setOutput("key", monitoring_value.msg.key);
  setOutput("description", monitoring_value.description);
  setOutput("blocked_groups", _blocked_groups);
  //ROS_INFO_STREAM("Processing key"<<monitoring_value.msg.key <<" desc "<<monitoring_value.description<<" lvl "<<monitoring_value.msg.errorlevel);

  //Clear all received values to avoid fill up with nonrelevant values
  _mon_aggregated.clear();

  return BT::NodeStatus::SUCCESS;
}

void Monitoring::setBlockedFilter()
{
  try {
    //Get the groups from blackboard which are currently blocked
    _blocked_groups = cmr_bt::checkInput(getInput<StartStopNodes::stt>("blocked_groups"));
  } catch (BT::RuntimeError) {
    //If port not yet written, do nothing
    return;
  }
  //If there are no blocked groups, we do not need to do anything
  if(_blocked_groups.empty()) return;

  //Iterate through maps of blocked groups (string -> ros::Time)

  for (auto it = _blocked_groups.cbegin(); it != _blocked_groups.cend();)
  {
    //Get launch file from group
    const auto &launch = _maps->group_to_launch.at(it->first);
    //Get delay for launch file
    const auto &blocked_time = _maps->launch_to_delay.at(launch.second);

    if ((ros::Time::now() - it->second).toSec() > blocked_time)
    {
      //If blocked element is older than delay time from config, delete from blocked map
      it = _blocked_groups.erase(it);
    }
    else
    {
      ++it;
    }
  }
}

void Monitoring::reset()
{
  //Reset all values and resubscribe to monitoring
  if(!_reset_done){ //Only do this once after returning from IDLE
    //_level = 0;
    //_key = "";
    //_description = "";
    // _status_found = false;
    //Check which groups are blocked
    setBlockedFilter();
    //subscribe();
  }
  _reset_done = true;
}

Monitoring::MonitoringValues Monitoring::getMostRelevantMonitoringVal()
{
  MonitoringValues relevant_val;
  // Start with negative error level, since smallest error level to receive is zero
  relevant_val.msg.errorlevel = -1;
  //Iterate through aggregated values. Stop when max errorlevel found
  //If no value has max value (1.0) the youngest message with largest value is used
  while(!_mon_aggregated.empty() && relevant_val.msg.errorlevel < 1.){

    //Start with last added (youngest) value
    auto top = _mon_aggregated.back();
    try {
      //Do not consider values with equal level with already found value
      if(top.msg.errorlevel <= relevant_val.msg.errorlevel)
        throw 0;
      if(top.description == "nodemonitor"){
        try {
          auto group = _maps->groupByKey(top.msg.key);
          if(_blocked_groups.find(group) != _blocked_groups.end()){
            throw 1; //If group is blocked, do not consider this value
          }
        } catch (BT::RuntimeError) {} //If no group is found for key, this is fine
      }
      relevant_val = top;
    } catch (int) {}
    _mon_aggregated.pop_back();
  }
  return relevant_val;
}

void Monitoring::subCallback(const monitoring_msgs::MonitoringArray::ConstPtr &msg)
{
  //Multithreaded operation: Lock aggregated values
  std::lock_guard<std::mutex> lock(_agg_mutex);
  MonitoringValues values;
  //Iterate through monitors
  for(const auto &info : msg->info){
    values.description = info.description;
    //Iterate through values
    for(const auto &value : info.values){
      values.msg = value;
      // Convert key to a valid ROS String
      values.msg.key = complyString(values.msg.key);
      //ROS_INFO_STREAM("Pushing key"<<values.msg.key <<" desc "<<values.description<<" lvl "<<values.msg.errorlevel);

      _mon_aggregated.push_back(values);
    }
  }
}


BT::NodeStatus StatusToParamServer::tick()
{
  //Get Input keys
  auto rewrite_stamp = cmr_bt::checkInput(getInput<bool>("rewrite_stamp"));
  auto key = cmr_bt::checkInput(getInput<std::string>("key"));
  auto description = cmr_bt::checkInput(getInput<std::string>("description"));

  this->stringsFromKey(key);


  //Only write stamp if not written yet, or key says so
  //Used to not rewrite the stamp for reoccuring errors
  //Stamp will still be rewritten by info/warn
  if(rewrite_stamp || !_node->hasParam(_param_stamp)){
    _node->setParam(_param_stamp, ros::Time::now().toSec());
    ROS_DEBUG_STREAM("StatusToParamServer: Writing '"<<_param_stamp<<"'");
  }
  ROS_DEBUG_STREAM("StatusToParamServer: Writing '"<<_param_desc<<"'");
  _node->setParam(_param_desc, description);
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus ErrorIsNew::tick()
{
  auto key = cmr_bt::checkInput(getInput<std::string>("key"));
  auto description = cmr_bt::checkInput(getInput<std::string>("description"));

  if(!_node->hasParam("max_dur_error")) throw BT::RuntimeError("Max error duration not set");
  double max_dur, time_insert;
  ros::Time t;


  std::string desc;
  this->stringsFromKey(key);

  _node->getParam(_param_stamp, time_insert);
  _node->getParam("max_dur_error", max_dur);
  _node->getParam(_param_desc, desc);

  t.fromSec(time_insert);

  if((ros::Time::now() - t).toSec() < max_dur){
    return BT::NodeStatus::SUCCESS;
  }
  else {
    return BT::NodeStatus::FAILURE;
  }

}

BT::NodeStatus DeleteStatus::tick()
{
  auto key = cmr_bt::checkInput(getInput<std::string>("key"));
  this->stringsFromKey(key);

  if(!_node->hasParam(_param_desc) || !_node->hasParam(_param_stamp)){
    ROS_ERROR_STREAM("Can not delete params '"<<_param_desc<<"' and '"<<_param_stamp<<"'. Parameters do not exist.");
    return BT::NodeStatus::FAILURE;
  }

  _node->deleteParam(_param_desc);
  _node->deleteParam(_param_stamp);
  ROS_DEBUG_STREAM("Deleted parameters '"<<_param_desc<<"' and '"<<_param_stamp<<"'");


  return BT::NodeStatus::SUCCESS;
}


BT::NodeStatus ReconfigureMonitoring::tick()
{

  //Get the input which defines the set of parameters to load
  auto parameter_set = cmr_bt::checkInput(getInput<std::string>("parameter_set"));
  monitoring_msgs::SetParameters params;

  //Lambda to call a client and check if params were changed successfully
  auto call_and_check = [&] (auto &client, auto &srv){
    if(client.call(srv)){
      if(srv.response.params_set){
        ROS_INFO_STREAM("Reconfigured monitoring to '"<<parameter_set<<"'");
        setOutput("parameter_set_out", parameter_set);
        return BT::NodeStatus::SUCCESS;
      }
      else return BT::NodeStatus::FAILURE;
    }
    else return BT::NodeStatus::FAILURE;
  };

  //Load monitoring params for normal operation
  if(parameter_set == "OPERATING"){

    params.request.param_server_name = "nodes_operating";
    return call_and_check(_srv_client_node, params);

  }
  //Load monitoring parameters for charging
  else if(parameter_set == "CHARGING"){
    params.request.param_server_name = "nodes_charging";
    return call_and_check(_srv_client_node, params);
  }
  else {
    ROS_ERROR_STREAM("Parameter set '"<<parameter_set<<"' not defined. Monitoring not reconfigured");
    return BT::NodeStatus::FAILURE;
  }

}

BT::NodeStatus SetKillGroups::tick()
{

  if(!_configured){
    //Get a ROS parameter if it exists
    auto param_if_exists = [&](const std::string &param, std::vector<std::string> &nodes){
      if(!_node->hasParam(param)) return false;
      _node->getParam(param, nodes);
      return true;
    };
    _kill_groups.clear();

    auto parameter_set = cmr_bt::checkInput(getInput<std::string>("parameter_set"));
    std::vector<std::string> nodes_operating, nodes_charging;

    //Get vectors of monitored nodes for all cases
    if(!param_if_exists("/monitors/ros/node_monitor/nodes_operating", nodes_operating)) return BT::NodeStatus::FAILURE;
    if(!param_if_exists("/monitors/ros/node_monitor/nodes_charging", nodes_charging)) return BT::NodeStatus::FAILURE;

    //Convert node names to groups. Creates a set, so that each group is only included once
    auto set_operating = groupSetFromNodes(nodes_operating);
    auto set_charging = groupSetFromNodes(nodes_charging);

    //Based on the parameter set, determine groups to be killed by comparing which nodes are not monitored anymore
    if(parameter_set == "OPERATING"){
      //Add the groups that are in 'charging' but not in 'operating' to set of groups to kill
      std::set_difference(set_charging.begin(), set_charging.end(), set_operating.begin(), set_operating.end(),
                          std::inserter(_kill_groups, _kill_groups.begin()));

    }
    else if(parameter_set == "CHARGING"){
      //Add the groups that are in 'operating' but not in 'charging' to set of groups to kill
      std::set_difference(set_operating.begin(), set_operating.end(), set_charging.begin(), set_charging.end(),
                          std::inserter(_kill_groups, _kill_groups.begin()));
    }
    else {
      ROS_ERROR_STREAM("Parameter set '"<<parameter_set<<"' not defined. No Group to kill");
      return BT::NodeStatus::FAILURE;
    }
    _configured = true;
  }

  //Return FAILURE if no nodes left to kill
  if(_kill_groups.empty()){
    _configured = false;
    return BT::NodeStatus::FAILURE;
  }

  //Set next group to kill to output, then erase
  auto set_it = _kill_groups.begin();
  setOutput("curr_kill_group", *set_it);
  _kill_groups.erase(*set_it);

  //Return SUCCESS when new node was written to output
  return BT::NodeStatus::SUCCESS;



}

std::set<std::string> SetKillGroups::groupSetFromNodes(const std::vector<std::string> &nodes)
{
  std::set<std::string> group_set;
  //Iterate through nodes, get group name from map and insert into a set
  for(const auto& node : nodes){
    std::string group = _maps->nsnode_to_group.at(node);
    group_set.insert(group);
  }
  return group_set;
}

}




