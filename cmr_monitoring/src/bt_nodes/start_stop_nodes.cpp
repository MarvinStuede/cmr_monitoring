#include "cmr_monitoring/bt_nodes/start_stop_nodes.h"

BT_REGISTER_NODES(factory)
{
  StartStopNodes::registerStartStop(factory);
}

namespace StartStopNodes{
BT::NodeStatus StartLaunchFile::tick()
{
  if(!_sent){
    auto pkg = cmr_bt::checkInput(getInput<std::string>("pkg"));
    auto group = cmr_bt::checkInput(getInput<std::string>("group_block"));
    stt blocked_groups = cmr_bt::checkInput(getInput<stt>("blocked_groups"));
    auto launch_file = cmr_bt::checkInput(getInput<std::string>("launch_file"));


    //if no server is present, fail
    if (!_client.waitForServer(ros::Duration(4.0))) {
      ROS_ERROR("Can't contact roslaunch_axserver");
      return BT::NodeStatus::FAILURE;
    }

    //Reset this flag
    _aborted = false;
    ROS_INFO_STREAM("Starting file '"<<launch_file<<"' in '"<<pkg<<"'");
    _goal.pkg = pkg;
    _goal.launch_file = launch_file;
    _client.sendGoal(_goal);
    _sent = true;
    blocked_groups[group] = ros::Time::now();
    setOutput("blocked_groups", blocked_groups);
  }

  if (_aborted) {
    // this happens only if method halt() was invoked
    _client.cancelAllGoals();
    ROS_ERROR("Start Launch aborted");
    return BT::NodeStatus::FAILURE;
  }

  if(_client.getState() != actionlib::SimpleClientGoalState::ACTIVE)
    setStatusRunningAndYield();


  _sent = false;
  return BT::NodeStatus::SUCCESS;



}


BT::NodeStatus KillROSNode(BT::TreeNode &self)
{
  auto msg = self.getInput<std::string>("node");
  if(!msg.has_value()){
    throw BT::RuntimeError("error reading port [node]:", msg.error());
  }
  ROS_INFO_STREAM("Killing node "<<msg.value());
  bool killed = killnode(msg.value());
  if(killed){
    ROS_INFO_STREAM("Node '"<<msg.value()<<"' killed");
    return BT::NodeStatus::SUCCESS;
  }
  else{
    ROS_WARN_STREAM("Node '"<<msg.value()<<"' not running and therefore not killed");
    return BT::NodeStatus::FAILURE;
  }
}

BT::NodeStatus LaunchFileFromGroup::tick()
{
  std::string group = cmr_bt::checkInput(getInput<std::string>("group"));

  if(_maps->group_to_launch.find(group) == _maps->group_to_launch.end()){
    ROS_ERROR_STREAM("Group '"<<group<<"' not found in launch file map");
    return BT::NodeStatus::FAILURE;
  }
  const auto& launch_pair = _maps->group_to_launch.at(group);
  ROS_DEBUG_STREAM("LaunchFileFromGroup: Receiving '"<<launch_pair.second<<"' in '"<<launch_pair.first<<"' from group '"<<group<<"'");
  setOutput("pkg", launch_pair.first);
  setOutput("launch_file", launch_pair.second);
  return BT::NodeStatus::SUCCESS;
}


BT::NodeStatus KillNodeFromGroup::tick()
{
  std::string group = cmr_bt::checkInput(getInput<std::string>("group"));

  if(_maps->group_to_kill.find(group) == _maps->group_to_kill.end()){
    ROS_ERROR_STREAM("Group '"<<group<<"' not found in kill map");
    return BT::NodeStatus::FAILURE;
  }

  const auto& kill_node = _maps->group_to_kill.at(group);
  ROS_DEBUG_STREAM("KillNodeFromROSNode: Receiving '"<<kill_node<<"' from group '"<<group<<"'");

  setOutput("kill_node", kill_node);
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus GroupFromKey::tick()
{
  auto key = cmr_bt::checkInput(getInput<std::string>("key"));
  std::string group;
  try {
    group = _maps->groupByKey(key);
  } catch (BT::RuntimeError) {
    return BT::NodeStatus::FAILURE;
  }
  setOutput("group", group);
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus GroupFromTopic::tick()
{
  auto topic = cmr_bt::checkInput(getInput<std::string>("topic"));
  const auto& group = _maps->top_to_group.at(topic);
  ROS_DEBUG_STREAM("ROSNodeFromTopic: Receiving '"<<group<<"' from topic '"<<topic<<"'");
  setOutput("group", group);
  return BT::NodeStatus::SUCCESS;
}


std::string NodeMapBase::receiveGroupName()
{
  std::string group;
  try {
    group = cmr_bt::checkInput(getInput<std::string>("group"));
    if (group=="NONE") throw BT::RuntimeError();
  } catch (BT::RuntimeError) {
    auto key = cmr_bt::checkInput(getInput<std::string>("key"));
    group = _maps->groupByKey(key);

  }
  return group;
}

BT::NodeStatus SetDelayAfterLaunch::tick()
{
  auto launch_file = cmr_bt::checkInput(getInput<std::string>("launch_file"));
  unsigned int delay;
  if(_maps->launch_to_delay.find(launch_file) == _maps->launch_to_delay.end()){
    ROS_WARN_STREAM("No mapping to delay found for launch_file '"<<launch_file<<"'. Setting to 10s");
    delay = 10000;
  }
  else{
    delay = _maps->launch_to_delay.at(launch_file) * 1000;
  }
  ROS_INFO("Blocking for %d s after launch", delay/1000);
  setOutput("delay", delay);
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus GroupNotBlocked::tick()
{
  std::string group = cmr_bt::checkInput(getInput<std::string>("group"));
  stt blocked_groups = cmr_bt::checkInput(getInput<stt>("blocked_groups"));
  if(blocked_groups.find(group) == blocked_groups.end())
    return BT::NodeStatus::SUCCESS;
  else
    return BT::NodeStatus::FAILURE;
}




}
