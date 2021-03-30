#include "cmr_monitoring/monitoring_bt.h"

#include <behaviortree_cpp_v3/bt_factory.h>
#include "behaviortree_cpp_v3/loggers/bt_cout_logger.h"
#include "behaviortree_cpp_v3/loggers/bt_file_logger.h"
#include "behaviortree_cpp_v3/loggers/bt_minitrace_logger.h"
#include "behaviortree_cpp_v3/loggers/bt_zmq_publisher.h"
#include "cmr_monitoring/bt_nodes/start_stop_nodes.h"
#include "cmr_monitoring/bt_nodes/monitoring_nodes.h"
#include "cmr_bt_generic/system_nodes.h"
#include "cmr_bt_generic/general.h"
#include "cmr_bt_generic/social_nodes.h"
#include "behaviortree_cpp_v3/utils/shared_library.h"
#include "behaviortree_ros/loggers/ros_topic_logger.h"
/**
  This is the implementation of the monitoring BT as a ROS node.
 */

//########## CONSTRUCTOR ###############################################################################################
MonitoringBT::MonitoringBT(ros::NodeHandle &node_handle):
  node_(&node_handle)
{
}

/**
* printError prints an error if the tick is not executed or doesn´t any reply from the nodes in a defined timeinterval
* lag_time is the corresponding lag time
*/
void printError(int &lag_time)
{
  ROS_FATAL("Tick is not executed in defined interval!");
  ROS_FATAL("Lag Time is currently: %i ms",lag_time);
}

//########## MAIN ######################################################################################################
int main(int argc, char** argv)
{
  ros::init(argc,argv,"monitoring_bt");
  ros::NodeHandle node_handle("~");
  MonitoringBT monitoring_bt(node_handle);

  // We use the BehaviorTreeFactory to register our custom nodes
  BT::BehaviorTreeFactory factory;

  //Register the nodes from plugins
  const std::vector<std::string> plugin_libs = {
    "cmr_bt_start_stop_nodes",
    "cmr_bt_monitoring_conditions",
    "cmr_bt_monitoring_nodes",
    "cmr_bt_general_nodes",
    "cmr_bt_system_nodes",
    "cmr_bt_social_nodes",
  };
  BT::SharedLibrary loader;
  for(const auto &plugin : plugin_libs)
    factory.registerFromPlugin(loader.getOSName(plugin));


  // Build Tree from File
  std::string ppath = ros::package::getPath("cmr_monitoring");
  auto tree = factory.createTreeFromFile(ppath+"/monitoring_main.xml");

  // Uncomment the next line to show the status of BT nodes in real time
  //BT::StdCoutLogger logger_cout(tree);

  // This logger publish status changes using ZeroMQ. Used by Groot
  BT::PublisherZMQ publisher_zmq(tree);

  // This logger stores the execution time of each node
  //std::string path_trace = ppath + "/bt_trace.json";
  //BT::MinitraceLogger logger_minitrace(tree, path_trace.c_str());
  BT::RosTopicLogger logger_rostopic(node_handle, tree);


  ros::Rate r(3);
  bool first = true;
  ros::Time time_last_log = ros::Time(0);
  ros::Duration dur_flush_log = ros::Duration(5.0);
  ros::AsyncSpinner spinner(4);
  spinner.start();
  // Execute behavior Tree until ros stops
  while(ros::ok())
  {
    //ros::spinOnce();
    tree.rootNode()->executeTick();

    r.sleep();

    if(ros::Time::now() > (time_last_log + dur_flush_log)){
      //Publish state changes to topic
      logger_rostopic.flush();
      time_last_log = ros::Time::now();
    }
    first = false;
  }
  ros::waitForShutdown();
  return 0;
}
