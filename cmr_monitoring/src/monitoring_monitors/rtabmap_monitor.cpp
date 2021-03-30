//#include <ros/ros.h>
//#include <monitoring_core/monitor.h>

//int main(int argc, char **argv)
//{
//  ros::init(argc, argv, "simple_monitor");
//  ros::NodeHandle nh;

//  Monitor monitor(nh, "simple_monitor_example");

//  ros::Rate loop_rate(10);
//  int c = 0;
//  while (ros::ok()){
//    monitor.addValue("count", c, "SIUnits", 0.0);
//    c++;
//    ros::spinOnce();
//    loop_rate.sleep();
//  }


//  ROS_INFO("Hello world!");
//}

#include "cmr_monitoring/monitoring_monitors/rtabmap_monitor.h""

//########## CONSTRUCTOR #####################################################################################
RTABMAPMonitor::RTABMAPMonitor(ros::NodeHandle &node_handle):
  node_(&node_handle)
{
  monitor_ = std::make_unique<Monitor>(node_handle, "RTABMAP-Monitor");
  // === SUBSCRIBER ===
  sub_rt_info_ = node_->subscribe("/rtabmap/info", 10, &RTABMAPMonitor::rtInfoSubCallback, this);

  // === TIMER ===
  tim_mon_ = node_->createTimer(ros::Duration(0.1), &RTABMAPMonitor::timerCallback, this);

  countdown=std::make_pair(false, ros::Time(0));
  node_->param<float>("warn_dur", warn_dur_, 10.);
  node_->param<float>("error_dur", err_dur_, 20.);
}

//########## CALLBACK: SUBSCRIBER ############################################################################
void RTABMAPMonitor::rtInfoSubCallback(const rtabmap_ros::InfoConstPtr &info_msg)
{
  auto el_ptr = std::find(std::begin(info_msg->statsKeys), std::end(info_msg->statsKeys), "Loop/Id/");
  if (el_ptr == std::end(info_msg->statsKeys)) {
    ROS_ERROR("String not in array");
    return;
  }
  unsigned int index = std::distance(info_msg->statsKeys.begin(), el_ptr);
  loop_id_ = info_msg->statsValues[index];
  if(loop_id_ == 0.){
    if(!countdown.first){
      countdown.first = true;
      countdown.second = ros::Time::now();
    }
  }
  else countdown.first = false;

}

//########## CALLBACK: TIMER #################################################################"################
void RTABMAPMonitor::timerCallback(const ros::TimerEvent &evt)
{

  //Only proceed if LoopID is zero
  if(!countdown.first){
    monitor_->addValue("rtabmap_loop", loop_id_,"ID",0.0);
    return;
  }
  float sec_since_loop0 = (ros::Time::now() - countdown.second).toSec();
  if(sec_since_loop0 > err_dur_)
    monitor_->addValue("rtabmap_loop", loop_id_,"ID",1.0);
  else if(sec_since_loop0 > warn_dur_)
    monitor_->addValue("rtabmap_loop", loop_id_,"ID",0.5);


}



//########## MAIN ############################################################################################
int main(int argc, char** argv)
{
  ros::init(argc, argv, "rtabmap_monitor");

  ros::NodeHandle node_handle;
  RTABMAPMonitor my_node(node_handle);

  ros::spin();

  return 0;
}
