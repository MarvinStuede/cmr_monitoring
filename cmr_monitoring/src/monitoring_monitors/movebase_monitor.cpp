
#include "cmr_monitoring/monitoring_monitors/movebase_monitor.h""


MoveBaseMonitor::MoveBaseMonitor(ros::NodeHandle &node_handle):
  node_(&node_handle)
{
  monitor_ = std::make_unique<Monitor>(node_handle, "MoveBase-Monitor");
  // === SUBSCRIBER ===
  sub_log_ = node_->subscribe("/rosout_agg", 10, &MoveBaseMonitor::logCallback, this);
  sub_mbfeedback_ = node_->subscribe("/move_base/feedback", 10, &MoveBaseMonitor::mbfeedbackCallback, this);


  // === TIMER ===
  tim_mon_ = node_->createTimer(ros::Duration(0.3), &MoveBaseMonitor::timerCallback, this);
  not_updated_.first = false;
  not_updated_.second = 0;
  node_->param<int>("warn_dur_not_updated", warn_dur_not_updated_, 8);
  node_->param<int>("error_dur_not_updated", err_dur_not_updated_, 20);
  node_->param<int>("warn_dur_no_feedback", warn_dur_no_feedback_, 30);
  node_->param<int>("error_dur_no_feedback", err_dur_no_feedback_, 60);
}


void MoveBaseMonitor::logCallback(const rosgraph_msgs::LogConstPtr &log)
{
  if(log->name == "/move_base" && (log->msg.find("observation buffer has not been updated for")!=std::string::npos))
  {
    not_updated_.first = true;
    not_updated_.second = getTimeNotUpdated(log);
    
  }
}

void MoveBaseMonitor::cmdvelCallback(const geometry_msgs::TwistConstPtr &vel)
{

}

void MoveBaseMonitor::mbfeedbackCallback(const move_base_msgs::MoveBaseActionFeedbackConstPtr &mbfeedback)
{
	time_last_mbfb_ = ros::Time::now();
}


void MoveBaseMonitor::timerCallback(const ros::TimerEvent &evt)
{
  double error_not_updated = getErrorValNotUpdated(not_updated_);
  monitor_->addValue("move_base_monitor", not_updated_.second,"s", error_not_updated, AggregationStrategies::MAX);

  double error_no_feedback = getErrorValNoFeedback(time_last_mbfb_);
  double dur_last_fb = (ros::Time::now() - time_last_mbfb_).toSec();
  monitor_->addValue("move_base_monitor_fb", dur_last_fb ,"s", error_no_feedback, AggregationStrategies::MAX);

  not_updated_.first = false;
  not_updated_.second = 0;

}

double MoveBaseMonitor::getErrorValNotUpdated(const std::pair<bool, int> &not_updated)
{

  if(!not_updated.first){
    return 0.0;
  }
  else  if(not_updated.first){
    if(not_updated.second > err_dur_not_updated_)
      return 1.0;
    else if(not_updated.second > warn_dur_not_updated_)
      return 0.5;
    else {
      return 0.0;
    }

  }
}

double MoveBaseMonitor::getErrorValNoFeedback(const ros::Time &time_last_fb)
{
  //If there is no publisher (MoveBase is off, there is no error
  if(sub_mbfeedback_.getNumPublishers() == 0){
    return 0.0;
  }

  auto time_passed = [&](int dur){
    return ros::Time::now() > (time_last_fb + ros::Duration(double(dur)));
  };

  //Check if warn or error duration was surpassed
  if(time_passed(err_dur_no_feedback_))
    return 1.0;
  else if(time_passed(warn_dur_no_feedback_))
    return 0.5;
  else {
    return 0.;
  }
}

int MoveBaseMonitor::getTimeNotUpdated(const rosgraph_msgs::LogConstPtr &log)
{
  int observ_buff_time_not_updated = 0;
  std::string str = "";
  for (int i=0; i<strlen(log->msg.c_str()); i++)
  {
    if (isdigit(log->msg[i]))
    {
      str += log->msg[i];
    }
    else if(log->msg[i] == '.')
      break;
  }

  observ_buff_time_not_updated = atof(str.c_str());

  return observ_buff_time_not_updated;
}




int main(int argc, char** argv)
{
  ros::init(argc, argv, "move_base_monitor");

  ros::NodeHandle node_handle;
  MoveBaseMonitor my_node(node_handle);

  ros::spin();

  return 0;
}
