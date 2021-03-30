
#include "cmr_monitoring/launch_tools.h"

bool killnode(std::string node){
  std::string command = std::string("rosnode kill ")+ node;
  std::string stdcout = exec(command);
  ros::Duration(0.5).sleep();
  //If output contains this, the node was not found
  if (stdcout.find("ERROR:") != std::string::npos) {
    return false;
  }
  else {
    return true;
  }


}

std::string getBaseNodeName(std::string full_name)
{
  full_name[0]='0';
  std::string base_name;
  if (full_name.find("/")){base_name=full_name.substr(1, full_name.find("/")-1);}
  else {base_name=full_name.substr(1,full_name.length()-1);}
  return base_name;
}

std::string exec(std::string cmd) {
  std::string data;
  FILE * stream;
  const int max_buffer = 256;
  char buffer[max_buffer];
  cmd.append(" 2>&1");

  stream = popen(cmd.c_str(), "r");

  if (stream) {
    while (!feof(stream))
      if (fgets(buffer, max_buffer, stream) != NULL) data.append(buffer);
    pclose(stream);
  }
  return data;
}


std::string complyString(std::string str)
{
  //ROS forbids '-' in param names
  std::replace( str.begin(), str.end(), '-', '_');
  //Always start with '/'
  if(str[0] != '/') str.insert(str.begin(),'/');
  return str;
}
