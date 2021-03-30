# CMR MONITORING

CMR Monitoring is a Repository for a monitoring and controlling interface for ROS-Systems. It contains three ROS-Packages.
* cmr_monitoring - contains a monitoring interface as a Behavior Tree and using the Monitors of the monitoring Package.
* cmr_logging - contains configurations and starters to log ros messages to a mongo db
* cmr_gui_bridge - contains the user interface as a ROS Web Tool that helps monitor and control the robot remotely.



In the following all packages are presented in detail.
This Package is based on the messages of [the monitoring](https://github.com/luhrts/monitoring) package. The Monitors has to be configured to correspond to the robot specifications. The configured version of the package is located under this [fork](https://github.com/MarvinStuede/monitoring).

The Behavior Tree implementation of the cmr_monitoring package is based on the library [BehaviorTreeCPP](https://github.com/BehaviorTree/BehaviorTree.CPP). This is used to model behaviors within a Behavior Tree. For a documentation of the library this [documentation](https://www.behaviortree.dev/) is recommended.

The package [roslaunch_axserver](https://github.com/strands-project/strands_apps/tree/indigo-devel/roslaunch_axserver) will also be used to implement the restart of .launch files. Further documentation can be found [here](https://github.com/strands-project/strands_apps/blob/indigo-devel/roslaunch_axserver/README.md).
## Prerequisites and Installing
See general instructions [here](https://marvinstuede.github.io/Sobi/software/).
Then install the following additional dependencies:
```
 sudo apt-get install libzmq3-dev libdw-dev
```
## CMR MONITORING
This package contains a monitoring interface as a Behavior Tree (BT) using the Monitors of the monitoring Package and a remote controlling user interface. For this, the library [BehaviorTreeCPP](https://github.com/BehaviorTree/BehaviorTree.CPP)
is used to model behaviors within a Behavior Tree.

To run the monitoring the following three components are needed:
#### MONITORING
Runs the system, ROS and custom monitors. In case of errors or warnings the aggregated values are published on the `/monitoring` topic.

The standard configuration can be started via:
```
 roslaunch cmr_monitoring monitoring.launch
```
#### AXSERVER

The axserver provides an interface to run launch files via action calls.
You can run it with the following launch file:
```
 roslaunch cmr_monitoring axserver.launch
```
#### MONITORING BEHAVIOR TREE
The BT subscribes to the `/monitoring` topic and implements recovery behavior. If nodes are not pingable or topics are not within a tolerance band, in restarts the corresponding nodes. Which nodes should be restarted (i.e. which launch files should be started) in case a node is not pingable is defined via mappings.
The mappings are defined in `cmr_monitoring/cfg/mappings.yaml`. Each node belongs to a specified group. Each group then maps to a specific shutdown node to kill (which will quit a launch process) and a launch file to restart afterwards.
The BT also implements recoveries for lost localization and navigation errors. You can check the file `monitoring_main.xml` with [Groot](https://github.com/BehaviorTree/Groot) to see how the recoveries are implemented.

You can run the Monitoring Behavior Tree via:
```
 roslaunch cmr_monitoring monitoring_bt.launch
```

For testing: try to kill the imu node by
```
 rosnode kill /imu_shutdown
```
then the monitoring BT will detect the missing node and restart the roslaunch file.
## CMR GUI BRIDGE
This package contains the user interface as a ROS Web Tool that helps monitor and control the robot remotely. It has :
1. Graphical interface to simplify the remote controlling and the monitoring of the robot.
2. Connection status field.
3. Joystick for remote driving manoeuvres
4. Live video strems of the front and back cameras of the robot

Start the Webserver and all corresponding nodes via
```
 roslaunch cmr_gui_bridge web_gui.launch
```


## CMR LOGGING
This package heavily depends on the [mongodb_store](https://github.com/MarvinStuede/mongodb_store) package which was forked from the STRANDS project. The package can be used to store ROS messages to a local or remote MongoDB Server for persistent storage. The fork adds MongoDB authentication and SSL encrypted data transport.
The cmr_logging package contains configurations of the topics to log and utility programs to run throttlers if messages should not be stored with full frequency.
The folder `cmr_logging/cfg` contains yaml files with topics to log. Config files with a `*_unthrottled.yaml` suffix contain the topics that should be stored with full frequency. Config files with a `*_throttled.yaml` suffix contain topics that should be throttled and therefore define different groups with frequencies the topics should be throttled to.
In order for the package to work you need to provide Server and User certificates and create a config file inside the `cmr_logging/cfg folder` named `mongodb_config.yaml` with the following content
```
mongodb_name: 'DB_NAME'
mongodb_host: 'SERVER_HOSTNAME'
mongodb_port: PORT # Standard port is 27017
mongodb_username: 'MONGODB_USERNAME'
mongodb_password: 'MONGODB_PASSWORD'
mongodb_authsource: 'MONGODB_AUTH_DATABASE'
mongodb_certfile: 'PATH_TO_USER_CERTIFICATE'
mongodb_ca_certs: 'PATH_TO_CA_CERTIFICATE'
```

The following launch file can be used to start the logging processes:
```
 roslaunch cmr_logging log_to_db.launch
```
This will also start the throttling nodes (one node for each topic)

If you want to download the saved topics to a rosbag, you can use the following launch file. Check the launch file for config options
```
 roslaunch cmr_logging mongodb_to_rosbag.launch PARAMETERS...
```
