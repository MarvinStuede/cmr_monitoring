cmake_minimum_required(VERSION 3.5.1)
project(cmr_monitoring)
set(CMAKE_CXX_FLAGS "-Wl,--no-undefined")
set(CMAKE_CXX_FLAGS "-std=c++14 ${CMAKE_CXX_FLAGS}")
set(CMAKE_DEBUG_POSTFIX "")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  std_srvs
  behaviortree_cpp_v3
  behaviortree_ros
  message_generation
  monitoring_msgs
  monitoring_core
  roslaunch_axserver
  actionlib_msgs
  actionlib
  cmr_api
  cmr_bt_generic
  rtabmap_ros
  neo_msgs
  rosgraph_msgs
)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES cmr_bt_start_stop_nodes cmr_bt_monitoring_conditions cmr_bt_monitoring_nodes
    CATKIN_DEPENDS
    roscpp
    std_msgs
    std_srvs
    monitoring_msgs
    monitoring_core
    cmr_api
    rtabmap_ros
    )

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${cmr_api_INCLUDE_DIRS}
  ${cmr_bt_generic_INCLUDE_DIRS}
)
add_service_files(
  FILES
  KillNode.srv
)

# BehaviorTree.CPP Plugins

function(set_plugin_properties plugin)
    target_link_libraries(${plugin} PRIVATE
      ${catkin_LIBRARIES}
      ${BEHAVIOR_TREE_LIBRARY}
    )
    target_include_directories(${plugin} PUBLIC ${catkin_INCLUDE_DIRS})
    target_compile_definitions(${plugin} PRIVATE  BT_PLUGIN_EXPORT )
    set_target_properties(${plugin} PROPERTIES ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_LIBRARY_OUTPUT_DIRECTORY} )
endfunction()

add_library(cmr_bt_start_stop_nodes SHARED
  src/bt_nodes/start_stop_nodes.cpp
  include/cmr_monitoring/bt_nodes/start_stop_nodes.h
  src/launch_tools.cpp
  include/cmr_monitoring/launch_tools.h
)

set_plugin_properties(cmr_bt_start_stop_nodes)

add_library(cmr_bt_monitoring_conditions SHARED
    include/cmr_monitoring/bt_nodes/monitoring_conditions.h
    src/bt_nodes/monitoring_conditions.cpp
    src/launch_tools.cpp

)

set_plugin_properties(cmr_bt_monitoring_conditions)

add_library(cmr_bt_monitoring_nodes SHARED
    src/bt_nodes/monitoring_nodes.cpp
    include/cmr_monitoring/bt_nodes/monitoring_nodes.h
     src/launch_tools.cpp
      include/cmr_monitoring/launch_tools.h
)

set_plugin_properties(cmr_bt_monitoring_nodes)

add_executable(monitoring_bt
  src/monitoring_bt.cpp
)


target_link_libraries(monitoring_bt
  ${catkin_LIBRARIES}
  ${BEHAVIOR_TREE_LIBRARY}
)

# Custom monitors
add_executable(rtabmap_monitor src/monitoring_monitors/rtabmap_monitor.cpp)

add_dependencies(rtabmap_monitor ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(rtabmap_monitor
    ${catkin_LIBRARIES}
)

add_executable(move_base_monitor src/monitoring_monitors/movebase_monitor.cpp)

add_dependencies(move_base_monitor ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(move_base_monitor
    ${catkin_LIBRARIES}
)
