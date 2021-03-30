cmake_minimum_required(VERSION 2.8.3)
project(cmr_logging)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  mongodb_store
  topic_tools
)

catkin_package(
    CATKIN_DEPENDS mongodb_store topic_tools
    )

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${topic_tools_INCLUDE_DIRS}
)

add_executable(latch_republisher
  src/latch_republisher.cpp
)
target_link_libraries(latch_republisher
  ${catkin_LIBRARIES}
)
