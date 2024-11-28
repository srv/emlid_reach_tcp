cmake_minimum_required(VERSION 3.0.2)
project(emlid_reach_tcp)

find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
)


catkin_package(

)


include_directories(
  ${catkin_INCLUDE_DIRS}
)
