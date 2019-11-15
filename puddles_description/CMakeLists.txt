cmake_minimum_required(VERSION 2.8.3)
project(puddles_description)

## Add support for C++11, supported in ROS Kinetic and newer
# add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS 
   common_robot_description
   xacro
)

catkin_package(
   CATKIN_DEPENDS 
      common_robot_description
      xacro
)