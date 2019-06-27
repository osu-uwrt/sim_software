cmake_minimum_required(VERSION 2.8.3)
project(riptide_simulation)
find_package(catkin REQUIRED)

catkin_package(
   CATKIN_DEPENDS
      uuv_sensor_ros_plugins_msgs
      nortek_dvl
      geometry_msgs
      std_msgs
)